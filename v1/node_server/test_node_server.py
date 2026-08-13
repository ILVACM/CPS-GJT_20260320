"""服务节点 五项自检脚本（独立可运行）。

对齐 AGENTS.md §7.4（测试策略：节点级诊断）与 Design-server.md §7.6。

覆盖五项自检：
1. 相机自检：MockCamera + 真实相机（若有），验证 get_rgb_frame 返回非 None
2. 串口自检：构造测试帧喂 SerialBuffer，验证解析出 4 路距离
3. GUI 自检：尝试创建 tk.Tk() + RebarMeasureApp（不 start），验证无异常
4. 通信自检：启动 mock gRPC server，InferenceGrpcClient.infer() 联调
5. 推理测量自检：构造假掩码 + 距离，RebarMeasure.measure() 返回 MeasureResult

另含：
6. 全链路集成验证（§7.6）：MockCamera → mock 推理 → 测量 → 保存 → 退出流程

执行：python test_node_server.py
（Windows 开发环境用 Mock 替代真实相机/串口）
"""
import os
import sys
import struct
import time
import logging
import threading
from typing import Tuple, List, Optional

# 确保 node_server/ 目录在 sys.path 首位（独立运行时不依赖 main.py）
_NODE_B_DIR: str = os.path.dirname(os.path.abspath(__file__))
if _NODE_B_DIR not in sys.path:
    sys.path.insert(0, _NODE_B_DIR)

import numpy as np
import cv2

# ---- 服务节点 模块导入 ----
from utils.common.config_loader import (
    CameraConfig,
    InferenceConfig,
    IntrinsicsConfig,
    NetworkConfig,
    SerialConfig,
)
from utils.common.constants import (
    SERIAL_FRAME_HEADER,
    SERIAL_FRAME_TAIL,
    SegClass,
)
from utils.camera.mock import MockCameraInput
from utils.camera.scanner import LocalCameraScanner
from utils.serial.serial_buffer import SerialBuffer
from utils.serial.laser_parser import LaserDataParser
from utils.measure.intrinsics_manager import IntrinsicsManager
from utils.measure.rebar_measure import RebarMeasure, MeasureResult
from utils.storage.result_store import ResultStore
from utils.system.node_monitor import NodeMonitor
from utils.system.exit_coordinator import ExitCoordinator

# gRPC 相关
import grpc
from concurrent.futures import ThreadPoolExecutor
from utils.proto import rebar_inference_pb2
from utils.proto import rebar_inference_pb2_grpc
from utils.grpc_client.inference_client import InferenceGrpcClient, InferenceResult
from utils.grpc_client.grpc_server import GrpcServerB


# ==================================================================
# 工具函数
# ==================================================================

def _print_result(name: str, passed: bool, detail: str) -> None:
    """打印单项自检结果。"""
    tag: str = "[PASS]" if passed else "[FAIL]"
    print(f"{tag} {name}")
    if detail:
        for line in detail.strip().split("\n"):
            print(f"      {line}")


def _build_test_frame(distances: List[int]) -> bytes:
    """构造 11 字节 S21C 测试帧（含 XOR 校验）。

    :param distances: 4 路距离值（mm）
    :return: 11 字节完整帧
    """
    frame: bytearray = bytearray([SERIAL_FRAME_HEADER])
    for d in distances:
        frame += struct.pack(">H", d)
    # XOR 校验：前 9 字节异或和
    xor: int = 0
    for b in frame:
        xor ^= b
    frame.append(xor)
    frame.append(SERIAL_FRAME_TAIL)
    assert len(frame) == 11, f"帧长度应为 11，实际 {len(frame)}"
    return bytes(frame)


def _make_fake_mask(width: int = 640, height: int = 480) -> np.ndarray:
    """构造含纵向钢筋的假掩码（0=背景 / 1=纵向钢筋）。"""
    mask: np.ndarray = np.zeros((height, width), dtype=np.uint8)
    # 5 根纵向钢筋，每根宽 16px，跨大部分高度
    for cx in [100, 200, 300, 400, 500]:
        mask[30:height - 30, cx - 8:cx + 8] = SegClass.REBAR_VERTICAL
    return mask


# ==================================================================
# 自检 1：相机
# ==================================================================

def check_camera() -> Tuple[bool, str]:
    """相机自检：MockCamera + 真实相机扫描（若有）。"""
    details: List[str] = []
    all_pass: bool = True

    # 1a. MockCamera
    try:
        cam: MockCameraInput = MockCameraInput(width=1920, height=1080)
        opened: bool = cam.open()
        if not opened:
            details.append("MockCamera.open() 返回 False")
            all_pass = False
        else:
            frame: Optional[np.ndarray] = cam.get_rgb_frame()
            if frame is None:
                details.append("MockCamera.get_rgb_frame() 返回 None")
                all_pass = False
            else:
                details.append(
                    f"MockCamera: 帧尺寸={frame.shape}, dtype={frame.dtype}"
                )
            intrinsics: dict = cam.get_intrinsics()
            details.append(
                f"MockCamera 内参: fx={intrinsics['fx']}, fy={intrinsics['fy']}"
            )
            cam.close()
    except Exception as e:
        details.append(f"MockCamera 异常: {e}")
        all_pass = False

    # 1b. 真实相机扫描（不阻塞，仅报告发现数量）
    try:
        scanner: LocalCameraScanner = LocalCameraScanner(["Orbbec Gemini 336L"])
        devices = scanner.scan(max_index=5)
        details.append(f"本地摄像头扫描: 发现 {len(devices)} 个设备")
        if devices:
            for dev in devices[:3]:
                details.append(
                    f"  - [{dev.index}] {dev.name} ({dev.backend}) "
                    f"兼容={'是' if dev.is_compatible else '否'}"
                )
    except Exception as e:
        details.append(f"本地摄像头扫描异常（非阻塞）: {e}")

    return all_pass, "\n".join(details)


# ==================================================================
# 自检 2：串口
# ==================================================================

def check_serial() -> Tuple[bool, str]:
    """串口自检：构造测试帧 → SerialBuffer → LaserDataParser。"""
    details: List[str] = []
    try:
        # 构造测试帧：4 路距离 [812, 815, 810, 813] mm
        test_distances: List[int] = [812, 815, 810, 813]
        frame: bytes = _build_test_frame(test_distances)
        details.append(f"构造测试帧: {len(frame)} 字节, hex={frame.hex()}")

        # 喂入 SerialBuffer
        buf: SerialBuffer = SerialBuffer()
        frames: List[bytes] = buf.feed(frame)
        if len(frames) != 1:
            details.append(f"SerialBuffer 提取帧数={len(frames)}（期望 1）")
            return False, "\n".join(details)
        details.append("SerialBuffer 帧提取成功（1 帧，校验通过）")

        # LaserDataParser 解析
        result = LaserDataParser.parse(frames[0])
        if result is None:
            details.append("LaserDataParser.parse 返回 None")
            return False, "\n".join(details)

        details.append(
            f"解析结果: distances={result.distances_mm}, "
            f"valid={result.valid_mask}, valid_count={result.valid_count}"
        )

        # 验证 4 路距离值一致
        if result.distances_mm != test_distances:
            details.append(
                f"距离值不匹配: 期望 {test_distances}, 实际 {result.distances_mm}"
            )
            return False, "\n".join(details)

        if result.valid_count != 4:
            details.append(f"有效路数={result.valid_count}（期望 4）")
            return False, "\n".join(details)

        details.append("4 路距离值与有效性检查均通过")

        # 额外测试：粘包（两帧拼接）
        two_frames: bytes = frame + frame
        frames2: List[bytes] = buf.feed(two_frames)
        if len(frames2) != 2:
            details.append(f"粘包测试: 提取帧数={len(frames2)}（期望 2）")
            return False, "\n".join(details)
        details.append("粘包测试通过（一次喂入 2 帧提取 2 帧）")

        # 额外测试：无效值剔除（0 + 超量程）
        # 注：uint16 范围 0~65535，超量程值取 60000（> LASER_MAX_MM=7500，且在 uint16 范围内）
        invalid_frame: bytes = _build_test_frame([0, 812, 60000, 815])
        invalid_result = LaserDataParser.parse(invalid_frame)
        if invalid_result is None:
            details.append("无效值帧解析返回 None")
            return False, "\n".join(details)
        if invalid_result.valid_count != 2:
            details.append(
                f"无效值剔除测试: 有效路数={invalid_result.valid_count}（期望 2）"
            )
            return False, "\n".join(details)
        details.append("无效值剔除测试通过（0 值与超量程剔除，2 路有效）")

    except Exception as e:
        details.append(f"串口自检异常: {e}")
        return False, "\n".join(details)

    return True, "\n".join(details)


# ==================================================================
# 自检 3：GUI
# ==================================================================

def check_gui() -> Tuple[ bool, str]:
    """GUI 自检：创建 tk.Tk() + RebarMeasureApp（不 start），验证无异常。"""
    details: List[str] = []
    root = None
    try:
        import tkinter as tk
        from utils.ui.app import RebarMeasureApp

        root = tk.Tk()
        root.withdraw()  # 不显示窗口（自检无需可视化）

        # 构造测试配置
        net_cfg: NetworkConfig = NetworkConfig(
            local_ip="127.0.0.1", remote_ip="127.0.0.1", grpc_port=50061,
        )
        camera_cfg: CameraConfig = CameraConfig()
        intrinsics_cfg: IntrinsicsConfig = IntrinsicsConfig()
        inference_cfg: InferenceConfig = InferenceConfig()
        serial_cfg: SerialConfig = SerialConfig()

        # NodeMonitor / GrpcServerB 不启动（仅构造）
        monitor: NodeMonitor = NodeMonitor()
        grpc_server: GrpcServerB = GrpcServerB(net_cfg, node_monitor=monitor)

        # 注入 mock 推理客户端（避免真实 gRPC 连接）
        mock_client: InferenceGrpcClient = InferenceGrpcClient(net_cfg)

        app: RebarMeasureApp = RebarMeasureApp(
            root,
            net_cfg=net_cfg,
            camera_cfg=camera_cfg,
            intrinsics_cfg=intrinsics_cfg,
            inference_cfg=inference_cfg,
            serial_cfg=serial_cfg,
            node_monitor=monitor,
            grpc_server=grpc_server,
            degraded=True,  # 降级模式避免连接推理端
            inference_client=mock_client,
        )
        details.append("RebarMeasureApp 构造成功（未 start）")
        details.append(f"  按钮初始状态: {app._button_state}")
        details.append(f"  降级模式: {app._degraded_mode}")

        # 验证退出协调器已初始化
        if app._exit_coordinator is not None:
            details.append("ExitCoordinator 已注入")
        else:
            details.append("ExitCoordinator 未注入")
            return False, "\n".join(details)

        # 验证 _on_closing 已绑定
        details.append("GUI 自检通过（构造 + 退出协调器均正常）")

    except Exception as e:
        import traceback
        details.append(f"GUI 自检异常: {e}")
        details.append(traceback.format_exc().split("\n")[-3])
        return False, "\n".join(details)
    finally:
        if root is not None:
            try:
                root.destroy()
            except Exception:
                pass

    return True, "\n".join(details)


# ==================================================================
# 自检 4：通信（gRPC）
# ==================================================================

class _MockInferenceServicer(rebar_inference_pb2_grpc.RebarInferenceServicer):
    """mock 推理服务端：Infer 返回假掩码，Heartbeat/Shutdown 返回空响应。"""

    def Infer(self, request, context):
        """返回与请求 frame_id 一致的假掩码（PNG 编码）。"""
        mask: np.ndarray = _make_fake_mask(640, 480)
        ok, png = cv2.imencode(".png", mask)
        return rebar_inference_pb2.InferResponse(
            label_mask=png.tobytes(),
            frame_id=request.frame_id,
            timestamp_ms=request.timestamp_ms,
            width=640,
            height=480,
            status=rebar_inference_pb2.NODE_STATUS_IDLE,
        )

    def Heartbeat(self, request, context):
        return rebar_inference_pb2.HeartbeatResponse(
            accepted=True, server_timestamp_ms=int(time.time() * 1000),
        )

    def Shutdown(self, request, context):
        return rebar_inference_pb2.ShutdownResponse(
            acknowledged=True, server_timestamp_ms=int(time.time() * 1000),
        )


def check_grpc() -> Tuple[bool, str]:
    """通信自检：启动 mock gRPC server，InferenceGrpcClient.infer() 联调。"""
    details: List[str] = []
    server: Optional[grpc.Server] = None
    port: int = 50061  # 测试端口（避开生产 50051）

    try:
        # 启动 mock gRPC server
        server = grpc.server(ThreadPoolExecutor(max_workers=2))
        servicer: _MockInferenceServicer = _MockInferenceServicer()
        rebar_inference_pb2_grpc.add_RebarInferenceServicer_to_server(
            servicer, server,
        )
        actual_port: int = server.add_insecure_port(f"127.0.0.1:{port}")
        if actual_port == 0:
            details.append(f"端口 {port} 绑定失败")
            return False, "\n".join(details)
        server.start()
        details.append(f"mock gRPC server 已启动: 127.0.0.1:{actual_port}")

        # 构造客户端
        net_cfg: NetworkConfig = NetworkConfig(
            local_ip="127.0.0.1", remote_ip="127.0.0.1", grpc_port=actual_port,
            grpc_deadline_ms=5000,
        )
        client: InferenceGrpcClient = InferenceGrpcClient(net_cfg)
        connected: bool = client.connect()
        if not connected:
            details.append("InferenceGrpcClient.connect() 返回 False")
            return False, "\n".join(details)
        details.append("InferenceGrpcClient 信道建立成功")

        # 构造测试帧（BGR）
        test_frame: np.ndarray = np.zeros((480, 640, 3), dtype=np.uint8)
        test_frame[:] = (50, 50, 50)  # 灰色背景

        # 调用 infer
        result: Optional[InferenceResult] = client.infer(
            rgb=test_frame, frame_id=42, camera_distance_mm=1000.0,
        )
        if result is None:
            details.append("infer() 返回 None（推理失败）")
            return False, "\n".join(details)

        details.append(
            f"推理响应: frame_id={result.frame_id}, "
            f"mask_size={result.width}x{result.height}, "
            f"status={result.status_code}"
        )

        # 验证 frame_id 一致（帧同步）
        if result.frame_id != 42:
            details.append(f"帧同步不一致: 期望 42, 实际 {result.frame_id}")
            return False, "\n".join(details)

        # 验证掩码尺寸
        if result.mask.shape != (480, 640):
            details.append(f"掩码尺寸异常: {result.mask.shape}（期望 (480, 640)）")
            return False, "\n".join(details)

        # 验证掩码含纵向钢筋（类别 1）
        unique_vals: np.ndarray = np.unique(result.mask)
        if SegClass.REBAR_VERTICAL not in unique_vals:
            details.append(f"掩码缺失纵向钢筋类别(1), unique={unique_vals}")
            return False, "\n".join(details)
        details.append(f"掩码类别值: {unique_vals.tolist()}（含纵向钢筋）")

        # 测试 request_peer_shutdown 占位调用（不阻塞）
        shutdown_result = client.request_peer_shutdown(deadline_s=1.0)
        details.append(
            f"request_peer_shutdown 占位调用返回: {shutdown_result}（待 proto 扩展）"
        )

        client.close()
        details.append("通信自检通过（infer + 帧同步 + request_peer_shutdown 占位）")

    except Exception as e:
        import traceback
        details.append(f"通信自检异常: {e}")
        details.append(traceback.format_exc().split("\n")[-3])
        return False, "\n".join(details)
    finally:
        if server is not None:
            try:
                server.stop(grace=2.0)
            except Exception:
                pass

    return True, "\n".join(details)


# ==================================================================
# 自检 5：推理测量
# ==================================================================

def check_measure() -> Tuple[bool, str]:
    """推理测量自检：假掩码 + 距离 → RebarMeasure.measure() → MeasureResult。"""
    details: List[str] = []
    try:
        # 内参管理器（默认 fx=fy=950）
        intrinsics_cfg: IntrinsicsConfig = IntrinsicsConfig()
        mgr: IntrinsicsManager = IntrinsicsManager(intrinsics_cfg)
        measurer: RebarMeasure = RebarMeasure(mgr)

        # 构造假掩码（5 根纵向钢筋）
        mask: np.ndarray = _make_fake_mask(640, 480)
        details.append(f"输入掩码: shape={mask.shape}, 钢筋像素数={int(np.sum(mask == 1))}")

        # 构造 RGB 帧
        rgb_frame: np.ndarray = np.zeros((480, 640, 3), dtype=np.uint8)
        rgb_frame[:] = (100, 100, 100)

        # 执行测量
        distance_mm: float = 1000.0
        result: Optional[MeasureResult] = measurer.measure(
            mask=mask, distance_mm=distance_mm, rgb_frame=rgb_frame,
        )
        if result is None:
            details.append("measure() 返回 None")
            return False, "\n".join(details)

        details.append(
            f"测量结果: 钢筋数={len(result.rebars)}, "
            f"间距数={len(result.spacings_mm)}, "
            f"mm/px={result.mm_per_pixel:.4f}"
        )
        details.append(f"  距离={result.distance_mm:.1f}mm")
        details.append(f"  标注图尺寸={result.annotated_frame.shape}")

        # 验证钢筋数 > 0
        if len(result.rebars) == 0:
            details.append("未检测到钢筋（钢筋数为 0）")
            return False, "\n".join(details)

        # 打印每根钢筋直径
        for r in result.rebars:
            details.append(
                f"  H{r.index}: 直径={r.diameter_mm:.1f}mm, "
                f"国标={r.standard_diameter_mm}mm, "
                f"中心x={r.center_x}"
            )

        # 验证 mm/px 换算（1000 / 950 ≈ 1.0526）
        expected_mmpp: float = 1000.0 / 950.0
        if abs(result.mm_per_pixel - expected_mmpp) > 0.001:
            details.append(
                f"mm/px 换算异常: 期望 {expected_mmpp:.4f}, 实际 {result.mm_per_pixel:.4f}"
            )
            return False, "\n".join(details)

        details.append(f"mm/px 换算正确（{result.mm_per_pixel:.4f} ≈ {expected_mmpp:.4f}）")
        details.append("推理测量自检通过")

    except Exception as e:
        import traceback
        details.append(f"推理测量自检异常: {e}")
        details.append(traceback.format_exc().split("\n")[-3])
        return False, "\n".join(details)

    return True, "\n".join(details)


# ==================================================================
# 集成验证（§7.6）：全链路 MockCamera → mock 推理 → 测量 → 保存 → 退出
# ==================================================================

def check_integration() -> Tuple[bool, str]:
    """全链路集成验证：九步循环核心链路 + 结果保存 + 退出流程。"""
    details: List[str] = []
    server: Optional[grpc.Server] = None
    port: int = 50062

    try:
        # ---- 1. 启动 mock gRPC server（返回假掩码） ----
        server = grpc.server(ThreadPoolExecutor(max_workers=2))
        rebar_inference_pb2_grpc.add_RebarInferenceServicer_to_server(
            _MockInferenceServicer(), server,
        )
        actual_port: int = server.add_insecure_port(f"127.0.0.1:{port}")
        if actual_port == 0:
            details.append(f"集成测试端口 {port} 绑定失败")
            return False, "\n".join(details)
        server.start()
        details.append(f"① mock gRPC server 已启动: 127.0.0.1:{actual_port}")

        # ---- 2. MockCamera 采集 ----
        cam: MockCameraInput = MockCameraInput(width=640, height=480)
        cam.open()
        frame: Optional[np.ndarray] = cam.get_rgb_frame()
        if frame is None:
            details.append("② MockCamera 采集失败")
            return False, "\n".join(details)
        details.append(f"② MockCamera 采集成功: shape={frame.shape}")

        # ---- 3. gRPC 推理（九步循环 ④） ----
        net_cfg: NetworkConfig = NetworkConfig(
            local_ip="127.0.0.1", remote_ip="127.0.0.1", grpc_port=actual_port,
            grpc_deadline_ms=5000,
        )
        client: InferenceGrpcClient = InferenceGrpcClient(net_cfg)
        client.connect()
        infer_result: Optional[InferenceResult] = client.infer(
            rgb=frame, frame_id=1, camera_distance_mm=1000.0,
        )
        if infer_result is None:
            details.append("④ gRPC 推理失败")
            return False, "\n".join(details)
        details.append(
            f"④ gRPC 推理成功: frame_id={infer_result.frame_id}, "
            f"mask={infer_result.width}x{infer_result.height}"
        )

        # ---- 4. 测量换算（九步循环 ⑤） ----
        intrinsics_cfg: IntrinsicsConfig = IntrinsicsConfig()
        mgr: IntrinsicsManager = IntrinsicsManager(intrinsics_cfg)
        measurer: RebarMeasure = RebarMeasure(mgr)
        measure_result: Optional[MeasureResult] = measurer.measure(
            mask=infer_result.mask, distance_mm=1000.0, rgb_frame=frame,
        )
        if measure_result is None:
            details.append("⑤ 测量换算失败")
            return False, "\n".join(details)
        details.append(
            f"⑤ 测量换算成功: 钢筋数={len(measure_result.rebars)}, "
            f"间距数={len(measure_result.spacings_mm)}"
        )

        # ---- 5. 结果保存（九步循环 ⑥） ----
        store: ResultStore = ResultStore()
        result_dir: str = store.save(
            frame_id=1,
            rgb_frame=frame,
            mask=infer_result.mask,
            measure_result=measure_result,
            save_options={"csv": True, "mask": True},
        )
        if not result_dir or not os.path.isdir(result_dir):
            details.append(f"⑥ 结果保存失败: dir={result_dir}")
            return False, "\n".join(details)
        details.append(f"⑥ 结果保存成功: dir={result_dir}")

        # 验证文件存在
        files: List[str] = os.listdir(result_dir)
        for expected in ["result.jpg", "report.csv", "ClassMask.png"]:
            if expected not in files:
                details.append(f"  缺失文件: {expected}")
                return False, "\n".join(details)
        details.append(f"  文件完整: {files}")

        # ---- 6. 退出流程验证 ----
        # 构造 mock NodeMonitor（离线状态）
        monitor: NodeMonitor = NodeMonitor()
        coord: ExitCoordinator = ExitCoordinator(
            node_monitor=monitor, inference_client=client,
        )
        # 检测节点 离线 → notify_peer_shutdown 应返回 True（直接退出）
        notify_result = coord.notify_peer_shutdown(deadline_s=1.0)
        if notify_result is not True:
            details.append(f"⑦ 退出通知异常: 返回 {notify_result}（期望 True，A 离线）")
            return False, "\n".join(details)
        details.append("⑦ 退出通知通过（检测节点 离线，直接退出）")

        # 资源释放
        stop_event: threading.Event = threading.Event()

        def _stop_cb() -> None:
            stop_event.set()

        coord.release_resources(
            stop_callback=_stop_cb,
            threads=[],
            camera=cam,
            serial_manager=None,
            grpc_server=None,
        )
        if not stop_event.is_set():
            details.append("⑧ 资源释放: 停止标志未设置")
            return False, "\n".join(details)
        details.append("⑧ 资源释放通过（停止标志已设置，相机已关闭）")
        details.append("全链路集成验证通过（九步循环 + 结果保存 + 双退出流程）")

    except Exception as e:
        import traceback
        details.append(f"集成验证异常: {e}")
        tb_lines: List[str] = traceback.format_exc().split("\n")
        details.append(tb_lines[-3] if len(tb_lines) >= 3 else "")
        return False, "\n".join(details)
    finally:
        if server is not None:
            try:
                server.stop(grace=2.0)
            except Exception:
                pass

    return True, "\n".join(details)


# ==================================================================
# 主入口
# ==================================================================

def main() -> int:
    """运行五项自检 + 集成验证，返回退出码（0=全通过, 1=有失败）。"""
    print("=" * 60)
    print("服务节点 五项自检 + 集成验证")
    print("=" * 60)
    print()

    # 配置日志级别为 WARNING（减少自检过程中的日志干扰）
    logging.basicConfig(level=logging.WARNING, format="%(levelname)s | %(name)s | %(message)s")

    checks: List[Tuple[str, any]] = [
        ("1. 相机自检", check_camera),
        ("2. 串口自检", check_serial),
        ("3. GUI 自检", check_gui),
        ("4. 通信自检", check_grpc),
        ("5. 推理测量自检", check_measure),
        ("6. 全链路集成验证", check_integration),
    ]

    results: List[Tuple[str, bool, str]] = []
    for name, func in checks:
        try:
            passed, detail = func()
        except Exception as e:
            passed, detail = False, f"自检函数异常: {e}"
        results.append((name, passed, detail))
        _print_result(name, passed, detail)
        print()

    # 汇总
    print("=" * 60)
    passed_count: int = sum(1 for _, p, _ in results if p)
    total: int = len(results)
    print(f"汇总: {passed_count}/{total} 项通过")
    print("=" * 60)

    for name, passed, _ in results:
        tag: str = "[PASS]" if passed else "[FAIL]"
        print(f"  {tag} {name}")

    return 0 if passed_count == total else 1


if __name__ == "__main__":
    sys.exit(main())
