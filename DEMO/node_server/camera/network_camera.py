"""网络摄像头输入（TCP 客户端）。

对齐 Design-server.md §3.3.4 与 orbbec-336L/orbbec-336L-win-protocol.md。
作为 Orbbec336LInput（USB 直连）之外的补充接入方式：
服务节点 主动 TCP 连接网络摄像头端，握手后被动接收 JPEG 视频推流。

协议要点：
- 应用层握手：客户端发 ``HELLO\\n`` → 服务器回 ``OK <port>\\n`` → 开始推流
- 帧格式：28 字节定长帧头（小端序）+ 变长 JPEG 数据
- 帧头 struct 格式 ``<IBBHQIHHI``，magic=0x4F524242("ORBB")
- 粘包/半包：用 ``_recv_exactly`` 精确读取；magic 不匹配时逐字节扫描重新对齐
"""
import logging
import socket
import struct
import threading
import time
from typing import Dict, Optional

import cv2
import numpy as np

from .base import BaseCameraInput
from common.config_loader import IntrinsicsConfig

logger = logging.getLogger("node_server.camera")

# 28 字节帧头 struct 格式（小端序）
# '<I B B H Q I H H I' → 4+1+1+2+8+4+2+2+4 = 28 字节
_FRAME_HEADER_FMT: str = "<IBBHQIHHI"
_FRAME_HEADER_SIZE: int = 28
_MAGIC_ORBB: int = 0x4F524242  # "ORBB" 帧起始魔数
_HELLO: bytes = b"HELLO\n"
_HANDSHAKE_TIMEOUT: float = 5.0  # 服务器 accept 后 5 秒内未收 HELLO 则关闭
_CONNECT_TIMEOUT: float = 5.0  # TCP connect 超时


class NetworkCameraInput(BaseCameraInput):
    """网络摄像头输入（TCP 客户端）。

    主动连接网络摄像头端，握手后被动接收 JPEG 视频推流。
    默认服务器地址 ``192.168.1.100:8080``，构造参数允许 GUI 手动指定 host/port。
    断线重连：最多 5 次，间隔 3 秒。
    """

    def __init__(
        self,
        intrinsics_cfg: IntrinsicsConfig,
        host: str = "192.168.1.100",
        port: int = 8080,
    ):
        """初始化网络摄像头输入。

        :param intrinsics_cfg: 相机内参配置
        :param host: 网络摄像头服务器 IP，默认 192.168.1.100
        :param port: 网络摄像头服务器端口，默认 8080
        """
        self._host: str = host
        self._port: int = port
        self._intrinsics_cfg: IntrinsicsConfig = intrinsics_cfg
        self._sock: Optional[socket.socket] = None
        self._is_opened: bool = False
        self._latest_frame: Optional[np.ndarray] = None
        self._frame_lock: threading.Lock = threading.Lock()
        self._read_thread: Optional[threading.Thread] = None
        self._is_running: bool = False
        # 断线重连参数
        self._reconnect_interval: float = 3.0
        self._max_reconnect_attempts: int = 5
        self._reconnect_attempts: int = 0
        # magic 失步时的逐字节扫描缓冲
        self._resync_buffer: bytearray = bytearray()

    def open(self) -> bool:
        """建立 TCP 连接 + 应用层握手 + 启动后台读帧线程。

        :return: 连接并握手成功返回 True
        """
        try:
            self._sock = socket.create_connection(
                (self._host, self._port), timeout=_CONNECT_TIMEOUT
            )
            self._sock.settimeout(None)  # 连接建立后设为阻塞
            # 1. 应用层握手：发 HELLO
            self._sock.sendall(_HELLO)
            # 2. 等待 OK <port>\n 响应
            ok_resp = self._recv_handshake_response()
            if ok_resp is None or not ok_resp.startswith(b"OK"):
                logger.error("网络摄像头握手失败: host=%s port=%d resp=%s", self._host, self._port, ok_resp)
                self._close_socket()
                return False
            self._is_opened = True
            self._is_running = True
            self._reconnect_attempts = 0
            self._resync_buffer = bytearray()
            self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self._read_thread.start()
            logger.info("网络摄像头连接成功: %s:%d", self._host, self._port)
            return True
        except (socket.error, OSError) as e:
            logger.error("网络摄像头连接异常: %s:%d %s", self._host, self._port, e)
            self._close_socket()
            return False

    def close(self) -> None:
        """关闭连接并停止后台读帧线程。"""
        self._is_running = False
        if self._read_thread is not None and self._read_thread.is_alive():
            self._read_thread.join(timeout=2.0)
            self._read_thread = None
        self._close_socket()
        self._is_opened = False
        with self._frame_lock:
            self._latest_frame = None
        logger.info("网络摄像头已关闭: %s:%d", self._host, self._port)

    def is_opened(self) -> bool:
        return self._is_opened

    def get_rgb_frame(self) -> Optional[np.ndarray]:
        """获取最新帧的拷贝（线程安全）。"""
        with self._frame_lock:
            if self._latest_frame is None:
                return None
            return self._latest_frame.copy()

    def get_depth_frame(self) -> Optional[np.ndarray]:
        """原型阶段不实现，返回 None。"""
        return None

    def get_intrinsics(self) -> Dict[str, float]:
        """从 IntrinsicsConfig 返回内参。"""
        return {
            "fx": self._intrinsics_cfg.fx,
            "fy": self._intrinsics_cfg.fy,
            "cx": self._intrinsics_cfg.cx,
            "cy": self._intrinsics_cfg.cy,
        }

    def try_reconnect(self) -> bool:
        """断线重连：间隔 3 秒，最多 5 次。"""
        if self._reconnect_attempts >= self._max_reconnect_attempts:
            logger.error(
                "网络摄像头重连次数已达上限 %d 次，放弃重连", self._max_reconnect_attempts
            )
            return False
        self._reconnect_attempts += 1
        logger.warning(
            "网络摄像头尝试第 %d/%d 次重连（间隔 %.1fs）",
            self._reconnect_attempts,
            self._max_reconnect_attempts,
            self._reconnect_interval,
        )
        time.sleep(self._reconnect_interval)
        return self.open()

    def _close_socket(self) -> None:
        """关闭并清理 socket。"""
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    def _recv_handshake_response(self) -> Optional[bytes]:
        """接收握手响应，读到 ``\\n`` 结束。

        :return: 响应字节（含 ``\\n``）；连接断开返回 None
        """
        buf: bytearray = bytearray()
        self._sock.settimeout(_HANDSHAKE_TIMEOUT)
        try:
            while b"\n" not in buf:
                chunk: bytes = self._sock.recv(64)
                if not chunk:
                    return None
                buf.extend(chunk)
            return bytes(buf)
        except (socket.timeout, socket.error, OSError):
            return None
        finally:
            try:
                self._sock.settimeout(None)
            except (socket.error, OSError):
                pass

    def _recv_exactly(self, n: int) -> Optional[bytes]:
        """精确读取 n 字节，解决 TCP 半包问题。

        :param n: 需读取的字节数
        :return: 读满 n 字节返回 bytes；连接断开返回 None
        """
        buf: bytearray = bytearray()
        while len(buf) < n:
            try:
                chunk: bytes = self._sock.recv(n - len(buf))
            except (socket.error, OSError):
                return None
            if not chunk:
                return None
            buf.extend(chunk)
        return bytes(buf)

    def _read_loop(self) -> None:
        """后台线程：读帧头 → 读 JPEG → 解码 → 更新 _latest_frame。

        magic 不匹配时逐字节扫描重新对齐（resync）。
        连接断开时触发重连。
        """
        while self._is_running:
            # 读 28 字节帧头
            header: Optional[bytes] = self._recv_exactly(_FRAME_HEADER_SIZE)
            if header is None:
                logger.warning("网络摄像头读帧头失败，连接断开")
                self._handle_disconnect()
                return
            # 解包帧头
            try:
                (magic, ver, flags, reserved, ts_ms, fidx,
                 w, h, jpg_sz) = struct.unpack(_FRAME_HEADER_FMT, header)
            except struct.error:
                logger.warning("网络摄像头帧头解包失败，跳过该帧")
                continue
            # magic 校验：不匹配时逐字节扫描重新对齐
            if magic != _MAGIC_ORBB:
                self._resync(header)
                continue
            # 读 JPEG 数据
            jpeg: Optional[bytes] = self._recv_exactly(jpg_sz)
            if jpeg is None:
                logger.warning("网络摄像头读 JPEG 数据失败，连接断开")
                self._handle_disconnect()
                return
            # 解码为 BGR 图像
            frame: Optional[np.ndarray] = cv2.imdecode(
                np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR
            )
            if frame is not None:
                with self._frame_lock:
                    self._latest_frame = frame
                self._reconnect_attempts = 0
            # 帧丢弃时不阻塞，继续读下一帧

    def _resync(self, header: bytes) -> None:
        """magic 不匹配时，逐字节扫描重新对齐到下一个 magic。

        将帧头字节存入 _resync_buffer，逐字节查找 magic 字节序列，
        找到则丢弃之前的字节，从 magic 位置重新开始读帧头。

        :param header: 收到的 28 字节帧头（magic 不匹配）
        """
        # magic 的 4 字节小端序表示
        magic_bytes: bytes = struct.pack("<I", _MAGIC_ORBB)
        buf: bytearray = bytearray(header)
        # 逐字节查找 magic
        for i in range(len(buf) - 3):
            if bytes(buf[i:i + 4]) == magic_bytes:
                # 找到 magic，丢弃之前的字节，从 magic 位置补齐帧头
                need: int = _FRAME_HEADER_SIZE - (len(buf) - i)
                if need > 0:
                    extra: Optional[bytes] = self._recv_exactly(need)
                    if extra is None:
                        self._handle_disconnect()
                        return
                    buf = bytearray(buf[i:]) + extra
                # 重新校验补齐后的帧头
                try:
                    (magic2, *_rest) = struct.unpack(_FRAME_HEADER_FMT, bytes(buf[:_FRAME_HEADER_SIZE]))
                    if magic2 == _MAGIC_ORBB:
                        # 对齐成功，读 JPEG
                        jpg_sz2: int = struct.unpack(_FRAME_HEADER_FMT, bytes(buf[:_FRAME_HEADER_SIZE]))[8]
                        jpeg2: Optional[bytes] = self._recv_exactly(jpg_sz2)
                        if jpeg2 is not None:
                            frame2: Optional[np.ndarray] = cv2.imdecode(
                                np.frombuffer(jpeg2, np.uint8), cv2.IMREAD_COLOR
                            )
                            if frame2 is not None:
                                with self._frame_lock:
                                    self._latest_frame = frame2
                                self._reconnect_attempts = 0
                except struct.error:
                    pass
                return
        # 未找到 magic，丢弃整个帧头，下次循环重新读 28 字节

    def _handle_disconnect(self) -> None:
        """处理连接断开：标记离线 + 尝试重连。"""
        self._is_opened = False
        self._close_socket()
        if not self.try_reconnect():
            self._is_running = False
