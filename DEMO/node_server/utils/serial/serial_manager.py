"""串口设备管理器。

对齐 Design-server.md §3.4.4 与 AGENTS.md §5.1 / §6.1。
基于 pyserial 后台读串口线程，封装端口打开、后台读取、帧解析、数据获取与重连。

数据链路（AGENTS.md §6.1）：
4 路 STP23L → S21C 主控（汇总）→ Type-C 串口 → 服务节点 SerialManager
→ SerialBuffer（11 字节帧解析）→ LaserDataParser（4 路 uint16 距离值 + 有效性检查）

设备识别（兼容 USB 直连与扩展坞接入）：
- 支持 /dev/ttyUSB* 与 /dev/ttyACM* 两种节点类型
- 按 VID/PID 白名单识别 CH9102 系列（VID=0x1A86, PID∈{0x55D4, 0x55D5}）
- 多候选时取 device_path 字典序首个，其余忽略
"""
import logging
import os
import sys
import threading
import time
from typing import List, Optional, Tuple

# ---- 导入 pyserial（绕过本地 serial/ 包同名遮蔽）----
# 项目 node_server/serial/ 包与 pyserial 库（安装名 ``serial``）同名，
# 从 node_server/ 目录运行程序时 ``import serial`` 会找到本地包而非 pyserial。
# 以下代码临时调整 sys.path 与 sys.modules，导入 pyserial 后恢复原始状态。
# 设计文档（Design-server.md §3.4.4）原使用 ``import serial``，
# 因本地包遮蔽在此调整为显式加载，确保 pyserial 的 Serial 类可用。


def _load_pyserial():
    """加载 pyserial 模块，绕过本地 serial/ 包的同名遮蔽。

    临时从 sys.path 移除当前目录、清除 sys.modules 中已加载的本地 serial
    模块缓存，使 Python 重新从 site-packages 找到 pyserial。导入完成后
    恢复 sys.path 与 sys.modules 原始状态，不影响其他模块的本地 serial 引用。

    :return: pyserial 模块对象
    """
    cwd_abs = os.path.abspath(os.getcwd())
    # 临时移除当前目录及空路径
    saved_path = sys.path[:]
    sys.path = [p for p in sys.path
                if p not in ('', '.') and os.path.abspath(p) != cwd_abs]
    # 清除本地 serial 模块缓存
    saved_modules = {}
    for key in list(sys.modules.keys()):
        if key == 'serial' or key.startswith('serial.'):
            saved_modules[key] = sys.modules.pop(key)
    try:
        import serial as _pyserial
        import serial.tools.list_ports  # noqa: F401
        return _pyserial
    finally:
        sys.path = saved_path
        # 恢复本地 serial 模块缓存
        sys.modules.update(saved_modules)


_pyserial = _load_pyserial()
Serial = _pyserial.Serial
SerialException = _pyserial.SerialException
list_ports = _pyserial.tools.list_ports

from .serial_buffer import SerialBuffer
from .laser_parser import LaserDataParser, LaserParseResult
from utils.common.config_loader import SerialConfig

logger = logging.getLogger("node_server.serial")


# ---- 串口连接状态机 ----
# UNCONNECTED：未连接（初始状态）
# CONNECTING：正在连接（open_port 调用中）
# CONNECTED：已连接，后台读线程运行中
# RECONNECTING：断线后等待重连（_reconnect_daemon 退避重试中）
_STATE_UNCONNECTED: str = "UNCONNECTED"
_STATE_CONNECTING: str = "CONNECTING"
_STATE_CONNECTED: str = "CONNECTED"
_STATE_RECONNECTING: str = "RECONNECTING"


class SerialManager:
    """串口设备管理类。

    封装端口打开、后台读取、帧解析、数据获取与重连。
    后台线程持续读串口 → SerialBuffer 喂入 → LaserDataParser 解析 → 更新最新结果。
    线程安全：``_data_lock`` 保护最新解析结果与时间戳；``_state_lock`` 保护连接状态。

    设备识别策略：
    1. ``cfg.device_path`` 非空 → 直接使用
    2. ``cfg.auto_scan=True`` → ``auto_detect()`` 按 VID/PID 白名单扫描
    3. 多候选时取 ``device_path`` 字典序首个
    """

    def __init__(self, serial_cfg: Optional[SerialConfig] = None):
        """初始化串口管理器。

        :param serial_cfg: 串口配置；None 时使用默认配置
        """
        cfg: SerialConfig = serial_cfg if serial_cfg is not None else SerialConfig()
        self._cfg: SerialConfig = cfg
        self._baudrate: int = cfg.baudrate
        self._timeout: float = 0.1
        self._serial_conn: Optional[Serial] = None
        self._port_name: str = ""
        self._buffer: SerialBuffer = SerialBuffer()
        self._data_lock: threading.Lock = threading.Lock()
        self._latest_result: Optional[LaserParseResult] = None
        self._data_timestamp: float = 0.0
        self._read_thread: Optional[threading.Thread] = None

        # 连接状态机
        self._state_lock: threading.Lock = threading.Lock()
        self._state: str = _STATE_UNCONNECTED
        # 统一调度线程（热插拔扫描 + 断线重扫），替代原 _reconnect_thread
        self._scan_thread: Optional[threading.Thread] = None
        self._scan_interval: float = cfg.scan_interval_seconds  # 热插拔扫描间隔
        self._reconnect_attempts: int = 0
        # [已弃用] max_reconnect_attempts 仅保留向后兼容，热插拔改为无限重连
        self._max_reconnect_attempts: int = cfg.max_reconnect_attempts
        self._reconnect_interval: float = cfg.reconnect_interval_seconds
        self._max_age_seconds: float = cfg.max_age_seconds

        # USB 稳定性监测（5 分钟内断线次数）
        self._usb_unstable_threshold: int = cfg.usb_unstable_threshold
        self._disconnect_timestamps: List[float] = []

        # 可观测性：周期性心跳日志
        self._stats_lock: threading.Lock = threading.Lock()
        self._stats_frames: int = 0
        self._stats_bytes: int = 0
        self._stats_last_time: float = time.time()

    # ------------------------------------------------------------------
    # 设备自动扫描
    # ------------------------------------------------------------------

    @staticmethod
    def auto_detect(cfg: SerialConfig) -> Optional[str]:
        """按配置规则扫描 S21C 设备，返回设备路径或 None。

        扫描顺序：
        1. 枚举所有串口
        2. 路径前缀黑名单排除（ttyAMA*/ttyS*/ttyconsole*）
        3. 路径前缀白名单过滤（ttyUSB*/ttyACM*）
        4. VID+PID 白名单匹配 → 候选集 A
        5. VID 匹配 + 描述关键字兜底 → 候选集 B
        6. 合并候选集，按 device_path 字典序排序
        7. select_first_when_multiple=True 取第一个
        8. 其余候选记 INFO 日志忽略

        :param cfg: 串口配置
        :return: 选中的设备路径；无候选返回 None
        """
        logger.info("扫描 S21C 设备...（VID 白名单=%s, PID 白名单=%s）",
                    [hex(v) for v in cfg.vid_whitelist],
                    [hex(p) for p in cfg.pid_whitelist])

        try:
            all_ports: list = list(list_ports.comports())
        except Exception as e:
            logger.error("枚举串口失败: %s", e)
            return None

        candidates: List[str] = []
        for p in all_ports:
            device: str = p.device or ""
            description: str = p.description or ""
            vid: Optional[int] = p.vid
            pid: Optional[int] = p.pid

            # 路径前缀黑名单排除
            if any(device.startswith(prefix) for prefix in cfg.path_prefix_blacklist):
                continue

            # 路径前缀白名单过滤
            if not any(device.startswith(prefix) for prefix in cfg.path_prefix_whitelist):
                continue

            # 匹配规则：VID+PID 白名单（候选集 A）
            matched: bool = False
            if vid in cfg.vid_whitelist and pid in cfg.pid_whitelist:
                matched = True
                logger.info("发现 CH9102 设备（VID/PID 白名单匹配）: %s "
                            "（VID=0x%04X, PID=0x%04X, %s）",
                            device, vid or 0, pid or 0, description)
            # 兜底：VID 匹配 + 描述关键字（候选集 B）
            elif vid in cfg.vid_whitelist:
                desc_upper: str = description.upper()
                if any(kw.upper() in desc_upper for kw in cfg.description_keywords):
                    matched = True
                    logger.info("发现 CH9102 设备（VID+描述兜底匹配）: %s "
                                "（VID=0x%04X, PID=%s, %s）",
                                device, vid or 0,
                                hex(pid) if pid else "N/A", description)

            if matched:
                candidates.append(device)

        if not candidates:
            logger.warning("未发现 S21C 设备，请检查接线和供电")
            return None

        # 按 device_path 字典序排序
        candidates.sort()

        if len(candidates) == 1:
            logger.info("选中: %s", candidates[0])
            return candidates[0]

        # 多候选处理
        if cfg.select_first_when_multiple:
            selected: str = candidates[0]
            logger.info("发现 %d 个 S21C 候选，取字典序首个: %s",
                        len(candidates), selected)
            for extra in candidates[1:]:
                logger.info("忽略多余 S21C: %s", extra)
            return selected
        else:
            logger.warning("发现 %d 个 S21C 候选，select_first_when_multiple=False，"
                           "不自动选择: %s", len(candidates), candidates)
            return None

    # ------------------------------------------------------------------
    # 端口打开与连接
    # ------------------------------------------------------------------

    def resolve_and_open(self) -> Tuple[bool, str]:
        """解析设备路径并打开串口。

        优先级：
        1. ``cfg.device_path`` 非空 → 直接打开
        2. ``cfg.auto_scan=True`` → ``auto_detect()`` → 打开
        3. 均失败 → 返回 ``(False, 原因)``

        :return: ``(是否成功, 描述消息)``
        """
        # 优先级 1：配置显式指定
        if self._cfg.device_path:
            logger.info("使用配置指定的串口设备: %s", self._cfg.device_path)
            return self.open_port(self._cfg.device_path)

        # 优先级 2：自动扫描
        if self._cfg.auto_scan:
            port: Optional[str] = self.auto_detect(self._cfg)
            if port is None:
                msg: str = "自动扫描未发现 S21C 设备"
                logger.warning(msg)
                return False, msg
            return self.open_port(port)

        # 优先级 3：均未启用
        msg = "串口未配置：device_path 为空且 auto_scan=False"
        logger.error(msg)
        return False, msg

    def open_port(self, port_name: str) -> Tuple[bool, str]:
        """打开指定串口并启动后台读取线程。

        :param port_name: 串口设备路径（如 ``/dev/ttyUSB0`` 或 ``/dev/ttyACM0``）
        :return: ``(是否成功, 描述消息)``
        """
        with self._state_lock:
            if self._state == _STATE_CONNECTED:
                logger.warning("串口已连接，跳过重复打开: %s", self._port_name)
                return True, f"串口已连接: {self._port_name}"
            self._state = _STATE_CONNECTING

        try:
            self._serial_conn = Serial(
                port=port_name,
                baudrate=self._baudrate,
                timeout=self._timeout,
            )
            self._port_name = port_name
            self._reconnect_attempts = 0
            self._buffer = SerialBuffer()
            # ★ 修复竞争条件：先将状态设为 CONNECTED，再启动读线程。
            #   原顺序（先 start 后设 CONNECTED）在 OrangePi 单核调度下，
            #   daemon 线程可能在主线程转移状态之前瞬间执行 _read_loop，
            #   看到 CONNECTING 后直接 return，导致 read-loop 从未真正运转。
            with self._state_lock:
                self._state = _STATE_CONNECTED
            self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self._read_thread.start()

            msg: str = f"成功打开串口 {port_name}（波特率 {self._baudrate}）"
            logger.info(msg)
            return True, msg
        except (SerialException, OSError) as e:
            self._serial_conn = None
            with self._state_lock:
                self._state = _STATE_UNCONNECTED
            msg = f"打开串口 {port_name} 失败: {e}"
            logger.error(msg)
            return False, msg

    def close(self) -> None:
        """关闭串口并停止后台读取线程。"""
        with self._state_lock:
            self._state = _STATE_UNCONNECTED

        if self._read_thread is not None and self._read_thread.is_alive():
            self._read_thread.join(timeout=1.0)
            self._read_thread = None
        if self._serial_conn is not None and self._serial_conn.is_open:
            try:
                self._serial_conn.close()
            except (SerialException, OSError):
                pass
        self._serial_conn = None
        logger.info("串口已关闭: %s", self._port_name)

    def is_running(self) -> bool:
        """串口是否正在运行（后台线程活跃）。"""
        with self._state_lock:
            return self._state == _STATE_CONNECTED

    def get_port_name(self) -> str:
        """获取当前串口名称。"""
        return self._port_name

    def get_state(self) -> str:
        """获取当前连接状态。"""
        with self._state_lock:
            return self._state

    # ------------------------------------------------------------------
    # 数据获取
    # ------------------------------------------------------------------

    def get_latest(
        self, max_age_seconds: Optional[float] = None
    ) -> Tuple[Optional[LaserParseResult], float]:
        """获取最新解析结果与时间戳（线程安全拷贝）。

        :param max_age_seconds: 新鲜度阈值（秒）；None 时使用配置的 max_age_seconds。
                                数据超过此阈值视为过期，返回 None。
        :return: ``(LaserParseResult 或 None, 数据时间戳)``；
                 无数据或过期时返回 ``(None, 数据时间戳)``
        """
        threshold: float = (
            max_age_seconds if max_age_seconds is not None
            else self._max_age_seconds
        )
        with self._data_lock:
            if self._latest_result is None:
                return None, 0.0
            age: float = time.time() - self._data_timestamp
            if age > threshold:
                return None, self._data_timestamp
            return self._latest_result, self._data_timestamp

    # ------------------------------------------------------------------
    # 热插拔扫描 + 断线重连（统一 _scan_daemon 线程，无限循环）
    # ------------------------------------------------------------------

    def start_scanning(self) -> None:
        """启动后台热插拔扫描守护线程（幂等）。

        统一处理两种场景：
        1. 未连接（UNCONNECTED）：周期 auto_detect() 扫描，发现 S21C 即自动 open_port
        2. 断线（RECONNECTING）：重新 auto_detect()（应对热插拔换接口/换路径），无限重试

        由 _scan_daemon 单一线程驱动，避免扫描与重连并发 open_port 的竞态。
        """
        with self._state_lock:
            if self._scan_thread is not None and self._scan_thread.is_alive():
                logger.warning("热插拔扫描线程已在运行，忽略重复 start_scanning()")
                return
            # 仅在 UNCONNECTED 态启动扫描；CONNECTED/CONNECTING 时由 read_loop 异常接管
            if self._state == _STATE_UNCONNECTED:
                self._state = _STATE_CONNECTING  # 进入连接流程
        self._scan_thread = threading.Thread(
            target=self._scan_daemon,
            name="serial-hotplug-scan",
            daemon=True,
        )
        self._scan_thread.start()
        logger.info("S21C 热插拔扫描已启动（间隔 %.1fs，无设备将持续扫描）",
                    self._scan_interval)

    def stop_scanning(self) -> None:
        """停止热插拔扫描线程（幂等，等待最多 2 秒）。"""
        with self._state_lock:
            # 若处于 CONNECTING（扫描中未连接），回退到 UNCONNECTED
            if self._state == _STATE_CONNECTING:
                self._state = _STATE_UNCONNECTED
        if self._scan_thread is not None and self._scan_thread.is_alive():
            self._scan_thread.join(timeout=2.0)
        self._scan_thread = None
        logger.info("S21C 热插拔扫描已停止")

    def _scan_daemon(self) -> None:
        """热插拔扫描守护线程主循环。

        分支处理：
        - UNCONNECTED：已被外部 close()，退出线程
        - CONNECTING：尝试 auto_detect() → 发现设备则 open_port → 转 CONNECTED
        - RECONNECTING：记录断线 + 重新 auto_detect()（换路径），发现则 open_port
        - CONNECTED：正常连接，退出（由 read_loop 接管，断线时再进入 RECONNECTING）
        """
        while True:
            with self._state_lock:
                state: str = self._state

            if state == _STATE_UNCONNECTED:
                return  # 已被 close()，退出扫描线程

            if state == _STATE_CONNECTED:
                return  # 连接成功，read_loop 已接管，退出扫描线程

            # state ∈ {CONNECTING, RECONNECTING}：尝试扫描并连接
            if state == _STATE_RECONNECTING:
                # USB 稳定性监测（仅断线重扫时记录）
                self._record_disconnect()
                self._check_usb_stability()

            # 重新扫描（RECONNECTING 时也重新扫描以应对热插拔换路径）
            port: Optional[str] = self.auto_detect(self._cfg)
            if port is not None:
                ok, msg = self.open_port(port)
                if ok:
                    logger.info("S21C 连接成功: %s", port)
                    return  # open_port 已启动 read_loop，扫描线程退出
                # open_port 失败（权限/被占）→ 记 ERROR，回退重扫
                logger.error("S21C 打开失败: %s，将重扫", msg)
            else:
                # 未发现设备，持续扫描（DEBUG 避免刷屏）
                logger.debug("未发现 S21C 设备，%.1fs 后重扫", self._scan_interval)

            # 防抖：sleep 一个扫描间隔后再试
            time.sleep(self._scan_interval)

    def try_reconnect(self) -> bool:
        """断线重连：重新扫描当前最优设备路径（支持热插拔换接口）。

        由 _scan_daemon 在 RECONNECTING 态调用（每次自动重新 auto_detect），
        不再复用固定 _port_name，避免换 USB 接口/路径变化后永远连不上。

        :return: 重连成功返回 True；未发现设备或打开失败返回 False
        """
        # [已弃用] max_reconnect_attempts 不再作为上限，热插拔改为无限重连
        port: Optional[str] = self.auto_detect(self._cfg)
        if port is None:
            logger.warning("断线重扫未发现 S21C 设备，持续扫描等待重连")
            return False
        self._reconnect_attempts += 1
        logger.warning(
            "串口尝试第 %d 次重连（重新扫描）: %s",
            self._reconnect_attempts, port,
        )
        ok, _ = self.open_port(port)
        return ok

    def _reconnect_daemon(self) -> None:
        """断线重连守护线程：无限重扫，直到成功或状态变更。

        [已弃用] 为兼容旧调用保留；新逻辑由 _scan_daemon 统一处理。
        此方法作为 _scan_daemon 的 RECONNECTING 分支等价实现保留，
        若被外部显式调用则委托给 _scan_daemon 的扫描逻辑。
        """
        while True:
            with self._state_lock:
                if self._state != _STATE_RECONNECTING:
                    return  # 状态变更（已连接或已关闭），退出守护
            self._record_disconnect()
            self._check_usb_stability()
            ok: bool = self.try_reconnect()
            if ok:
                logger.info("串口重连成功: %s", self._port_name)
                return  # read_loop 已在 open_port 中启动，守护线程退出
            time.sleep(self._reconnect_interval)  # 重连失败，等待后重扫

    def _record_disconnect(self) -> None:
        """记录断线时间戳，清理 5 分钟外的旧记录。"""
        now: float = time.time()
        self._disconnect_timestamps.append(now)
        # 清理 5 分钟（300 秒）外的记录
        cutoff: float = now - 300.0
        self._disconnect_timestamps = [
            t for t in self._disconnect_timestamps if t > cutoff
        ]

    def _check_usb_stability(self) -> None:
        """检查 5 分钟内断线次数，超阈值告警。"""
        count: int = len(self._disconnect_timestamps)
        if count >= self._usb_unstable_threshold:
            logger.warning(
                "5 分钟内串口断线 %d 次（阈值 %d），请检查 USB 供电/扩展坞连接稳定性",
                count, self._usb_unstable_threshold,
            )

    # ------------------------------------------------------------------
    # 串口枚举
    # ------------------------------------------------------------------

    @staticmethod
    def list_ports() -> list:
        """枚举系统可用串口。

        :return: 串口设备信息列表（pyserial ListPortInfo 对象）
        """
        return list(list_ports.comports())

    # ------------------------------------------------------------------
    # 后台读取线程
    # ------------------------------------------------------------------

    def _read_loop(self) -> None:
        """后台线程：读取串口 → 缓冲区 → 帧解析 → 更新结果。

        异常捕获 SerialException → 设置状态为 RECONNECTING → 确保扫描线程
        (_scan_daemon) 在运行以重新扫描重连。不再内联起新线程调用 open_port，
        避免嵌套线程问题。
        """
        while True:
            with self._state_lock:
                if self._state != _STATE_CONNECTED:
                    return  # 状态变更，退出读线程

            try:
                if self._serial_conn is None:
                    return
                if self._serial_conn.in_waiting > 0:
                    data: bytes = self._serial_conn.read(self._serial_conn.in_waiting)
                    if data:
                        frames = self._buffer.feed(data)
                        for frame in frames:
                            result: Optional[LaserParseResult] = LaserDataParser.parse(frame)
                            if result is not None:
                                with self._data_lock:
                                    self._latest_result = result
                                    self._data_timestamp = result.timestamp
                                self._reconnect_attempts = 0

                        # 可观测性统计
                        with self._stats_lock:
                            self._stats_frames += len(frames)
                            self._stats_bytes += len(data)

                        # 周期性心跳日志（每 10 秒）
                        self._emit_stats_if_due()

            except (SerialException, OSError) as e:
                logger.error("串口读取异常: %s", e, exc_info=True)
                # 设置状态为 RECONNECTING，确保扫描线程在运行以重新扫描重连
                with self._state_lock:
                    self._state = _STATE_RECONNECTING
                # 仅当扫描线程未运行时启动（避免与 start_scanning 并发重复）
                need_new_scan: bool = (
                    self._scan_thread is None or not self._scan_thread.is_alive()
                )
                if need_new_scan:
                    self._scan_thread = threading.Thread(
                        target=self._scan_daemon,
                        name="serial-hotplug-scan",
                        daemon=True,
                    )
                    self._scan_thread.start()
                return  # 退出当前读线程，由扫描线程重连后起新读线程

            time.sleep(0.001)

    def _emit_stats_if_due(self) -> None:
        """每 10 秒输出一次接收统计日志，便于排查"串口已开但无数据"。"""
        now: float = time.time()
        with self._stats_lock:
            elapsed: float = now - self._stats_last_time
            if elapsed >= 10.0:
                if self._stats_frames > 0:
                    logger.info(
                        "串口接收统计: %d 帧 / %d 字节 / %.1fs（%.1f 帧/秒）",
                        self._stats_frames, self._stats_bytes, elapsed,
                        self._stats_frames / elapsed,
                    )
                else:
                    logger.warning(
                        "串口 10 秒内未收到任何帧（已收到 %d 字节），"
                        "请检查 S21C 是否上电与探头连接",
                        self._stats_bytes,
                    )
                self._stats_frames = 0
                self._stats_bytes = 0
                self._stats_last_time = now
