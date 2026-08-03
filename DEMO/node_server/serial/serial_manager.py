"""串口设备管理器。

对齐 Design-server.md §3.4.4 与 AGENTS.md §5.1 / §6.1。
基于 pyserial 后台读串口线程，封装端口打开、后台读取、帧解析、数据获取与重连。

数据链路（AGENTS.md §6.1）：
4 路 STP23L → S21C 主控（汇总）→ Type-C 串口 → 服务节点 SerialManager
→ SerialBuffer（11 字节帧解析）→ LaserDataParser（4 路 uint16 距离值 + 有效性检查）
"""
import logging
import os
import sys
import threading
import time
from typing import Optional, Tuple

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
from common.constants import SERIAL_BAUDRATE

logger = logging.getLogger("node_server.serial")


class SerialManager:
    """串口设备管理类。

    封装端口打开、后台读取、帧解析、数据获取与重连。
    后台线程持续读串口 → SerialBuffer 喂入 → LaserDataParser 解析 → 更新最新结果。
    线程安全：``_data_lock`` 保护最新解析结果与时间戳。
    """

    def __init__(self, baudrate: int = SERIAL_BAUDRATE, timeout: float = 0.1):
        """初始化串口管理器。

        :param baudrate: 波特率，默认 115200（从 constants.py 读取）
        :param timeout: 串口读超时（秒），默认 0.1
        """
        self._baudrate: int = baudrate
        self._timeout: float = timeout
        self._serial_conn: Optional[Serial] = None
        self._port_name: str = ""
        self._is_running: bool = False
        self._buffer: SerialBuffer = SerialBuffer()
        self._data_lock: threading.Lock = threading.Lock()
        self._latest_result: Optional[LaserParseResult] = None
        self._data_timestamp: float = 0.0
        self._read_thread: Optional[threading.Thread] = None
        # 断线重连参数
        self._reconnect_attempts: int = 0
        self._max_reconnect_attempts: int = 5
        self._reconnect_interval: float = 3.0

    def open_port(self, port_name: str) -> Tuple[bool, str]:
        """打开指定串口并启动后台读取线程。

        :param port_name: 串口设备路径（如 ``/dev/ttyUSB0`` 或 ``COM3``）
        :return: ``(是否成功, 描述消息)``
        """
        try:
            self._serial_conn = Serial(
                port=port_name,
                baudrate=self._baudrate,
                timeout=self._timeout,
            )
            self._port_name = port_name
            self._is_running = True
            self._reconnect_attempts = 0
            self._buffer = SerialBuffer()
            self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self._read_thread.start()
            msg: str = f"成功打开串口 {port_name}（波特率 {self._baudrate}）"
            logger.info(msg)
            return True, msg
        except (SerialException, OSError) as e:
            self._serial_conn = None
            msg = f"打开串口 {port_name} 失败: {e}"
            logger.error(msg)
            return False, msg

    def close(self) -> None:
        """关闭串口并停止后台读取线程。"""
        self._is_running = False
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
        return self._is_running

    def get_port_name(self) -> str:
        """获取当前串口名称。"""
        return self._port_name

    def get_latest(self) -> Tuple[Optional[LaserParseResult], float]:
        """获取最新解析结果与时间戳（线程安全拷贝）。

        :return: ``(LaserParseResult 或 None, 数据时间戳)``；
                 无数据时返回 ``(None, 0.0)``
        """
        with self._data_lock:
            return self._latest_result, self._data_timestamp

    def try_reconnect(self) -> bool:
        """断线重连：间隔 3 秒，最多 5 次。

        :return: 重连成功返回 True；达到上限返回 False
        """
        if self._reconnect_attempts >= self._max_reconnect_attempts:
            logger.error("串口重连次数已达上限 %d 次，放弃重连", self._max_reconnect_attempts)
            return False
        self._reconnect_attempts += 1
        logger.warning(
            "串口尝试第 %d/%d 次重连（间隔 %.1ls）: %s",
            self._reconnect_attempts,
            self._max_reconnect_attempts,
            self._reconnect_interval,
            self._port_name,
        )
        time.sleep(self._reconnect_interval)
        ok, _ = self.open_port(self._port_name)
        return ok

    @staticmethod
    def list_ports() -> list:
        """枚举系统可用串口。

        :return: 串口设备信息列表（pyserial ListPortInfo 对象）
        """
        return list(list_ports.comports())

    def _read_loop(self) -> None:
        """后台线程：读取串口 → 缓冲区 → 帧解析 → 更新结果。

        异常捕获 SerialException → 标记离线 → try_reconnect。
        """
        while self._is_running and self._serial_conn is not None:
            try:
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
            except (SerialException, OSError) as e:
                logger.error("串口读取异常: %s", e, exc_info=True)
                self._is_running = False
                # 尝试重连（在新线程中执行，避免阻塞当前退出流程）
                reconnect_thread = threading.Thread(target=self.try_reconnect, daemon=True)
                reconnect_thread.start()
                break
            time.sleep(0.001)
