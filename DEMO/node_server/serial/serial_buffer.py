"""串口数据缓冲区，处理粘包与半包。

对齐 Design-server.md §3.4.1 与 AGENTS.md §5.1（11 字节帧协议）。
帧结构：帧头 0x7B + 4×2 字节大端 uint16 距离值 + 1 字节 XOR 校验 + 帧尾 0x7D。
XOR 校验：前 9 字节异或和等于第 10 字节（手册称 RCC）。
"""
import threading
from typing import List, Optional, Tuple

from common.constants import (
    SERIAL_FRAME_HEADER,
    SERIAL_FRAME_LENGTH,
    SERIAL_FRAME_TAIL,
)


class SerialBuffer:
    """串口字节流缓冲区，从中提取完整且校验通过的 11 字节帧。

    线程安全：内部 ``_buffer`` 由 ``_lock`` 保护，``feed()`` 可在多线程下安全调用。
    """

    def __init__(self):
        self._buffer: bytearray = bytearray()
        self._lock = threading.Lock()

    def feed(self, data: bytes) -> List[bytes]:
        """喂入原始字节流，返回从中提取出的所有完整帧。

        处理粘包（一次喂入多帧）与半包（一帧分多次到达）：
        内部累积缓冲，循环调用 ``find_frame()`` 提取所有完整帧。

        :param data: 串口读到的原始字节
        :return: 本次新增解析出的完整帧列表（可能为空）
        """
        frames: List[bytes] = []
        with self._lock:
            self._buffer.extend(data)
            while True:
                frame, remaining = self.find_frame(bytes(self._buffer))
                if frame is None:
                    break
                frames.append(frame)
                # 保留剩余字节供下次拼接
                self._buffer = bytearray(remaining)
        return frames

    @staticmethod
    def find_frame(buffer: bytes) -> Tuple[Optional[bytes], bytes]:
        """从给定缓冲区中提取一个完整且校验通过的帧。

        扫描逻辑：
        1. 在可构成完整帧的范围内寻找帧头 0x7B
        2. 找到后取 11 字节候选帧，校验帧尾 0x7D 与 XOR 校验
        3. 校验失败则跳过该帧头字节继续扫描
        4. 无帧头时保留尾部（可能含未确认帧头）供下次拼接

        :param buffer: 输入字节缓冲区
        :return: ``(完整帧 bytes 或 None, 剩余字节)``
                 - 找到完整帧 → ``(帧数据, 帧之后的剩余字节)``
                 - 未找到 → ``(None, 需保留供下次拼接的尾部字节)``
        """
        n: int = len(buffer)
        pos: int = 0
        while pos + SERIAL_FRAME_LENGTH <= n:
            # 在可构成完整帧的范围内寻找帧头
            header_pos: int = -1
            scan_end: int = n - SERIAL_FRAME_LENGTH + 1
            for i in range(pos, scan_end):
                if buffer[i] == SERIAL_FRAME_HEADER:
                    header_pos = i
                    break
            if header_pos == -1:
                # 可扫描范围内无帧头：保留尾部（可能含未确认帧头）供下次拼接
                keep: int = min(n, SERIAL_FRAME_LENGTH - 1)
                if keep > 0:
                    return None, bytes(buffer[n - keep:])
                return None, b""
            # 提取候选帧
            candidate: bytes = buffer[header_pos:header_pos + SERIAL_FRAME_LENGTH]
            if candidate[-1] != SERIAL_FRAME_TAIL:
                # 帧尾不匹配，跳过此帧头继续扫描
                pos = header_pos + 1
                continue
            if SerialBuffer._verify_checksum(candidate):
                # 校验通过
                return candidate, bytes(buffer[header_pos + SERIAL_FRAME_LENGTH:])
            # 校验失败，跳过此帧头继续扫描
            pos = header_pos + 1
        # 剩余字节不足一帧，保留待下次拼接
        return None, bytes(buffer[pos:])

    @staticmethod
    def _verify_checksum(frame: bytes) -> bool:
        """XOR 校验：前 9 字节异或和等于第 10 字节（索引 9）。

        :param frame: 11 字节完整帧
        :return: 校验通过返回 True
        """
        xor_sum: int = 0
        for i in range(9):
            xor_sum ^= frame[i]
        return xor_sum == frame[9]
