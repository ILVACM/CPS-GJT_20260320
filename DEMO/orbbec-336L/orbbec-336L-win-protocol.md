# Demo2 TCP 视频流传输协议规范

> **文档用途**：本文件定义本机服务器与 Jetson Nano 客户端之间的 TCP 视频流传输协议。
> 可单独交付给 Jetson 端开发者，无需阅读主设计文档即可实现客户端。
>
> **协议版本**：v0.01
> **传输层**：TCP（局域网）
> **数据内容**：JPEG 压缩的 RGB 图像帧

---

## 1. 协议总览

### 1.1 通信模型

```
┌─────────────┐       TCP 连接       ┌─────────────┐
│  Jetson 客户 │ ──────────────────→ │  本机服务器  │
│  (主动连接)  │                     │  (被动监听)  │
└─────────────┘                     └─────────────┘
       │                                   │
       │  1. TCP 三次握手（操作系统完成）   │
       │  2. 应用层握手 HELLO / OK          │
       │  3. 持续接收帧数据流               │
       │  4. 任意一方关闭则结束             │
```

### 1.2 数据流向

- **握手阶段**：客户端 → 服务器（HELLO），服务器 → 客户端（OK）
- **推流阶段**：服务器 → 客户端（持续单向推送帧）
- 客户端在推流阶段**不需要发送任何数据**（纯接收）

---

## 2. 应用层握手

### 2.1 握手流程

```
客户端                                      服务器
  │                                          │
  │── TCP 连接 (SYN/SYN-ACK/ACK) ──────────→│  accept() 返回
  │                                          │
  │── "HELLO\n"  (6 字节 ASCII) ───────────→│  等待 HELLO（5秒超时）
  │                                          │  标记该连接"已就绪"
  │←── "OK <port>\n"  (例 "OK 8080\n") ─────│  回应实际监听端口
  │                                          │
  │════════════ 帧数据流开始 ════════════════│
  │←── [帧1头][帧1 jpeg] ────────────────────│
  │←── [帧2头][帧2 jpeg] ────────────────────│
  │   ...                                    │
```

### 2.2 握手报文格式

| 方向 | 报文 | 字节数 | 说明 |
|------|------|--------|------|
| C → S | `HELLO\n` | 6 | ASCII 文本，以 `\n` 结尾 |
| S → C | `OK <port>\n` | 可变 | 例 `OK 8080\n`，port 为服务器实际监听端口 |

### 2.3 握手规则

1. **客户端必须先发 `HELLO\n`**，服务器在收到前**不发送任何帧数据**
2. 服务器收到 `HELLO\n` 后立即回 `OK <port>\n`，之后开始推流
3. **握手超时**：服务器 accept 后 5 秒内未收到 `HELLO`，主动关闭连接
4. 客户端实现：`connect` → `sendall(b"HELLO\n")` → 循环 `recv` 直到遇到 `\n` → 进入读帧循环

---

## 3. 帧格式

### 3.1 帧结构

每一帧由 **28 字节定长帧头** + **变长 JPEG 数据** 组成：

```
┌──────────────────────────────────────────────────────────┐
│  帧头 (28 字节，定长)                                     │
├──────────────────────────────────────────────────────────┤
│  magic(4) │ ver(1) │ flags(1) │ reserved(2)             │
│  timestamp_ms(8) │ frame_index(4)                       │
│  width(2) │ height(2) │ jpeg_size(4)                    │
├──────────────────────────────────────────────────────────┤
│  JPEG 数据 (jpeg_size 字节，变长)                         │
└──────────────────────────────────────────────────────────┘
```

### 3.2 帧头字段定义（小端序）

| 偏移 | 长度 | 字段 | 类型 | 字节序 | 说明 |
|------|------|------|------|--------|------|
| 0 | 4 | `magic` | uint32 | LE | 魔数 `0x4F524242`（ASCII "ORBB"），帧起始标识 |
| 4 | 1 | `version` | uint8 | — | 协议版本，当前 `0x01` |
| 5 | 1 | `flags` | uint8 | — | 保留位，当前 `0x00` |
| 6 | 2 | `reserved` | uint16 | LE | 保留字段，当前 `0x0000` |
| 8 | 8 | `timestamp_ms` | uint64 | LE | 采集时间戳（Unix epoch 毫秒） |
| 16 | 4 | `frame_index` | uint32 | LE | 帧序号，服务器端单调递增 |
| 20 | 2 | `width` | uint16 | LE | JPEG 图像宽度（像素） |
| 22 | 2 | `height` | uint16 | LE | JPEG 图像高度（像素） |
| 24 | 4 | `jpeg_size` | uint32 | LE | 紧随其后的 JPEG 数据字节数 N |
| 28 | N | `jpeg_data` | bytes | — | JPEG 字节流 |

**帧头总长 = 28 字节**（定长，便于精确读取）

### 3.3 struct 打包格式字符串

服务器端打包 / 客户端解包使用如下 Python `struct` 格式：

```python
# '<' 表示小端序
HEADER_FORMAT = '<I B B H Q I H H I'
#  I  - uint32  magic
#  B  - uint8   version
#  B  - uint8   flags
#  H  - uint16  reserved
#  Q  - uint64  timestamp_ms
#  I  - uint32  frame_index
#  H  - uint16  width
#  H  - uint16  height
#  I  - uint32  jpeg_size
# 总字节数: 4+1+1+2+8+4+2+2+4 = 28
assert struct.calcsize(HEADER_FORMAT) == 28
```

---

## 4. 关键设计说明

### 4.1 为什么用 Magic Bytes

**问题**：TCP 是字节流，没有"帧边界"概念。如果客户端中途连接或网络出错后重连，如何知道下一帧从哪里开始？

**解决**：每帧开头 4 字节魔数 `0x4F524242`（"ORBB"）。客户端失步时可逐字节扫描，找到 magic 即重新对齐到一帧的开始。

> **类比**：像收音机找台。频段里全是杂音，你旋转旋钮直到听到熟悉的"滴——"标志音，就知道找到了电台。Magic 就是每帧开头的"标志音"。

### 4.2 如何解决粘包/半包

**粘包**：两帧数据粘在一起到达，客户端不知道从哪里切开。
**半包**：一帧数据被拆成多个 TCP 包到达，客户端收到的不完整。

**解决**：帧头中 `jpeg_size` 明确告知"这帧的 JPEG 数据有多少字节"。客户端读取规则：

1. 先精确读 28 字节帧头（用 `recv_exactly` 循环读直到满 28 字节）
2. 解析出 `jpeg_size = N`
3. 再精确读 N 字节 JPEG 数据
4. 一帧完整，交给应用层处理
5. 回到步骤 1 读下一帧

> **类比**：TCP 像一根水管，水连续流出，水管本身不告诉你"这是一杯、那是一杯"。我们每杯水前放一个固定大小的标签（28 字节帧头），标签上写"这杯有 N 毫升"。喝水的人先读标签，再按 N 精确接水，绝不会把两杯混在一起，也不会喝到半杯。

### 4.3 时间戳与帧序号的作用

| 字段 | 作用 | 客户端用法 |
|------|------|-----------|
| `timestamp_ms` | 采集时刻（epoch 毫秒） | `网络延迟 = 当前时间(ms) - timestamp_ms`，用于监控抖动 |
| `frame_index` | 服务器端帧序号，单调递增 | `丢帧数 = 当前 idx - 上次 idx - 1`，>0 表示有跳帧 |

**为什么需要两个？**
- 时间戳反映"绝对延迟"，但客户端与服务器时钟可能有偏差
- 帧序号反映"相对丢帧"，不依赖时钟同步，更可靠
- 两者结合：帧序号判断是否丢帧，时间戳判断延迟量级

---

## 5. 客户端实现参考

### 5.1 完整 Python 客户端示例

```python
import socket
import struct
import time
import cv2
import numpy as np

SERVER_IP = "192.168.1.100"   # 改为本机服务器实际 IP
SERVER_PORT = 8080

HEADER_FORMAT = '<I B B H Q I H H I'
HEADER_SIZE = 28
MAGIC = 0x4F524242


def recv_exactly(sock, n):
    """精确读取 n 字节，解决 TCP 半包问题。"""
    data = b""
    while len(data) < n:
        chunk = sock.recv(n - len(data))
        if not chunk:           # 对方关闭连接
            return None
        data += chunk
    return data


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((SERVER_IP, SERVER_PORT))
    print(f"[客户端] 已连接 {SERVER_IP}:{SERVER_PORT}")

    # 1. 应用层握手
    sock.sendall(b"HELLO\n")
    buf = b""
    while b"\n" not in buf:
        chunk = sock.recv(64)
        if not chunk:
            print("[客户端] 握手阶段连接断开")
            return
        buf += chunk
    print(f"[客户端] 握手回应: {buf.decode().strip()}")

    # 2. 读帧循环
    last_index = -1
    while True:
        # 2.1 读 28 字节帧头
        header = recv_exactly(sock, HEADER_SIZE)
        if header is None:
            print("[客户端] 连接已关闭")
            break

        (magic, ver, flags, reserved, ts_ms, idx,
         w, h, jpeg_size) = struct.unpack(HEADER_FORMAT, header)

        if magic != MAGIC:
            print(f"[客户端] 帧头 magic 不匹配: 0x{magic:08X}，可能失步")
            continue

        # 2.2 读 jpeg_size 字节 JPEG 数据
        jpeg = recv_exactly(sock, jpeg_size)
        if jpeg is None:
            print("[客户端] 读 JPEG 时连接断开")
            break

        # 2.3 解码显示
        img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            print(f"[客户端] 帧 {idx} JPEG 解码失败")
            continue

        # 2.4 延迟与丢帧监控
        latency = time.time() * 1000 - ts_ms
        lost = (idx - last_index - 1) if last_index >= 0 else 0
        if lost > 0:
            print(f"[客户端] ⚠ 帧 {idx} 检测到丢帧 {lost} 帧")
        cv2.putText(img, f"idx={idx} latency={latency:.0f}ms", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.imshow("Jetson Client", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        last_index = idx

    sock.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
```

### 5.2 接收循环要点

1. **必须用 `recv_exactly`**，不能假设一次 `recv` 就能读满一帧——TCP 会按 MTU 分片
2. **magic 校验**：收到帧头后先校验 magic，不匹配说明失步，可逐字节扫描重新对齐
3. **JPEG 解码失败处理**：网络数据偶尔损坏时 `cv2.imdecode` 返回 None，跳过该帧即可
4. **延迟计算**：`timestamp_ms` 是服务器采集时刻，与客户端时钟可能有偏差，但量级参考有价值

---

## 6. 协议版本演进预留

当前版本 v0.01，以下字段为未来扩展预留：

| 字段 | 当前值 | 未来用途 |
|------|--------|---------|
| `version` | `0x01` | 新版本协商（握手时可声明支持的版本范围） |
| `flags` | `0x00` | 位标志，如 bit0=含深度、bit1=关键帧 |
| `reserved` | `0x0000` | 16 位保留，可扩展为深度段长度等 |

**版本兼容策略**：客户端若收到 `version > 0x01` 的帧，应忽略无法识别的字段，按 v0.01 解析已知字段，保证向前兼容。

---

## 7. 调试技巧

### 7.1 用 nc 测试握手

```bash
# Linux/Mac
nc <服务器IP> 8080
# 连上后手动输入 HELLO 回车，应收到 OK <port>
```

### 7.2 用 Python 抓取一帧保存

```python
# 连接 + 握手后，读一帧并存为 jpg
header = recv_exactly(sock, 28)
magic, ver, flags, reserved, ts, idx, w, h, size = struct.unpack('<IBBHQIHHI', header)
jpeg = recv_exactly(sock, size)
with open("test_frame.jpg", "wb") as f:
    f.write(jpeg)
print(f"已保存帧 {idx}，尺寸 {w}x{h}，{size} 字节")
```

### 7.3 常见问题

| 现象 | 可能原因 |
|------|---------|
| 连接后无响应 | 未发 `HELLO\n`，或未以 `\n` 结尾 |
| 收到乱码 | 未先读 28 字节帧头，直接按文本解析了 |
| JPEG 解码失败 | 帧头 magic 不匹配（失步），或 `jpeg_size` 读错 |
| 延迟越来越大 | 正常情况下不应出现（最新帧策略）；若出现检查服务器实现 |
| frame_index 跳变 | 服务器应用层主动丢旧帧导致，属正常，可统计丢帧率 |
