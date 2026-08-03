"""
node_detect — 钢筋直径测量设备推理端（AI Inference Node）。

部署于 NVIDIA Jetson Nano (Ubuntu 22.04)，专用 AI 计算节点。
职责：接收服务节点 的 JPEG 帧 → UNet 推理 → 返回 PNG 类别掩码。
"""
__version__ = "0.1.0"
