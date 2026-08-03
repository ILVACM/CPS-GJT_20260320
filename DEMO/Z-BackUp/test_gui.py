#!/usr/bin/env python3
import tkinter as tk
import os

print(f"DISPLAY = {os.environ.get('DISPLAY', '未设置')}")
print(f"WAYLAND_DISPLAY = {os.environ.get('WAYLAND_DISPLAY', '未设置')}")

try:
    print("尝试创建 Tk 窗口...")
    root = tk.Tk()
    root.title("测试")
    label = tk.Label(root, text="如果看到这个，GUI 正常")
    label.pack()
    print("窗口创建成功，2秒后关闭...")
    root.after(2000, root.destroy)
    root.mainloop()
    print("✅ GUI 环境正常")
except Exception as e:
    print(f"❌ GUI 环境失败：{e}")