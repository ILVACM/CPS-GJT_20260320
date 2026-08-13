#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
USB摄像头视频流与激光测距串口数据实时融合显示程序
支持 Windows (ARM/x64) 和 Linux (Ubuntu/Jetson) 跨平台运行
"""

import sys
import os
import threading
import time
import struct
import json
from collections import deque
from typing import Optional, Tuple, List, Callable, Dict, Any

import cv2
import serial
import serial.tools.list_ports
from PIL import Image, ImageTk, ImageDraw, ImageFont
import tkinter as tk
from tkinter import ttk, messagebox


class SerialBuffer:
    """串口数据缓冲区类，处理粘包和断包问题"""
    
    FRAME_HEADER = 0x7B
    FRAME_TAIL = 0x7D
    FRAME_LENGTH = 11
    
    def __init__(self):
        self.buffer = bytearray()
        self.lock = threading.Lock()
    
    def append(self, data: bytes) -> None:
        """添加数据到缓冲区"""
        with self.lock:
            self.buffer.extend(data)
    
    def find_frame(self) -> Optional[bytes]:
        """从缓冲区中查找并提取完整帧"""
        with self.lock:
            while len(self.buffer) >= self.FRAME_LENGTH:
                header_index = -1
                for i in range(len(self.buffer) - self.FRAME_LENGTH + 1):
                    if self.buffer[i] == self.FRAME_HEADER:
                        header_index = i
                        break
                
                if header_index == -1:
                    self.buffer = bytearray()
                    return None
                
                if header_index > 0:
                    del self.buffer[:header_index]
                
                if len(self.buffer) < self.FRAME_LENGTH:
                    return None
                
                potential_frame = bytes(self.buffer[:self.FRAME_LENGTH])
                
                if potential_frame[-1] != self.FRAME_TAIL:
                    del self.buffer[0]
                    continue
                
                if self._verify_checksum(potential_frame):
                    del self.buffer[:self.FRAME_LENGTH]
                    return potential_frame
                else:
                    del self.buffer[0]
            
            return None
    
    def _verify_checksum(self, frame: bytes) -> bool:
        """校验帧的XOR校验位"""
        if len(frame) != self.FRAME_LENGTH:
            return False
        
        xor_sum = 0
        for i in range(9):
            xor_sum ^= frame[i]
        
        return xor_sum == frame[9]


class LaserDataParser:
    """激光测距数据解析器"""
    
    @staticmethod
    def parse(frame: bytes) -> Tuple[int, int, int, int]:
        """
        解析11字节数据帧
        返回: (传感器1, 传感器2, 传感器3, 传感器4) 单位mm
        """
        if len(frame) != 11:
            raise ValueError("帧长度错误")
        
        sensor1 = struct.unpack('>H', frame[1:3])[0]
        sensor2 = struct.unpack('>H', frame[3:5])[0]
        sensor3 = struct.unpack('>H', frame[5:7])[0]
        sensor4 = struct.unpack('>H', frame[7:9])[0]
        
        return sensor1, sensor2, sensor3, sensor4


class CameraManager:
    """摄像头设备管理类"""
    
    MAX_SCAN_INDEX = 10
    SCAN_TIMEOUT = 2.0
    NETWORK_CAMERA_PATHS = ["/video", "/videofeed", "/mjpegfeed", "/video/feed", "/stream/video.mjpeg"]
    DEFAULT_NETWORK_PORTS = [8080, 80, 8888, 9000]
    CONFIG_FILE = "camera_config.json"
    RECONNECT_INTERVAL = 3.0
    MAX_RECONNECT_ATTEMPTS = 5
    
    def __init__(self):
        self.cap: Optional[cv2.VideoCapture] = None
        self.camera_index: int = -1
        self.is_running: bool = False
        self.frame_lock = threading.Lock()
        self.latest_frame: Optional[any] = None
        self.read_thread: Optional[threading.Thread] = None
        self.backend = self._get_preferred_backend()
        self.available_cameras: List[dict] = []
        
        self.camera_type: str = "usb"
        self.network_url: str = ""
        self.scan_thread: Optional[threading.Thread] = None
        self.scan_progress: int = 0
        self.scan_total: int = self.MAX_SCAN_INDEX
        self.scan_complete: bool = False
        self.scan_cancelled: bool = False
        self.saved_cameras: List[dict] = []
        self.scan_lock = threading.Lock()
        
        self.last_camera_info: Optional[dict] = None
        self.reconnect_attempts: int = 0
    
    def _get_preferred_backend(self) -> int:
        """获取平台首选后端"""
        if sys.platform == 'win32':
            return cv2.CAP_DSHOW
        else:
            return cv2.CAP_V4L2
    
    def _get_available_backends(self) -> List[int]:
        """获取可用的摄像头后端列表"""
        backends = []
        
        if sys.platform == 'win32':
            backends = [cv2.CAP_DSHOW, cv2.CAP_MSMF, cv2.CAP_ANY]
        else:
            backends = [cv2.CAP_V4L2, cv2.CAP_GSTREAMER, cv2.CAP_ANY]
        
        return backends
    
    def _check_video_devices(self) -> List[str]:
        """检查Linux系统中的视频设备文件"""
        if sys.platform == 'win32':
            return []
        
        video_devices = []
        dev_path = "/dev"
        
        if os.path.exists(dev_path):
            for item in os.listdir(dev_path):
                if item.startswith("video"):
                    full_path = os.path.join(dev_path, item)
                    if os.path.exists(full_path):
                        video_devices.append(full_path)
        
        return sorted(video_devices)
    
    def _check_device_permission(self) -> Tuple[bool, str]:
        """检查设备访问权限"""
        if sys.platform == 'win32':
            return True, ""
        
        video_devices = self._check_video_devices()
        
        if not video_devices:
            return False, "未检测到 /dev/video* 设备文件，请检查摄像头连接或驱动"
        
        for device in video_devices:
            if os.access(device, os.R_OK | os.W_OK):
                return True, ""
        
        return False, (
            f"检测到视频设备 {video_devices} 但无访问权限\n"
            "解决方案:\n"
            "1. 将当前用户添加到 video 组: sudo usermod -aG video $USER\n"
            "2. 重新登录或重启系统\n"
            "3. 或使用 sudo 运行程序"
        )
    
    def _try_open_camera(self, index: int, backend: int, timeout: float = 2.0) -> Optional[cv2.VideoCapture]:
        """尝试打开指定索引的摄像头"""
        try:
            cap = cv2.VideoCapture(index, backend)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            
            # 设置读取超时
            start_time = time.time()
            while time.time() - start_time < timeout:
                ret, _ = cap.read()
                if ret:
                    return cap
                time.sleep(0.1)
            
            cap.release()
        except Exception as e:
            print(f"[摄像头] 打开失败：{e}")
        return None
    
    def _get_config_path(self) -> str:
        """获取配置文件路径"""
        if getattr(sys, 'frozen', False):
            base_path = os.path.dirname(sys.executable)
        else:
            base_path = os.path.dirname(os.path.abspath(__file__))
        return os.path.join(base_path, self.CONFIG_FILE)
    
    def _load_saved_cameras(self) -> List[dict]:
        """加载保存的摄像头配置"""
        config_path = self._get_config_path()
        try:
            if os.path.exists(config_path):
                with open(config_path, 'r', encoding='utf-8') as f:
                    config = json.load(f)
                    return config.get('cameras', [])
        except Exception as e:
            print(f"[摄像头] 加载配置失败: {e}")
        return []
    
    def _save_camera_config(self, camera_info: dict):
        """保存摄像头配置"""
        config_path = self._get_config_path()
        try:
            saved = self._load_saved_cameras()
            existing = next((c for c in saved if c.get('url') == camera_info.get('url')), None)
            if existing:
                saved.remove(existing)
            camera_info['last_used'] = time.strftime("%Y-%m-%d %H:%M:%S")
            saved.insert(0, camera_info)
            if len(saved) > 10:
                saved = saved[:10]
            with open(config_path, 'w', encoding='utf-8') as f:
                json.dump({'cameras': saved}, f, ensure_ascii=False, indent=2)
            self.saved_cameras = saved
        except Exception as e:
            print(f"[摄像头] 保存配置失败: {e}")
    
    def _remove_camera_config(self, url: str):
        """移除摄像头配置"""
        config_path = self._get_config_path()
        try:
            saved = self._load_saved_cameras()
            saved = [c for c in saved if c.get('url') != url]
            with open(config_path, 'w', encoding='utf-8') as f:
                json.dump({'cameras': saved}, f, ensure_ascii=False, indent=2)
            self.saved_cameras = saved
        except Exception as e:
            print(f"[摄像头] 移除配置失败: {e}")
    
    def _validate_network_url(self, url: str, timeout: float = 5.0) -> Tuple[bool, dict]:
        """验证网络摄像头URL"""
        print(f"[网络摄像头] 验证URL: {url}")
        try:
            cap = cv2.VideoCapture(url)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            
            start_time = time.time()
            while time.time() - start_time < timeout:
                ret, frame = cap.read()
                if ret:
                    height, width = frame.shape[:2]
                    cap.release()
                    camera_info = {
                        "type": "network",
                        "index": -1,
                        "name": f"网络摄像头",
                        "resolution": f"{width}x{height}",
                        "backend": "HTTP/RTSP",
                        "url": url,
                        "source": url
                    }
                    print(f"[网络摄像头] 验证成功: {width}x{height}")
                    return True, camera_info
                time.sleep(0.1)
            
            cap.release()
        except Exception as e:
            print(f"[网络摄像头] 验证失败: {e}")
        
        return False, {}
    
    def _probe_ip_camera(self, ip: str, ports: List[int] = None, 
                         paths: List[str] = None, timeout: float = 3.0,
                         progress_callback: Callable[[str], None] = None) -> Optional[str]:
        """探测IP地址的网络摄像头"""
        if ports is None:
            ports = self.DEFAULT_NETWORK_PORTS
        if paths is None:
            paths = self.NETWORK_CAMERA_PATHS
        
        for port in ports:
            for path in paths:
                url = f"http://{ip}:{port}{path}"
                if progress_callback:
                    progress_callback(url)
                print(f"[网络摄像头] 探测: {url}")
                success, _ = self._validate_network_url(url, timeout=timeout)
                if success:
                    return url
        
        return None
    
    def _open_network_camera(self, camera_info: dict) -> Tuple[bool, str]:
        """打开网络摄像头"""
        url = camera_info.get('url', '')
        if not url:
            return False, "无效的网络摄像头URL"
        
        try:
            self.cap = cv2.VideoCapture(url)
            if not self.cap.isOpened():
                return False, f"无法连接到 {url}"
            
            ret, _ = self.cap.read()
            if not ret:
                self.cap.release()
                return False, "无法从网络摄像头读取视频"
            
            self.camera_type = "network"
            self.network_url = url
            self.camera_index = -1
            self.last_camera_info = camera_info
            self.reconnect_attempts = 0
            
            self._save_camera_config(camera_info)
            
            self.is_running = True
            self.read_thread = threading.Thread(target=self._read_frames, daemon=True)
            self.read_thread.start()
            
            return True, f"成功连接网络摄像头: {url}"
            
        except Exception as e:
            return False, f"连接网络摄像头失败: {str(e)}"
    
    def _scan_cameras(self) -> List[dict]:
        """
        同步扫描所有可用摄像头（保留用于兼容）
        返回: 摄像头信息列表
        """
        cameras = []
        
        if sys.platform != 'win32':
            perm_ok, perm_msg = self._check_device_permission()
            if not perm_ok:
                print(f"[摄像头] 权限检查失败: {perm_msg}")
                return cameras
            
            video_devices = self._check_video_devices()
            print(f"[摄像头] 检测到视频设备: {video_devices}")
        
        backends = self._get_available_backends()
        print(f"[摄像头] 尝试后端: {backends}")
        
        for index in range(self.MAX_SCAN_INDEX):
            for backend in backends:
                cap = self._try_open_camera(index, backend)
                
                if cap is not None:
                    try:
                        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                        
                        try:
                            backend_name = cap.getBackendName()
                        except:
                            backend_name = "unknown"
                        
                        camera_name = f"摄像头 {index} ({backend_name})"
                        
                        cameras.append({
                            "type": "usb",
                            "index": index,
                            "name": camera_name,
                            "resolution": f"{width}x{height}",
                            "backend": backend_name,
                            "url": None,
                            "source": index
                        })
                        
                        print(f"[摄像头] 发现: 索引={index}, 后端={backend_name}, 分辨率={width}x{height}")
                        
                    except Exception as e:
                        print(f"[摄像头] 获取设备信息失败: {e}")
                    finally:
                        cap.release()
                    
                    break
        
        return cameras
    
    def _start_async_scan(self, on_progress: Callable[[int, int], None] = None,
                          on_camera_found: Callable[[dict], None] = None,
                          on_complete: Callable[[List[dict]], None] = None):
        """启动异步扫描"""
        self.scan_cancelled = False
        self.scan_complete = False
        self.scan_progress = 0
        self.available_cameras = []
        
        def scan_worker():
            cameras = []
            
            if sys.platform != 'win32':
                perm_ok, perm_msg = self._check_device_permission()
                if not perm_ok:
                    print(f"[摄像头] 权限检查失败: {perm_msg}")
                    self.scan_complete = True
                    if on_complete:
                        on_complete(cameras)
                    return
                
                video_devices = self._check_video_devices()
                print(f"[摄像头] 检测到视频设备: {video_devices}")
            
            backends = self._get_available_backends()
            print(f"[摄像头] 尝试后端: {backends}")
            
            for index in range(self.MAX_SCAN_INDEX):
                if self.scan_cancelled:
                    print("[摄像头] 扫描已取消")
                    break
                
                with self.scan_lock:
                    self.scan_progress = index + 1
                
                if on_progress:
                    on_progress(index + 1, self.MAX_SCAN_INDEX)
                
                for backend in backends:
                    if self.scan_cancelled:
                        break
                    
                    cap = self._try_open_camera(index, backend, timeout=1.0)
                    
                    if cap is not None:
                        try:
                            width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                            height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                            
                            try:
                                backend_name = cap.getBackendName()
                            except:
                                backend_name = "unknown"
                            
                            camera_name = f"摄像头 {index} ({backend_name})"
                            
                            camera_info = {
                                "type": "usb",
                                "index": index,
                                "name": camera_name,
                                "resolution": f"{width}x{height}",
                                "backend": backend_name,
                                "url": None,
                                "source": index
                            }
                            
                            cameras.append(camera_info)
                            
                            with self.scan_lock:
                                self.available_cameras = cameras.copy()
                            
                            if on_camera_found:
                                on_camera_found(camera_info)
                            
                            print(f"[摄像头] 发现: 索引={index}, 后端={backend_name}, 分辨率={width}x{height}")
                            
                        except Exception as e:
                            print(f"[摄像头] 获取设备信息失败: {e}")
                        finally:
                            cap.release()
                        
                        break
            
            self.scan_complete = True
            if on_complete:
                on_complete(cameras)
        
        self.scan_thread = threading.Thread(target=scan_worker, daemon=True)
        self.scan_thread.start()
    
    def _stop_scan(self):
        """停止扫描"""
        self.scan_cancelled = True
        if self.scan_thread and self.scan_thread.is_alive():
            self.scan_thread.join(timeout=2.0)
    
    def _show_camera_selection_dialog(self, cameras: List[dict]) -> Optional[dict]:
        """
        显示摄像头选择对话框（同步版本，用于兼容）
        返回: 选择的摄像头信息字典，取消返回None
        """
        dialog = tk.Toplevel()
        dialog.title("选择摄像头")
        dialog.geometry("500x400")
        dialog.resizable(False, False)
        dialog.transient()
        dialog.grab_set()
        
        dialog.update_idletasks()
        x = (dialog.winfo_screenwidth() - 500) // 2
        y = (dialog.winfo_screenheight() - 400) // 2
        dialog.geometry(f"+{x}+{y}")
        
        selected_camera = [None]
        
        ttk.Label(dialog, text="请选择要使用的摄像头设备：", 
                  font=("Arial", 11, "bold")).pack(pady=10)
        
        list_frame = ttk.Frame(dialog)
        list_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=5)
        
        scrollbar = ttk.Scrollbar(list_frame)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        listbox = tk.Listbox(list_frame, font=("Consolas", 10), 
                             selectmode=tk.SINGLE, height=8,
                             yscrollcommand=scrollbar.set)
        listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.config(command=listbox.yview)
        
        for cam in cameras:
            type_tag = "[USB]" if cam.get('type') == 'usb' else "[NET]"
            display_text = f"{type_tag} {cam['name']} - {cam['resolution']}"
            listbox.insert(tk.END, display_text)
        
        if cameras:
            listbox.selection_set(0)
        
        info_frame = ttk.LabelFrame(dialog, text="设备详情", padding=5)
        info_frame.pack(fill=tk.X, padx=20, pady=5)
        
        info_label = ttk.Label(info_frame, text="请选择摄像头查看详情", font=("Arial", 9))
        info_label.pack(fill=tk.X)
        
        def on_select(event):
            selection = listbox.curselection()
            if selection:
                cam = cameras[selection[0]]
                type_text = "USB" if cam.get('type') == 'usb' else "网络"
                info_text = f"类型: {type_text} | 分辨率: {cam['resolution']} | 后端: {cam.get('backend', 'N/A')}"
                if cam.get('url'):
                    info_text += f"\nURL: {cam['url']}"
                info_label.config(text=info_text)
        
        listbox.bind('<<ListboxSelect>>', on_select)
        
        def on_double_click(event):
            selection = listbox.curselection()
            if selection:
                selected_camera[0] = cameras[selection[0]]
                dialog.destroy()
        
        listbox.bind('<Double-Button-1>', on_double_click)
        
        button_frame = ttk.Frame(dialog)
        button_frame.pack(pady=15)
        
        def on_add_network():
            camera_info = self._show_network_camera_dialog(dialog)
            if camera_info:
                selected_camera[0] = camera_info
                dialog.destroy()
        
        def on_confirm():
            selection = listbox.curselection()
            if selection:
                selected_camera[0] = cameras[selection[0]]
                dialog.destroy()
            else:
                messagebox.showwarning("提示", "请选择一个摄像头", parent=dialog)
        
        def on_cancel():
            dialog.destroy()
        
        ttk.Button(button_frame, text="添加网络摄像头", command=on_add_network).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="确定", width=10, command=on_confirm).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="取消", width=10, command=on_cancel).pack(side=tk.LEFT, padx=5)
        
        dialog.wait_window()
        
        return selected_camera[0]
    
    def _show_camera_selection_dialog_async(self, parent: tk.Tk, 
                                            on_selected: Callable[[Optional[dict]], None]) -> tk.Toplevel:
        """
        显示摄像头选择对话框（异步版本，支持后台扫描）
        返回: 对话框对象
        """
        dialog = tk.Toplevel(parent)
        dialog.title("选择摄像头")
        dialog.geometry("550x450")
        dialog.resizable(False, False)
        dialog.transient(parent)
        dialog.grab_set()
        
        dialog.update_idletasks()
        x = (dialog.winfo_screenwidth() - 550) // 2
        y = (dialog.winfo_screenheight() - 450) // 2
        dialog.geometry(f"+{x}+{y}")
        
        self.saved_cameras = self._load_saved_cameras()
        camera_list = []
        usb_cameras = []
        refreshing_list = False
        
        status_frame = ttk.LabelFrame(dialog, text="扫描状态", padding=5)
        status_frame.pack(fill=tk.X, padx=20, pady=10)
        
        scan_status_label = ttk.Label(status_frame, text="正在扫描USB摄像头...", font=("Arial", 10))
        scan_status_label.pack(anchor=tk.W)
        
        progress_var = tk.DoubleVar(value=0)
        progress_bar = ttk.Progressbar(status_frame, variable=progress_var, maximum=self.MAX_SCAN_INDEX)
        progress_bar.pack(fill=tk.X, pady=5)
        
        progress_label = ttk.Label(status_frame, text="0/10")
        progress_label.pack(anchor=tk.E)
        
        list_frame = ttk.LabelFrame(dialog, text="摄像头列表", padding=5)
        list_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=5)
        
        listbox_frame = ttk.Frame(list_frame)
        listbox_frame.pack(fill=tk.BOTH, expand=True)
        
        scrollbar = ttk.Scrollbar(listbox_frame)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        listbox = tk.Listbox(listbox_frame, font=("Consolas", 10), 
                             selectmode=tk.SINGLE, height=10,
                             yscrollcommand=scrollbar.set)
        listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.config(command=listbox.yview)
        
        info_frame = ttk.LabelFrame(dialog, text="设备详情", padding=5)
        info_frame.pack(fill=tk.X, padx=20, pady=5)
        
        info_label = ttk.Label(info_frame, text="请选择摄像头查看详情", font=("Arial", 9))
        info_label.pack(fill=tk.X)
        
        def update_listbox(camera_info: dict):
            if refreshing_list:
                return
            if camera_info not in camera_list:
                camera_list.append(camera_info)
                if camera_info.get('type') == 'usb':
                    usb_cameras.append(camera_info)
                type_tag = "[USB]" if camera_info.get('type') == 'usb' else "[NET]"
                display_text = f"{type_tag} {camera_info['name']} - {camera_info['resolution']}"
                listbox.insert(tk.END, display_text)
                if listbox.size() == 1:
                    listbox.selection_set(0)
        
        def on_select(event):
            selection = listbox.curselection()
            if selection and selection[0] < len(camera_list):
                cam = camera_list[selection[0]]
                type_text = "USB" if cam.get('type') == 'usb' else "网络"
                info_text = f"类型: {type_text} | 分辨率: {cam['resolution']} | 后端: {cam.get('backend', 'N/A')}"
                if cam.get('url'):
                    info_text += f"\nURL: {cam['url']}"
                info_label.config(text=info_text)
        
        listbox.bind('<<ListboxSelect>>', on_select)
        
        def on_progress(current: int, total: int):
            if dialog.winfo_exists():
                progress_var.set(current)
                progress_label.config(text=f"{current}/{total}")
        
        def on_camera_found(camera_info: dict):
            if dialog.winfo_exists() and not refreshing_list:
                dialog.after(0, lambda: update_listbox(camera_info))
        
        def on_complete(cameras: List[dict]):
            if dialog.winfo_exists() and not refreshing_list:
                if cameras:
                    scan_status_label.config(text=f"扫描完成，发现 {len(cameras)} 个USB摄像头")
                else:
                    scan_status_label.config(text="扫描完成，未发现USB摄像头")
                
                for saved in self.saved_cameras:
                    if saved not in camera_list:
                        saved['type'] = 'network'
                        dialog.after(0, lambda s=saved: update_listbox(s))
        
        self._start_async_scan(
            on_progress=on_progress,
            on_camera_found=on_camera_found,
            on_complete=on_complete
        )
        
        button_frame = ttk.Frame(dialog)
        button_frame.pack(pady=15)
        
        def on_add_network():
            camera_info = self._show_network_camera_dialog(dialog)
            if camera_info:
                self._stop_scan()
                on_selected(camera_info)
                dialog.destroy()
        
        def on_manage_history():
            nonlocal refreshing_list
            
            def rebuild_list():
                listbox.delete(0, tk.END)
                camera_list.clear()
                for cam in usb_cameras:
                    camera_list.append(cam)
                    listbox.insert(tk.END, f"[USB] {cam['name']} - {cam['resolution']}")
                self.saved_cameras = self._load_saved_cameras()
                for saved in self.saved_cameras:
                    if saved not in camera_list:
                        saved['type'] = 'network'
                        camera_list.append(saved)
                        listbox.insert(tk.END, f"[NET] {saved['name']} - {saved['resolution']}")
                if listbox.size() > 0:
                    listbox.selection_set(0)
                    on_select(None)
            
            refreshing_list = True
            self._show_history_manager_dialog(dialog, on_changed=rebuild_list)
            refreshing_list = False
            rebuild_list()
        
        def on_confirm():
            selection = listbox.curselection()
            if selection and selection[0] < len(camera_list):
                self._stop_scan()
                on_selected(camera_list[selection[0]])
                dialog.destroy()
            else:
                messagebox.showwarning("提示", "请选择一个摄像头", parent=dialog)
        
        def on_cancel():
            self._stop_scan()
            on_selected(None)
            dialog.destroy()
        
        def on_closing():
            self._stop_scan()
            on_selected(None)
            dialog.destroy()
        
        dialog.protocol("WM_DELETE_WINDOW", on_closing)
        
        ttk.Button(button_frame, text="添加网络摄像头", command=on_add_network).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="管理历史", command=on_manage_history).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="确定", width=10, command=on_confirm).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="取消", width=10, command=on_cancel).pack(side=tk.LEFT, padx=5)
        
        return dialog
    
    def _show_history_manager_dialog(self, parent: tk.Toplevel, on_changed: Callable[[], None] = None):
        """显示历史记录批量管理对话框"""
        dialog = tk.Toplevel(parent)
        dialog.title("管理历史记录")
        dialog.geometry("520x380")
        dialog.resizable(False, False)
        dialog.transient(parent)
        dialog.grab_set()
        
        dialog.update_idletasks()
        x = parent.winfo_x() + (parent.winfo_width() - 520) // 2
        y = parent.winfo_y() + (parent.winfo_height() - 380) // 2
        dialog.geometry(f"+{x}+{y}")
        
        self.saved_cameras = self._load_saved_cameras()
        
        list_frame = ttk.LabelFrame(dialog, text=f"历史记录（共 {len(self.saved_cameras)} 条）", padding=5)
        list_frame.pack(fill=tk.BOTH, expand=True, padx=15, pady=10)
        
        list_scrollbar = ttk.Scrollbar(list_frame)
        list_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        listbox = tk.Listbox(list_frame, font=("Consolas", 9),
                             selectmode=tk.MULTIPLE, height=10,
                             yscrollcommand=list_scrollbar.set)
        listbox.pack(fill=tk.BOTH, expand=True)
        list_scrollbar.config(command=listbox.yview)
        
        def refresh_list():
            listbox.delete(0, tk.END)
            self.saved_cameras = self._load_saved_cameras()
            for cam in self.saved_cameras:
                listbox.insert(tk.END, f"{cam.get('name', '网络摄像头')} - {cam.get('url', '')}")
            list_frame.config(text=f"历史记录（共 {len(self.saved_cameras)} 条）")
            if on_changed:
                on_changed()
        
        refresh_list()
        
        btn_frame = ttk.Frame(dialog)
        btn_frame.pack(fill=tk.X, padx=15, pady=5)
        
        def select_all():
            listbox.select_set(0, tk.END)
        
        def deselect_all():
            listbox.selection_clear(0, tk.END)
        
        def delete_selected():
            selections = list(listbox.curselection())
            if not selections:
                messagebox.showinfo("提示", "请先选中要删除的记录", parent=dialog)
                return
            count = len(selections)
            if not messagebox.askyesno("确认删除", f"确定删除选中的 {count} 条记录？", parent=dialog):
                return
            urls_to_delete = []
            for idx in selections:
                if idx < len(self.saved_cameras):
                    urls_to_delete.append(self.saved_cameras[idx].get('url', ''))
            for url in urls_to_delete:
                self._remove_camera_config(url)
            refresh_list()
        
        def clear_all():
            if not self.saved_cameras:
                messagebox.showinfo("提示", "没有可清空的记录", parent=dialog)
                return
            count = len(self.saved_cameras)
            if not messagebox.askyesno("确认清空", f"确定清空全部 {count} 条历史记录？", parent=dialog):
                return
            config_path = self._get_config_path()
            try:
                with open(config_path, 'w', encoding='utf-8') as f:
                    json.dump({'cameras': []}, f, ensure_ascii=False, indent=2)
            except Exception as e:
                print(f"[摄像头] 清空配置失败: {e}")
            self.saved_cameras = []
            refresh_list()
        
        ttk.Button(btn_frame, text="全选", width=8, command=select_all).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="取消全选", width=8, command=deselect_all).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="删除选中", width=8, command=delete_selected).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="清空全部", width=8, command=clear_all).pack(side=tk.LEFT, padx=5)
        
        close_btn = ttk.Button(dialog, text="关闭", width=10, command=dialog.destroy)
        close_btn.pack(pady=10)
    
    def _show_network_camera_dialog(self, parent: tk.Toplevel) -> Optional[dict]:
        """显示网络摄像头输入对话框"""
        dialog = tk.Toplevel(parent)
        dialog.title("添加网络摄像头")
        dialog.geometry("500x520")
        dialog.resizable(False, False)
        dialog.transient(parent)
        dialog.grab_set()
        
        dialog.update_idletasks()
        x = parent.winfo_x() + (parent.winfo_width() - 500) // 2
        y = parent.winfo_y() + (parent.winfo_height() - 520) // 2
        dialog.geometry(f"+{x}+{y}")
        
        result = [None]
        
        mode_frame = ttk.LabelFrame(dialog, text="输入方式", padding=10)
        mode_frame.pack(fill=tk.X, padx=20, pady=10)
        
        mode_var = tk.StringVar(value="url")
        ttk.Radiobutton(mode_frame, text="完整URL", variable=mode_var, value="url").pack(anchor=tk.W)
        ttk.Radiobutton(mode_frame, text="IP地址 + 自动探测", variable=mode_var, value="ip").pack(anchor=tk.W)
        ttk.Radiobutton(mode_frame, text="历史记录", variable=mode_var, value="history").pack(anchor=tk.W)
        
        input_frame = ttk.Frame(dialog)
        input_frame.pack(fill=tk.X, padx=20, pady=5)
        
        url_frame = ttk.Frame(input_frame)
        url_frame.pack(fill=tk.X)
        
        ttk.Label(url_frame, text="URL地址:").pack(anchor=tk.W)
        url_entry = ttk.Entry(url_frame, width=60)
        url_entry.pack(fill=tk.X, pady=2)
        url_entry.insert(0, "http://192.168.1.100:8080/video")
        
        quick_frame = ttk.Frame(url_frame)
        quick_frame.pack(fill=tk.X, pady=5)
        ttk.Label(quick_frame, text="快速填充:").pack(side=tk.LEFT)
        
        def fill_url(path):
            url_entry.delete(0, tk.END)
            url_entry.insert(0, f"http://:8080{path}")
            url_entry.icursor(7)
        
        ttk.Button(quick_frame, text="/video", command=lambda: fill_url("/video")).pack(side=tk.LEFT, padx=2)
        ttk.Button(quick_frame, text="/videofeed", command=lambda: fill_url("/videofeed")).pack(side=tk.LEFT, padx=2)
        
        ip_frame = ttk.Frame(input_frame)
        
        ip_row = ttk.Frame(ip_frame)
        ip_row.pack(fill=tk.X, pady=2)
        ttk.Label(ip_row, text="IP地址:").pack(side=tk.LEFT)
        ip_entry = ttk.Entry(ip_row, width=20)
        ip_entry.pack(side=tk.LEFT, padx=5)
        ip_entry.insert(0, "192.168.1.100")
        
        ttk.Label(ip_row, text="端口:").pack(side=tk.LEFT, padx=(10, 0))
        port_entry = ttk.Entry(ip_row, width=8)
        port_entry.pack(side=tk.LEFT, padx=5)
        port_entry.insert(0, "8080")
        
        auto_port_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(ip_frame, text="自动探测常见端口", variable=auto_port_var).pack(anchor=tk.W, pady=5)
        
        history_frame = ttk.LabelFrame(input_frame, text="历史记录", padding=5)
        
        history_scrollbar = ttk.Scrollbar(history_frame)
        history_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        history_listbox = tk.Listbox(history_frame, height=5, font=("Consolas", 9),
                                      selectmode=tk.MULTIPLE, yscrollcommand=history_scrollbar.set)
        history_listbox.pack(fill=tk.BOTH, expand=True)
        history_scrollbar.config(command=history_listbox.yview)
        
        def refresh_history_list():
            history_listbox.delete(0, tk.END)
            for cam in self.saved_cameras:
                history_listbox.insert(tk.END, f"{cam.get('name', '网络摄像头')} - {cam.get('url', '')}")
        
        refresh_history_list()
        
        history_btn_frame = ttk.Frame(history_frame)
        history_btn_frame.pack(fill=tk.X, pady=(5, 0))
        
        def select_all():
            history_listbox.select_set(0, tk.END)
        
        def deselect_all():
            history_listbox.selection_clear(0, tk.END)
        
        def delete_selected():
            selections = list(history_listbox.curselection())
            if not selections:
                return
            count = len(selections)
            if not messagebox.askyesno("确认删除", f"确定删除选中的 {count} 条记录？", parent=dialog):
                return
            urls_to_delete = []
            for idx in selections:
                if idx < len(self.saved_cameras):
                    urls_to_delete.append(self.saved_cameras[idx].get('url', ''))
            for url in urls_to_delete:
                self._remove_camera_config(url)
            refresh_history_list()
            status_label.config(text=f"已删除 {count} 条记录", foreground="green")
        
        def clear_all():
            if not self.saved_cameras:
                return
            count = len(self.saved_cameras)
            if not messagebox.askyesno("确认清空", f"确定清空全部 {count} 条历史记录？", parent=dialog):
                return
            config_path = self._get_config_path()
            try:
                with open(config_path, 'w', encoding='utf-8') as f:
                    json.dump({'cameras': []}, f, ensure_ascii=False, indent=2)
            except Exception as e:
                print(f"[摄像头] 清空配置失败: {e}")
            self.saved_cameras = []
            refresh_history_list()
            status_label.config(text=f"已清空 {count} 条记录", foreground="green")
        
        ttk.Button(history_btn_frame, text="全选", width=6, command=select_all).pack(side=tk.LEFT, padx=2)
        ttk.Button(history_btn_frame, text="取消全选", width=8, command=deselect_all).pack(side=tk.LEFT, padx=2)
        ttk.Button(history_btn_frame, text="删除选中", width=8, command=delete_selected).pack(side=tk.LEFT, padx=2)
        ttk.Button(history_btn_frame, text="清空全部", width=8, command=clear_all).pack(side=tk.LEFT, padx=2)
        
        def update_input_frame():
            for widget in [url_frame, ip_frame, history_frame]:
                widget.pack_forget()
            
            mode = mode_var.get()
            if mode == "url":
                url_frame.pack(fill=tk.X)
            elif mode == "ip":
                ip_frame.pack(fill=tk.X)
            else:
                history_frame.pack(fill=tk.X, pady=5)
                refresh_history_list()
        
        mode_var.trace('w', lambda *args: update_input_frame())
        update_input_frame()
        
        status_frame = ttk.LabelFrame(dialog, text="连接状态", padding=5)
        status_frame.pack(fill=tk.X, padx=20, pady=10)
        
        status_label = ttk.Label(status_frame, text="等待输入...", font=("Arial", 9))
        status_label.pack(fill=tk.X)
        
        button_frame = ttk.Frame(dialog)
        button_frame.pack(pady=15)
        
        def on_verify():
            mode = mode_var.get()
            status_label.config(text="正在验证连接...", foreground="blue")
            dialog.update()
            
            if mode == "url":
                url = url_entry.get().strip()
                if not url:
                    status_label.config(text="请输入URL地址", foreground="red")
                    return
                success, camera_info = self._validate_network_url(url)
                
            elif mode == "ip":
                ip = ip_entry.get().strip()
                if not ip:
                    status_label.config(text="请输入IP地址", foreground="red")
                    return
                
                if auto_port_var.get():
                    ports = self.DEFAULT_NETWORK_PORTS
                else:
                    try:
                        ports = [int(port_entry.get().strip())]
                    except ValueError:
                        status_label.config(text="端口格式错误", foreground="red")
                        return
                
                found_url = None
                for port in ports:
                    for path in self.NETWORK_CAMERA_PATHS:
                        test_url = f"http://{ip}:{port}{path}"
                        status_label.config(text=f"探测: {test_url}")
                        dialog.update()
                        success, camera_info = self._validate_network_url(test_url, timeout=2.0)
                        if success:
                            found_url = test_url
                            break
                    if found_url:
                        break
                
                if not found_url:
                    success = False
                    camera_info = {}
            
            else:
                selection = history_listbox.curselection()
                if not selection:
                    status_label.config(text="请选择历史记录", foreground="red")
                    return
                
                cam = self.saved_cameras[selection[0]]
                success, camera_info = self._validate_network_url(cam.get('url', ''))
            
            if success:
                status_label.config(text="连接成功！", foreground="green")
                camera_info['name'] = f"网络摄像头 ({camera_info.get('url', '').split('//')[-1].split('/')[0]})"
                result[0] = camera_info
                dialog.destroy()
            else:
                status_label.config(text="连接失败，请检查地址或网络", foreground="red")
        
        def on_cancel():
            dialog.destroy()
        
        ttk.Button(button_frame, text="验证连接", command=on_verify).pack(side=tk.LEFT, padx=10)
        ttk.Button(button_frame, text="取消", command=on_cancel).pack(side=tk.LEFT, padx=10)
        
        dialog.wait_window()
        
        return result[0]
    
    def scan_and_open(self) -> Tuple[bool, str]:
        """
        扫描并打开摄像头（同步版本，保留兼容）
        返回: (是否成功, 消息)
        """
        self.available_cameras = self._scan_cameras()
        
        saved = self._load_saved_cameras()
        for cam in saved:
            cam['type'] = 'network'
            self.available_cameras.append(cam)
        
        if not self.available_cameras:
            if sys.platform != 'win32':
                perm_ok, perm_msg = self._check_device_permission()
                if not perm_ok:
                    return False, perm_msg
            return False, "未找到可用摄像头，请检查USB连接或添加网络摄像头"
        
        selected = self._show_camera_selection_dialog(self.available_cameras)
        
        if selected is None:
            return False, "用户取消了摄像头选择"
        
        if selected.get('type') == 'network':
            return self._open_network_camera(selected)
        else:
            return self._open_usb_camera(selected)
    
    def _open_usb_camera(self, camera_info: dict) -> Tuple[bool, str]:
        """打开USB摄像头"""
        index = camera_info.get('index', 0)
        print(f"[摄像头] 尝试打开: 索引={index}")
        
        backends = self._get_available_backends()
        opened = False
        
        for backend in backends:
            try:
                self.cap = cv2.VideoCapture(index, backend)
                if self.cap.isOpened():
                    ret, _ = self.cap.read()
                    if ret:
                        print(f"[摄像头] 成功打开: 后端={backend}")
                        opened = True
                        break
                    self.cap.release()
            except Exception as e:
                print(f"[摄像头] 后端 {backend} 打开失败: {e}")
                continue
        
        if not opened:
            return False, f"无法打开摄像头索引 {index}，请检查设备权限或驱动"
        
        self.camera_type = "usb"
        self.camera_index = index
        self.last_camera_info = camera_info
        self.reconnect_attempts = 0
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        self.is_running = True
        self.read_thread = threading.Thread(target=self._read_frames, daemon=True)
        self.read_thread.start()
        
        return True, f"成功打开摄像头索引 {index}"
    
    def _read_frames(self):
        """独立线程中读取视频帧，检测断线"""
        consecutive_failures = 0
        while self.is_running and self.cap:
            ret, frame = self.cap.read()
            if ret:
                consecutive_failures = 0
                with self.frame_lock:
                    self.latest_frame = frame
            else:
                consecutive_failures += 1
                if consecutive_failures >= 30:
                    print("[摄像头] 连续读取失败，判定为断线")
                    self.is_running = False
                    break
                time.sleep(0.01)
    
    def get_frame(self) -> Optional[any]:
        """获取最新帧"""
        with self.frame_lock:
            if self.latest_frame is not None:
                return self.latest_frame.copy()
            return None
    
    def _get_permission_error_msg(self, device: str) -> str:
        """生成权限错误提示消息"""
        if sys.platform == 'win32':
            return f"{device}权限不足，请尝试以管理员身份运行程序"
        else:
            return f"{device}权限不足，请尝试:\n1. 使用 sudo 运行程序\n2. 将用户添加到 video 组: sudo usermod -aG video $USER\n3. 重新登录后生效"
    
    def try_reconnect(self) -> Tuple[bool, str]:
        """尝试重连摄像头"""
        if not self.last_camera_info:
            return False, "无重连信息"
        
        if self.reconnect_attempts >= self.MAX_RECONNECT_ATTEMPTS:
            return False, f"已达最大重连次数 ({self.MAX_RECONNECT_ATTEMPTS})"
        
        self.reconnect_attempts += 1
        info = self.last_camera_info
        print(f"[摄像头] 第 {self.reconnect_attempts} 次重连尝试: {info.get('name', '')}")
        
        self.release()
        time.sleep(self.RECONNECT_INTERVAL)
        
        if info.get('type') == 'network':
            success, msg = self._open_network_camera(info)
        else:
            success, msg = self._open_usb_camera(info)
        
        if success:
            self.reconnect_attempts = 0
        return success, msg
    
    def release(self):
        """释放摄像头资源"""
        self.is_running = False
        self._stop_scan()
        if self.read_thread and self.read_thread.is_alive():
            self.read_thread.join(timeout=1.0)
        if self.cap:
            self.cap.release()
            self.cap = None


class SerialManager:
    """串口设备管理类"""
    
    MAX_SCAN_PORTS = 20
    RECONNECT_INTERVAL = 3.0
    MAX_RECONNECT_ATTEMPTS = 5
    
    def __init__(self, baudrate: int = 115200, timeout: float = 0.1):
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_conn: Optional[serial.Serial] = None
        self.port_name: str = ""
        self.is_running: bool = False
        self.buffer = SerialBuffer()
        self.data_lock = threading.Lock()
        self.latest_data: Tuple[int, int, int, int] = (0, 0, 0, 0)
        self.data_timestamp: float = 0
        self.read_thread: Optional[threading.Thread] = None
        self.frame_count: int = 0
        self.error_count: int = 0
        
        self.scan_thread: Optional[threading.Thread] = None
        self.scan_cancelled: bool = False
        self.scan_complete: bool = False
        self.available_ports: List[str] = []
        
        self.reconnect_attempts: int = 0
        self.reconnect_info: Optional[dict] = None
    
    def _try_open_port(self, port_name: str) -> Optional[serial.Serial]:
        """尝试打开指定串口"""
        try:
            conn = serial.Serial(
                port=port_name,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            return conn
        except PermissionError:
            return None
        except serial.SerialException:
            return None
        except Exception:
            return None
    
    def scan_ports(self) -> List[str]:
        """同步扫描可用串口（仅列出，不打开）"""
        ports = list(serial.tools.list_ports.comports())
        available = []
        for port in ports:
            if self._try_open_port(port.device) is not None:
                available.append(port.device)
        return available
    
    def _start_async_scan(self, on_progress: Callable[[int, int], None] = None,
                          on_port_found: Callable[[str], None] = None,
                          on_complete: Callable[[List[str]], None] = None):
        """启动异步串口扫描"""
        self.scan_cancelled = False
        self.scan_complete = False
        self.available_ports = []
        
        def scan_worker():
            ports = serial.tools.list_ports.comports()
            total = len(ports) if ports else 0
            found = []
            
            for i, port in enumerate(ports):
                if self.scan_cancelled:
                    break
                
                if on_progress:
                    on_progress(i + 1, total)
                
                conn = self._try_open_port(port.device)
                if conn is not None:
                    conn.close()
                    found.append(port.device)
                    self.available_ports = found.copy()
                    if on_port_found:
                        on_port_found(port.device)
                    print(f"[串口] 发现: {port.device}")
            
            self.scan_complete = True
            if on_complete:
                on_complete(found)
        
        self.scan_thread = threading.Thread(target=scan_worker, daemon=True)
        self.scan_thread.start()
    
    def _stop_scan(self):
        """停止扫描"""
        self.scan_cancelled = True
        if self.scan_thread and self.scan_thread.is_alive():
            self.scan_thread.join(timeout=2.0)
    
    def open_port(self, port_name: str) -> Tuple[bool, str]:
        """打开指定串口并启动读取线程"""
        try:
            self.serial_conn = serial.Serial(
                port=port_name,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            self.port_name = port_name
            self.reconnect_info = {"port": port_name}
            self.reconnect_attempts = 0
            
            self.is_running = True
            self.buffer = SerialBuffer()
            self.read_thread = threading.Thread(target=self._read_data, daemon=True)
            self.read_thread.start()
            
            return True, f"成功打开串口 {port_name}"
        except PermissionError:
            return False, self._get_permission_error_msg("串口")
        except serial.SerialException as e:
            return False, f"打开串口 {port_name} 失败: {e}"
        except Exception as e:
            return False, f"打开串口异常: {e}"
    
    def scan_and_open(self) -> Tuple[bool, str]:
        """扫描并打开串口（同步版本，保留兼容）"""
        ports = list(serial.tools.list_ports.comports())
        
        if not ports:
            return False, "未找到可用串口设备，请检查USB连接"
        
        for port in ports:
            conn = self._try_open_port(port.device)
            if conn is not None:
                self.serial_conn = conn
                self.port_name = port.device
                self.reconnect_info = {"port": port.device}
                self.reconnect_attempts = 0
                
                self.is_running = True
                self.read_thread = threading.Thread(target=self._read_data, daemon=True)
                self.read_thread.start()
                
                return True, f"成功打开串口 {port.device}"
        
        return False, "无法打开任何串口设备"
    
    def _read_data(self):
        """独立线程中读取串口数据"""
        while self.is_running and self.serial_conn:
            try:
                if self.serial_conn.in_waiting > 0:
                    data = self.serial_conn.read(self.serial_conn.in_waiting)
                    self.buffer.append(data)
                    self.reconnect_attempts = 0
                    
                    while True:
                        frame = self.buffer.find_frame()
                        if frame is None:
                            break
                        
                        try:
                            sensor_data = LaserDataParser.parse(frame)
                            with self.data_lock:
                                self.latest_data = sensor_data
                                self.data_timestamp = time.time()
                                self.frame_count += 1
                        except Exception as e:
                            self.error_count += 1
                else:
                    time.sleep(0.001)
                    
            except (serial.SerialException, OSError):
                print("[串口] 连接断开")
                self.is_running = False
                break
            except Exception as e:
                time.sleep(0.01)
    
    def try_reconnect(self) -> Tuple[bool, str]:
        """尝试重连串口"""
        if not self.reconnect_info:
            return False, "无重连信息"
        
        if self.reconnect_attempts >= self.MAX_RECONNECT_ATTEMPTS:
            return False, f"已达最大重连次数 ({self.MAX_RECONNECT_ATTEMPTS})"
        
        self.reconnect_attempts += 1
        port = self.reconnect_info["port"]
        print(f"[串口] 第 {self.reconnect_attempts} 次重连尝试: {port}")
        
        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.close()
            except Exception:
                pass
            self.serial_conn = None
        
        time.sleep(self.RECONNECT_INTERVAL)
        
        success, msg = self.open_port(port)
        if success:
            self.reconnect_attempts = 0
        return success, msg
    
    def get_data(self) -> Tuple[Tuple[int, int, int, int], float]:
        """获取最新数据和时间戳"""
        with self.data_lock:
            return self.latest_data, self.data_timestamp
    
    def _get_permission_error_msg(self, device: str) -> str:
        """生成权限错误提示消息"""
        if sys.platform == 'win32':
            return f"{device}权限不足，请尝试以管理员身份运行程序"
        else:
            return f"{device}权限不足，请尝试:\n1. 使用 sudo 运行程序\n2. 将用户添加到 dialout 组: sudo usermod -aG dialout $USER\n3. 重新登录后生效"
    
    def release(self):
        """释放串口资源"""
        self.is_running = False
        self._stop_scan()
        if self.read_thread and self.read_thread.is_alive():
            self.read_thread.join(timeout=1.0)
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            self.serial_conn = None


class FusionDisplayApp:
    """融合显示主应用程序"""
    
    def __init__(self, root: tk.Tk):
        print("[信息] FusionDisplayApp 初始化中...")
        self.root = root
        self.root.title("激光测距视频融合系统")
        self.root.geometry("800x600")
        self.root.resizable(True, True)
        
        print("[信息] 创建设备管理器...")
        self.camera_manager = CameraManager()
        self.serial_manager = SerialManager()
        
        self.is_running = False
        self.update_thread: Optional[threading.Thread] = None
        self.camera_ready = False
        self.serial_ready = False
        
        print("[信息] 设置UI界面...")
        self._setup_ui()
        print("[信息] 启动系统...")
        self._start_system()
    
    def _setup_ui(self):
        """设置UI界面"""
        main_frame = ttk.Frame(self.root, padding="5")
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        main_frame.grid_rowconfigure(0, weight=4, minsize=300)
        main_frame.grid_rowconfigure(1, weight=0, minsize=80)
        main_frame.grid_rowconfigure(2, weight=0, minsize=40)
        main_frame.grid_columnconfigure(0, weight=1)
        
        video_frame = ttk.LabelFrame(main_frame, text="视频显示", padding="5")
        video_frame.grid(row=0, column=0, sticky="nsew", pady=(0, 5))
        
        self.video_label = ttk.Label(video_frame, anchor="center")
        self.video_label.pack(fill=tk.BOTH, expand=True)
        
        data_frame = ttk.LabelFrame(main_frame, text="激光测距数据", padding="5")
        data_frame.grid(row=1, column=0, sticky="ew", pady=(0, 5))
        
        sensor_frame = ttk.Frame(data_frame)
        sensor_frame.pack(fill=tk.X)
        
        self.sensor_labels = []
        sensor_names = ["传感器1", "传感器2", "传感器3", "传感器4"]
        for i, name in enumerate(sensor_names):
            frame = ttk.Frame(sensor_frame)
            frame.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=5)
            
            ttk.Label(frame, text=name, font=("Arial", 10, "bold")).pack()
            label = ttk.Label(frame, text="0 mm", font=("Arial", 14), foreground="blue")
            label.pack()
            self.sensor_labels.append(label)
        
        status_frame = ttk.LabelFrame(main_frame, text="系统状态", padding="5")
        status_frame.grid(row=2, column=0, sticky="ew")
        
        status_row = ttk.Frame(status_frame)
        status_row.pack(fill=tk.X)
        
        self.status_label = ttk.Label(status_row, text="正在初始化...", font=("Arial", 10))
        self.status_label.pack(side=tk.LEFT, fill=tk.X, expand=True)
        
        self.serial_retry_btn = ttk.Button(status_row, text="重连串口", width=10,
                                            command=self._retry_serial, state=tk.DISABLED)
        self.serial_retry_btn.pack(side=tk.RIGHT, padx=(5, 0))
        
        self.switch_camera_btn = ttk.Button(status_row, text="切换摄像头", width=10,
                                            command=self._switch_camera, state=tk.DISABLED)
        self.switch_camera_btn.pack(side=tk.RIGHT, padx=(5, 0))
        
        self.root.protocol("WM_DELETE_WINDOW", self._on_closing)
    
    def _start_system(self):
        """启动系统（异步模式）"""
        print("[信息] 启动摄像头选择流程...")
        
        def on_camera_selected(camera_info: Optional[dict]):
            if camera_info is None:
                print("[信息] 用户取消了摄像头选择")
                self.status_label.config(text="用户取消了摄像头选择")
                return
            
            print(f"[信息] 用户选择: {camera_info.get('name', 'Unknown')}")
            
            if camera_info.get('type') == 'network':
                success, msg = self.camera_manager._open_network_camera(camera_info)
            else:
                success, msg = self.camera_manager._open_usb_camera(camera_info)
            
            if not success:
                messagebox.showerror("摄像头错误", msg)
                self.status_label.config(text=f"摄像头错误: {msg}")
                return
            
            self.camera_ready = True
            cam_info = f"网络: {self.camera_manager.network_url}" if self.camera_manager.camera_type == 'network' else f"USB索引: {self.camera_manager.camera_index}"
            self.status_label.config(text=f"摄像头: {cam_info} | 串口: 扫描中...")
            self.switch_camera_btn.config(state=tk.NORMAL)
            
            print("[信息] 摄像头就绪，启动串口异步扫描")
            self._start_serial_scan()
            
            print("[信息] 启动视频更新循环")
            self.is_running = True
            self.update_thread = threading.Thread(target=self._update_loop, daemon=True)
            self.update_thread.start()
        
        self.camera_manager._show_camera_selection_dialog_async(self.root, on_camera_selected)
    
    def _start_serial_scan(self):
        """异步扫描串口，不阻塞主线程"""
        def on_port_found(port_name: str):
            if self.root.winfo_exists():
                cam_info = f"网络: {self.camera_manager.network_url}" if self.camera_manager.camera_type == 'network' else f"USB索引: {self.camera_manager.camera_index}"
                self.root.after(0, lambda: self.status_label.config(
                    text=f"摄像头: {cam_info} | 串口: 发现 {port_name}，连接中..."))
        
        def on_complete(found_ports: List[str]):
            if not self.root.winfo_exists():
                return
            
            if found_ports:
                def do_open():
                    success, msg = self.serial_manager.open_port(found_ports[0])
                    cam_info = f"网络: {self.camera_manager.network_url}" if self.camera_manager.camera_type == 'network' else f"USB索引: {self.camera_manager.camera_index}"
                    if success:
                        self.serial_ready = True
                        self.root.after(0, lambda: self.status_label.config(
                            text=f"摄像头: {cam_info} | 串口: {self.serial_manager.port_name}"))
                        self.root.after(0, lambda: self.serial_retry_btn.config(state=tk.DISABLED))
                    else:
                        self.root.after(0, lambda: self.status_label.config(
                            text=f"摄像头: {cam_info} | 串口: {msg}"))
                        self.root.after(0, lambda: self.serial_retry_btn.config(state=tk.NORMAL))
                self.root.after(0, do_open)
            else:
                cam_info = f"网络: {self.camera_manager.network_url}" if self.camera_manager.camera_type == 'network' else f"USB索引: {self.camera_manager.camera_index}"
                self.root.after(0, lambda: self.status_label.config(
                    text=f"摄像头: {cam_info} | 串口: 未找到设备"))
                self.root.after(0, lambda: self.serial_retry_btn.config(state=tk.NORMAL))
        
        self.serial_manager._start_async_scan(
            on_port_found=on_port_found,
            on_complete=on_complete
        )
    
    def _retry_serial(self):
        """重试串口连接"""
        self.serial_retry_btn.config(state=tk.DISABLED, text="扫描中...")
        self._start_serial_scan()
    
    def _switch_camera(self):
        """切换摄像头"""
        self.switch_camera_btn.config(state=tk.DISABLED)
        
        old_camera_info = self.camera_manager.last_camera_info
        self.camera_manager.release()
        self.camera_ready = False
        
        self.status_label.config(text="正在选择新摄像头...")
        
        def on_camera_selected(camera_info: Optional[dict]):
            if camera_info is None:
                if old_camera_info:
                    print("[信息] 用户取消，恢复原摄像头")
                    if old_camera_info.get('type') == 'network':
                        success, msg = self.camera_manager._open_network_camera(old_camera_info)
                    else:
                        success, msg = self.camera_manager._open_usb_camera(old_camera_info)
                    if success:
                        self.camera_ready = True
                        cam_info = f"网络: {self.camera_manager.network_url}" if self.camera_manager.camera_type == 'network' else f"USB索引: {self.camera_manager.camera_index}"
                        self.status_label.config(text=f"摄像头: {cam_info} | 串口: {'就绪' if self.serial_ready else '未连接'}")
                    else:
                        self.status_label.config(text=f"恢复摄像头失败: {msg}")
                self.switch_camera_btn.config(state=tk.NORMAL)
                return
            
            print(f"[信息] 切换到: {camera_info.get('name', 'Unknown')}")
            
            if camera_info.get('type') == 'network':
                success, msg = self.camera_manager._open_network_camera(camera_info)
            else:
                success, msg = self.camera_manager._open_usb_camera(camera_info)
            
            if not success:
                messagebox.showerror("摄像头错误", msg)
                if old_camera_info:
                    if old_camera_info.get('type') == 'network':
                        self.camera_manager._open_network_camera(old_camera_info)
                    else:
                        self.camera_manager._open_usb_camera(old_camera_info)
                    self.camera_ready = True
                self.switch_camera_btn.config(state=tk.NORMAL)
                return
            
            self.camera_ready = True
            cam_info = f"网络: {self.camera_manager.network_url}" if self.camera_manager.camera_type == 'network' else f"USB索引: {self.camera_manager.camera_index}"
            self.status_label.config(text=f"摄像头: {cam_info} | 串口: {'就绪' if self.serial_ready else '未连接'}")
            self.switch_camera_btn.config(state=tk.NORMAL)
        
        self.camera_manager._show_camera_selection_dialog_async(self.root, on_camera_selected)
    
    def _update_loop(self):
        """主更新循环"""
        while self.is_running:
            try:
                frame = self.camera_manager.get_frame()
                if frame is not None:
                    sensor_data, timestamp = self.serial_manager.get_data()
                    
                    if self.is_running:
                        self.root.after(0, self._update_video_display, frame, sensor_data, timestamp)
                        self.root.after(0, self._update_sensor_labels, sensor_data)
                
                if self.camera_ready and not self.camera_manager.is_running:
                    self.camera_ready = False
                    print("[摄像头] 检测到断线，尝试重连...")
                    self.root.after(0, self._handle_camera_disconnect)
                
                if self.serial_ready and not self.serial_manager.is_running:
                    self.serial_ready = False
                    print("[串口] 检测到断线，尝试重连...")
                    self.root.after(0, self._handle_serial_disconnect)
                
                time.sleep(0.033)
                
            except Exception as e:
                print(f"[错误] 更新循环异常：{e}")
                import traceback
                traceback.print_exc()
                time.sleep(0.1)
    
    def _handle_camera_disconnect(self):
        """处理摄像头断线"""
        self.switch_camera_btn.config(state=tk.DISABLED)
        self.status_label.config(text="摄像头断线，正在重连...")
        
        def do_reconnect():
            success, msg = self.camera_manager.try_reconnect()
            cam_info = f"网络: {self.camera_manager.network_url}" if self.camera_manager.camera_type == 'network' else f"USB索引: {self.camera_manager.camera_index}"
            if success:
                self.camera_ready = True
                self.root.after(0, lambda: self.status_label.config(
                    text=f"摄像头: {cam_info} (已重连) | 串口: {'就绪' if self.serial_ready else '未连接'}"))
                self.root.after(0, lambda: self.switch_camera_btn.config(state=tk.NORMAL))
            else:
                self.root.after(0, lambda: self.status_label.config(text=f"摄像头重连失败: {msg}"))
                self.root.after(0, lambda: self.switch_camera_btn.config(state=tk.NORMAL))
        
        threading.Thread(target=do_reconnect, daemon=True).start()
    
    def _handle_serial_disconnect(self):
        """处理串口断线"""
        cam_info = f"网络: {self.camera_manager.network_url}" if self.camera_manager.camera_type == 'network' else f"USB索引: {self.camera_manager.camera_index}"
        self.status_label.config(text=f"摄像头: {cam_info} | 串口: 断线，正在重连...")
        self.serial_retry_btn.config(state=tk.NORMAL)
        
        def do_reconnect():
            success, msg = self.serial_manager.try_reconnect()
            if success:
                self.serial_ready = True
                self.root.after(0, lambda: self.status_label.config(
                    text=f"摄像头: {cam_info} | 串口: {self.serial_manager.port_name} (已重连)"))
                self.root.after(0, lambda: self.serial_retry_btn.config(state=tk.DISABLED))
            else:
                self.root.after(0, lambda: self.status_label.config(
                    text=f"摄像头: {cam_info} | 串口: {msg}"))
                self.root.after(0, lambda: self.serial_retry_btn.config(state=tk.NORMAL))
        
        threading.Thread(target=do_reconnect, daemon=True).start()
    
    def _draw_sensor_data(self, frame, sensor_data: Tuple[int, int, int, int], timestamp: float) -> any:
        """在视频帧上绘制传感器数据"""
        display_frame = frame.copy()
        
        sensor_names = ["S1", "S2", "S3", "S4"]
        y_offset = 30
        
        for i, (name, value) in enumerate(zip(sensor_names, sensor_data)):
            text = f"{name}: {value} mm"
            
            if value > 0:
                color = (0, 255, 0)
            else:
                color = (128, 128, 128)
            
            cv2.putText(display_frame, text, (10, y_offset + i * 25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        current_time = time.strftime("%H:%M:%S", time.localtime())
        cv2.putText(display_frame, current_time, (display_frame.shape[1] - 100, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        data_age = time.time() - timestamp
        if data_age < 1.0:
            status_color = (0, 255, 0)
            status_text = "DATA OK"
        elif data_age < 3.0:
            status_color = (0, 255, 255)
            status_text = "DATA OLD"
        else:
            status_color = (0, 0, 255)
            status_text = "NO DATA"
        
        cv2.putText(display_frame, status_text, (display_frame.shape[1] - 100, 50),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, status_color, 1)
        
        return display_frame
    
    def _update_video_display(self, frame, sensor_data: Tuple[int, int, int, int], timestamp: float):
        """更新视频显示（在主线程中调用，同时完成绘制）"""
        try:
            display_frame = self._draw_sensor_data(frame, sensor_data, timestamp)
            
            label_width = self.video_label.winfo_width()
            label_height = self.video_label.winfo_height()
            
            if label_width <= 1 or label_height <= 1:
                label_width, label_height = 640, 480
            
            frame_rgb = cv2.cvtColor(display_frame, cv2.COLOR_BGR2RGB)
            if (frame_rgb.shape[1], frame_rgb.shape[0]) != (label_width, label_height):
                frame_rgb = cv2.resize(frame_rgb, (label_width, label_height), 
                                       interpolation=cv2.INTER_LINEAR)
            
            img = Image.fromarray(frame_rgb)
            imgtk = ImageTk.PhotoImage(image=img)
            
            self.video_label.imgtk = imgtk
            self.video_label.config(image=imgtk)
            
        except Exception as e:
            print(f"[错误] 视频显示更新失败：{e}")
    
    def _update_sensor_labels(self, sensor_data: Tuple[int, int, int, int]):
        """更新传感器数据标签"""
        for i, value in enumerate(sensor_data):
            self.sensor_labels[i].config(text=f"{value} mm")
    
    def _on_closing(self):
        """关闭窗口时的清理操作"""
        self.is_running = False
        
        if self.update_thread and self.update_thread.is_alive():
            self.update_thread.join(timeout=1.0)
        
        self.camera_manager.release()
        self.serial_manager.release()
        
        self.root.destroy()


def check_gui_environment() -> Tuple[bool, str]:
    """检查GUI环境是否可用"""
    if sys.platform == 'win32':
        return True, ""
    
    display = os.environ.get('DISPLAY', '')
    wayland = os.environ.get('WAYLAND_DISPLAY', '')
    session_type = os.environ.get('XDG_SESSION_TYPE', '')
    
    if not display and not wayland:
        hints = [
            "未检测到图形显示环境",
            "",
            "解决方案:",
            "1. 如果使用SSH远程连接，请使用 -X 参数: ssh -X user@host",
            "2. 设置DISPLAY环境变量: export DISPLAY=:0",
            "3. 确保X11服务正在运行",
            "4. 或在本地终端运行程序",
            "",
        ]
        if session_type == 'wayland':
            hints.append("检测到Wayland会话，tkinter可能需要XWayland支持")
            hints.append("尝试: export GDK_BACKEND=x11 再运行程序")
        return False, "\n".join(hints)
    
    if session_type == 'wayland' and not display:
        os.environ['GDK_BACKEND'] = 'x11'
        try:
            test_root = tk.Tk()
            test_root.destroy()
            return True, ""
        except tk.TclError as e:
            return False, (
                f"Wayland会话下无法创建GUI窗口: {e}\n"
                "解决方案:\n"
                "1. 安装XWayland: sudo dnf install xorg-x11-server-Xwayland\n"
                "2. 或切换到X11会话: 在登录界面选择X11"
            )
    
    try:
        test_root = tk.Tk()
        test_root.destroy()
        return True, ""
    except tk.TclError as e:
        return False, f"无法创建 GUI 窗口：{e}"


def main():
    """主函数"""
    print("=" * 50)
    print("激光测距视频融合系统")
    print("=" * 50)
    print(f"Python版本: {sys.version}")
    print(f"平台: {sys.platform}")
    print(f"工作目录: {os.getcwd()}")
    print("-" * 50)
    
    if sys.platform != 'win32':
        session_type = os.environ.get('XDG_SESSION_TYPE', '')
        wayland = os.environ.get('WAYLAND_DISPLAY', '')
        display = os.environ.get('DISPLAY', '')
        print(f"[信息] 显示会话: {session_type or '未知'}")
        print(f"[信息] DISPLAY={display or '未设置'}")
        print(f"[信息] WAYLAND_DISPLAY={wayland or '未设置'}")
        
        if session_type == 'wayland' and not display:
            print("[信息] 检测到Wayland会话，自动设置GDK_BACKEND=x11以兼容tkinter")
            os.environ['GDK_BACKEND'] = 'x11'
    
    gui_ok, gui_msg = check_gui_environment()
    if not gui_ok:
        print(f"[错误] {gui_msg}")
        sys.exit(1)
    
    print("[信息] GUI环境检查通过")
    
    try:
        print("[信息] 初始化Tkinter...")
        root = tk.Tk()
        print("[信息] Tkinter初始化成功")
    except tk.TclError as e:
        print(f"[错误] Tkinter初始化失败: {e}")
        session_type = os.environ.get('XDG_SESSION_TYPE', '')
        if sys.platform != 'win32':
            print("Linux系统安装tkinter:")
            if 'openEuler' in open('/etc/os-release', 'r').read() if os.path.exists('/etc/os-release') else False:
                print("  sudo dnf install python3-tkinter")
            else:
                print("  sudo apt install python3-tk        # Debian/Ubuntu")
                print("  sudo dnf install python3-tkinter    # openEuler/Fedora")
                print("  sudo pacman -S tk                   # Arch Linux")
            if session_type == 'wayland':
                print("\n当前为Wayland会话，尝试设置:")
                print("  export GDK_BACKEND=x11")
                print("  或安装XWayland: sudo dnf install xorg-x11-server-Xwayland")
        sys.exit(1)
    except Exception as e:
        print(f"[错误] 初始化失败: {e}")
        sys.exit(1)
    
    try:
        print("[信息] 启动应用程序...")
        app = FusionDisplayApp(root)
        print("[信息] 进入主循环...")
        root.mainloop()
    except KeyboardInterrupt:
        print("\n[信息] 用户中断")
    except Exception as e:
        print(f"[错误] 程序运行错误: {e}")
        import traceback
        traceback.print_exc()
        try:
            messagebox.showerror("错误", f"程序运行错误: {str(e)}")
        except:
            pass
        sys.exit(1)


if __name__ == "__main__":
    main()
