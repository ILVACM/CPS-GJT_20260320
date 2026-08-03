"""系统服务管理脚本调用入口。

对齐 Design-server.md §3.8.4 + §9。
封装 ``scripts/service-manager.sh`` 的程序内部调用，供 GUI 设置页与 CLI 命令统一调用。
所有方法均需 root/sudo 权限（脚本内部已校验 root）。
"""
import subprocess
import sys
from typing import Tuple


class ServiceManager:
    """系统服务管理器：封装 scripts/service-manager.sh 的程序内部调用。

    所有方法均需 root/sudo 权限（见 §9.3）。
    Windows 平台下不支持，直接返回失败。
    """

    # 脚本相对路径（相对当前工作目录）
    SCRIPT_PATH: str = "scripts/service-manager.sh"

    def install(self) -> Tuple[bool, str]:
        """注册为系统服务并设置开机自启。

        :return: (是否成功, 脚本 stdout 输出或错误说明)
        """
        if sys.platform != "linux" and not sys.platform.startswith("linux"):
            return False, "仅 Linux 支持"
        try:
            result = subprocess.run(
                ["sudo", "bash", self.SCRIPT_PATH, "install"],
                capture_output=True,
                text=True,
                timeout=30,
            )
            return result.returncode == 0, result.stdout
        except subprocess.TimeoutExpired:
            return False, "安装超时（>30s）"
        except FileNotFoundError as e:
            return False, f"脚本或解释器未找到: {e}"
        except Exception as e:
            return False, f"调用异常: {e}"

    def uninstall(self) -> Tuple[bool, str]:
        """卸载系统服务并取消开机自启。

        :return: (是否成功, 脚本 stdout 输出或错误说明)
        """
        if sys.platform != "linux" and not sys.platform.startswith("linux"):
            return False, "仅 Linux 支持"
        try:
            result = subprocess.run(
                ["sudo", "bash", self.SCRIPT_PATH, "uninstall"],
                capture_output=True,
                text=True,
                timeout=30,
            )
            return result.returncode == 0, result.stdout
        except subprocess.TimeoutExpired:
            return False, "卸载超时（>30s）"
        except FileNotFoundError as e:
            return False, f"脚本或解释器未找到: {e}"
        except Exception as e:
            return False, f"调用异常: {e}"
