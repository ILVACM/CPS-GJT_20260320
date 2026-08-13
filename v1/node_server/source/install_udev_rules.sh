#!/bin/sh

# Copyright (c) Orbbec Inc. All Rights Reserved.
# Licensed under the MIT License.
#
# node_server 内嵌版：从本脚本所在目录安装 udev 规则，
# 不再依赖外部 pyorbbecsdk 子目录，便于独立迁移部署。
# 来源：OrbbecSDK-Linux-ARM64-python39/source/install_udev_rules.sh（方案 F2/F6）。

# Check if user is root/running with sudo
if [ `whoami` != root ]; then
    echo "Please run this script with sudo"
    exit 1
fi

# 获取本脚本所在目录的绝对路径
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
RULES_FILE="${SCRIPT_DIR}/99-obsensor-libusb.rules"
TARGET="/etc/udev/rules.d/99-obsensor-libusb.rules"

if [ ! -f "${RULES_FILE}" ]; then
    echo "Error: ${RULES_FILE} not found"
    exit 1
fi

# 安装 udev 规则文件
cp "${RULES_FILE}" "${TARGET}"
echo "udev rules installed to ${TARGET}"

# 重新加载 udev 规则并触发设备重枚举
udevadm control --reload
udevadm trigger
echo "udev rules reloaded and devices triggered"
