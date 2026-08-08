#!/bin/sh

# Copyright (c) Orbbec Inc. All Rights Reserved.
# Licensed under the MIT License.
#
# Demo 3 内嵌版：从本脚本所在目录安装 udev 规则，
# 不再依赖 pyorbbecsdk-v2.0.18 子目录，便于独立迁移部署。

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
