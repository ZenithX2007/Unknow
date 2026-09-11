#!/usr/bin/env bash
set -euo pipefail

rosbridge_port="${1:-9090}"
web_port="${2:-8000}"
local_ip="$(hostname -I 2>/dev/null | awk '{print $1}')"

if [[ -z "${local_ip}" ]]; then
  echo "无法自动取得局域网 IP，请执行: ip -4 addr" >&2
  exit 1
fi

echo "============================================================"
echo "GEN0 手机连接信息"
echo "电脑局域网 IP : ${local_ip}"
echo "APP 中填写 IP : ${local_ip}"
echo "rosbridge      : ws://${local_ip}:${rosbridge_port}"
echo "浏览器测试网页: http://${local_ip}:${web_port}/"
echo "============================================================"
echo "请确保手机和电脑连接同一个 Wi-Fi。"

if command -v ss >/dev/null 2>&1; then
  if ss -lnt | grep -q ":${rosbridge_port} "; then
    echo "[OK] rosbridge 端口 ${rosbridge_port} 正在监听。"
  else
    echo "[等待] 端口 ${rosbridge_port} 尚未监听，请先启动完整系统。"
  fi
fi

if command -v qrencode >/dev/null 2>&1; then
  echo "可扫描下面的二维码，在手机浏览器中测试网页："
  qrencode -t ANSIUTF8 "http://${local_ip}:${web_port}/"
else
  echo "可选安装 qrencode 后显示测试网页二维码: sudo apt install qrencode"
fi
