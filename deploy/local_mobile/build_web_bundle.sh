#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repository_root="$(cd -- "${script_dir}/../.." && pwd)"
web_root="${repository_root}/web_control"
output_file="${1:-${repository_root}/gen0-mobile-web.zip}"
if [[ "${output_file}" != /* ]]; then
  output_file="$(pwd)/${output_file}"
fi
temporary_dir="$(mktemp -d)"
trap 'rm -rf -- "${temporary_dir}"' EXIT

if ! command -v zip >/dev/null 2>&1; then
  echo "缺少 zip，请安装: sudo apt install zip" >&2
  exit 1
fi

mkdir -p "${temporary_dir}/web_control"
cp "${web_root}/index.html" \
   "${web_root}/app.js" \
   "${web_root}/style.css" \
   "${web_root}/manifest.webmanifest" \
   "${web_root}/service-worker.js" \
   "${web_root}/runtime-config.js" \
   "${temporary_dir}/web_control/"
cp -R "${web_root}/vendor" "${web_root}/static_map" "${temporary_dir}/web_control/"

(cd "${temporary_dir}/web_control" && zip -qr "${output_file}" .)
echo "已生成离线网页包: ${output_file}"
echo "在 WebToApp 中选择 Local HTML / Offline Website，并将 index.html 设为入口。"
