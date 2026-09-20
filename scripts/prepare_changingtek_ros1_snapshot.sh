#!/usr/bin/env bash
set -euo pipefail

workspace_root="${REALMAN_WS_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
target="$workspace_root/upstream/changingtek_ros1"
expected_sha=8535b4f70ea8292977f3666964405949de3db25c5e2835702436544f0dd70b00
url=https://gitee.com/Ice_Panda/changingtek/repository/archive/master.zip

if [[ -e "$target" ]]; then
  echo "Snapshot already exists: $target" >&2
  exit 1
fi

task_temp="$(mktemp -d)"
trap 'rm -rf -- "$task_temp"' EXIT
curl -fL --retry 2 --max-time 120 -o "$task_temp/master.zip" "$url"
printf '%s  %s\n' "$expected_sha" "$task_temp/master.zip" | sha256sum -c -
unzip -q "$task_temp/master.zip" -d "$task_temp"
test -d "$task_temp/changingtek-master"
mkdir -p "$workspace_root/upstream"
mv "$task_temp/changingtek-master" "$target"
echo "Imported checksum-locked ROS1 snapshot to $target"
