#!/usr/bin/env bash
set -euo pipefail

workspace_root="${REALMAN_WS_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
target="$workspace_root/upstream/realman_ros2/rm_description"
link="$workspace_root/src/rm_description"
expected="$target"

if [[ ! -d "$target" ]]; then
  echo "FAIL missing $target; run scripts/import_upstream.sh first" >&2
  exit 1
fi
if [[ -L "$link" ]]; then
  actual="$(readlink -f "$link")"
  if [[ "$actual" == "$expected" ]]; then
    echo "PASS $link -> $actual"
    exit 0
  fi
  echo "FAIL $link points to $actual; refusing to replace it" >&2
  exit 1
fi
if [[ -e "$link" ]]; then
  echo "FAIL $link already exists and is not a symbolic link; refusing to replace it" >&2
  exit 1
fi

ln -s ../upstream/realman_ros2/rm_description "$link"
echo "PASS created $link -> $(readlink -f "$link")"
