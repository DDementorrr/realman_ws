#!/usr/bin/env bash
# Register an existing CJK font for this user; no sudo, download or font bundling.
set -euo pipefail

for tool in fc-list fc-cache; do
  command -v "$tool" >/dev/null || { echo "缺少 fontconfig：请安装 fontconfig fonts-noto-cjk。" >&2; exit 1; }
done

if [[ -n "$(fc-list :lang=zh family)" ]]; then
  echo '中文字体已就绪。'
  exit 0
fi

# WSL does not necessarily register the fonts already available in Windows.
for font in /mnt/c/Windows/Fonts/msyh.ttc /mnt/c/Windows/Fonts/simsun.ttc; do
  [[ -r "$font" ]] || continue
  font_dir="${XDG_DATA_HOME:-$HOME/.local/share}/fonts/rm65b_operator_panel"
  mkdir -p "$font_dir"
  font_link="$font_dir/$(basename "$font")"
  if [[ -e "$font_link" || -L "$font_link" ]]; then
    [[ "$(readlink "$font_link")" == "$font" ]] || { echo "保留已有字体文件：$font_link" >&2; exit 1; }
  else
    ln -s "$font" "$font_link"
  fi
  fc-cache -f "$font_dir"
  if [[ -n "$(fc-list :lang=zh family)" ]]; then
    echo '已注册本机 Windows 中文字体；请重新启动 RViz。'
    exit 0
  fi
done

echo '未找到中文字体。Ubuntu 请先执行：sudo apt install fonts-noto-cjk，然后重新启动 RViz。' >&2
exit 1
