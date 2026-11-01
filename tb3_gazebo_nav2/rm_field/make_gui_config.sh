#!/usr/bin/env bash
# 生成场地专用 GUI 配置：把 3D 视图初始相机改成指定位姿
# 用法: make_gui_config.sh ["x y z roll pitch yaw"] [输出配置文件]
#   默认位姿 = 低机位 3/4 斜视（能看见 0.503 m 侧壁）
#   默认输出 = config/gui_field.config
set +u
ROOT=/mnt/e/RM/tb3_gazebo_nav2/rm_field
POSE="${1:--8.5 7.5 2.6 0 0.226 -0.723}"
CFG="${2:-$ROOT/config/gui_field.config}"
mkdir -p "$ROOT/config"
if [ -f "$CFG" ]; then SRC="$CFG"; elif [ -f "$ROOT/config/gui_field.config" ]; then SRC="$ROOT/config/gui_field.config"; else SRC="$HOME/.gz/sim/8/gui.config"; fi
if [ ! -f "$SRC" ]; then echo "找不到 gui.config"; exit 1; fi
# 用 python 精确替换 <camera_pose> 行
python3 - "$SRC" "$CFG" "$POSE" <<'PY'
import re
import sys

src, dst, pose = sys.argv[1], sys.argv[2], sys.argv[3]
with open(src, "r", encoding="utf-8") as f:
    txt = f.read()
new, n = re.subn(r"<camera_pose>[^<]*</camera_pose>",
                 f"<camera_pose>{pose}</camera_pose>", txt)
if n == 0:
    # 没有就插在 <plugin filename="MinimalScene" 之后
    new = re.sub(r'(<plugin filename="MinimalScene"[^>]*>)',
                 r"\1\n  <camera_pose>%s</camera_pose>" % pose, txt)
with open(dst, "w", encoding="utf-8") as f:
    f.write(new)
print(f"写入 {dst}\ncamera_pose={pose}  (替换 {n} 处)")
PY
grep -n 'camera_pose' "$CFG"