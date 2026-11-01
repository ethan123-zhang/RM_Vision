#!/usr/bin/env bash
# 列出 WSLg(:0) 上的 X 窗口，并按窗口抓图（证明 Gazebo/RViz 真的在出图）
WS=/mnt/e/RM/tb3_gazebo_nav2
export DISPLAY="${DISPLAY:-:0}"
mkdir -p "$WS/shots"
echo "=== X 窗口清单 ==="
for w in $(xdotool search --onlyvisible --name '.' 2>/dev/null); do
  name=$(xdotool getwindowname "$w" 2>/dev/null)
  geo=$(xdotool getwindowgeometry --shell "$w" 2>/dev/null | tr '\n' ' ')
  printf '  id=%-10s name=%-28s %s\n' "$w" "${name:-<无名>}" "$geo"
done
echo
echo "=== 按名字抓图 ==="
shot() {
  local id="$1" tag="$2"
  [ -n "$id" ] || return
  import -display :0 -window "$id" "$WS/shots/$tag.png" 2>/dev/null \
    && echo "  已抓 $tag <- window $id ($(stat -c %s "$WS/shots/$tag.png") bytes)" \
    || echo "  $tag 抓图失败 (window $id)"
}
for name in Gazebo RViz 'rviz2' 'TurtleBot3'; do
  id=$(xdotool search --onlyvisible --name "$name" 2>/dev/null | head -1)
  [ -n "$id" ] && shot "$id" "win_$(echo "$name" | tr 'A-Z' 'a-z')"
done