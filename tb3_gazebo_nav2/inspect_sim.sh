#!/usr/bin/env bash
# 查看 nav2_minimal_tb3_sim 包结构与 Gazebo 资源引用
P=/opt/ros/jazzy/share/nav2_minimal_tb3_sim
echo "--- tree"
find "$P" -maxdepth 2 | sed "s|$P|<pkg>|" | head -40
echo "--- gz_waffle.sdf.xacro 资源引用"
grep -oE '(model|file)://[A-Za-z0-9_./-]+' "$P/urdf/gz_waffle.sdf.xacro" | sort -u | head -20
echo "--- world 资源引用"
grep -oE '(model|file)://[A-Za-z0-9_./-]+' "$P/worlds/tb3_sandbox.sdf.xacro" | sort -u | head -20
echo "--- 其它 xacro 依赖"
grep -rnoE '\$\(find [a-z0-9_]+\)' "$P/urdf" "$P/worlds" 2>/dev/null | head -10
echo "--- mesh 文件"
find "$P" -name '*.dae' -o -name '*.stl' -o -name '*.obj' 2>/dev/null | head -5