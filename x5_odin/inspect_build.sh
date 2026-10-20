#!/usr/bin/env bash
# inspect_build.sh — 看当前编译单元 / 内存压力（板端 3 GB 内存，编译会吃 swap）
date +%T
for p in $(pgrep -x cc1plus); do
  tu=$(tr '\0' ' ' < "/proc/$p/cmdline" | grep -o 'CMakeFiles/[^ ]*' | head -1)
  rss=$(awk '/^VmRSS/{print $2" kB"}' "/proc/$p/status")
  st=$(awk '/^State/{print $2}' "/proc/$p/status")
  pcpu=$(ps -o pcpu= -p "$p")
  echo "$tu  state=$st cpu=${pcpu}% rss=$rss  elapsed=$(ps -o etime= -p "$p" | tr -d ' ')"
done
echo "---- mem ----"
free -m | sed -n '1,3p'
echo "---- objects ----"
find "$HOME/odin_ws/build" -name '*.o' | wc -l
