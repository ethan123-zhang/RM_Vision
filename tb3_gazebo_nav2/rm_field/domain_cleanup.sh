#!/usr/bin/env bash
# 清场工具：只停"当前 ROS_DOMAIN_ID"的进程，只删没人用的 FastRTPS 共享内存。
#
# 为什么要有这个文件（2026-11-21）：nav2_field.sh / stop_nav2_field.sh 原先用裸
# pkill -f 清场。pkill 不认 ROS domain —— 并行调试另一个 domain 时会把那边的
# bridge、组件容器一起 SIGKILL；脚本还 rm -f /dev/shm/fastrtps_*，会把别的 domain
# 正在用的 DDS 共享内存删掉（症状：action server 看着 ready，但 /clock 收不到、
# bridge 永远不 READY）。实测踩坑记录见
# docs/handoffs/2026-11-20-amcl-unmapped-obstacle-drift.md §5。
#
# 归属判定：
#   进程   —— 读 /proc/<pid>/environ 里的 ROS_DOMAIN_ID（未设置 = ROS 默认 0）；
#             environ 读不到（权限）时按"不知道"处理，一律不动。
#   共享内存 —— 端口协调文件 fastrtps_port* 用 flock 试探（能锁到 = 没有活 participant）；
#            数据段/信号量扫 /proc/<pid>/maps 与 /proc/<pid>/fd，只要还有活进程映射或
#            打开就保留（可能是别的 domain 的栈）。
#
# 用法（source 之后调用）：
#   kill_in_domain TERM 'gz sim' 'component_container'
#   clean_stale_fastrtps_shm
#   echo "残留 $(count_in_domain 'gz sim')"
# 回退：RM_CLEAN_ALL_DOMAINS=1 恢复旧的全 domain 行为（现场抢救用，会把所有 domain
#       的 nav2/gz 一起清掉，别在并行调试时用）。

rc_self_domain() {
  printf '%s' "${ROS_DOMAIN_ID:-0}"
}

rc_pid_domain() {
  # 该 pid 的 ROS_DOMAIN_ID；environ 读不到时输出空串
  tr '\0' '\n' < "/proc/$1/environ" 2>/dev/null \
    | sed -n 's/^ROS_DOMAIN_ID=//p' | head -n1
}

kill_in_domain() {
  local sig="$1"; shift
  local cur pat pid dom killed=0 skipped=0
  cur=$(rc_self_domain)
  if [ "${RM_CLEAN_ALL_DOMAINS:-0}" = 1 ]; then
    for pat in "$@"; do
      pkill -"$sig" -f -- "$pat" 2>/dev/null
    done
    echo "  [清场] RM_CLEAN_ALL_DOMAINS=1：已对全部 domain 发 SIG$sig"
    return 0
  fi
  for pat in "$@"; do
    for pid in $(pgrep -f -- "$pat" 2>/dev/null); do
      [ "$pid" = "$$" ] && continue
      [ "$pid" = "$PPID" ] && continue
      dom=$(rc_pid_domain "$pid")
      if [ -z "$dom" ]; then
        skipped=$((skipped+1))
        continue
      fi
      if [ "$dom" = "$cur" ]; then
        kill -s "$sig" "$pid" 2>/dev/null && killed=$((killed+1))
      else
        skipped=$((skipped+1))
      fi
    done
  done
  echo "  [清场] SIG$sig（domain $cur）：命中本 domain $killed 个，跳过 $skipped 个（其它/未知 domain）"
}

rc_pid_alive() {
  # 进程还活着（僵尸 Z 不算：SIGTERM 后父进程还没 wait 时会短暂是 Z）
  local st
  st=$(ps -o stat= -p "$1" 2>/dev/null | tr -d ' ')
  [ -n "$st" ] && [ "${st#Z}" = "$st" ]
}

count_in_domain() {
  local cur pat pid dom n=0
  cur=$(rc_self_domain)
  for pat in "$@"; do
    for pid in $(pgrep -f -- "$pat" 2>/dev/null); do
      [ "$pid" = "$$" ] && continue
      [ "$pid" = "$PPID" ] && continue
      rc_pid_alive "$pid" || continue
      dom=$(rc_pid_domain "$pid")
      # 只看本 domain：environ 读不到的（其它用户的进程）不算我们的残留，
      # 否则"无 ROS_DOMAIN_ID 的进程"（ROS 默认 domain 0）会被误报成没停干净
      [ "$dom" = "$cur" ] && n=$((n+1))
    done
  done
  printf '%s' "$n"
}

rc_shm_in_use() {
  # 仍被活进程映射或打开、且文件还在的 /dev/shm/{sem.}fastrtps* 文件名。
  # 注意：FastRTPS/glibc 会先 unlink 再继续用映射，maps 里就是 "<path> (deleted)"；
  # 这种"已删除但还映射着"的旧痕迹磁盘上没文件可删，不能算进"在用文件"，
  # 否则会保护同名的后来者。取整串路径名（grep -o 到空格），不是子串匹配：
  # fastrtps_port1234 不会命中 fastrtps_port1234_el。
  local n
  {
    grep -h -E '/dev/shm/(sem\.)?fastrtps' /proc/[0-9]*/maps 2>/dev/null
    ls -l /proc/[0-9]*/fd/* 2>/dev/null | grep -E '/dev/shm/(sem\.)?fastrtps'
  } | grep -o -E '/dev/shm/(sem\.)?fastrtps[^[:space:]]*' \
    | sed -e 's#.*/##' -e 's/(deleted)//' \
    | sort -u \
    | while read -r n; do
        [ -e "/dev/shm/$n" ] && printf '%s\n' "$n"
      done
}

clean_stale_fastrtps_shm() {
  # FastRTPS 不是靠删文件来"释放"共享内存的；这里只清崩溃/强杀留下的孤儿文件。
  # 活 participant 的痕迹（实测，2026-11-21）：
  #   数据段 fastrtps_<guid>      —— 被 mmap（maps 扫得到）
  #   数据段/端口 fastrtps_*_el   —— 被 flock 锁住 + fd 打开
  #   端口段 fastrtps_port<n>     —— 被 mmap
  #   sem.fastrtps_port<n>_mutex  —— POSIX 信号量，没有文件锁、也没 fd/mapping 可查
  #     → 一律保留（删掉活栈的信号量比留个孤儿文件危险得多）
  local f base inuse kept=0 removed=0
  if [ "${RM_CLEAN_ALL_DOMAINS:-0}" = 1 ]; then
    rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
    echo "  [shm] RM_CLEAN_ALL_DOMAINS=1：已清掉全部 fastrtps 共享内存"
    return 0
  fi
  inuse=$(rc_shm_in_use)
  # 端口/数据段是成对的：<name> 与 <name>_el。实测 FastRTPS 只把锁打在 _el 上、
  # plain 文件既不 mmap 也不打开（例如 ros2 daemon 常驻的端口 7000/7001）——
  # 按单文件判会以为 plain 没人用而删掉，下一次 participant init 该端口就报
  # "Failed init_port fastrtps_portNNNN: open_and_lock_file failed"（非致命，但丢 SHM
  # 通道 + 刷日志）。所以这里把配对关系补进"在用"集合。
  inuse=$(printf '%s\n%s\n' "$inuse" \
          "$(printf '%s\n' "$inuse" | sed -n 's/^\(.*\)_el$/\1/p')" \
          "$(printf '%s\n' "$inuse" | sed -n 's/^\(.*\)$/\1_el/p')" | sort -u)
  for f in /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*; do
    [ -e "$f" ] || continue
    base=${f##*/}
    case "$base" in
      sem.fastrtps_*) kept=$((kept+1)); continue ;;
    esac
    if printf '%s\n' "$inuse" | grep -qxF -- "$base"; then
      kept=$((kept+1))
      continue
    fi
    if ! flock -n "$f" true 2>/dev/null; then
      kept=$((kept+1))
      continue
    fi
    rm -f "$f" 2>/dev/null && removed=$((removed+1))
  done
  echo "  [shm] FastRTPS 共享内存：删除 $removed 个没人用的，保留 $kept 个（在用/信号量）"
}