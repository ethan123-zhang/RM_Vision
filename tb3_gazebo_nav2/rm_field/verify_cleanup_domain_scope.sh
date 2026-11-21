#!/usr/bin/env bash
# 回归：清场只在本 ROS_DOMAIN_ID 内生效、共享内存只删没人用的。
# 不需要 Gazebo / nav2 —— 用同 cmdline 的假进程 + 假共享内存段覆盖两个坑：
#   ① 裸 pkill -f 跨 domain 杀进程（并行调试时误杀别的 domain 的 bridge/容器）
#   ② rm -f /dev/shm/fastrtps_* 切断别的 domain 正在用的 DDS 共享内存
# 跑法：bash verify_cleanup_domain_scope.sh          （PASS/FAIL 逐项打印）
# 注意：最后一项会真的执行 stop_nav2_field.sh（本 domain）；执行前请确认该 domain
#       上没有你想留着跑的栈。
set +u
ROOT=/mnt/e/RM/tb3_gazebo_nav2/rm_field
cd "$ROOT" || exit 1
export ROS_DOMAIN_ID="${CLEAN_TEST_DOMAIN:-42}"
OTHER_DOMAIN=$((ROS_DOMAIN_ID + 1))
source "$ROOT/domain_cleanup.sh" || { echo "致命: 缺 $ROOT/domain_cleanup.sh"; exit 1; }

PASS=0
FAIL=0
ok()  { echo "  PASS  $*"; PASS=$((PASS + 1)); }
bad() { echo "  FAIL  $*"; FAIL=$((FAIL + 1)); }

is_alive() {  # 僵尸进程不算活着（TERM 之后父进程还没 wait 时会短暂是 Z）
  local st
  st=$(ps -o stat= -p "$1" 2>/dev/null | tr -d ' ')
  [ -n "$st" ] && [ "${st#Z}" = "$st" ]
}

spawn_fake() {  # $1=domain（"-" = 不设 ROS_DOMAIN_ID，即 ROS 默认 0）；$2=argv0；回显 pid
  local dom="$1" name="$2"
  # 假进程的 stdout/stderr 必须重定向：否则它会继承 $(...) 的管道，命令替换
  # 等不到管道关闭、整段测试卡死（sleep 300 期间一直挂着）。
  if [ "$dom" = "-" ]; then
    env -u ROS_DOMAIN_ID bash -c 'exec -a "$0" sleep 300' "$name" >/dev/null 2>&1 &
  else
    ROS_DOMAIN_ID="$dom" bash -c 'exec -a "$0" sleep 300' "$name" >/dev/null 2>&1 &
  fi
  echo $!
}

fake_files=(/dev/shm/fastrtps_seg_stale_cleantest \
            /dev/shm/fastrtps_seg_live_cleantest \
            /dev/shm/fastrtps_port9901_cleantest \
            /dev/shm/fastrtps_port9902_cleantest \
            /dev/shm/sem.fastrtps_live_cleantest \
            /dev/shm/fastrtps_seg_open_cleantest \
            /dev/shm/fastrtps_port9903_cleantest \
            /dev/shm/fastrtps_port9903_cleantest_el)
holders=()
cleanup() {
  for p in "${holders[@]}" "${fakes[@]}"; do kill -9 "$p" 2>/dev/null; done
  for f in "${fake_files[@]}"; do rm -f "$f" 2>/dev/null; done
  sleep 0.2
}
trap cleanup EXIT

before_shm=$(ls /dev/shm 2>/dev/null | sort)
fakes=()

echo "=== 1. 进程 domain 过滤（当前 domain=$ROS_DOMAIN_ID，另一个=$OTHER_DOMAIN）==="
p_same=$(spawn_fake "$ROS_DOMAIN_ID" fake_component_container); fakes+=("$p_same")
p_other=$(spawn_fake "$OTHER_DOMAIN" fake_component_container); fakes+=("$p_other")
p_none=$(spawn_fake - fake_component_container); fakes+=("$p_none")
sleep 0.5
kill_in_domain TERM 'fake_component_container'
sleep 0.5
if is_alive "$p_same"; then bad "本 domain 的假容器没被 TERM 掉 (pid $p_same)"; else ok "本 domain 进程被 TERM（pid $p_same）"; fi
if is_alive "$p_other"; then ok "其它 domain 进程未被误杀（pid $p_other）"; else bad "其它 domain 进程被误杀（pid $p_other）"; fi
if is_alive "$p_none"; then ok "未设 ROS_DOMAIN_ID（=0）的进程未被误杀（pid $p_none）"; else bad "未设 ROS_DOMAIN_ID 的进程被误杀（pid $p_none）"; fi

echo "=== 2. 用户入口 stop_nav2_field.sh 同样只清本 domain ==="
s_same=$(spawn_fake "$ROS_DOMAIN_ID" goal_pose_bridge.py); fakes+=("$s_same")
s_other=$(spawn_fake "$OTHER_DOMAIN" goal_pose_bridge.py); fakes+=("$s_other")
sleep 0.5
ROS_DOMAIN_ID="$ROS_DOMAIN_ID" bash "$ROOT/stop_nav2_field.sh"
sleep 0.5
if is_alive "$s_same"; then bad "stop_nav2_field.sh 没停掉本 domain 的 goal_pose_bridge（pid $s_same）"; else ok "stop_nav2_field.sh 停掉本 domain 的 goal_pose_bridge"; fi
if is_alive "$s_other"; then ok "stop_nav2_field.sh 未误杀其它 domain 的 goal_pose_bridge"; else bad "stop_nav2_field.sh 误杀了其它 domain 的 goal_pose_bridge（pid $s_other）"; fi

echo "=== 3. RM_CLEAN_ALL_DOMAINS=1 回退路径（现场抢救用）==="
RM_CLEAN_ALL_DOMAINS=1 kill_in_domain TERM 'goal_pose_bridge[.]py'
sleep 0.5
if is_alive "$s_other"; then bad "RM_CLEAN_ALL_DOMAINS=1 没有清到其它 domain（pid $s_other）"; else ok "RM_CLEAN_ALL_DOMAINS=1 能清其它 domain"; fi
unset RM_CLEAN_ALL_DOMAINS

echo "=== 4. FastRTPS 共享内存：只删没人用的 ==="
touch "${fake_files[0]}"
python3 - "${fake_files[1]}" >/dev/null 2>&1 <<'PY' &
import mmap, sys, time
path = sys.argv[1]
with open(path, 'wb') as f:
    f.write(b'\0' * 4096)
with open(path, 'r+b') as f:
    m = mmap.mmap(f.fileno(), 4096)   # 必须留引用，否则 mmap 立刻被回收、映射就没了
    time.sleep(300)
PY
holders+=($!); disown $! 2>/dev/null
python3 - "${fake_files[4]}" >/dev/null 2>&1 <<'PY' &
import sys, time
f = open(sys.argv[1], 'wb')
f.write(b'x')
f.flush()
time.sleep(300)
PY
holders+=($!); disown $! 2>/dev/null
touch "${fake_files[2]}" "${fake_files[3]}"
flock "${fake_files[2]}" sleep 300 >/dev/null 2>&1 &
holders+=($!); disown $! 2>/dev/null
python3 - "${fake_files[5]}" >/dev/null 2>&1 <<'PY' &
import sys, time
f = open(sys.argv[1], 'wb')   # 只打开不 mmap：考验 fd 扫描
f.write(b'y')
f.flush()
time.sleep(300)
PY
holders+=($!); disown $! 2>/dev/null
# 端口成对保护：只锁住 _el、plain 文件谁都没映射 → 也必须保留（FastRTPS 下一轮 init 要用）
touch "${fake_files[6]}" "${fake_files[7]}"
flock "${fake_files[7]}" sleep 300 >/dev/null 2>&1 &
holders+=($!); disown $! 2>/dev/null
sleep 1
inuse_before=$(rc_shm_in_use)
clean_stale_fastrtps_shm
[ -e "${fake_files[0]}" ] && bad "没人用的段没被删（${fake_files[0]}）" || ok "没人用的段被删掉"
[ -e "${fake_files[1]}" ] && ok "被 mmap 的段保留（有活进程在用）" || bad "被 mmap 的段被误删（可能切断别的 domain 的 DDS）"
[ -e "${fake_files[2]}" ] && ok "被 flock 持有的文件保留" || bad "被 flock 持有的文件被误删"
[ -e "${fake_files[3]}" ] && bad "没人用的端口文件没被删（${fake_files[3]}）" || ok "没人用的端口文件被删掉"
[ -e "${fake_files[4]}" ] && ok "sem 信号量一律保留（没有文件锁可查，保守）" || bad "sem 文件被误删"
[ -e "${fake_files[5]}" ] && ok "被进程打开（未 mmap）的段保留" || bad "被 fd 打开的段被误删"
[ -e "${fake_files[6]}" ] && [ -e "${fake_files[7]}" ] \
  && ok "端口成对文件：只锁 _el 时 plain 文件也保留" \
  || bad "端口成对文件被拆开（plain 被删、_el 保留 → 下次 init 报 open_and_lock_file failed）"

echo "=== 5. 清场前所有"在用"的共享内存都不能被删 ==="
gone_inuse=""
for name in $inuse_before; do
  if [ ! -e "/dev/shm/$name" ]; then gone_inuse="$gone_inuse $name"; fi
done
if [ -z "$gone_inuse" ]; then
  ok "清场前判定在用的 $(echo "$inuse_before" | wc -l) 个文件全部还在"
else
  bad "清场删掉了在用文件:$gone_inuse"
fi

echo
echo "=== 结果：PASS=$PASS FAIL=$FAIL ==="
[ "$FAIL" = 0 ] || exit 1
echo "OK: 清场 domain 隔离回归全过"