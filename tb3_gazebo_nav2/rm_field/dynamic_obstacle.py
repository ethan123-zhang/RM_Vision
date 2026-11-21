#!/usr/bin/env python3
"""动态障碍物驱动：按 config/avoid_obstacles.json 驱动 world 里的 dynamic_box。

两种模式（配置里的 dynamic.mode）：
  1. triggered_retreat（绕障验收默认）：箱体**一开始停在基线走道正中**挡路；机器人进到
     trigger_dist 以内后，箱体沿 start->end 单程移动（横向让开，方向垂直于机器人来向），
     到位后停住。为什么不用"连续往返横穿"（2026-11-20 第一版）：盲动的障碍物会反复
     切进机器人走廊 → nav2 默认 MPPI 反复触发 "Failed to make progress"/recovery，形成
     livelock（实测第三段 154 s、出现 0.077 m 级接近）。触发式单程退让仍有"会动的障碍物"，
     但不会追着机器人跑，可复现且可判定。
  2. patrol（手动试验用）：在 p0<->p1 之间以固定速度往返，永不主动停。

档位（2026-11-21）：config.dynamic.tiers 里每个档位有自己的速度/让开方向，
--tier 选档（默认 default_tier=slow，与 2026-11-20 基线同几何）。

做法：直接调 Gazebo 的 /world/<world>/set_pose 服务（gz.msgs.Pose），10 Hz 设定位置。
为什么不给障碍物加轮子：验收要的是"机器人看见会动的东西并避开"，障碍物自身动力学
越简单越可复现；位置由服务直接设定（位置误差 0），速度由本脚本按墙钟积分，
验收侧再从 /world/<world>/pose/info **独立**测速（不读本脚本日志，防自证）。

stdout 每 0.5 s 一行，例如：
  WAIT  t=12.3 robot_dist=3.42
  MOVING t=18.1 x=2.50 y=-2.21 res=1
  PARKED t=24.6 x=2.50 y=-3.50
CTRL-C 停止（停在最后位置）。
"""
import argparse
import json
import math
import os
import signal
import time

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node

HERE = os.path.dirname(os.path.abspath(__file__))
DEF_CFG = os.path.join(HERE, "config", "avoid_obstacles.json")


def load_dyn(cfg_path):
    with open(cfg_path, "r", encoding="utf-8") as f:
        return json.load(f)["dynamic"]


def resolve_tier(dyn, name=None):
    """把配置里的"档位"解析成实际几何 {name, speed, start, end}。

    config.dynamic.tiers 是唯一源（每个档位有自己的速度与让开方向）；没选档位时用
    default_tier。老配置（只有顶层 speed/start/end）仍支持，便于对比历史数据。
    """
    tiers = dyn.get("tiers") or {}
    if not tiers:
        return {"name": "legacy", "speed": dyn["speed"],
                "start": list(dyn["start"]), "end": list(dyn["end"])}
    name = name or dyn.get("default_tier") or sorted(tiers)[0]
    if name not in tiers:
        raise SystemExit(f"未知档位 {name!r}；可选: {', '.join(sorted(tiers))}")
    t = tiers[name]
    return {"name": name, "speed": float(t["speed"]),
            "start": list(t["start"]), "end": list(t["end"])}


def interp(t, p0, p1, speed):
    """t 秒后沿 p0->p1 走到哪；返回 (x, y, done)。"""
    lx, ly = p1[0] - p0[0], p1[1] - p0[1]
    L = math.hypot(lx, ly)
    if L < 1e-6:
        return p0[0], p0[1], True
    d = t * speed
    if d >= L:
        return p1[0], p1[1], True
    return p0[0] + lx / L * d, p0[1] + ly / L * d, False


class RobotWatcher:
    """从 gz pose/info 里读机器人（模型名含 waffle）的平面位置。"""

    def __init__(self, world):
        self.pose = None
        self.node = Node()
        self.node.subscribe(Pose_V, f"/world/{world}/pose/info", self._cb)

    def _cb(self, msg):
        for p in msg.pose:
            if "waffle" in p.name and "::" not in p.name:
                self.pose = (p.position.x, p.position.y)

    def dist_to(self, xy):
        if self.pose is None:
            return None
        return math.hypot(self.pose[0] - xy[0], self.pose[1] - xy[1])


def load_cfg(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def run_popup(args, cfg):
    """刹车验收：宽挡板先藏在角落，机器人靠近后瞬移到它正前方。"""
    pop = cfg["popup"]
    size = pop["size"]
    z = pop.get("z_center", size[2] / 2.0)
    hidden = list(pop["hidden"])
    appear = list(pop["appear"])
    trigger = float(pop.get("trigger_dist", 0.7))

    stopping = {"flag": False}

    def _stop(_sig, _frm):
        stopping["flag"] = True

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    svc = f"/world/{args.world}/set_pose"
    node = Node()
    req = Pose()
    req.name = pop["name"]
    req.position.z = z
    req.orientation.w = 1.0
    watcher = RobotWatcher(args.world)

    def put(x, y):
        req.position.x, req.position.y = x, y
        ok, _ = node.request(svc, req, Pose, Boolean, 200)
        return ok

    for _ in range(20):
        if put(hidden[0], hidden[1]):
            break
        time.sleep(0.2)
    print(f"HIDDEN x={hidden[0]:.2f} y={hidden[1]:.2f} (appear={appear})", flush=True)

    period = 1.0 / max(args.rate, 1.0)
    t0 = time.monotonic()
    next_tick = time.monotonic()
    next_log = 0.0
    popped = False
    n_ok = n_bad = 0
    while not stopping["flag"]:
        t = time.monotonic() - t0
        if not popped:
            d = watcher.dist_to(appear)
            if d is not None and d <= trigger:
                n_ok += put(appear[0], appear[1])
                popped = True
                print(f"POPPED t={t:.2f} robot_xy="
                      f"{tuple(round(v, 3) for v in (watcher.pose or (float('nan'),) * 2))} "
                      f"dist_before={d:.3f}", flush=True)
            elif t >= next_log:
                next_log = t + 0.5
                print(f"WAIT  t={t:.1f} robot_dist={'-' if d is None else round(d, 2)}",
                      flush=True)
        if args.duration > 0 and t >= args.duration:
            break
        next_tick += period
        dt = next_tick - time.monotonic()
        if dt > 0:
            time.sleep(dt)
        else:
            next_tick = time.monotonic()
    print(f"STOP popped={popped} ok={n_ok} bad={n_bad}", flush=True)
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="rmul2027_3v3")
    ap.add_argument("--config", default=DEF_CFG)
    ap.add_argument("--tier", default=None,
                    help="动态箱档位（config.dynamic.tiers 的键；默认用 default_tier）")
    ap.add_argument("--rate", type=float, default=10.0)     # 服务调用频率（Hz）
    ap.add_argument("--duration", type=float, default=0.0)  # >0 时到时退出（自检用）
    ap.add_argument("--patrol-p0", nargs=2, type=float, default=None)
    ap.add_argument("--patrol-p1", nargs=2, type=float, default=None)
    ap.add_argument("--patrol-speed", type=float, default=None)
    ap.add_argument("--popup", action="store_true",
                    help="刹车验收：把 config.popup 的宽挡板藏在 hidden，机器人进到 appear 前方 "
                         "trigger_dist 内瞬移出现，逼 collision_monitor 刹车")
    args = ap.parse_args()

    if args.popup:
        return run_popup(args, load_cfg(args.config))

    dyn = load_dyn(args.config)
    tier = resolve_tier(dyn, args.tier)
    mode = "patrol" if args.patrol_p0 else dyn.get("mode", "triggered_retreat")
    speed = args.patrol_speed or tier["speed"]
    size = dyn["size"]
    z = dyn.get("z_center", size[2] / 2.0)
    if mode == "patrol":
        p0 = args.patrol_p0 or dyn.get("p0") or tier["start"]
        p1 = args.patrol_p1 or dyn.get("p1") or tier["end"]
        trigger_dist = 0.0
    else:
        p0 = list(tier["start"])
        p1 = list(tier["end"])
        trigger_dist = float(dyn.get("trigger_dist", 1.5))
    print(f"TIER {tier['name']} speed={tier['speed']} m/s {p0}->{p1}", flush=True)

    stopping = {"flag": False}

    def _stop(_sig, _frm):
        stopping["flag"] = True

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    svc = f"/world/{args.world}/set_pose"
    node = Node()
    req = Pose()
    req.name = dyn["name"]
    req.position.z = z
    req.orientation.w = 1.0
    watcher = RobotWatcher(args.world)

    def put(x, y):
        req.position.x, req.position.y = x, y
        ok, _ = node.request(svc, req, Pose, Boolean, 200)
        return ok

    period = 1.0 / max(args.rate, 1.0)
    t0 = None
    next_tick = time.monotonic()
    next_log = 0.0
    n_ok = n_bad = 0
    started = False
    parked = False

    # 开局先把箱体摆到 p0（patrol 模式也先摆位，保证与配置一致）
    for _ in range(20):
        if put(p0[0], p0[1]):
            break
        time.sleep(0.2)

    while not stopping["flag"]:
        if t0 is None:
            t0 = time.monotonic()
        t = time.monotonic() - t0
        if not started and trigger_dist > 0.0:
            d = watcher.dist_to(p0)
            if d is not None and d <= trigger_dist:
                started = True
                t = 0.0
                t0 = time.monotonic()
                next_log = 0.0
                print(f"TRIGGER robot_dist={d:.2f} "
                      f"robot_xy={tuple(round(v, 2) for v in watcher.pose)}", flush=True)
            else:
                if t >= next_log:
                    next_log = t + 0.5
                    print(f"WAIT  t={t:.1f} robot_dist={'-' if d is None else round(d, 2)}",
                          flush=True)
        else:
            started = True

        if started and not parked:
            if mode == "patrol":
                lx, ly = p1[0] - p0[0], p1[1] - p0[1]
                L = math.hypot(lx, ly)
                phase = (t * speed) % (2.0 * L) if L > 1e-6 else 0.0
                d = phase if phase <= L else 2.0 * L - phase
                if L > 1e-6:
                    x, y = p0[0] + lx / L * d, p0[1] + ly / L * d
                else:
                    x, y = p0
                done = False
            else:
                x, y, done = interp(t, p0, p1, speed)
            ok = put(x, y)
            n_ok += ok
            n_bad += (not ok)
            if done:
                parked = True
                print(f"PARKED t={t:.1f} x={x:.3f} y={y:.3f}", flush=True)
            elif t >= next_log:
                next_log = t + 0.5
                print(f"MOVING t={t:.1f} x={x:.3f} y={y:.3f} res={int(ok)}", flush=True)
        if args.duration > 0 and t >= args.duration:
            break
        next_tick += period
        dt = next_tick - time.monotonic()
        if dt > 0:
            time.sleep(dt)
        else:
            next_tick = time.monotonic()
    print(f"STOP ok={n_ok} bad={n_bad}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())