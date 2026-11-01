#!/usr/bin/env python3
"""验证 workaround：sdf_filename 方式 spawn 探针 + /cmd_vel 驱动机器。"""
import os
import time

import gz.msgs10.boolean_pb2 as bool_pb2
import gz.msgs10.entity_factory_pb2 as ef_pb2
import gz.msgs10.pose_v_pb2 as pose_v_pb2
import gz.msgs10.twist_pb2 as twist_pb2
from gz.transport13 import Node

WORLD = "rmul2027_3v3"
node = Node()
poses = {}
node.subscribe(pose_v_pb2.Pose_V, f"/world/{WORLD}/pose/info",
               lambda m: [poses.__setitem__(p.name, (round(p.position.x, 4),
                                                     round(p.position.y, 4),
                                                     round(p.position.z, 4)))
                          for p in m.pose])
time.sleep(1.0)

PROBE_SDF = """<?xml version="1.0"?>
<sdf version="1.9"><model name="dbg_probe2"><static>false</static>
<link name="l"><inertial><mass>0.01</mass>
<inertia><ixx>1e-7</ixx><iyy>1e-7</iyy><izz>1e-7</izz></inertia></inertial>
<collision name="c"><geometry><box><size>0.03 0.03 0.03</size></box></geometry></collision>
<visual name="v"><geometry><box><size>0.03 0.03 0.03</size></box></geometry></visual>
</link></model></sdf>
"""
path = "/tmp/dbg_probe2.sdf"
with open(path, "w") as f:
    f.write(PROBE_SDF)

req = ef_pb2.EntityFactory()
req.name = "dbg_probe2"
req.sdf_filename = path
req.allow_renaming = False
req.pose.position.x = 1.5
req.pose.position.y = 1.5
req.pose.position.z = 0.4
req.pose.orientation.w = 1.0
ok, rep = node.request(f"/world/{WORLD}/create/blocking", req,
                       ef_pb2.EntityFactory, bool_pb2.Boolean, 10000)
print("spawn probe by filename -> ok=%s rep=%s" % (ok, None if rep is None else rep.data))
time.sleep(1.5)
print("probe pose:", poses.get("dbg_probe2"))

pub = node.advertise("/cmd_vel", twist_pb2.Twist)
time.sleep(0.5)
tw = twist_pb2.Twist()
tw.linear.x = 0.5
print("robot before:", poses.get("waffle"))
t0 = time.time()
while time.time() - t0 < 2.0:
    pub.publish(tw)
    time.sleep(0.05)
pub.publish(twist_pb2.Twist())
time.sleep(1.0)
print("robot after :", poses.get("waffle"))