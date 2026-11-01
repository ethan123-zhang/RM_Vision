#!/usr/bin/env python3
"""在已运行的 Gazebo 世界里生成一个模型（默认 TB3 waffle）。

用法: python3 spawn_model.py <name> <sdf 或 model.sdf 路径> [x y z yaw]
"""
import os
import sys
import time

import gz.msgs10.boolean_pb2 as bool_pb2
import gz.msgs10.entity_factory_pb2 as ef_pb2
import gz.msgs10.pose_v_pb2 as pose_v_pb2
from gz.transport13 import Node

WORLD = os.environ.get("GZ_WORLD", "rmul2027_3v3")
name = sys.argv[1] if len(sys.argv) > 1 else "waffle"
path = sys.argv[2] if len(sys.argv) > 2 else ""
if not path:
    path = ("/opt/ros/jazzy/share/turtlebot3_gazebo/models/"
            "turtlebot3_waffle/model.sdf")
x = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
y = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
z = float(sys.argv[5]) if len(sys.argv) > 5 else 0.10
yaw = float(sys.argv[6]) if len(sys.argv) > 6 else 0.0

node = Node()
poses = {}
node.subscribe(pose_v_pb2.Pose_V, f"/world/{WORLD}/pose/info",
               lambda m: [poses.__setitem__(p.name, (round(p.position.x, 3),
                                                     round(p.position.y, 3),
                                                     round(p.position.z, 3)))
                          for p in m.pose])
time.sleep(0.8)
req = ef_pb2.EntityFactory()
req.name = name
req.sdf_filename = path
req.allow_renaming = True
req.pose.position.x = x
req.pose.position.y = y
req.pose.position.z = z
req.pose.orientation.z = __import__("math").sin(yaw / 2)
req.pose.orientation.w = __import__("math").cos(yaw / 2)
ok, rep = node.request(f"/world/{WORLD}/create/blocking", req,
                       ef_pb2.EntityFactory, bool_pb2.Boolean, 15000)
print("create:", ok, None if rep is None else rep.data)
time.sleep(2.0)
print("pose:", poses.get(name))