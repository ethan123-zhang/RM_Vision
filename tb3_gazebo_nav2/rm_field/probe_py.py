#!/usr/bin/env python3
"""探测：EntityFactory 字段、模型列表、pose/info 是否工作、tb3 资源路径。"""
import os
import time

from gz.transport13 import Node
from gz.msgs10 import entity_factory_pb2, pose_v_pb2

WORLD = "rmul2027_3v3"
req = entity_factory_pb2.EntityFactory()
print("EntityFactory fields:")
for f in req.DESCRIPTOR.fields:
    print(f"  {f.name:22s} type={f.type}")

node = Node()
print("\nservices:", [s for s in node.service_list() if 'world' in s][:10])
print("topics:", [t for t in node.topic_list() if 'world' in t][:15])

poses = {}
def cb(msg):
    for p in msg.pose:
        poses[p.name] = (round(p.position.x, 4), round(p.position.y, 4), round(p.position.z, 4))
sub = node.subscribe(pose_v_pb2.Pose_V, f"/world/{WORLD}/pose/info", cb)
time.sleep(1.5)
print("\npose/info models:", {k: v for k, v in list(poses.items())[:10]}, "count", len(poses))
os.system("ls /opt/ros/jazzy/share/turtlebot3_gazebo/models/ 2>/dev/null")
os.system("find /opt/ros/jazzy/share -maxdepth 4 -type d -name 'turtlebot3_common' 2>/dev/null | head -3")