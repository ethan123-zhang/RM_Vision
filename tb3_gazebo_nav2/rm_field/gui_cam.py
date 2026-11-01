#!/usr/bin/env python3
"""设置 gz GUI 相机位姿并可选截 3D 视图。

用法:
  python3 gui_cam.py pose  x y z roll pitch yaw
  python3 gui_cam.py shot  /path/out.png
  python3 gui_cam.py both  x y z roll pitch yaw  /path/out.png
"""
import math
import sys
import time

import gz.msgs10.boolean_pb2 as bool_pb2
import gz.msgs10.gui_camera_pb2 as gui_cam_pb2
import gz.msgs10.stringmsg_pb2 as str_pb2
from gz.transport13 import Node

node = Node()


def set_pose(x, y, z, roll, pitch, yaw):
    msg = gui_cam_pb2.GUICamera()
    msg.name = "user_camera"
    msg.view_controller = "orbit"
    msg.projection_type = "perspective"
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z
    # RPY -> quaternion
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    msg.pose.orientation.w = cr * cp * cy + sr * sp * sy
    msg.pose.orientation.x = sr * cp * cy - cr * sp * sy
    msg.pose.orientation.y = cr * sp * cy + sr * cp * sy
    msg.pose.orientation.z = cr * cp * sy - sr * sp * cy
    pub = node.advertise("/gui/camera/pose", gui_cam_pb2.GUICamera)
    time.sleep(0.5)
    ok = pub.publish(msg)
    print("publish /gui/camera/pose ->", ok, (x, y, z, roll, pitch, yaw))


def shot(path):
    req = str_pb2.StringMsg()
    req.data = path
    ok, rep = node.request("/gui/screenshot", req, str_pb2.StringMsg,
                           bool_pb2.Boolean, 10000)
    print("screenshot:", ok, None if rep is None else rep.data, path)


cmd = sys.argv[1] if len(sys.argv) > 1 else "both"
if cmd in ("pose", "both"):
    vals = [float(v) for v in sys.argv[2:8]]
    set_pose(*vals)
    time.sleep(1.5)
    path = sys.argv[8] if len(sys.argv) > 8 else None
else:
    path = sys.argv[2] if len(sys.argv) > 2 else None
if path:
    time.sleep(0.5)
    shot(path)