#!/usr/bin/env python3
"""调试 EntityFactory：字符串 SDF spawn 到底卡在哪。"""
import time

import gz.msgs10.boolean_pb2 as bool_pb2
import gz.msgs10.entity_factory_pb2 as ef_pb2
import gz.msgs10.pose_v_pb2 as pose_v_pb2
from gz.transport13 import Node

WORLD = "rmul2027_3v3"
node = Node()
poses = {}
node.subscribe(pose_v_pb2.Pose_V, f"/world/{WORLD}/pose/info",
               lambda m: [poses.__setitem__(p.name, (round(p.position.x, 3),
                                                     round(p.position.y, 3),
                                                     round(p.position.z, 3)))
                          for p in m.pose])

SDF = """<sdf version="1.9"><model name="dbg_probe"><static>false</static>
<link name="l"><inertial><mass>0.01</mass><inertia><ixx>1e-7</ixx><iyy>1e-7</iyy><izz>1e-7</izz></inertia></inertial>
<collision name="c"><geometry><box><size>0.03 0.03 0.03</size></box></geometry></collision>
<visual name="v"><geometry><box><size>0.03 0.03 0.03</size></box></geometry></visual>
</link></model></sdf>"""

for svc in ("/world/%s/create" % WORLD, "/world/%s/create/blocking" % WORLD):
    req = ef_pb2.EntityFactory()
    req.name = "dbg_probe"
    req.sdf = SDF
    req.sdf_filename = ""
    req.allow_renaming = True
    req.pose.position.x = 1.0
    req.pose.position.y = 1.0
    req.pose.position.z = 0.4
    req.pose.orientation.w = 1.0
    print(f"\n--- {svc} ---")
    print("  req fields set: name=%r sdf_len=%d sdf_filename=%r" %
          (req.name, len(req.sdf), req.sdf_filename))
    t0 = time.time()
    ok, rep = node.request(svc, req, ef_pb2.EntityFactory, bool_pb2.Boolean, 5000)
    print(f"  ok={ok} rep={None if rep is None else rep.data} elapsed={time.time()-t0:.2f}s")
    time.sleep(0.5)
    print("  poses now:", {k: v for k, v in poses.items() if "dbg" in k})