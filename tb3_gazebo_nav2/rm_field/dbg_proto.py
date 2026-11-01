#!/usr/bin/env python3
"""弄清 gz.msgs EntityFactory 的 sdf 字段为什么赋值后为空。"""
import gz.msgs10.entity_factory_pb2 as ef_pb2

req = ef_pb2.EntityFactory()
print("class:", type(req))
print("has sdf attr:", hasattr(req, "sdf"))
print("instance dict:", [k for k in getattr(req, "__dict__", {}) if "sdf" in k])

try:
    req.sdf = "<sdf>hello</sdf>"
    print("after assign: repr(req.sdf)=%r len=%d" % (req.sdf, len(req.sdf)))
except Exception as e:
    print("assign failed:", type(e).__name__, e)

req.sdf_filename = "/tmp/x.sdf"
req.name = "abc"
print("sdf_filename repr=%r len=%d" % (req.sdf_filename, len(req.sdf_filename)))
print("text format:")
print(req)
print("serialized len:", len(req.SerializeToString()))

# 试试构造函数 / CopyFrom
try:
    r2 = ef_pb2.EntityFactory(name="p2", sdf="<sdf>hi</sdf>")
    print("ctor: repr(sdf)=%r len=%d" % (r2.sdf, len(r2.sdf)))
except Exception as e:
    print("ctor failed:", type(e).__name__, e)

r3 = ef_pb2.EntityFactory()
try:
    r3.ParseFromString(req.SerializeToString())
    print("roundtrip: sdf=%r name=%r filename=%r" % (r3.sdf, r3.name, r3.sdf_filename))
except Exception as e:
    print("roundtrip failed:", e)

# 看看官方 gz 源码对应的字段：EntityFactory 里到底叫什么
for f in ef_pb2.EntityFactory.DESCRIPTOR.fields:
    print("field", f.name, f.number, "type", f.type, "label", f.label)