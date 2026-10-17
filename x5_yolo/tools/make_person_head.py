#!/usr/bin/env python3
"""Build a 1-class (person) YOLOv8n head by slicing the COCO cls branch.

No retraining: the person logits of the original 80-class model are exactly
output channel 0 of each cv3 final 1x1 conv, so slicing preserves them.

Outputs (NCHW, raw logits, no Sigmoid/DFL) per scale:
    box_80 [1,64,80,80], cls_80 [1,1,80,80]
    box_40 [1,64,40,40], cls_40 [1,1,40,40]
    box_20 [1,64,20,20], cls_20 [1,1,20,20]
"""
import argparse
import os

import numpy as np
import onnx
from onnx import helper, numpy_helper


def find_cls_convs(graph):
    """Return {scale_index: node} for /model.22/cv3.{i}/cv3.{i}.2/Conv."""
    out = {}
    for n in graph.node:
        if n.op_type != "Conv":
            continue
        if not n.name.startswith("/model.22/cv3."):
            continue
        if not n.name.endswith(".2/Conv"):
            continue
        idx = int(n.name.split("/model.22/cv3.")[1].split("/")[0])
        out[idx] = n
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True)
    ap.add_argument("--dst", required=True)
    ap.add_argument("--class-id", type=int, default=0, help="0 = person in COCO")
    ap.add_argument("--layout", choices=("nchw", "nhwc"), default="nchw",
                    help="nhwc matches the official RDK X5 runtime protocol")
    args = ap.parse_args()

    model = onnx.load(args.src)
    graph = model.graph
    inits = {i.name: numpy_helper.to_array(i) for i in graph.initializer}

    cls_convs = find_cls_convs(graph)
    assert sorted(cls_convs) == [0, 1, 2], f"unexpected cls convs: {sorted(cls_convs)}"

    outputs = []
    new_nodes, new_inits = [], []
    for i in (0, 1, 2):
        n = cls_convs[i]
        w_name, b_name = n.input[1], n.input[2]
        W, b = inits[w_name], inits[b_name]
        assert W.shape[0] == 80 and W.shape[1] == 80 and W.shape[2:] == (1, 1), W.shape
        sw = np.ascontiguousarray(W[args.class_id:args.class_id + 1])
        sb = np.ascontiguousarray(b[args.class_id:args.class_id + 1])

        out_name = f"cls_{80 >> i}"
        wname = f"person_cls_w_{i}"
        bname = f"person_cls_b_{i}"
        new_inits.append(numpy_helper.from_array(sw, wname))
        new_inits.append(numpy_helper.from_array(sb, bname))

        attrs = {a.name: helper.get_attribute_value(a) for a in n.attribute}
        box_src = f"/model.22/cv2.{i}/cv2.{i}.2/Conv_output_0"
        if args.layout == "nchw":
            new_nodes.append(helper.make_node("Conv", [n.input[0], wname, bname],
                                              [out_name], name=f"person_{out_name}", **attrs))
            new_nodes.append(helper.make_node("Identity", [box_src], [f"box_{80 >> i}"],
                                             name=f"person_box_{i}"))
            outputs += [f"box_{80 >> i}", out_name]
        else:
            cls_raw = out_name + "_raw"
            new_nodes.append(helper.make_node("Conv", [n.input[0], wname, bname],
                                              [cls_raw], name=f"person_{out_name}", **attrs))
            new_nodes.append(helper.make_node("Transpose", [cls_raw], [out_name],
                                              name=f"person_{out_name}_nhwc", perm=[0, 2, 3, 1]))
            new_nodes.append(helper.make_node("Transpose", [box_src], [f"box_{80 >> i}"],
                                              name=f"person_box_{i}", perm=[0, 2, 3, 1]))
            outputs += [out_name, f"box_{80 >> i}"]

    graph.node.extend(new_nodes)
    graph.initializer.extend(new_inits)
    model = onnx.shape_inference.infer_shapes(model)

    tmp = args.dst + ".tmp.onnx"
    onnx.save(model, tmp)
    onnx.utils.extract_model(tmp, args.dst, ["images"], outputs)
    os.remove(tmp)

    m = onnx.load(args.dst)
    onnx.checker.check_model(m)
    m = onnx.shape_inference.infer_shapes(m)
    print("== saved", args.dst, "nodes", len(m.graph.node))
    for o in m.graph.output:
        print("  out", o.name, [d.dim_value or d.dim_param for d in o.type.tensor_type.shape.dim])
    print("  size %.2f MB" % (os.path.getsize(args.dst) / 1e6))


if __name__ == "__main__":
    main()
