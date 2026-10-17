#!/usr/bin/env python3
"""Inspect a YOLOv8 ONNX graph: outputs, Sigmoid/Softmax, Detect-head convs."""
import sys

import onnx
from onnx import numpy_helper

MODEL = sys.argv[1] if len(sys.argv) > 1 else r"E:\RM\x5_yolo\model\yolov8n.onnx"


def dims(vi):
    t = vi.type.tensor_type
    return [d.dim_value or d.dim_param for d in t.shape.dim], t.elem_type


def main():
    m = onnx.load(MODEL)
    g = m.graph
    print("== file:", MODEL, "opset", m.opset_import[0].version, "nodes", len(g.node))
    print("\n== graph inputs ==")
    for i in g.input:
        print(" ", i.name, dims(i))
    print("\n== graph outputs ==")
    for o in g.output:
        print(" ", o.name, dims(o))
    prod = {n.output[0]: n for n in g.node if n.output}
    print("\n== output producers ==")
    for o in g.output:
        n = prod.get(o.name)
        if n is None:
            print(f"  {o.name}: <initializer or input>")
            continue
        print(f"  {o.name} <- {n.op_type} ({n.name}) inputs={list(n.input)}")
        for inp in n.input:
            p = prod.get(inp)
            if p:
                print(f"      {inp} <- {p.op_type} ({p.name}) inputs={list(p.input)}")

    inits = {i.name: numpy_helper.to_array(i) for i in g.initializer}
    print("\n== Sigmoid / Softmax nodes ==")
    for n in g.node:
        if n.op_type in ("Sigmoid", "Softmax"):
            print(f"  {n.op_type:8s} {n.name:45s} in={list(n.input)} out={list(n.output)}")
            for w in n.input:
                if w in inits:
                    print("      weight", w, inits[w].shape)

    print("\n== Conv nodes with 1x1 kernel (candidate cls/box heads) ==")
    for n in g.node:
        if n.op_type != "Conv":
            continue
        ws = [inits[w].shape for w in n.input if w in inits]
        if ws and list(ws[0][-2:]) == [1, 1]:
            print(f"  {n.name:50s} W={tuple(ws[0])} in={n.input[0]} out={n.output[0]}")
            if len(n.input) > 2 and n.input[2] in inits:
                print("      bias", inits[n.input[2]].shape)

    print("\n== op type counts ==")
    from collections import Counter
    for k, v in sorted(Counter(n.op_type for n in g.node).items()):
        print(f"  {k:15s} {v}")


if __name__ == "__main__":
    main()
