#!/usr/bin/env python3
"""Rewrite a size-agnostic YOLO ONNX input from 640 to another square size."""
import argparse

import onnx


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True)
    ap.add_argument("--dst", required=True)
    ap.add_argument("--size", type=int, default=512)
    args = ap.parse_args()

    m = onnx.load(args.src)
    del m.graph.value_info[:]
    inp = m.graph.input[0]
    dims = inp.type.tensor_type.shape.dim
    dims[2].dim_value = args.size
    dims[3].dim_value = args.size
    m = onnx.shape_inference.infer_shapes(m)
    # ONNX keeps the old declared output shapes; rewrite them for the new size.
    for o in m.graph.output:
        name = o.name
        suffix = int(name.split("_")[-1])      # 80/40/20 = 640-size feature map
        stride = 640 // suffix                 # 8/16/32
        n = args.size // stride
        c = 1 if name.startswith("cls") else 64
        dims = o.type.tensor_type.shape.dim
        dims[0].dim_value = 1
        dims[1].dim_value = n
        dims[2].dim_value = n
        dims[3].dim_value = c
    onnx.checker.check_model(m)
    onnx.save(m, args.dst)
    print("saved", args.dst)
    for o in m.graph.output:
        print("  ", o.name, [d.dim_value or d.dim_param for d in o.type.tensor_type.shape.dim])


if __name__ == "__main__":
    main()
