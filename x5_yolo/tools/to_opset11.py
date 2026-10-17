#!/usr/bin/env python3
"""Downgrade the exported graph (opset 19) to opset 11 for hb_mapper / OE 1.2.8.

PyTorch's exporter emits Split-18 (split sizes as a second input); opset 11
expects them as a `split` attribute. Resize here already uses the 3-input
(X, roi, scales) form, which opset 11 accepts. Everything else is opset <= 11.
"""
import argparse

import numpy as np
import onnx
from onnx import numpy_helper


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True)
    ap.add_argument("--dst", required=True)
    args = ap.parse_args()

    m = onnx.load(args.src)
    inits = {i.name: numpy_helper.to_array(i) for i in m.graph.initializer}
    converted = 0
    for n in m.graph.node:
        if n.op_type != "Split" or len(n.input) < 2:
            continue
        sizes = inits[n.input[1]].astype("int64").tolist()
        attrs = [a for a in n.attribute if a.name != "split"]
        attrs.append(onnx.helper.make_attribute("split", sizes))
        del n.attribute[:]
        n.attribute.extend(attrs)
        del n.input[1:]
        converted += 1

    for opset in m.opset_import:
        opset.version = 11

    # opset-11 Resize requires a real (possibly empty) roi tensor, not "".
    roi_name = "resize_roi_empty"
    m.graph.initializer.append(
        numpy_helper.from_array(np.zeros((0,), dtype=np.float32), roi_name))
    for n in m.graph.node:
        if n.op_type == "Resize" and len(n.input) >= 2 and n.input[1] == "":
            n.input[1] = roi_name

    onnx.checker.check_model(m, full_check=True)
    onnx.save(m, args.dst)
    print(f"converted {converted} Split nodes; opset -> 11; saved {args.dst}")


if __name__ == "__main__":
    main()
