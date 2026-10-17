#!/usr/bin/env python3
"""Numerical parity: original 80-class YOLOv8n vs 1-class person head.

Compares, on every one of the 8400 anchors:
  - person score  : sigmoid(person cls logit)  vs  original output[:, 4]
  - box (xywh)    : DFL(person box logit)      vs  original output[:, 0:4]
"""
import argparse
import glob
import os
import sys

import cv2
import numpy as np
import onnxruntime as ort

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from yolo_person import INPUT_SIZE, STRIDES, _dfl, preprocess, sigmoid  # noqa: E402


def session(path, threads=8):
    so = ort.SessionOptions()
    so.intra_op_num_threads = threads
    so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    return ort.InferenceSession(path, so, providers=["CPUExecutionProvider"])


def person_raw(outs):
    scores, xywh = [], []
    for i, s in enumerate(STRIDES):
        n = INPUT_SIZE // s
        box = np.asarray(outs[2 * i])[0].astype(np.float32)
        logit = np.asarray(outs[2 * i + 1])[0].reshape(-1).astype(np.float32)
        d = _dfl(box)                                  # [4, n*n]
        g = (np.arange(n, dtype=np.float32) + 0.5) * s
        ax = np.tile(g, n)
        ay = np.repeat(g, n)
        # box center = anchor + (right-left)/2, in pixels
        cx = ax + (d[2] - d[0]) / 2 * s
        cy = ay + (d[3] - d[1]) / 2 * s
        scores.append(logit)
        xywh.append(np.stack([cx, cy, (d[0] + d[2]) * s, (d[1] + d[3]) * s], 1))
    return np.concatenate(scores), np.concatenate(xywh)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--orig", default=r"E:\RM\x5_yolo\model\yolov8n.onnx")
    ap.add_argument("--person", default=r"E:\RM\x5_yolo\model\yolov8n_person1.onnx")
    ap.add_argument("--images", default=r"E:\RM\_probe\precision_analysis\data\images")
    ap.add_argument("--limit", type=int, default=20)
    args = ap.parse_args()

    files = sorted(glob.glob(os.path.join(args.images, "*.jpg")))[: args.limit]
    assert files, "no images"
    so, sp = session(args.orig), session(args.person)
    iname = so.get_inputs()[0].name

    max_ds = max_db = 0.0
    bad = 0
    for k, f in enumerate(files):
        im = cv2.imread(f)
        x, _, _ = preprocess(im)
        o = so.run(None, {iname: x})[0][0]             # [84,8400]
        outs = sp.run(None, {iname: x})
        sc, xywh = person_raw(outs)

        ds = float(np.abs(sigmoid(sc) - o[4]).max())
        db = float(np.abs(xywh - o[:4].T).max())
        max_ds, max_db = max(max_ds, ds), max(max_db, db)
        n_pos = int((o[4] > 0.25).sum())
        if ds > 1e-3 or db > 0.05:
            bad += 1
        if k < 5 or ds > 1e-3:
            print(f"  [{k:3d}] {os.path.basename(f):24s} max|dscore|={ds:.2e} "
                  f"max|dbox|={db:.2e} orig_person>{0.25}: {n_pos}")

    print(f"\nimages={len(files)}  max score diff={max_ds:.3e}  "
          f"max box diff={max_db:.3e} px  bad={bad}")


if __name__ == "__main__":
    main()
