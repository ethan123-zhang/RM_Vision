#!/usr/bin/env python3
"""Person-only AP@0.5 on the 494-image COCO val subset.

Compares original 80-class model (person = class 0) with the 1-class surgery.
"""
import argparse
import glob
import os
import sys

import cv2
import numpy as np
import onnxruntime as ort

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import yolo_person  # noqa: E402
from yolo_person import INPUT_SIZE, decode, letterbox, nms  # noqa: E402

CONF = 0.001
IOU = 0.45


def session(path, threads=8):
    so = ort.SessionOptions()
    so.intra_op_num_threads = threads
    so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    return ort.InferenceSession(path, so, providers=["CPUExecutionProvider"])


def preprocess(im, size=INPUT_SIZE):
    lb, r, pad = letterbox(im, size)
    x = lb[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0
    return np.ascontiguousarray(x), r, pad


def xywh2xyxy(b):
    out = b.copy()
    out[:, 0] = b[:, 0] - b[:, 2] / 2
    out[:, 1] = b[:, 1] - b[:, 3] / 2
    out[:, 2] = b[:, 0] + b[:, 2] / 2
    out[:, 3] = b[:, 1] + b[:, 3] / 2
    return out


def unletterbox(b, r, pad, shape):
    b = b.copy()
    b[:, [0, 2]] -= pad[0]
    b[:, [1, 3]] -= pad[1]
    b /= r
    h, w = shape[:2]
    b[:, [0, 2]] = b[:, [0, 2]].clip(0, w)
    b[:, [1, 3]] = b[:, [1, 3]].clip(0, h)
    return b


def dets_orig(out, r, pad, shape):
    o = out[0]
    s = o[4]
    keep = s >= CONF
    b = xywh2xyxy(o[:4].T[keep])
    s = s[keep]
    b = unletterbox(b, r, pad, shape)
    idx = nms(b, s, IOU)
    return b[idx][:300], s[idx][:300]


def dets_person(outs, r, pad, shape):
    b, s = decode(outs, CONF, IOU, shape, r, pad)
    return b[:300], s[:300]


def load_gt(path, w, h):
    g = []
    if not os.path.exists(path):
        return np.zeros((0, 4), np.float32)
    for line in open(path):
        p = line.split()
        if len(p) < 5 or int(float(p[0])) != 0:
            continue
        cx, cy, bw, bh = map(float, p[1:5])
        g.append([(cx - bw / 2) * w, (cy - bh / 2) * h,
                  (cx + bw / 2) * w, (cy + bh / 2) * h])
    return np.asarray(g, np.float32).reshape(-1, 4)


def iou_matrix(a, b):
    if len(a) == 0 or len(b) == 0:
        return np.zeros((len(a), len(b)), np.float32)
    lt = np.maximum(a[:, None, :2], b[None, :, :2])
    rb = np.minimum(a[:, None, 2:], b[None, :, 2:])
    wh = np.clip(rb - lt, 0, None)
    inter = wh[..., 0] * wh[..., 1]
    area_a = (a[:, 2] - a[:, 0]) * (a[:, 3] - a[:, 1])
    area_b = (b[:, 2] - b[:, 0]) * (b[:, 3] - b[:, 1])
    return inter / (area_a[:, None] + area_b[None, :] - inter + 1e-9)


def evaluate(dets, gts, npos):
    """dets: list of (boxes, scores); gts: list of boxes."""
    records = []
    for i, (b, s) in enumerate(dets):
        for j in range(len(s)):
            records.append((float(s[j]), i, b[j]))
    records.sort(key=lambda x: -x[0])
    matched = [np.zeros(len(g), bool) for g in gts]
    tp = np.zeros(len(records), np.float32)
    fp = np.zeros(len(records), np.float32)
    for k, (_, i, box) in enumerate(records):
        g = gts[i]
        if len(g) == 0:
            fp[k] = 1
            continue
        ious = iou_matrix(box[None], g)[0]
        ious[matched[i]] = -1
        j = int(ious.argmax())
        if ious[j] >= 0.5:
            matched[i][j] = True
            tp[k] = 1
        else:
            fp[k] = 1
    if npos == 0:
        return 0.0
    ctp, cfp = np.cumsum(tp), np.cumsum(fp)
    rec = ctp / npos
    prec = ctp / np.maximum(ctp + cfp, 1e-9)
    # all-point interpolation
    mrec = np.concatenate([[0], rec, [1]])
    mpre = np.concatenate([[0], prec, [0]])
    for i in range(len(mpre) - 2, -1, -1):
        mpre[i] = max(mpre[i], mpre[i + 1])
    idx = np.where(mrec[1:] != mrec[:-1])[0]
    return float(np.sum((mrec[idx + 1] - mrec[idx]) * mpre[idx + 1]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--orig", default=r"E:\RM\x5_yolo\model\yolov8n.onnx")
    ap.add_argument("--person", default=r"E:\RM\x5_yolo\model\yolov8n_person1.onnx")
    ap.add_argument("--data", default=r"E:\RM\_probe\precision_analysis\data")
    ap.add_argument("--size", type=int, default=640, help="person model input size")
    ap.add_argument("--limit", type=int, default=0)
    args = ap.parse_args()

    yolo_person.INPUT_SIZE = args.size
    imgs = sorted(glob.glob(os.path.join(args.data, "images", "*.jpg")))
    if args.limit:
        imgs = imgs[: args.limit]
    so, sp = session(args.orig), session(args.person)
    iname = so.get_inputs()[0].name

    gts, d_orig, d_per = [], [], []
    npos = 0
    for k, f in enumerate(imgs):
        im = cv2.imread(f)
        if im is None:
            continue
        h, w = im.shape[:2]
        x, r, pad = preprocess(im)
        xp, rp, padp = preprocess(im, args.size)
        gt = load_gt(os.path.join(args.data, "labels",
                                  os.path.splitext(os.path.basename(f))[0] + ".txt"), w, h)
        gts.append(gt)
        npos += len(gt)
        d_orig.append(dets_orig(so.run(None, {iname: x})[0], r, pad, (h, w)))
        d_per.append(dets_person(sp.run(None, {iname: xp}), rp, padp, (h, w)))
        if (k + 1) % 50 == 0:
            print(f"  {k+1}/{len(imgs)} images, gt={npos}", flush=True)

    ap_o = evaluate(d_orig, gts, npos)
    ap_p = evaluate(d_per, gts, npos)
    print(f"\nimages={len(imgs)}  person GT={npos}")
    print(f"  original 80-class, person AP@0.5 = {ap_o:.4f}")
    print(f"  1-class surgery    AP@0.5 = {ap_p:.4f}")
    print(f"  delta = {ap_p - ap_o:+.4f}")


if __name__ == "__main__":
    main()
