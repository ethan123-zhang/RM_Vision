#!/usr/bin/env python3
"""Shared YOLOv8 person-only preprocessing / decode / NMS.

Model outputs are 6 raw-logit tensors in NCHW:
    box_80, cls_80, box_40, cls_40, box_20, cls_20
"""
import cv2
import numpy as np

INPUT_SIZE = 640
REG_MAX = 16
STRIDES = (8, 16, 32)


def letterbox(im, new_size=INPUT_SIZE, color=(114, 114, 114)):
    h, w = im.shape[:2]
    r = min(new_size / h, new_size / w)
    nw, nh = int(round(w * r)), int(round(h * r))
    if (nw, nh) != (w, h):
        im = cv2.resize(im, (nw, nh), interpolation=cv2.INTER_LINEAR)
    dw, dh = (new_size - nw) / 2, (new_size - nh) / 2
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    out = np.full((new_size, new_size, 3), color, dtype=im.dtype)
    out[top:top + nh, left:left + nw] = im
    return out, r, (left, top)


def preprocess(im):
    lb, r, pad = letterbox(im)
    x = lb[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0
    return np.ascontiguousarray(x), r, pad


def _dfl(box):
    """box [64,H,W] -> distances [4,N] in grid units."""
    c, h, w = box.shape
    b = box.reshape(4, REG_MAX, h * w).astype(np.float32, copy=True)
    b -= b.max(axis=1, keepdims=True)
    e = np.exp(b, out=b)
    e /= e.sum(axis=1, keepdims=True)
    return np.dot(e.transpose(0, 2, 1), np.arange(REG_MAX, dtype=np.float32))


def sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x))


def _chw(t, channels):
    """Normalise an ONNX output to CHW (accepts NCHW or NHWC, batch optional)."""
    a = np.asarray(t)
    if a.ndim == 4:
        a = a[0]
    if a.shape[0] == channels:
        return a
    if a.shape[-1] == channels:
        return np.ascontiguousarray(a.transpose(2, 0, 1))
    raise ValueError(f"cannot infer layout for shape {a.shape} with C={channels}")


def decode(outputs, conf=0.25, iou=0.45, orig_shape=None, ratio=1.0, pad=(0, 0),
           max_det=300):
    """Decode 6 raw-logit tensors (any order, NCHW or NHWC) into one-class boxes."""
    logit_thr = np.log(conf / (1.0 - conf))
    boxes_by_stride, cls_by_stride = {}, {}
    for t in outputs:
        a = np.asarray(t)
        if a.ndim == 4:
            a = a[0]
        if a.ndim != 3:
            continue
        if a.shape[0] in (1, 64) and a.shape[1] == a.shape[2] and a.shape[1] in (20, 40, 80):
            c, h = a.shape[0], a.shape[1]
        elif a.shape[-1] in (1, 64) and a.shape[0] == a.shape[1] and a.shape[0] in (20, 40, 80):
            h, c = a.shape[0], a.shape[-1]
            a = np.ascontiguousarray(a.transpose(2, 0, 1))
        else:
            continue
        (boxes_by_stride if c == 64 else cls_by_stride)[INPUT_SIZE // h] = a

    all_xyxy, all_score = [], []
    for s in STRIDES:
        if s not in boxes_by_stride or s not in cls_by_stride:
            raise ValueError(f"missing output for stride {s}")
        box = boxes_by_stride[s]
        cls = cls_by_stride[s].reshape(-1)
        keep = cls >= logit_thr
        if not keep.any():
            continue
        idx = np.flatnonzero(keep)
        d = _dfl(box)[:, idx]                       # [4,k]
        n = INPUT_SIZE // s
        g = (np.arange(n, dtype=np.float32) + 0.5) * s
        cx = np.tile(g, n)[idx]
        cy = np.repeat(g, n)[idx]
        all_xyxy.append(np.stack([cx - d[0] * s, cy - d[1] * s,
                                  cx + d[2] * s, cy + d[3] * s], 1))
        all_score.append(sigmoid(cls[idx].astype(np.float32)))

    if not all_xyxy:
        return np.zeros((0, 4), np.float32), np.zeros((0,), np.float32)

    xyxy = np.concatenate(all_xyxy).astype(np.float32)
    score = np.concatenate(all_score).astype(np.float32)

    # undo letterbox
    xyxy[:, [0, 2]] -= pad[0]
    xyxy[:, [1, 3]] -= pad[1]
    xyxy /= ratio
    if orig_shape is not None:
        h0, w0 = orig_shape[:2]
        xyxy[:, [0, 2]] = xyxy[:, [0, 2]].clip(0, w0)
        xyxy[:, [1, 3]] = xyxy[:, [1, 3]].clip(0, h0)

    keep = score >= conf
    xyxy, score = xyxy[keep], score[keep]
    if len(score) == 0:
        return xyxy, score

    if len(score) > 3000:
        sel = np.argpartition(-score, 3000)[:3000]
        xyxy, score = xyxy[sel], score[sel]

    idx = nms(xyxy, score, iou)
    return xyxy[idx], score[idx]


def nms(boxes, scores, iou_thres):
    if len(boxes) == 0:
        return np.zeros((0,), np.int64)
    x1, y1, x2, y2 = boxes.T
    areas = (x2 - x1) * (y2 - y1)
    order = scores.argsort()[::-1]
    keep = []
    while order.size:
        i = order[0]
        keep.append(i)
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])
        w = np.maximum(0.0, xx2 - xx1)
        h = np.maximum(0.0, yy2 - yy1)
        inter = w * h
        ovr = inter / (areas[i] + areas[order[1:]] - inter + 1e-9)
        order = order[1:][ovr <= iou_thres]
    return np.asarray(keep, dtype=np.int64)
