#!/usr/bin/env python3
"""RDK X5 单类 person YOLOv8n 板端推理 / 测速（v2 优化预处理）.

依赖板端镜像自带的 hobot_dnn (pyeasy_dnn)。

用法:
  python3 yolov8_person1.py --model xxx.bin --img bus.jpg --bench 200
  python3 yolov8_person1.py --model xxx.bin --img bus.jpg --bench 200 --pipeline
  python3 yolov8_person1.py --model xxx.bin --img bus.jpg --profile
"""
import argparse
import queue
import threading
import time

import cv2
import numpy as np

try:
    from hobot_dnn import pyeasy_dnn as dnn
except ImportError as e:  # pragma: no cover
    raise SystemExit("board image has no hobot_dnn.pyeasy_dnn: %s" % e)

INPUT = 640
STRIDES = (8, 16, 32)
REG_MAX = 16
PAD_COLOR = (114, 114, 114)

PROFILE = {"resize": 0.0, "cvt": 0.0, "pack": 0.0}


def letterbox_nv12(img, size=INPUT):
    """BGR -> letterboxed 640x640 NV12. Returns (nv12, ratio, (left, top))."""
    h, w = img.shape[:2]
    r = min(size / h, size / w)
    nw, nh = int(round(w * r)), int(round(h * r))
    dw, dh = (size - nw) / 2, (size - nh) / 2

    t0 = time.perf_counter()
    resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_NEAREST)
    canvas = np.full((size, size, 3), PAD_COLOR, np.uint8)
    left, top = int(round(dw - 0.1)), int(round(dh - 0.1))
    canvas[top:top + nh, left:left + nw] = resized
    t1 = time.perf_counter()

    yuv = cv2.cvtColor(canvas, cv2.COLOR_BGR2YUV_I420).reshape(-1)
    t2 = time.perf_counter()

    area = size * size
    nv12 = np.empty(area * 3 // 2, np.uint8)
    nv12[:area] = yuv[:area]
    nv12[area:] = yuv[area:].reshape(2, area // 4).T.reshape(-1)
    t3 = time.perf_counter()

    PROFILE["resize"] += t1 - t0
    PROFILE["cvt"] += t2 - t1
    PROFILE["pack"] += t3 - t2
    return nv12, r, (left, top)


def _dfl(box):
    b = box.reshape(-1, 4, REG_MAX).astype(np.float32)
    b -= b.max(axis=2, keepdims=True)
    e = np.exp(b)
    e /= e.sum(axis=2, keepdims=True)
    return (e * np.arange(REG_MAX, dtype=np.float32)).sum(axis=2)


def nms(boxes, scores, iou, topk=3000):
    if len(scores) == 0:
        return np.zeros((0,), np.int64)
    if len(scores) > topk:
        sel = np.argpartition(-scores, topk)[:topk]
        boxes, scores = boxes[sel], scores[sel]
    idx = cv2.dnn.NMSBoxes(boxes.tolist(), scores.tolist(), 0.0, iou)
    return np.asarray(idx).reshape(-1).astype(np.int64)


class PersonYOLO:
    def __init__(self, model_path, conf=0.25, iou=0.45):
        t0 = time.time()
        self.model = dnn.load(model_path)[0]
        print(f"load model: {1000*(time.time()-t0):.1f} ms")
        print("inputs:")
        for i, t in enumerate(self.model.inputs):
            print(f"  [{i}] {t.name} {t.properties.dtype} {tuple(t.properties.shape)}")
        print("outputs:")
        for i, t in enumerate(self.model.outputs):
            print(f"  [{i}] {t.name} {t.properties.dtype} {tuple(t.properties.shape)}")
        self.conf = conf
        self.iou = iou
        self.logit_thr = float(np.log(conf / (1.0 - conf)))

    def preprocess(self, img):
        return letterbox_nv12(img)

    def forward(self, nv12):
        return self.model.forward(nv12)

    def postprocess(self, outputs, ratio, pad, orig_shape):
        boxes, clses = {}, {}
        for t in outputs:
            shp = tuple(int(x) for x in t.properties.shape)
            a = np.asarray(t.buffer).reshape(shp)
            if shp[-1] in (1, 64):
                c, h = shp[-1], shp[1]
            else:
                c, h = shp[1], shp[2]
            if a.dtype != np.float32:
                a = a.astype(np.float32)
                sc = getattr(t.properties, "scale_data", None)
                if sc is not None:
                    a = a * np.asarray(sc, np.float32).reshape(1, 1, 1, -1)
            (boxes if c == 64 else clses)[INPUT // h] = a[0]

        xyxy, score = [], []
        for s in STRIDES:
            b = boxes[s].reshape(-1, 64)
            cl = clses[s].reshape(-1)
            idx = np.flatnonzero(cl >= self.logit_thr)
            if len(idx) == 0:
                continue
            d = _dfl(b[idx])
            n = INPUT // s
            g = (np.arange(n, dtype=np.float32) + 0.5) * s
            cx = np.tile(g, n)[idx]
            cy = np.repeat(g, n)[idx]
            xyxy.append(np.stack([cx - d[:, 0] * s, cy - d[:, 1] * s,
                                  cx + d[:, 2] * s, cy + d[:, 3] * s], 1))
            score.append(1.0 / (1.0 + np.exp(-cl[idx])))

        if not xyxy:
            return np.zeros((0, 4), np.float32), np.zeros((0,), np.float32)
        xyxy = np.concatenate(xyxy).astype(np.float32)
        score = np.concatenate(score).astype(np.float32)

        xyxy[:, [0, 2]] -= pad[0]
        xyxy[:, [1, 3]] -= pad[1]
        xyxy /= ratio
        h0, w0 = orig_shape[:2]
        xyxy[:, [0, 2]] = xyxy[:, [0, 2]].clip(0, w0)
        xyxy[:, [1, 3]] = xyxy[:, [1, 3]].clip(0, h0)

        keep = score >= self.conf
        xyxy, score = xyxy[keep], score[keep]
        if len(score) == 0:
            return xyxy, score
        idx = nms(xyxy, score, self.iou)
        return xyxy[idx][:300], score[idx][:300]


def draw(img, boxes, scores):
    for (x1, y1, x2, y2), s in zip(boxes, scores):
        cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (56, 56, 255), 2)
        cv2.putText(img, f"person {s:.2f}", (int(x1), max(0, int(y1) - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)


def bench_serial(m, img, n):
    tp = ti = tq = 0.0
    for _ in range(n):
        t0 = time.perf_counter(); nv12, r, pad = m.preprocess(img)
        t1 = time.perf_counter(); outs = m.forward(nv12)
        t2 = time.perf_counter(); m.postprocess(outs, r, pad, img.shape)
        t3 = time.perf_counter()
        tp += t1 - t0; ti += t2 - t1; tq += t3 - t2
    print(f"serial x{n}: pre {1000*tp/n:.2f} | bpu {1000*ti/n:.2f} | "
          f"post {1000*tq/n:.2f} | total {1000*(tp+ti+tq)/n:.2f} ms | "
          f"{n/(tp+ti+tq):.1f} FPS")


def bench_pipeline(m, img, n):
    """One producer (pre) + main (bpu+post): overlaps 1080p resize with BPU."""
    q = queue.Queue(maxsize=4)

    def producer():
        for _ in range(n):
            q.put(m.preprocess(img))

    th = threading.Thread(target=producer, daemon=True)
    th.start()
    t0 = time.perf_counter()
    for _ in range(n):
        nv12, r, pad = q.get()
        outs = m.forward(nv12)
        m.postprocess(outs, r, pad, img.shape)
    dt = time.perf_counter() - t0
    th.join()
    print(f"pipeline x{n}: {1000*dt/n:.2f} ms/frame | {n/dt:.1f} FPS")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True)
    ap.add_argument("--img", required=True)
    ap.add_argument("--save", default="person_result.jpg")
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--iou", type=float, default=0.45)
    ap.add_argument("--bench", type=int, default=0)
    ap.add_argument("--pipeline", action="store_true")
    ap.add_argument("--profile", action="store_true")
    args = ap.parse_args()

    m = PersonYOLO(args.model, args.conf, args.iou)
    img = cv2.imread(args.img)
    if img is None:
        raise SystemExit(f"cannot read {args.img}")
    print("image", img.shape)

    nv12, ratio, pad = m.preprocess(img)
    outs = m.forward(nv12)
    t0 = time.perf_counter()
    boxes, scores = m.postprocess(outs, ratio, pad, img.shape)
    print(f"detections: {len(boxes)}, postprocess {1000*(time.perf_counter()-t0):.2f} ms")
    for b, s in zip(boxes, scores):
        print("  ", [int(v) for v in b], f"{s:.3f}")
    draw(img, boxes, scores)
    cv2.imwrite(args.save, img)
    print("saved", args.save)

    if args.profile:
        for k in ("resize", "cvt", "pack"):
            print(f"profile {k}: {1000*PROFILE[k]:.2f} ms (n=1)")
    if args.bench:
        if args.pipeline:
            bench_pipeline(m, img, args.bench)
        else:
            bench_serial(m, img, args.bench)


if __name__ == "__main__":
    main()
