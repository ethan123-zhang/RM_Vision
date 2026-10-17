#!/usr/bin/env python3
"""Generate X5 hb_mapper calibration tensors (.rgbchw, float32 RGB CHW 0-255).

Matches the official RDK X5 YOLO conversion protocol:
  input_type_train: rgb / NCHW, norm_type: data_scale 1/255,
  calibration data stored un-normalised (0..255), stretch-resized to 640.
"""
import argparse
import glob
import os

import cv2
import numpy as np


def person_count(label_path):
    if not os.path.exists(label_path):
        return 0
    n = 0
    for line in open(label_path):
        p = line.split()
        if len(p) >= 5 and int(float(p[0])) == 0:
            n += 1
    return n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--images", required=True)
    ap.add_argument("--labels", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--num", type=int, default=32)
    ap.add_argument("--size", type=int, default=640)
    ap.add_argument("--pad", type=int, default=127)
    ap.add_argument("--letterbox", action="store_true",
                    help="letterbox instead of the official stretch resize")
    args = ap.parse_args()

    files = sorted(glob.glob(os.path.join(args.images, "*.jpg")))
    ranked = []
    for f in files:
        stem = os.path.splitext(os.path.basename(f))[0]
        ranked.append((person_count(os.path.join(args.labels, stem + ".txt")), f))
    ranked.sort(key=lambda x: (-x[0], x[1]))
    pool = [f for n, f in ranked if n > 0] or [f for _, f in ranked]
    if len(pool) > args.num:
        idx = np.linspace(0, len(pool) - 1, args.num).round().astype(int)
        pool = [pool[i] for i in sorted(set(idx.tolist()))]

    os.makedirs(args.out, exist_ok=True)
    names = []
    for f in pool:
        im = cv2.imread(f)
        if im is None:
            continue
        im = cv2.cvtColor(im, cv2.COLOR_BGR2RGB)
        h, w = im.shape[:2]
        if args.letterbox:
            r = min(args.size / h, args.size / w)
            nw, nh = int(round(w * r)), int(round(h * r))
            im = cv2.resize(im, (nw, nh), interpolation=cv2.INTER_LINEAR)
            canvas = np.full((args.size, args.size, 3), args.pad, np.uint8)
            top, left = (args.size - nh) // 2, (args.size - nw) // 2
            canvas[top:top + nh, left:left + nw] = im
            im = canvas
        else:
            im = cv2.resize(im, (args.size, args.size), interpolation=cv2.INTER_LINEAR)
        tensor = np.transpose(im, (2, 0, 1)).astype(np.float32)
        name = os.path.splitext(os.path.basename(f))[0] + ".rgbchw"
        tensor.tofile(os.path.join(args.out, name))
        names.append(name)

    with open(os.path.join(args.out, "calib_list.txt"), "w") as fp:
        fp.write("\n".join(names) + "\n")
    total = sum(os.path.getsize(os.path.join(args.out, n)) for n in names)
    print(f"wrote {len(names)} calibration tensors to {args.out} "
          f"({total / 1e6:.1f} MB), {args.size}x{args.size}, "
          f"{'letterbox' if args.letterbox else 'stretch'}")


if __name__ == "__main__":
    main()
