#!/usr/bin/env python3
"""Probe a V4L2 camera: negotiated format + pure cap.read() cost (no model).

Usage:
  python3 cam_probe.py --camera 0
"""
import argparse
import time

import cv2

MODES = [
    ("MJPG", 1920, 1080, 30),
    ("MJPG", 1280, 720, 60),
    ("MJPG", 640, 480, 120),
    ("YUYV", 640, 480, 30),
    ("YUYV", 320, 240, 60),
]


def probe(idx, fourcc, w, h, fps, frames):
    cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
    if not cap.isOpened():
        return None
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_FPS, fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    aw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    ah = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    af = cap.get(cv2.CAP_PROP_FPS)

    for _ in range(5):
        cap.read()
    n = frames
    okc = 0
    t0 = time.perf_counter()
    for _ in range(n):
        ok, _ = cap.read()
        okc += 1 if ok else 0
    dt = time.perf_counter() - t0
    cap.release()
    return (fourcc, w, h, fps, aw, ah, af, okc, n, dt)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--frames", type=int, default=60)
    args = ap.parse_args()

    print(f"{'request':28s} {'negotiated':18s} {'read ms':>8s} {'fps':>6s} ok")
    for fourcc, w, h, fps in MODES:
        r = probe(args.camera, fourcc, w, h, fps, args.frames)
        if r is None:
            print(f"{fourcc} {w}x{h}@{fps}: open failed")
            continue
        fourcc, w, h, fps, aw, ah, af, okc, n, dt = r
        print(f"{fourcc:4s} {w:4d}x{h:<4d}@{fps:<3d}          "
              f"{aw:4d}x{ah:<4d}@{af:5.1f}   {1000*dt/n:8.1f} {n/dt:6.1f} {okc}/{n}")


if __name__ == "__main__":
    main()
