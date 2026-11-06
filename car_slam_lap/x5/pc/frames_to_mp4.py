#!/usr/bin/env python3
"""frames_to_mp4 —— 把 record_odin_rgb.py 落盘的 JPEG 帧合成成可双击播放的 mp4。

在 **PC（Windows）** 上而不是 X5 上：合成编码在这里做，不拖慢建图。

用法：
  python frames_to_mp4.py --frames E:\\RM\\...\\video\\frames --out E:\\RM\\...\\mapping.mp4
  python frames_to_mp4.py --frames ... --out ... --fps 5 --width 1280

依赖：opencv-python（本机已装 4.x）。

两个坑：
  1. H.264 要求宽高都是偶数，奇数高度会让编码器直接报 "Slice parameters ... invalid"，
     写出 0 字节；这里统一裁成偶数。
  2. opencv 自带 avc1 在缺 openh264 dll 时 isOpened() 仍返回 True，但写不出可读文件；
     所以写完用 VideoCapture 读回一帧校验，失败再换 mp4v。
"""
import argparse
import glob
import os
import sys

import cv2
import numpy as np

CANDIDATES = (("H.264", "avc1"), ("MPEG-4", "mp4v"))


def log(msg):
    print(msg, flush=True)


def load_frame(path):
    # Windows 下 cv2.imread 对中文路径不友好，用 imdecode
    buf = np.fromfile(path, dtype=np.uint8)
    return cv2.imdecode(buf, cv2.IMREAD_COLOR)


def even(value):
    return value - (value % 2)


def render(files, out_path, fps, size, first):
    """用指定编码器写 out_path；返回读回的帧数，失败返回 0。"""
    writer = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*size[2]), fps, size[:2])
    if not writer.isOpened():
        writer.release()
        return 0
    written = 0
    try:
        for i, path in enumerate(files, 1):
            img = first if i == 1 else load_frame(path)
            if img is None:
                continue
            if (img.shape[1], img.shape[0]) != size[:2]:
                img = cv2.resize(img, size[:2], interpolation=cv2.INTER_AREA)
            writer.write(img)
            written += 1
            if i % 500 == 0:
                log("  写入 %d/%d" % (i, len(files)))
    finally:
        writer.release()

    if not os.path.exists(out_path):
        return 0
    cap = cv2.VideoCapture(out_path)
    ok = False
    if cap.isOpened():
        ok, _ = cap.read()
    cap.release()
    return written if ok else 0


def main(argv=None):
    ap = argparse.ArgumentParser(description="JPEG 帧序列 -> mp4")
    ap.add_argument("--frames", required=True, help="frames/ 目录")
    ap.add_argument("--out", required=True, help="输出 .mp4")
    ap.add_argument("--fps", type=float, default=10.0, help="帧率，默认 10")
    ap.add_argument("--width", type=int, default=0, help="缩到指定宽度，0 = 原图")
    ap.add_argument("--limit", type=int, default=0, help="只处理前 N 帧（调试）")
    args = ap.parse_args(argv)

    files = sorted(glob.glob(os.path.join(args.frames, "frame_*.jpg")))
    if not files:
        log("[!] %s 里没有 frame_*.jpg" % args.frames)
        return 1
    if args.limit:
        files = files[: args.limit]
    log("帧数: %d  首帧: %s" % (len(files), os.path.basename(files[0])))

    first = load_frame(files[0])
    if first is None:
        log("[!] 首帧解不开: %s" % files[0])
        return 1
    h, w = even(first.shape[0]), even(first.shape[1])
    if args.width and args.width != w:
        w = even(args.width)
        h = even(int(round(first.shape[0] * w / first.shape[1])))
    if w != first.shape[1] or h != first.shape[0]:
        first = cv2.resize(first, (w, h), interpolation=cv2.INTER_AREA)
    log("分辨率: %dx%d" % (w, h))

    os.makedirs(os.path.dirname(os.path.abspath(args.out)) or ".", exist_ok=True)

    written = 0
    for name, tag in CANDIDATES:
        tmp = os.path.abspath(args.out + ".tmp.mp4")
        log("尝试编码器: %s (%s)" % (name, tag))
        written = render(files, tmp, args.fps, (w, h, tag), first)
        if written and os.path.exists(tmp):
            try:
                os.replace(tmp, args.out)
            except OSError as exc:
                log("[!] 替换输出失败: %s" % exc)
                return 1
            log("编码器可用: %s" % name)
            break
        log("  %s 写不出可读文件，换下一个" % name)
        if os.path.exists(tmp) and tmp != args.out:
            os.remove(tmp)

    if not written:
        log("[!] avc1 / mp4v 都不可用：opencv 缺编码器。")
        return 1

    size_mb = os.path.getsize(args.out) / 1e6
    log("[ok] %s  (%d 帧 @ %.2f fps, %.1f MB, 时长 %.1f s)"
        % (args.out, written, args.fps, size_mb, written / args.fps))
    return 0


if __name__ == "__main__":
    sys.exit(main())