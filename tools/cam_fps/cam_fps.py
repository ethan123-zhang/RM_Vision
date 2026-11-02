#!/usr/bin/env python3
"""Windows USB (UVC) camera frame-rate tester.

Modes:
  list    enumerate DirectShow devices: index, name, default format
  scan    sweep resolution x fps request x pixel format, measure real fps
  single  one config, long run, detailed interval stats (+ --show / --csv)

Only dependency: cv2 (OpenCV).  Measurement never trusts CAP_PROP_FPS: the
driver-reported value is printed as "negotiated" and looks can lie; the real
number is computed from wall-clock frame arrival timestamps.

Examples:
  python tools/cam_fps/cam_fps.py --list
  python tools/cam_fps/cam_fps.py --device ZSKJ --mode scan --fps 120,60,30
  python tools/cam_fps/cam_fps.py --device ZSKJ --mode single \
      --width 1280 --height 720 --fps 60 --fourcc MJPG --seconds 10 --csv out.csv
"""

import argparse
import ctypes
import statistics
import sys
import threading
import time
from ctypes import POINTER, byref, c_ubyte, c_ulong, c_ushort, c_void_p, c_wchar_p

import cv2

BACKENDS = {"dshow": cv2.CAP_DSHOW, "msmf": cv2.CAP_MSMF}

# Common UVC modes, largest first.  Presets are only *requests*; each driver
# negotiates what it actually supports (reported back in the table).
DEFAULT_RESOLUTIONS = [
    "3840x2160", "2560x1440", "1920x1080", "1280x720", "960x540",
    "848x480", "800x600", "640x480", "640x360", "320x240",
]

HELP_EPILOG = """
columns:
  req res / fps : what the script asked the driver for
  neg res / fps : what the driver negotiated (CAP_PROP_*, can be 0 or a lie)
  fps           : REAL frames/second measured from arrival timestamps
  read ms       : mean cost of one cap.read() call (decode+copy)
  p95 gap ms    : 95th percentile of inter-frame gaps
  gaps          : gaps > 1.5x the median gap (stalls / dropped frames)
  verdict       : OK >=95% of requested, LOW >=50%, FAIL = no frame at all
"""


# --------------------------------------------------------------------------
# DirectShow device enumeration (same ICreateDevEnum OpenCV's DSHOW uses, so
# the index order matches cv2.VideoCapture(index, CAP_DSHOW)).
# --------------------------------------------------------------------------
class _GUID(ctypes.Structure):
    _fields_ = [("Data1", c_ulong), ("Data2", c_ushort),
                ("Data3", c_ushort), ("Data4", c_ubyte * 8)]

    def __init__(self, text):
        super().__init__()
        _ole32().CLSIDFromString(c_wchar_p(text), byref(self))


class _VARIANT(ctypes.Structure):
    _fields_ = [("vt", c_ushort), ("r1", c_ushort), ("r2", c_ushort),
                ("r3", c_ushort), ("val", c_ubyte * 16)]


def _ole32():
    return ctypes.WinDLL("ole32")


def _oleaut32():
    return ctypes.WinDLL("oleaut32")


def _com_call(pobj, index, argtypes, *args, restype=ctypes.c_long):
    vtbl = ctypes.cast(pobj, POINTER(POINTER(c_void_p)))[0]
    proto = ctypes.WINFUNCTYPE(restype, c_void_p, *argtypes)
    return proto(vtbl[index])(pobj, *args)


def _release(pobj):
    try:
        _com_call(pobj, 2, [])
    except Exception:
        pass


def _read_bag_string(bag, name):
    var = _VARIANT()
    hr = _com_call(bag, 3, [c_wchar_p, POINTER(_VARIANT), c_void_p],
                   c_wchar_p(name), byref(var), None)
    try:
        if hr == 0 and var.vt == 8:  # VT_BSTR
            ptr = ctypes.cast(byref(var.val), POINTER(c_void_p)).contents
            return ctypes.wstring_at(ptr.value) if ptr.value else None
        return None
    finally:
        _oleaut32().VariantClear(byref(var))


def list_dshow_devices():
    """Return [{name, path}] in DirectShow (== OpenCV DSHOW) index order."""
    ole32 = _ole32()
    ole32.CoInitializeEx(None, 2)  # COINIT_APARTMENTTHREADED
    try:
        clsid = _GUID("{62BE5D10-60EB-11D0-BD3B-00A0C911CE86}")  # SystemDeviceEnum
        iid = _GUID("{29840822-5B84-11D0-BD3B-00A0C911CE86}")    # ICreateDevEnum
        cat = _GUID("{860BB310-5D01-11D0-BD3B-00A0C911CE86}")    # VideoInputDevice
        iid_bag = _GUID("{55272A00-42CB-11CE-8135-00AA004BB851}")  # IPropertyBag

        dev_enum = c_void_p()
        if ole32.CoCreateInstance(byref(clsid), None, 1, byref(iid),
                                  byref(dev_enum)) != 0:
            return []
        enum_moniker = c_void_p()
        if _com_call(dev_enum, 3, [POINTER(_GUID), POINTER(c_void_p), c_ulong],
                     byref(cat), byref(enum_moniker), 0) != 0:
            return []

        devices = []
        while True:
            moniker = c_void_p()
            fetched = c_ulong(0)
            hr = _com_call(enum_moniker, 3,
                           [c_ulong, POINTER(c_void_p), POINTER(c_ulong)],
                           1, byref(moniker), byref(fetched))
            if hr != 0 or fetched.value != 1:
                break
            bag = c_void_p()
            hr = _com_call(moniker, 9,
                           [c_void_p, c_void_p, POINTER(_GUID), POINTER(c_void_p)],
                           None, None, byref(iid_bag), byref(bag))
            if hr == 0:
                devices.append({"name": _read_bag_string(bag, "FriendlyName") or "",
                                "path": _read_bag_string(bag, "DevicePath") or ""})
                _release(bag)
            _release(moniker)

        _release(enum_moniker)
        _release(dev_enum)
        return devices
    finally:
        ole32.CoUninitialize()


def resolve_camera(args):
    """Pick a device index from --camera or by name fragment from --device."""
    devices = []
    if args.device:
        try:
            devices = list_dshow_devices()
        except Exception as exc:  # enumeration is a convenience, never fatal
            print(f"[warn] device-name enumeration failed: {exc}")
        needle = args.device.lower()
        matches = [i for i, d in enumerate(devices) if needle in d["name"].lower()]
        if not matches:
            sys.exit(f"[error] no DirectShow device name contains {args.device!r}. "
                     f"Try --list.")
        if len(matches) > 1:
            sys.exit(f"[error] {args.device!r} matches indices {matches}; "
                     f"be more specific or use --camera.")
        args.camera = matches[0]
        print(f"[info] --device {args.device!r} -> index {args.camera} "
              f"({devices[args.camera]['name']})")
    return args.camera


# --------------------------------------------------------------------------
# Capture + measurement
# --------------------------------------------------------------------------
def open_capture(index, backend, attempts=3, delay=0.6):
    """Open a capture; retry a few times because UVC drivers can wedge after
    rapid open/close cycles and need a moment."""
    for attempt in range(attempts):
        cap = cv2.VideoCapture(index, BACKENDS[backend])
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # ignored by DSHOW
            return cap
        cap.release()
        time.sleep(delay * (attempt + 1))
    return None


def open_any(index, backends):
    """Try backends in order; return (cap, backend_name) or (None, None)."""
    for backend in backends:
        cap = open_capture(index, backend)
        if cap is not None:
            return cap, backend
    return None, None


def apply_format(cap, width, height, fps, fourcc):
    if fourcc:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    neg_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    neg_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    neg_f = cap.get(cv2.CAP_PROP_FPS)
    raw = int(cap.get(cv2.CAP_PROP_FOURCC)) & 0xFFFFFFFF
    chars = [chr((raw >> (8 * i)) & 0xFF) for i in range(4)]
    neg_fourcc = "".join(chars) if all(32 <= ord(c) < 127 for c in chars) else "n/a"
    return neg_w, neg_h, neg_f, neg_fourcc


def measure_stream(cap, seconds, warmup, read_timeout):
    """Read frames for `seconds` wall-clock; return timing + duplicate stats.

    Duplicate-frame detection matters on Windows: when a driver rejects a
    requested mode it may keep handing back the same buffered frame, which
    inflates the naive count into a fake "176 fps".  A cheap 8x8 signature
    lets us report unique frames separately.

    A watchdog thread does the blocking reads so a wedged driver cannot hang
    the whole sweep: after `read_timeout` past the deadline the thread is
    abandoned and the caller releases the capture.
    """
    for _ in range(warmup):
        ok, _ = cap.read()
        if not ok:
            return {"frames": 0, "fail": "warmup read failed"}

    stamps = []          # (arrival timestamp, read call duration)
    shapes = set()
    sigs = []
    stop = threading.Event()

    def loop():
        while not stop.is_set():
            t0 = time.perf_counter()
            ok, frame = cap.read()
            t1 = time.perf_counter()
            if not ok:
                stop.set()
                return
            shapes.add((frame.shape[1], frame.shape[0]))
            # Sparse full-res sampling keeps sensor noise, unlike a downscale:
            # a 8x8 INTER_AREA signature averages noise away and flags every
            # frame of a static scene as a duplicate.
            h, w = frame.shape[:2]
            sig = frame[::max(1, h // 12), ::max(1, w // 12)][:12, :12].tobytes()
            sigs.append(sig)
            stamps.append((t1, t1 - t0))

    worker = threading.Thread(target=loop, daemon=True)
    worker.start()
    time.sleep(seconds)
    stop.set()
    worker.join(read_timeout)

    if len(stamps) < 2:
        return {"frames": len(stamps),
                "fail": "camera stalled" if worker.is_alive() else "only 0-1 frames"}

    times = [s[0] for s in stamps]
    gaps = [(times[i + 1] - times[i]) * 1000.0 for i in range(len(times) - 1)]
    median_gap = statistics.median(gaps)
    dups = sum(1 for i in range(1, len(sigs)) if sigs[i] and sigs[i] == sigs[i - 1])
    return {
        "frames": len(stamps),
        "fps": (len(times) - 1) / (times[-1] - times[0]),
        "uniq_fps": (len(stamps) - 1 - dups) / (times[-1] - times[0]),
        "dups": dups,
        "shape": next(iter(shapes)) if len(shapes) == 1 else None,
        "shapes": shapes,
        "read_ms": statistics.mean(s[1] for s in stamps) * 1000.0,
        "p95_gap_ms": sorted(gaps)[int(len(gaps) * 0.95) - 1] if gaps else 0.0,
        "max_gap_ms": max(gaps) if gaps else 0.0,
        "gaps": sum(1 for g in gaps if g > 1.5 * median_gap),
        "gaps_full": gaps,
    }


def verdict(fps, requested_fps):
    if not fps:
        return "FAIL"
    if fps >= 0.95 * requested_fps:
        return "OK"
    if fps >= 0.5 * requested_fps:
        return "LOW"
    return "LOW"


def fmt_row(rec):
    exact = "yes" if rec.get("exact") else "NO"
    if rec["fail"]:
        return (f"{rec['fourcc']:5s} {rec['backend']:5s} {rec['req_res']:10s}"
                f"@{rec['req_fps']:<4d} {rec['neg_res']:10s}@{rec['neg_fps']:<5.1f}"
                f" {'-':>7s} {'-':>8s} {'-':>5s} {exact:5s}  {rec['fail']}")
    return (f"{rec['fourcc']:5s} {rec['backend']:5s} {rec['req_res']:10s}"
            f"@{rec['req_fps']:<4d} {rec['neg_res']:10s}@{rec['neg_fps']:<5.1f}"
            f" {rec['fps']:7.1f} {rec['uniq_fps']:8.1f} {rec['dups']:5d}"
            f" {exact:5s}  {rec['verdict']}")


def probe_combo(index, backends, fourcc, width, height, req_fps, seconds, warmup,
                read_timeout, exact_only):
    record = {"backend": backends[0], "fourcc": fourcc,
              "req_res": f"{width}x{height}", "req_fps": req_fps}
    cap, used = open_any(index, backends)
    if cap is None:
        record.update(fail="open failed", neg_res="-", neg_fps=0.0, exact=False)
        return record
    record["backend"] = used
    try:
        nw, nh, nf, nfourcc = apply_format(cap, width, height, req_fps, fourcc)
        exact = (nw, nh) == (width, height)
        record.update(neg_res=f"{nw}x{nh}", neg_fps=nf, neg_fourcc=nfourcc,
                      exact=exact)
        if exact_only and not exact:
            record.update(fail="unsupported (fell back)", fps=0.0)
            return record
        stats = measure_stream(cap, seconds, warmup, read_timeout)
        if stats.get("fail"):
            record.update(fail=stats["fail"], fps=0.0)
            return record
        record.update(fail="", fps=stats["fps"], uniq_fps=stats["uniq_fps"],
                      dups=stats["dups"], shape=stats["shape"],
                      read_ms=stats["read_ms"],
                      p95_gap_ms=stats["p95_gap_ms"], max_gap_ms=stats["max_gap_ms"],
                      gaps=stats["gaps"], frames=stats["frames"])
        record["verdict"] = verdict(stats["uniq_fps"], req_fps)
        return record
    finally:
        cap.release()


def parse_resolutions(text):
    out = []
    for part in text.split(","):
        part = part.strip().lower()
        if not part:
            continue
        w, h = part.split("x")
        out.append((int(w), int(h)))
    return out


def print_header():
    print(f"{'fmt':5s} {'bkd':5s} {'req res':10s} {'req fps':7s} "
          f"{'neg res':10s} {'neg fps':8s} "
          f"{'real fps':>8s} {'uniq fps':>8s} {'dups':>5s} {'exact':5s}  verdict")
    print("-" * 104)


def record_to_csv_row(rec):
    esc = rec.get("verdict", rec.get("fail", ""))
    return [rec["backend"], rec["fourcc"], rec["req_res"], rec["req_fps"],
            rec["neg_res"], f"{rec['neg_fps']:.2f}",
            f"{rec.get('fps', 0):.2f}", f"{rec.get('uniq_fps', 0):.2f}",
            rec.get("dups", 0), "yes" if rec.get("exact") else "no",
            f"{rec.get('read_ms', 0):.2f}",
            f"{rec.get('p95_gap_ms', 0):.2f}", rec.get("gaps", 0), esc]


CSV_HEADER = ["backend", "fourcc", "req_res", "req_fps", "neg_res", "neg_fps",
              "real_fps", "uniq_fps", "dups", "exact", "read_ms", "p95_gap_ms",
              "gaps", "verdict"]


def run_list(max_devices):
    try:
        devices = list_dshow_devices()
    except Exception as exc:
        devices = []
        print(f"[warn] device enumeration failed: {exc}")
    n = max(len(devices), max_devices)
    print(f"{'idx':>3s}  {'name':32s} {'dshow':7s} {'msmf':7s}  default format")
    print("-" * 80)
    for i in range(n):
        name = devices[i]["name"] if i < len(devices) else "?"
        cells = []
        default = ""
        for backend in ("dshow", "msmf"):
            cap = open_capture(i, backend, attempts=1)
            if cap is None:
                cells.append("-")
                continue
            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            cells.append("open")
            if backend == "dshow":
                default = f"{w}x{h}"
            cap.release()
            time.sleep(0.1)
        print(f"{i:>3d}  {name[:32]:32s} {cells[0]:7s} {cells[1]:7s}  {default}")


def run_scan(args):
    resolutions = parse_resolutions(args.resolutions)
    fps_requests = [int(x) for x in args.fps.split(",") if x.strip()]
    fourccs = [x.strip().upper() for x in args.fourcc.split(",") if x.strip()]
    if args.backend == "both":
        backends = ["msmf", "dshow"]
    elif args.backend == "auto":
        backends = ["msmf", "dshow"]
    else:
        backends = [args.backend]

    print(f"[info] camera {args.camera}, backend={args.backend} "
          f"(try {' -> '.join(backends)}), fourcc={','.join(fourccs)}, "
          f"seconds={args.seconds}\n")
    print_header()
    records = []
    for fourcc in fourccs:
        for width, height in resolutions:
            for req_fps in fps_requests:
                rec = probe_combo(args.camera, backends, fourcc, width, height,
                                  req_fps, args.seconds, args.warmup,
                                  args.read_timeout, args.exact)
                records.append(rec)
                print(fmt_row(rec))
                sys.stdout.flush()
                time.sleep(args.settle)

    ok = [r for r in records if not r["fail"] and r.get("uniq_fps", 0) > 0]
    if ok:
        best = max(ok, key=lambda r: r["uniq_fps"])
        print(f"\n[best] {best['fourcc']} {best['req_res']} "
              f"requested {best['req_fps']} -> real {best['fps']:.1f} fps "
              f"unique {best['uniq_fps']:.1f} fps "
              f"(negotiated {best['neg_res']}@{best['neg_fps']:.1f}, "
              f"frame {best['shape']})")
    write_csv(args.csv, records, detailed=False)
    return records


def run_single(args):
    fourcc = args.fourcc.split(",")[0].strip().upper()
    backends = (["msmf", "dshow"] if args.backend in ("auto", "both")
                else [args.backend])
    rec = {"backend": backends[0], "fourcc": fourcc,
           "req_res": f"{args.width}x{args.height}", "req_fps": args.fps}
    cap, used = open_any(args.camera, backends)
    if cap is None:
        sys.exit(f"[error] cannot open camera {args.camera} "
                 f"({args.backend}); is another app using it?")
    args.backend = used
    rec["backend"] = used
    try:
        nw, nh, nf, nfourcc = apply_format(cap, args.width, args.height,
                                           args.fps, fourcc)
        print(f"[info] backend={used}, requested {fourcc} "
              f"{args.width}x{args.height}@{args.fps}")
        print(f"[info] negotiated {nfourcc} {nw}x{nh}@{nf:.1f} "
              f"(driver value, may be wrong)")
        stats = measure_stream(cap, args.seconds, args.warmup, args.read_timeout)
    finally:
        cap.release()

    if args.save_frame:
        cap = open_capture(args.camera, args.backend)
        if cap is not None:
            try:
                apply_format(cap, args.width, args.height, args.fps, fourcc)
                for _ in range(max(1, args.warmup)):
                    cap.read()
                ok, frame = cap.read()
                if ok:
                    cv2.imwrite(args.save_frame, frame)
                    print(f"[info] frame {frame.shape[1]}x{frame.shape[0]} -> "
                          f"{args.save_frame}")
            finally:
                cap.release()

    if stats.get("fail"):
        sys.exit(f"[error] measurement failed: {stats['fail']}")

    gaps = stats["gaps_full"]
    print(f"[result] frames={stats['frames']} real_fps={stats['fps']:.2f} "
          f"uniq_fps={stats['uniq_fps']:.2f} dups={stats['dups']} "
          f"frame={stats['shape']} read={stats['read_ms']:.2f} ms  "
          f"gaps: mean={statistics.mean(gaps):.2f} median="
          f"{statistics.median(gaps):.2f} p95={stats['p95_gap_ms']:.2f} "
          f"max={stats['max_gap_ms']:.2f} ms")
    rec.update(fail="", fps=stats["fps"], uniq_fps=stats["uniq_fps"],
               dups=stats["dups"], shape=stats["shape"], read_ms=stats["read_ms"],
               p95_gap_ms=stats["p95_gap_ms"], gaps=stats["gaps"],
               frames=stats["frames"], verdict=verdict(stats["uniq_fps"], args.fps))

    if args.csv:
        with open(args.csv, "w", encoding="utf-8", newline="") as fh:
            fh.write("frame,interval_ms\n")
            fh.write(f"0,{gaps[0]:.3f}\n")
            for i, g in enumerate(gaps):
                fh.write(f"{i + 1},{g:.3f}\n")
        print(f"[info] per-frame intervals -> {args.csv}")

    if args.show:
        cap = open_capture(args.camera, args.backend)
        if cap is None:
            print("[warn] cannot reopen camera for preview")
            return rec
        try:
            apply_format(cap, args.width, args.height, args.fps, fourcc)
            print("[info] preview window: press q to quit")
            while True:
                ok, frame = cap.read()
                if not ok:
                    break
                cv2.putText(frame, f"{nw}x{nh} {stats['fps']:.1f} fps",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                            (0, 255, 0), 2)
                cv2.imshow("cam_fps", frame)
                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    break
        finally:
            cap.release()
            cv2.destroyAllWindows()
    return rec


def write_csv(path, records, detailed):
    if not path:
        return
    with open(path, "w", encoding="utf-8", newline="") as fh:
        fh.write(",".join(CSV_HEADER) + "\n")
        for rec in records:
            fh.write(",".join(str(x) for x in record_to_csv_row(rec)) + "\n")
    print(f"[info] summary -> {path}")


def main():
    ap = argparse.ArgumentParser(
        description="Windows USB camera frame-rate tester (real measured fps).",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=HELP_EPILOG)
    ap.add_argument("--mode", choices=["list", "scan", "single"], default="scan")
    ap.add_argument("--camera", type=int, default=0, help="device index")
    ap.add_argument("--device", default=None,
                    help="pick camera by DirectShow name fragment, e.g. ZSKJ")
    ap.add_argument("--backend", choices=["auto", "dshow", "msmf", "both"],
                    default="auto",
                    help="auto = try MSMF then DSHOW (MSMF honours MJPG; "
                         "DSHOW often forces YUY2)")
    ap.add_argument("--resolutions", default=",".join(DEFAULT_RESOLUTIONS),
                    help="comma list WxH for --mode scan")
    ap.add_argument("--fps", default="120,60,30",
                    help="comma list of fps requests for --mode scan; single fps "
                         "for --mode single")
    ap.add_argument("--fourcc", default="MJPG",
                    help="comma list: MJPG,YUYV,... (scan) / one value (single)")
    ap.add_argument("--seconds", type=float, default=3.0,
                    help="measurement window per combo (default 3.0)")
    ap.add_argument("--warmup", type=int, default=10,
                    help="frames discarded before timing (default 10)")
    ap.add_argument("--settle", type=float, default=0.3,
                    help="pause between combos (default 0.3 s)")
    ap.add_argument("--read-timeout", type=float, default=3.0,
                    help="max wait for a stalled read thread (default 3 s)")
    ap.add_argument("--width", type=int, default=1280, help="--mode single")
    ap.add_argument("--height", type=int, default=720, help="--mode single")
    ap.add_argument("--exact", action="store_true",
                    help="scan: only report modes the driver really honoured; "
                         "a fallback resolution is marked unsupported")
    ap.add_argument("--csv", default=None, help="write results to CSV")
    ap.add_argument("--max-devices", type=int, default=8,
                    help="--mode list: how many indices to probe (default 8)")
    ap.add_argument("--show", action="store_true",
                    help="preview window (single mode; q quits)")
    ap.add_argument("--save-frame", default=None,
                    help="single mode: write one measured frame to this PNG")
    args = ap.parse_args()

    resolve_camera(args)

    if args.mode == "list":
        run_list(args.max_devices)
    elif args.mode == "single":
        args.fps = int(args.fps.split(",")[0])
        run_single(args)
    else:
        run_scan(args)


if __name__ == "__main__":
    main()