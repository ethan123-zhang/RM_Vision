"""地图路线的配套清单（bundle）读写与内容校验。

**不依赖 ROS**，所以能在 PC 上直接跑测试（见 test/test_route_bundle.py）。

它证明的只是"这份 path.yaml 与这个地图文件是配套内容"：
地图和路线都按文件字节算 SHA256，换目录不失效，改一个字节就失效。
它**不**证明重定位精度、不证明跨重启空间一致性，也不把历史 odom 路线改成 map。

清单固定写成 ``<path_file>.manifest.json``，与路线文件同目录。
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import sys
from typing import Any, Dict, List, Optional, Sequence, Tuple

from .path_io import Point, load_path, path_length

SCHEMA_VERSION = 1
MANIFEST_SUFFIX = ".manifest.json"
MAP_FRAME = "map"
DEFAULT_BASE_FRAME = "imu"


def file_sha256(path: str) -> str:
    """流式计算文件 SHA256（不把整张地图读进内存）。"""
    digest = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _require_file(path: str, label: str) -> None:
    if not path:
        raise ValueError(f"缺少 {label} 路径")
    if not os.path.isfile(path):
        raise ValueError(f"{label} 不存在或不是文件: {path}")


def _load_route(path_file: str) -> Tuple[str, List[Point], float]:
    """按严格规则读取路线，返回 (frame, points, length)。"""
    try:
        frame, pts = load_path(path_file, require_frame=True, require_finite=True)
    except FileNotFoundError as exc:
        raise ValueError(f"路线文件不存在: {path_file}") from exc
    except OSError as exc:
        raise ValueError(f"读路线文件失败: {path_file}: {exc}") from exc
    if str(frame).strip() != MAP_FRAME:
        raise ValueError(
            f"路线 frame_id={frame!r}，只接受显式 map；"
            "历史 odom 路线不能生成/校验 map bundle"
        )
    if len(pts) < 3:
        raise ValueError(f"路线只有 {len(pts)} 个点，至少需要 3 个")
    length = path_length(pts)
    if not math.isfinite(length) or length <= 0.0:
        raise ValueError(f"路线长度为 {length!r} m，不能是零长度")
    return frame, pts, length


def _base_frame_value(base_frame: str) -> str:
    value = "" if base_frame is None else str(base_frame).strip()
    if not value:
        raise ValueError("base_frame 不能为空（默认应为 imu）")
    return value


def _metadata(path_file: str, map_file: str, base_frame: str,
              pts: Sequence[Point]) -> Dict[str, Any]:
    first = pts[0]
    return {
        "schema_version": SCHEMA_VERSION,
        "map_sha256": file_sha256(map_file),
        "path_sha256": file_sha256(path_file),
        "frame_id": MAP_FRAME,
        "base_frame": base_frame,
        "point_count": len(pts),
        "start": {"x": float(first[0]), "y": float(first[1]), "yaw": float(first[2])},
    }


def _write_manifest_atomic(manifest_path: str, metadata: Dict[str, Any]) -> None:
    directory = os.path.dirname(os.path.abspath(manifest_path))
    os.makedirs(directory, exist_ok=True)
    tmp = manifest_path + ".tmp"
    try:
        with open(tmp, "w", encoding="utf-8") as fh:
            json.dump(metadata, fh, ensure_ascii=False, indent=2, sort_keys=True)
            fh.write("\n")
        os.replace(tmp, manifest_path)
    finally:
        try:
            if os.path.exists(tmp):
                os.unlink(tmp)
        except OSError:
            pass


def write_bundle(path_file: str, map_file: str,
                 base_frame: str = DEFAULT_BASE_FRAME) -> str:
    """校验路线/地图并写 ``path_file + '.manifest.json'``，返回清单路径。"""
    base = _base_frame_value(base_frame)
    _require_file(path_file, "路线 path_file")
    _require_file(map_file, "地图 map_file")
    _frame, pts, _length = _load_route(path_file)
    metadata = _metadata(path_file, map_file, base, pts)
    manifest_path = path_file + MANIFEST_SUFFIX
    _write_manifest_atomic(manifest_path, metadata)
    return manifest_path


def validate_bundle(path_file: str, map_file: str,
                    base_frame: str = DEFAULT_BASE_FRAME) -> Dict[str, Any]:
    """校验清单与当前地图/路线内容，返回 metadata；失败抛 ValueError。"""
    base = _base_frame_value(base_frame)
    _require_file(path_file, "路线 path_file")
    _require_file(map_file, "地图 map_file")
    manifest_path = path_file + MANIFEST_SUFFIX
    if not os.path.isfile(manifest_path):
        raise ValueError(f"缺少 manifest: {manifest_path}")

    try:
        with open(manifest_path, "r", encoding="utf-8") as fh:
            data = json.load(fh)
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"manifest 不是合法 JSON: {manifest_path}: {exc}") from exc
    if not isinstance(data, dict):
        raise ValueError(f"manifest 顶层应为 JSON 对象: {manifest_path}")

    schema = data.get("schema_version")
    if type(schema) is not int or schema != SCHEMA_VERSION:
        raise ValueError(
            f"manifest schema_version={schema!r} 不受支持（只支持 {SCHEMA_VERSION}）"
        )
    if data.get("frame_id") != MAP_FRAME:
        raise ValueError(
            f"manifest frame_id={data.get('frame_id')!r}，只接受 {MAP_FRAME}"
        )
    manifest_base = data.get("base_frame")
    if manifest_base != base:
        raise ValueError(
            f"manifest base_frame={manifest_base!r} 与当前 base_frame={base!r} 不一致"
        )

    _frame, pts, _length = _load_route(path_file)
    point_count = data.get("point_count")
    if not isinstance(point_count, int) or point_count != len(pts):
        raise ValueError(
            f"manifest point_count={point_count!r} 与当前路线点数 {len(pts)} 不一致"
        )

    start = data.get("start")
    if not isinstance(start, dict):
        raise ValueError("manifest 缺少 start 对象")
    try:
        start_values = (float(start["x"]), float(start["y"]), float(start["yaw"]))
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"manifest start 字段非法: {start!r}") from exc
    if not all(math.isfinite(value) for value in start_values):
        raise ValueError("manifest start 含 NaN/Inf")
    if not all(math.isclose(actual, expected, rel_tol=0.0, abs_tol=1e-6)
               for actual, expected in zip(start_values, pts[0][:3])):
        raise ValueError(
            f"manifest start={start_values} 与路线首点 {pts[0][:3]} 不一致"
        )

    path_sha = data.get("path_sha256")
    current_path_sha = file_sha256(path_file)
    if path_sha != current_path_sha:
        raise ValueError(
            f"路线文件 path_sha256 不匹配：manifest={path_sha!r} "
            f"当前={current_path_sha!r}（路线被修改？）"
        )

    map_sha = data.get("map_sha256")
    current_map_sha = file_sha256(map_file)
    if map_sha != current_map_sha:
        raise ValueError(
            f"地图文件 map_sha256 不匹配：manifest={map_sha!r} "
            f"当前={current_map_sha!r}（地图被修改或不是同一张？）"
        )

    return {
        "schema_version": SCHEMA_VERSION,
        "map_sha256": current_map_sha,
        "path_sha256": current_path_sha,
        "frame_id": MAP_FRAME,
        "base_frame": base,
        "point_count": len(pts),
        "start": {"x": start_values[0], "y": start_values[1], "yaw": start_values[2]},
    }


def main(argv: Optional[Sequence[str]] = None) -> int:
    """只读检查入口：python -m car_bridge.route_bundle validate ..."""
    parser = argparse.ArgumentParser(
        prog="python -m car_bridge.route_bundle",
        description="地图路线 bundle 只读校验（不转换历史 odom 数据）",
    )
    sub = parser.add_subparsers(dest="command", required=True)
    validate = sub.add_parser("validate", help="校验 path + manifest + map 内容")
    validate.add_argument("--path", required=True, help="route_map.yaml 路径")
    validate.add_argument("--map", required=True, help="Odin1 .bin 地图路径")
    validate.add_argument("--base-frame", default=DEFAULT_BASE_FRAME)
    args = parser.parse_args(argv)

    if args.command == "validate":
        try:
            metadata = validate_bundle(args.path, args.map, args.base_frame)
        except (OSError, ValueError) as exc:
            print(f"✗ bundle 校验失败: {exc}", file=sys.stderr)
            return 1
        print(
            "ok: "
            f"schema={metadata['schema_version']} "
            f"frame={metadata['frame_id']} "
            f"base_frame={metadata['base_frame']} "
            f"points={metadata['point_count']} "
            f"start=({metadata['start']['x']:.3f}, "
            f"{metadata['start']['y']:.3f}, "
            f"{metadata['start']['yaw']:.3f})"
        )
        return 0
    return 1


if __name__ == "__main__":
    sys.exit(main())
