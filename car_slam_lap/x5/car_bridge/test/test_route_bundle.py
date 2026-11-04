"""route_bundle 的离线测试（不依赖 ROS）。

跑法：
    python test_route_bundle.py
    python -m unittest discover -s x5/car_bridge/test -p 'test_route_bundle.py' -v
"""

from __future__ import annotations

import hashlib
import io
import json
import math
import os
import sys
import unittest
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from car_bridge import route_bundle as rb  # noqa: E402


class RouteBundleTest(unittest.TestCase):
    def setUp(self) -> None:
        self.test_dir = Path(__file__).resolve().parent
        self.other_dir = self.test_dir.parent
        self._seq = 0
        self._created: list[Path] = []
        self.addCleanup(self._cleanup)

    def _cleanup(self) -> None:
        for path in self._created:
            try:
                path.unlink()
            except FileNotFoundError:
                pass
        self._created.clear()

    def _file(self, directory: Path, suffix: str) -> Path:
        self._seq += 1
        path = directory / f".route_bundle_test_{os.getpid()}_{self._seq}{suffix}"
        self._created.append(path)
        return path

    def _write_text(self, directory: Path, suffix: str, content: str) -> Path:
        path = self._file(directory, suffix)
        path.write_text(content, encoding="utf-8")
        return path

    def _write_map(self, directory: Path, content: bytes = b"FAKE_MAP_BYTES") -> Path:
        path = self._file(directory, ".bin")
        path.write_bytes(content)
        return path

    def _write_route(self, directory: Path, points, frame: str = "map",
                     frame_line: bool = True) -> Path:
        lines = []
        if frame_line:
            lines.append(f"frame_id: {frame}" if frame else 'frame_id: ""')
        lines.append("points:")
        for x, y, yaw in points:
            lines.append(f"  - {{x: {x!r}, y: {y!r}, yaw: {yaw!r}}}")
        return self._write_text(directory, ".yaml", "\n".join(lines) + "\n")

    def _bundle(self, points=None, map_bytes: bytes = b"FAKE_MAP_BYTES",
                base_frame: str = "imu"):
        if points is None:
            points = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 1.0, math.pi / 2)]
        route = self._write_route(self.test_dir, points)
        map_file = self._write_map(self.test_dir, map_bytes)
        manifest = Path(rb.write_bundle(str(route), str(map_file), base_frame=base_frame))
        self._created.append(manifest)
        return route, map_file, manifest

    # ---------------------------------------------------------- 哈希
    def test_file_sha256_known_vectors(self) -> None:
        empty = self._file(self.test_dir, ".empty")
        empty.write_bytes(b"")
        self.assertEqual(rb.file_sha256(str(empty)),
                         hashlib.sha256(b"").hexdigest())

        data = self._file(self.test_dir, ".dat")
        data.write_bytes(b"abc")
        self.assertEqual(rb.file_sha256(str(data)),
                         hashlib.sha256(b"abc").hexdigest())

    # ---------------------------------------------------------- 正常往返
    def test_round_trip_and_manifest_metadata(self) -> None:
        route, map_file, manifest = self._bundle()
        metadata = rb.validate_bundle(str(route), str(map_file), base_frame="imu")
        self.assertEqual(metadata["schema_version"], 1)
        self.assertEqual(metadata["frame_id"], "map")
        self.assertEqual(metadata["base_frame"], "imu")
        self.assertEqual(metadata["point_count"], 3)
        self.assertAlmostEqual(metadata["start"]["x"], 0.0)
        self.assertAlmostEqual(metadata["start"]["y"], 0.0)
        self.assertAlmostEqual(metadata["start"]["yaw"], 0.0)
        self.assertEqual(metadata["path_sha256"], rb.file_sha256(str(route)))
        self.assertEqual(metadata["map_sha256"], rb.file_sha256(str(map_file)))
        self.assertTrue(manifest.exists())
        self.assertFalse(Path(str(manifest) + ".tmp").exists())

    def test_validation_does_not_depend_on_directory(self) -> None:
        route, map_file, manifest = self._bundle()
        moved_route = self._write_text(
            self.other_dir, ".yaml", route.read_text(encoding="utf-8"))
        moved_map = self._write_map(self.other_dir, map_file.read_bytes())
        moved_manifest = Path(str(moved_route) + ".manifest.json")
        moved_manifest.write_text(manifest.read_text(encoding="utf-8"),
                                  encoding="utf-8")
        self._created.append(moved_manifest)
        metadata = rb.validate_bundle(str(moved_route), str(moved_map), base_frame="imu")
        self.assertEqual(metadata["point_count"], 3)

    # ---------------------------------------------------------- 拒绝
    def test_modified_map_rejected(self) -> None:
        route, map_file, _ = self._bundle()
        map_file.write_bytes(b"CHANGED_MAP_BYTES")
        with self.assertRaises(ValueError) as ctx:
            rb.validate_bundle(str(route), str(map_file), base_frame="imu")
        self.assertIn("map", str(ctx.exception).lower())

    def test_modified_route_rejected(self) -> None:
        route, map_file, _ = self._bundle()
        route.write_text(route.read_text(encoding="utf-8") + "\n", encoding="utf-8")
        with self.assertRaises(ValueError) as ctx:
            rb.validate_bundle(str(route), str(map_file), base_frame="imu")
        self.assertIn("path", str(ctx.exception).lower())

    def test_missing_manifest_rejected(self) -> None:
        route = self._write_route(self.test_dir, [(0.0, 0.0, 0.0),
                                                  (1.0, 0.0, 0.0),
                                                  (1.0, 1.0, 0.0)])
        map_file = self._write_map(self.test_dir)
        with self.assertRaises(ValueError) as ctx:
            rb.validate_bundle(str(route), str(map_file), base_frame="imu")
        self.assertIn("manifest", str(ctx.exception).lower())

    def test_wrong_base_frame_rejected(self) -> None:
        route, map_file, _ = self._bundle(base_frame="imu")
        with self.assertRaises(ValueError) as ctx:
            rb.validate_bundle(str(route), str(map_file), base_frame="base_link")
        self.assertIn("base_frame", str(ctx.exception))

    def test_unknown_schema_rejected(self) -> None:
        route, map_file, manifest = self._bundle()
        data = json.loads(manifest.read_text(encoding="utf-8"))
        data["schema_version"] = 99
        manifest.write_text(json.dumps(data), encoding="utf-8")
        with self.assertRaises(ValueError) as ctx:
            rb.validate_bundle(str(route), str(map_file), base_frame="imu")
        self.assertIn("schema", str(ctx.exception).lower())

    def test_legacy_odom_route_rejected(self) -> None:
        route = self._write_route(self.test_dir, [(0.0, 0.0, 0.0),
                                                  (1.0, 0.0, 0.0),
                                                  (1.0, 1.0, 0.0)], frame="odom")
        map_file = self._write_map(self.test_dir)
        with self.assertRaises(ValueError) as ctx:
            rb.write_bundle(str(route), str(map_file))
        self.assertIn("map", str(ctx.exception).lower())

    def test_missing_frame_rejected(self) -> None:
        route = self._write_route(self.test_dir, [(0.0, 0.0, 0.0),
                                                  (1.0, 0.0, 0.0),
                                                  (1.0, 1.0, 0.0)],
                                  frame_line=False)
        map_file = self._write_map(self.test_dir)
        with self.assertRaises(ValueError):
            rb.write_bundle(str(route), str(map_file))

    def test_non_finite_route_rejected(self) -> None:
        route = self._write_route(self.test_dir, [(0.0, 0.0, 0.0),
                                                  (1.0, 0.0, 0.0),
                                                  (float("nan"), 1.0, 0.0)])
        map_file = self._write_map(self.test_dir)
        with self.assertRaises(ValueError) as ctx:
            rb.write_bundle(str(route), str(map_file))
        self.assertIn("NaN", str(ctx.exception))

    def test_zero_length_route_rejected(self) -> None:
        route = self._write_route(self.test_dir, [(0.0, 0.0, 0.0),
                                                  (0.0, 0.0, 0.0),
                                                  (0.0, 0.0, 0.0)])
        map_file = self._write_map(self.test_dir)
        with self.assertRaises(ValueError) as ctx:
            rb.write_bundle(str(route), str(map_file))
        self.assertIn("长度", str(ctx.exception))

    def test_missing_map_file_rejected(self) -> None:
        route = self._write_route(self.test_dir, [(0.0, 0.0, 0.0),
                                                  (1.0, 0.0, 0.0),
                                                  (1.0, 1.0, 0.0)])
        missing = self._file(self.test_dir, ".bin")
        with self.assertRaises(ValueError) as ctx:
            rb.write_bundle(str(route), str(missing))
        self.assertIn("map", str(ctx.exception).lower())

    def test_tampered_manifest_metadata_rejected(self) -> None:
        route, map_file, manifest = self._bundle()
        data = json.loads(manifest.read_text(encoding="utf-8"))
        data["point_count"] = 99
        manifest.write_text(json.dumps(data), encoding="utf-8")
        with self.assertRaises(ValueError) as ctx:
            rb.validate_bundle(str(route), str(map_file), base_frame="imu")
        self.assertIn("point_count", str(ctx.exception))

    # ---------------------------------------------------------- CLI
    def test_cli_validate_ok_and_failure(self) -> None:
        route, map_file, _ = self._bundle()
        out, err = io.StringIO(), io.StringIO()
        with redirect_stdout(out), redirect_stderr(err):
            code = rb.main(["validate", "--path", str(route), "--map", str(map_file),
                            "--base-frame", "imu"])
        self.assertEqual(code, 0)
        self.assertIn("ok", out.getvalue().lower())

        map_file.write_bytes(b"CHANGED")
        out2, err2 = io.StringIO(), io.StringIO()
        with redirect_stdout(out2), redirect_stderr(err2):
            code2 = rb.main(["validate", "--path", str(route), "--map", str(map_file),
                             "--base-frame", "imu"])
        self.assertEqual(code2, 1)
        self.assertIn("map", err2.getvalue().lower())


if __name__ == "__main__":
    unittest.main(verbosity=2)
