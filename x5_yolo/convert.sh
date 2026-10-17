#!/usr/bin/env bash
# Convert yolov8n_person1_op11.onnx -> RDK X5 .bin with the OE 1.2.8 toolchain.
# Run on an x86 Linux host with Docker. Calibration tensors must be in ./calib.
set -euo pipefail
cd "$(dirname "$0")"

IMAGE="${OE_IMAGE:-openexplorer/ai_toolchain_ubuntu_20_x5_cpu:v1.2.8}"

if command -v hb_mapper >/dev/null 2>&1; then
  hb_mapper makertbin --config config_x5.yaml --model-type onnx
elif command -v docker >/dev/null 2>&1; then
  docker run --rm --network host --shm-size=15g \
    -v "$PWD":/workspace --workdir /workspace \
    "$IMAGE" \
    bash -lc "hb_mapper makertbin --config config_x5.yaml --model-type onnx"
else
  echo "need hb_mapper (pip rdkx5-yolo-mapper) or docker" >&2
  exit 1
fi
