#!/usr/bin/env bash
# 实时避障的重定位入口：保留原模板，独立生成 raw 点云配置，不启动底盘。
set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/env.sh"
mkdir -p "$CAR_CONFIG/generated"
TEMPLATE="$CAR_CONFIG/generated/control_command.obstacles.template.yaml"
python3 - "$CAR_CONFIG/control_command.relocal.yaml" "$TEMPLATE" <<'PY'
import sys
import yaml
with open(sys.argv[1], encoding="utf-8") as f:
    config = yaml.safe_load(f)
keys = config["register_keys"]
keys.update(senddtof=1, sendcloudslam=0, sendcloudrender=0, showpath=0)
with open(sys.argv[2], "w", encoding="utf-8") as f:
    yaml.safe_dump(config, f, sort_keys=False)
PY
echo "== 避障重定位：开启 cloud_raw，关闭不参与控制的 cloud_slam/cloud_render/path"
RELOCAL_CONFIG_TEMPLATE="$TEMPLATE" bash "$SCRIPT_DIR/start_relocal.sh" "$@"
