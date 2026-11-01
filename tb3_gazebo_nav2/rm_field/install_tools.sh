#!/usr/bin/env bash
# 安装 STEP -> 网格 工具链（gmsh 带 OCCT STEP 导入）
set +u
export DEBIAN_FRONTEND=noninteractive
echo "=== apt-cache policy ==="
apt-cache policy gmsh python3-gmsh freecad 2>/dev/null
echo "=== install ==="
apt-get install -y --no-install-recommends gmsh python3-gmsh 2>&1 | tail -25
echo "=== verify ==="
command -v gmsh && gmsh --version
python3 -c "import gmsh; print('python gmsh', gmsh.__version__)"