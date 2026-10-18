#!/usr/bin/env bash
# Build the C++ postprocess shared library on the board.
set -euo pipefail
cd "$(dirname "$0")"
g++ -O3 -std=c++11 -shared -fPIC postprocess.cpp -o libpost.so
ls -lh libpost.so
