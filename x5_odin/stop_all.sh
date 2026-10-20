#!/usr/bin/env bash
# Stop the Odin1 web viewer and driver on the RDK X5.
for p in $(pgrep -f "[o]din_web_viewer.py"); do kill -9 "$p" 2>/dev/null || true; done
for p in $(pgrep -x host_sdk_sample); do kill "$p" 2>/dev/null || true; done
sleep 2
echo "viewer:    $(pgrep -f "[o]din_web_viewer.py" | tr '\n' ' ')  (empty = stopped)"
echo "driver:    $(pgrep -x host_sdk_sample | tr '\n' ' ')  (empty = stopped)"
