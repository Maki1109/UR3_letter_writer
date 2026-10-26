#!/usr/bin/env bash
# Phần 1 – hiển thị webcam bằng OpenCV (q / Esc để thoát)
source "$(dirname "$0")/env.sh"
exec $PY "$B6_DIR/scripts/webcam_test.py" "$@"
