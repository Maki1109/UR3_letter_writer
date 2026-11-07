#!/usr/bin/env bash
# Phần 1+2 – Phân vùng bản đồ + gắn nhãn -> config/zones.yaml, figures/zones.png, figures/zones_steps.png.
# Thêm --show để mở cửa sổ xem kết quả.
source "$(dirname "$0")/env.sh"
exec $PY "$B7_DIR/tools/segment_map.py" "$@"
