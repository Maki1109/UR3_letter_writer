#!/usr/bin/env bash
# AMCL + map_server với bản đồ bài 04 (../maps/map.yaml) – cùng bản đồ đã phân vùng.
D="$(cd "$(dirname "$0")" && pwd)"
source "$D/env.sh"
exec "$D/launch_retry.sh" "/map_server /amcl" "$B5_DIR/scripts/02_localization.sh" "$@"
