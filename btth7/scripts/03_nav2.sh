#!/usr/bin/env bash
# Nav2 với ../btth5/config/nav2.yaml (inflation 0.35 m để qua khe hẹp vào Phòng thí nghiệm).
D="$(cd "$(dirname "$0")" && pwd)"
source "$D/env.sh"
exec "$D/launch_retry.sh" "/bt_navigator" "$B5_DIR/scripts/03_nav2.sh" "$@"
