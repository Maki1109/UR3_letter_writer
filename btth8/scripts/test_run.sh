#!/usr/bin/env bash
# Kiểm thử không cần webcam: chạy Sim + emotion_motion_node ở nền (log vào btth8/logs/), phát lần lượt 6 biểu cảm
# lên /emotion, đo quỹ đạo bằng /odom và vẽ figures/trajectories.png. Xong thì tự tắt.
#   ./test_run.sh [happy sad ...]
D="$(cd "$(dirname "$0")" && pwd)"; L="$D/../logs"; mkdir -p "$L"; B5="$D"
"$D/stop_all.sh" >/dev/null 2>&1
source "$D/env.sh"
"$D/01_sim.sh" > "$L/sim.log" 2>&1 &
"$B5/wait_for.sh" robot
"$D/02_motion.sh" > "$L/motion.log" 2>&1 &
$PY "$B8_DIR/tests/run_trajectories.py" "$@" 2>&1 | tee "$L/test_run.log"
[ -n "$KEEP" ] || "$D/stop_all.sh" >/dev/null 2>&1
