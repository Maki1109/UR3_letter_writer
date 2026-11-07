#!/usr/bin/env bash
# Kiểm thử không cần micro: chạy Sim + Localization + Nav2 + Voice Nav ở nền (log vào btth7/logs/), rồi gửi lần
# lượt các câu lệnh văn bản (tests/send_and_wait.py) và đợi kết quả trên /voice_nav/status. Ctrl+C hoặc stop_all.sh để dừng.
#   ./test_run.sh ["câu 1" "câu 2" ...]
D="$(cd "$(dirname "$0")" && pwd)"; L="$D/../logs"; mkdir -p "$L"; B5="$D/../../btth5/scripts"
"$D/stop_all.sh" >/dev/null 2>&1
source "$D/env.sh"
"$D/01_sim.sh" > "$L/sim.log" 2>&1 &
"$B5/wait_for.sh" robot
"$D/02_localization.sh" > "$L/localization.log" 2>&1 &
"$B5/auto_initial_pose.sh" 0 0 0 > "$L/init_pose.log" 2>&1
"$D/03_nav2.sh" > "$L/nav2.log" 2>&1 &
"$B5/wait_for.sh" nav2
"$D/04_voice_nav.sh" > "$L/voice_nav.log" 2>&1 &
"$D/06_map_view.sh" -p show:=${SHOW:-true} -p snapshot:="$L/map_view.png" > "$L/map_view.log" 2>&1 &
sleep 5
[ $# -gt 0 ] || set -- "Đi đến khu vực E" "Đi đến phòng thí nghiệm" "Di chuyển từ khu vực A đến khu vực B" \
                       "Đi đến sảnh" "Di chuyển từ sảnh đến phòng học" \
                       "Đi từ phòng A qua hành lang C rồi đến phòng B"
$PY "$B7_DIR/tests/send_and_wait.py" "$@" 2>&1 | tee "$L/test_run.log"
echo "Log: $L/voice_nav.log"
