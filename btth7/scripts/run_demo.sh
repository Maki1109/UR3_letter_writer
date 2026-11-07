#!/usr/bin/env bash
# Chạy toàn bộ demo bài 07 trong một cửa sổ terminal nhiều tab, đúng thứ tự phụ thuộc:
#   Sim -> Localization -> Init pose -> Nav2 -> RViz -> Voice Nav -> Map view -> Speech.
# ENGINE=vosk ./run_demo.sh để dùng Vosk; ENGINE=text để gõ câu lệnh thay cho micro.
D="$(cd "$(dirname "$0")" && pwd)"; B5="$D/../../btth5/scripts"
"$D/stop_all.sh" >/dev/null 2>&1
source "$D/env.sh"; $PY "$B7_DIR/tools/segment_map.py" >/dev/null
tab() { gnome-terminal --tab --title="$1" -- bash -c "$2; exec bash"; sleep 1; }
gnome-terminal --window --title="Sim" -- bash -c "$D/01_sim.sh; exec bash"
sleep 1
tab Localization "$B5/wait_for.sh robot && $D/02_localization.sh"
tab "Init pose" "$B5/auto_initial_pose.sh 0 0 0"
tab Nav2 "$B5/wait_for.sh localization && $D/03_nav2.sh"
tab RViz "$B5/wait_for.sh nav2 && $D/rviz.sh"
tab "Voice Nav" "$B5/wait_for.sh nav2 && $D/04_voice_nav.sh"
tab "Map view" "$B5/wait_for.sh nav2 && $D/06_map_view.sh"
tab Speech "$B5/wait_for.sh rviz && $D/05_speech.sh -p engine:=${ENGINE:-whisper}"
echo "Đã mở các tab. Đợi 1–3 phút tới khi tab Speech báo \"Đang nghe...\", rồi nói câu lệnh."
