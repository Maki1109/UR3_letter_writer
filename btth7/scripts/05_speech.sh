#!/usr/bin/env bash
# Phần 3 – Node nhận dạng giọng nói: micro -> Whisper (mặc định) -> /voice_command.
#   ./05_speech.sh                       Whisper small, tiếng Việt
#   ./05_speech.sh -p engine:=vosk       Vosk (nhẹ, nhanh, chỉ nhận các câu lệnh mẫu)
#   ./05_speech.sh -p engine:=text       gõ câu lệnh bằng bàn phím (khi không có micro)
#   ./05_speech.sh -p device:=hw:1,0     chọn micro khác (xem: arecord -L)
source "$(dirname "$0")/env.sh"
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-4}"
exec $PY "$B7_DIR/nodes/speech_recognition_node.py" --ros-args "$@"
