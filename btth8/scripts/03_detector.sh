#!/usr/bin/env bash
# Node nhận diện biểu cảm: webcam -> DeepFace -> /emotion + cửa sổ hiển thị thời gian thực.
#   ./03_detector.sh -p device:=1                 webcam khác (v4l2-ctl --list-devices)
#   ./03_detector.sh -p detector:=ssd             bộ dò mặt khác của DeepFace (opencv | ssd | mtcnn | retinaface)
#   ./03_detector.sh -p backend:=fer              dùng thư viện FER (cần WITH_FER=1 ./setup.sh)
source "$(dirname "$0")/env.sh"
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-4}"
exec $PY "$B8_DIR/nodes/emotion_detector_node.py" --ros-args "$@"
