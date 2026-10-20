#!/usr/bin/env bash
# Phần 4 – Node YOLO: camera /oakd/rgb/preview/image_raw -> cửa sổ OpenCV + /yolo/image_annotated + terminal.
# Thêm tham số ROS nếu cần, ví dụ: ./04_yolo.sh -p conf:=0.5 -p show:=false
source "$(dirname "$0")/env.sh"
# Giới hạn luồng CPU của torch/OpenMP/OpenBLAS để Gazebo + Nav2 không bị thiếu CPU (mặc định torch dùng mọi nhân)
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-2}" MKL_NUM_THREADS="${MKL_NUM_THREADS:-2}" OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-1}"
exec $PY "$B5_DIR/nodes/yolo_detector.py" --ros-args -p use_sim_time:=true \
  -p model:="$B5_DIR/models/yolo11n.pt" "$@"
