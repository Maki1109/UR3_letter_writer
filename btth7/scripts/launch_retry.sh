#!/usr/bin/env bash
# Chạy một lệnh launch và tự chạy lại nếu các node lifecycle không lên "active" trong 90 s.
# Lý do: khi máy đang tải nặng (Gazebo vừa nạp), lifecycle_manager của Nav2 đôi khi bỏ dở giữa chừng (map_server
# inactive, amcl unconfigured) và đứng đó mãi, kéo theo mọi tab "wait_for" đứng theo.
#   launch_retry.sh "/map_server /amcl" lệnh ...
source "$(dirname "$0")/env.sh"
NODES="$1"; shift
for try in 1 2 3; do
  setsid "$@" &
  PID=$!
  trap 'kill -- -$PID 2>/dev/null; exit 0' INT TERM
  ok=0
  for _ in $(seq 18); do                      # 18 x 5 s = 90 s
    sleep 5
    kill -0 $PID 2>/dev/null || break
    all=1
    for n in $NODES; do
      timeout 10 ros2 lifecycle get "$n" 2>/dev/null | grep -q '^active' || { all=0; break; }
    done
    [ $all = 1 ] && { ok=1; break; }
  done
  if [ $ok = 1 ]; then wait $PID; exit $?; fi
  echo "[launch_retry] lần $try: $NODES chưa active sau 90 s -> khởi động lại" >&2
  kill -- -$PID 2>/dev/null; sleep 3; kill -9 -- -$PID 2>/dev/null; wait $PID 2>/dev/null
done
echo "[launch_retry] thất bại sau 3 lần" >&2; exit 1
