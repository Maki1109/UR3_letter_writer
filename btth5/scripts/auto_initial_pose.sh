#!/usr/bin/env bash
# Tự đặt initial pose cho AMCL ngay khi localization active, lặp lại tới khi TF map -> base_link xuất hiện.
# Nhờ vậy Nav2 active được và RViz mở ra đã có robot trên bản đồ, không báo lỗi TF.
# Dùng: auto_initial_pose.sh [x] [y] [yaw_rad]   (mặc định 0 0 0 = robot trên dock)
D="$(cd "$(dirname "$0")" && pwd)"
"$D/wait_for.sh" localization
for i in $(seq 1 20); do
  echo "Đặt initial pose ($*) lần $i"
  "$D/set_initial_pose.sh" "$@" >/dev/null 2>&1
  if timeout 8 "$D/wait_for.sh" pose >/dev/null 2>&1; then
    echo "AMCL đã định vị: TF map -> base_link sẵn sàng"
    exit 0
  fi
done
echo "Không đặt được initial pose – kiểm tra tab Localization" >&2
exit 1
