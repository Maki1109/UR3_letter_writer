# Môi trường dùng chung cho mọi script (source, không chạy trực tiếp)

# Terminal của VS Code bản snap gài biến môi trường làm RViz/Gazebo nạp nhầm thư viện snap
# (lỗi "snap/core20 ... libpthread: undefined symbol"). Gỡ chúng trước khi chạy ROS.
for v in $(env | grep -oE '^(SNAP[A-Z_]*|GTK_PATH|GTK_EXE_PREFIX|GTK_IM_MODULE_FILE|GIO_MODULE_DIR|GDK_PIXBUF_MODULE_FILE|GDK_PIXBUF_MODULEDIR|LOCPATH|GSETTINGS_SCHEMA_DIR[A-Z_]*)='); do
  unset "${v%=}"
done
[ -n "$XDG_DATA_DIRS_VSCODE_SNAP_ORIG" ] && export XDG_DATA_DIRS="$XDG_DATA_DIRS_VSCODE_SNAP_ORIG"
[ -n "$XDG_CONFIG_DIRS_VSCODE_SNAP_ORIG" ] && export XDG_CONFIG_DIRS="$XDG_CONFIG_DIRS_VSCODE_SNAP_ORIG"

source /opt/ros/humble/setup.bash

export WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Overlay: irobot_create_description với Sensors system render bằng ogre2 thay vì ogre1
# (ogre1 GPU lidar trả về toàn range_min trên Mesa/Intel -> /scan hỏng)
# Overlay được sinh từ /opt/ros bởi scripts/setup_overlay.sh (không nằm trong git); tự tạo nếu chưa có.
[ -f "$WS_DIR/overlay/.ready" ] || "$WS_DIR/scripts/setup_overlay.sh" >&2
export AMENT_PREFIX_PATH="$WS_DIR/overlay:$AMENT_PREFIX_PATH"

export MAP_DIR="$WS_DIR/maps"
export MAP_NAME="${MAP_NAME:-map}"            # -> maps/map.yaml + maps/map.pgm
export WORLD="${WORLD:-room}"                  # room (nhỏ, tự tạo) | warehouse | depot | maze
export MODEL="${MODEL:-standard}"             # standard | lite

# Ignition Gazebo GUI chạy ổn định hơn qua XWayland trên phiên Wayland
export QT_QPA_PLATFORM=xcb
# Tách khỏi các máy ROS khác trong cùng mạng LAN
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
