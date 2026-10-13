# Bài thực hành 04 – TurtleBot 4 Simulation: SLAM Toolbox + Nav2

Dựng bản đồ bằng SLAM Toolbox, lưu bản đồ, rồi dùng bản đồ đó để localization (AMCL) và điều hướng tự động
(Nav2) với TurtleBot 4 trong Ignition Gazebo. Môi trường mô phỏng là một **phòng nhỏ 7 x 5 m** (world `room`)
có hai vách ngăn tạo hành lang chữ S và ba hộp vật cản, nên quét hết bản đồ chỉ mất khoảng 1–2 phút.

## Yêu cầu

- Ubuntu 22.04 (trực tiếp hoặc WSL2), ROS 2 Humble Desktop
- Gói TurtleBot 4 simulator, SLAM Toolbox, Nav2:

```bash
sudo apt update
sudo apt install ros-humble-turtlebot4-simulator ros-humble-turtlebot4-desktop \
  ros-humble-turtlebot4-navigation ros-humble-slam-toolbox ros-humble-nav2-bringup \
  ros-humble-nav2-map-server ros-humble-teleop-twist-keyboard
```
(Hoặc `./scripts/00_install.sh`.) Các script dùng Python 3 hệ thống (`/usr/bin/python3`, có `rclpy`); nếu đang
dùng conda, hãy `conda deactivate` trước khi chạy.

## Cấu trúc repo

```
launch/
  sim.launch.py         Phần 1: TurtleBot 4 trong Gazebo (world room)
  mapping.launch.py     Phần 2: SLAM Toolbox (config/slam.yaml) + RViz2
  navigation.launch.py  Phần 3+4: AMCL + map_server (maps/map.yaml) + Nav2 + RViz2
config/
  slam.yaml             cấu hình SLAM Toolbox đã chỉnh cho phòng nhỏ (xem "Cấu hình")
maps/
  map.yaml, map.pgm     bản đồ đã tạo bằng SLAM Toolbox
scripts/                script chạy từng bước, teleop giữ-phím, lưu map, tạo world và overlay
docs/goals.png          3 điểm đích Nav2 đã dùng, vẽ trên bản đồ
```

## Cài đặt và chạy

Mọi lệnh chạy từ thư mục gốc của repo. Các script tự nạp môi trường (`scripts/env.sh`) và **lần chạy đầu tự
sinh thư mục `overlay/`** (vài giây) từ các gói ROS đã cài, nên không cần cài thêm gì.

```bash
git clone https://github.com/Maki1109/UR3_letter_writer.git
cd UR3_letter_writer && git checkout assignments_4
chmod +x scripts/*.sh scripts/*.py
```

Ba launch file chạy được trực tiếp sau khi `source scripts/env.sh` (nạp overlay), hoặc dùng các script bọc
tương ứng (khuyến nghị, vì `01_sim.sh` còn tự đặt camera Gazebo nhìn từ trên xuống).

### Phần 1 – Simulation

```bash
./scripts/01_sim.sh                 # tương đương: ros2 launch launch/sim.launch.py
./scripts/rviz.sh                   # RViz2 (đặt Fixed Frame = odom nếu chưa có SLAM/AMCL)
./scripts/check_topics.sh           # kiểm tra /scan, /odom, /cmd_vel, undock, gửi vận tốc
```
Đợi khoảng 30–60 giây cho robot xuất hiện trong Gazebo (bấm **Play** nếu sim đang pause).

### Phần 2 – Mapping

```bash
./scripts/run_mapping.sh            # mở 3 tab: SLAM + RViz + teleop (hoặc: ros2 launch launch/mapping.launch.py)
# rời dock (robot spawn trên dock):
source scripts/env.sh && ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"
```
Lái robot bằng teleop giữ-phím (`./scripts/teleop.sh`): giữ `W/A/S/D` hoặc phím mũi tên để chạy, nhả để dừng,
`Z/X` đổi tốc độ, `Space` dừng, `Q` thoát. **Tắt bộ gõ tiếng Việt** khi dùng chữ cái. Đi chậm (≤ 0.35 m/s), quay
≤ 0.4 rad/s và dừng hẳn trước khi quay. Thứ tự quét: phòng trái, khe phía trên vách thứ nhất, hành lang giữa,
khe phía dưới vách thứ hai, rồi vùng phải. Khi bản đồ đủ, **giữ SLAM đang chạy** và lưu:

```bash
./scripts/save_map.sh               # -> maps/map.yaml + maps/map.pgm (map cũ được sao lưu, không bị ghi đè)
```

### Phần 3 – Localization

```bash
./scripts/stop_all.sh               # tắt SLAM và toàn bộ mô phỏng
./scripts/01_sim.sh                 # robot spawn lại trên dock
./scripts/03_localization.sh        # AMCL + map_server, load maps/map.yaml
./scripts/set_initial_pose.sh 0 0 0 # robot đang trên dock = gốc bản đồ (hoặc dùng 2D Pose Estimate trong RViz)
./scripts/rviz.sh                   # Fixed Frame = map
```
Kiểm tra: robot trên RViz nằm đúng vị trí trong Gazebo, particle (mũi tên xanh) co cụm, tia laser đỏ trùng khít
tường đen của bản đồ.

### Phần 4 – Navigation với Nav2

```bash
./scripts/04_nav2.sh                # đợi log "Managed nodes are active"
# hoặc cả phần 3+4: ./scripts/run_navigation.sh  (tương đương: ros2 launch launch/navigation.launch.py)
```
Trong RViz bấm **Nav2 Goal**, **nhấn chuột tại đích** (một ô trắng cách xa robot) và **kéo** theo hướng mũi robot
muốn quay khi tới nơi, rồi thả. Không điều khiển tay sau khi đặt goal. Ba goal đã dùng (xem `docs/goals.png`):

| Goal | Khu vực | Tọa độ (map, m) |
|---|---|---|
| 1 | phòng trái, góc trên | (-1.2, 1.5) |
| 2 | hành lang giữa hai vách, nửa dưới | (2.4, -0.8) |
| 3 | vùng phải, sau vách thứ hai | (4.2, 1.5) |

## Cấu hình và các chỉnh sửa so với mặc định

Các chỉnh sửa này cần cho môi trường mô phỏng trên máy dùng GPU Intel tích hợp / Mesa. Chúng nằm trong
`overlay/` (sinh bởi `scripts/setup_overlay.sh`, không sửa gì trong `/opt/ros`):

- **Render cảm biến bằng ogre2:** với ogre1, LiDAR trong Gazebo trả về toàn `range_min` (0.164 m) nên `/scan` hỏng.
- **Hạ tần số cảm biến từ 62 Hz xuống 10 Hz** (RPLIDAR, cliff, ir_intensity; đúng với robot thật): tốc độ mô phỏng
  (real-time factor) tăng từ khoảng 0.25 lên khoảng 0.9.
- **World `room`:** sinh bởi `scripts/make_room_world.py` (được `setup_overlay.sh` gọi).
- **`config/slam.yaml`:** tắt loop closure (hai vách ngăn giống hệt nhau khiến loop closure ghép nhầm và làm bản
  đồ méo), chỉ xử lý scan khi robot đã đi ≥ 0.1 m hoặc quay ≥ 0.1 rad, thu hẹp vùng tìm ghép scan và tin
  odometry hơn. Các dòng đã đổi so với cấu hình mặc định của `turtlebot4_navigation` đều có chú thích trong file.
- **`scripts/env.sh`:** gỡ các biến `SNAP*`/`GTK_PATH`... do VS Code bản snap gài vào (làm RViz2 crash).

Đổi world bằng biến môi trường, ví dụ `WORLD=warehouse ./scripts/01_sim.sh` (30 x 50 m, quét lâu hơn nhiều).
`scripts/stop_all.sh` tắt sạch mọi tiến trình; hãy dùng giữa các phần vì Ctrl+C đôi khi để lại server Gazebo mồ côi.

## Kết quả

Bản đồ `maps/map.pgm` (khoảng 7 x 5 m, độ phân giải 0.05 m) khớp layout thật của phòng: khi chấm tự động với
layout trong world, không có pixel "ma" ngoài tường thật và độ phủ tường thật khoảng 99%.

## Xử lý sự cố

| Triệu chứng | Cách xử lý |
|---|---|
| RViz báo "Frame [map] does not exist" | Chưa chạy SLAM hoặc chưa đặt initial pose cho AMCL. Khi chỉ có sim, đặt Fixed Frame = `odom` |
| Giữ W/A/S/D mà robot không chạy hoặc hiện chữ lạ (ư, â, đ) | Bộ gõ tiếng Việt đang bật; tắt bằng Super+Space hoặc dùng phím mũi tên |
| Nav2 Goal đặt xong robot không chạy | Nhấn chuột đúng **tại đích**, không phải trên robot; kéo chỉ chọn hướng quay |
| Map bị méo / tường nhân đôi | Quét lại, quay chậm hơn; luôn chạy SLAM qua `02_slam.sh` hoặc `mapping.launch.py` (dùng `config/slam.yaml`) |
| `/scan` toàn giá trị 0.164 | Đang chạy ngoài script, thiếu overlay; `source scripts/env.sh` trước khi chạy |
| Log Gazebo báo `Found additional publishers on /clock` | Còn server Gazebo cũ: `./scripts/stop_all.sh` rồi chạy lại |
| Nav2 không báo `Managed nodes are active` | Chưa có initial pose cho AMCL: `./scripts/set_initial_pose.sh 0 0 0` |
| Sim chậm | GPU Intel tích hợp; cài driver NVIDIA nếu có card rời. Tốc độ teleop tính theo thời gian mô phỏng |
