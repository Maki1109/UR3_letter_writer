# Hướng dẫn chạy mô phỏng và quay video – Bài thực hành 05

Video cần thể hiện 3 yêu cầu:

1. Load lại bản đồ tuần trước và localization robot trên RViz2.
2. Code điều khiển robot tự đi qua ít nhất 3 navigation goal liên tiếp.
3. Cửa sổ camera chạy YOLO hoạt động song song khi robot di chuyển.

Mọi lệnh dưới đây gõ trong thư mục `btth5`:

```bash
cd ~/Downloads/turtlebot_simulation/btth5
```

---

## 0. Chuẩn bị (chỉ làm một lần)

```bash
./scripts/setup.sh
```

Script này cài Nav2 Simple Commander, cv_bridge, turtlebot4_navigation (cần sudo), ultralytics, numpy<2 vào
`.pydeps`, tải `models/yolo11n.pt` và sinh world `room_objects`. Đã làm trên máy này rồi thì bỏ qua.

Trước mỗi lần chạy, tắt sạch các tiến trình cũ (tránh 2 Gazebo / 2 Nav2 chạy cùng lúc):

```bash
./scripts/stop_all.sh
```

---

## Cách A – Mở tất cả bằng một lệnh (khuyên dùng)

### A1. Khởi động mọi thứ trừ node điều hướng

```bash
PATROL=0 ./scripts/run_demo.sh
```

Lệnh mở một cửa sổ terminal gồm các tab: **Sim**, **Localization**, **Init pose**, **Nav2**, **RViz**, **YOLO**.
Mỗi tab đợi bước trước sẵn sàng rồi mới chạy (dùng `scripts/wait_for.sh`):

1. **Sim**: Gazebo, robot nằm trên dock.
2. **Localization**: nạp `maps/map.yaml` (bản đồ bài 04), bật map_server và AMCL.
3. **Init pose**: tự đặt initial pose (0, 0, 0) cho AMCL, lặp lại tới khi có TF `map → base_link`. Xong thì in
   `AMCL đã định vị`.
4. **Nav2**: sau khi đã có pose thì Nav2 active.
5. **RViz**: chỉ mở khi Nav2 đã active, nên vừa mở đã có bản đồ, costmap và robot đúng vị trí trên dock.
   `Global Status: Ok`, panel Navigation 2 báo **Navigation: active, Localization: active**.
6. **YOLO**: mở cửa sổ OpenCV **"TurtleBot4 camera - YOLO"**, tab YOLO in ra các dòng như `person conf=0.67`.

Toàn bộ mất khoảng **1–2 phút**. RViz mở muộn là có chủ ý: mở RViz lúc Gazebo vừa khởi động thì máy quá tải, và
GNOME báo "RViz is not responding".

### A2. Sắp xếp màn hình

Bố trí để video thấy cùng lúc:

| Cửa sổ | Để làm gì |
|---|---|
| **RViz** (lớn nhất) | bản đồ, LaserScan, particle AMCL, đường đi của Nav2 |
| **TurtleBot4 camera - YOLO** | ảnh camera có bounding box |
| **Terminal** (tab mới) | chạy node điều hướng, in tiến độ từng goal |
| Gazebo (tùy chọn, có thể thu nhỏ) | thấy robot di chuyển trong world |

Mở tab terminal mới bằng `Ctrl+Shift+T` (vẫn ở thư mục `btth5`) và gõ sẵn lệnh, **chưa nhấn Enter**:

```bash
./scripts/05_patrol.sh
```

### A3. Bắt đầu quay video, rồi nhấn Enter

Node `patrol_navigator.py` tự làm toàn bộ, không cần chuột hay bàn phím:

| Thời điểm | Trên màn hình | Yêu cầu video |
|---|---|---|
| Ngay khi chạy | Terminal: `Đặt initial pose (0.0, 0.0, 0.0°)`. RViz: robot nằm đúng trên dock, tia laser (đỏ) trùng tường đen, particle AMCL co cụm quanh robot | **1 – localization** |
| ~5–20 s | Nav2 active, robot lùi khỏi dock (undock) | |
| Goal 1 | `==> [1/3] Goal 1 - bac hanh lang`, RViz vẽ đường đi xanh, robot qua khe vách A. Tới nơi: `ĐẾN NƠI`, đứng 4 s nhìn về người + ghế | **2** |
| Goal 2 | `==> [2/3] Goal 2 - nam hanh lang`, tới nơi nhìn về trụ cứu hỏa | **2** |
| Goal 3 | `==> [3/3] Goal 3 - phong trai`, quay về phòng trái, nhìn về sofa + người | **2** |
| Suốt quá trình | Cửa sổ YOLO vẽ box `person`, `chair`, `fire hydrant`, `couch` kèm độ tin cậy; tab YOLO in danh sách vật thể | **3 – YOLO song song** |
| Kết thúc | Bảng `===== Kết quả =====`, 3 dòng **OK** | **2 – thành công** |

Toàn bộ mất khoảng **1–2 phút**. Thấy bảng kết quả thì dừng quay.

**Gợi ý khi quay:**

- Lúc đầu video, nói/chú thích rằng bản đồ là `maps/map.yaml` của bài 04 (tab Localization in đường dẫn map).
- Muốn thể hiện localization thủ công: trước khi nhấn Enter ở bước A3, trên RViz bấm **2D Pose Estimate**, click
  vào vị trí dock và kéo theo hướng robot (hướng +x, sang phải). Particle sẽ co cụm và laser khớp tường. Sau đó
  mới chạy `05_patrol.sh` (node đặt lại đúng pose đó, không ảnh hưởng gì).
- Có thể phóng to cửa sổ YOLO: kéo giãn cửa sổ, hoặc xem ảnh tương tự ở panel **"YOLO Camera"** trong RViz.

### A4. Tắt

```bash
./scripts/stop_all.sh
```

---

## Cách B – Chạy từng bước, mỗi lệnh một terminal

Dùng khi muốn video thấy rõ từng bước. Mỗi lệnh mở trong một terminal/tab riêng, đều ở thư mục `btth5`.

| # | Lệnh | Đợi tới khi |
|---|---|---|
| 1 | `./scripts/01_sim.sh` | Gazebo hiện robot trên dock (30–60 s) |
| 2 | `./scripts/02_localization.sh` | in `Managed nodes are active` (map_server + AMCL) |
| 3 | `./scripts/03_nav2.sh` | Nav2 đã khởi động, đang chờ initial pose |
| 4 | `./scripts/rviz.sh` | RViz hiện bản đồ |
| 5 | `./scripts/set_initial_pose.sh 0 0 0` *(hoặc 2D Pose Estimate trên RViz)* | laser khớp tường, particle co cụm → **yêu cầu 1** |
| 6 | `./scripts/04_yolo.sh` | cửa sổ "TurtleBot4 camera - YOLO" hiện ra |
| 7 | **Bắt đầu quay**, rồi `./scripts/05_patrol.sh` | bảng kết quả 3 dòng OK → **yêu cầu 2, 3** |

Tắt: `Ctrl+C` từng terminal, rồi `./scripts/stop_all.sh`.

---

## Tùy chỉnh nhanh

| Muốn | Cách |
|---|---|
| Đổi tọa độ goal / thời gian dừng | sửa `config/goals.yaml` (x, y theo mét, yaw theo độ; `dwell` giây) |
| Lấy tọa độ goal mới | `./scripts/get_goal_coords.sh` rồi bấm **Publish Point** trên RViz và click lên bản đồ; hoặc `./scripts/get_goal_coords.sh amcl` để lấy pose hiện tại của robot |
| YOLO ít box hơn / chặt hơn | `./scripts/04_yolo.sh -p conf:=0.4` |
| YOLO nhận mọi lớp COCO | `./scripts/04_yolo.sh -p "classes:=['']"` |

## Xử lý sự cố khi quay

| Hiện tượng | Cách xử lý |
|---|---|
| Gazebo treo ở "downloading resources" hoặc 2 robot / 2 Gazebo | `./scripts/stop_all.sh` rồi chạy lại |
| Patrol đứng mãi ở `Waiting for amcl_pose` hoặc chờ Nav2 | Tab Localization hoặc Nav2 chưa chạy xong/bị lỗi; xem log tab đó, chạy lại từ đầu |
| RViz báo `Global Status: Error`, `RobotModel: Error` (No transform from ... to map) | Trước khi có initial pose thì đây là lỗi thoáng qua: AMCL chưa phát TF `map → odom`, và lỗi tự hết sau khi đặt pose. Nếu vẫn còn lỗi sau đó, hãy kiểm tra `config/nav_yolo.rviz`: mục RobotModel → Description Topic phải có Durability **Transient Local**, vì `/robot_description` chỉ được publish một lần. Có thể sửa ngay trên RViz rồi bấm Reset |
| GNOME báo "RViz is not responding" | Bấm **Wait**, không bấm Force Quit. RViz chỉ bị nghẽn một lúc khi nạp mesh robot hoặc khi CPU quá tải. Hãy dùng `run_demo.sh`, vì script mở RViz sau cùng, và đóng bớt ứng dụng khác |
| Robot không khớp bản đồ (laser lệch tường) | Bấm **2D Pose Estimate** đặt lại đúng vị trí, hoặc `./scripts/set_initial_pose.sh 0 0 0` khi robot còn trên dock |
| Lỗi `_ARRAY_API not found` | Chạy node qua `scripts/*.sh` (không chạy `python3 nodes/...` trực tiếp) |
| Mọi thứ rất chậm, robot giật | Đóng bớt ứng dụng khác; giảm tải YOLO: `./scripts/04_yolo.sh -p max_rate:=5.0` |
| Một goal báo `FAILED` | Chạy lại `./scripts/05_patrol.sh` (node đặt lại initial pose (0,0,0) – chỉ đúng khi robot ở dock; nếu robot đang ở chỗ khác thì `stop_all.sh` và chạy lại từ đầu) |
