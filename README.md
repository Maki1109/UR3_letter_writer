# Bài thực hành 06 – Giao tiếp phi ngôn ngữ: điều khiển TurtleBot 4 bằng cử chỉ tay

Webcam laptop → OpenCV → **MediaPipe Gesture Recognizer** → `/detected_gesture` (`std_msgs/String`) →
máy trạng thái → `/cmd_vel` → TurtleBot 4 trong Ignition Gazebo (ROS 2 Humble).

Repo chỉ chứa phần của bài 06: thư mục `btth6/` (node, launch file, script) và vài script dùng chung ở `scripts/`
(môi trường, overlay Gazebo, world `room` làm nền cho world `open_arena`). Mọi lệnh bên dưới chạy từ thư mục `btth6`.

## Cấu trúc

```
btth6/
  nodes/
    gesture_recognition_node.py   Phần 2: webcam -> MediaPipe -> /detected_gesture + cửa sổ hiển thị
    robot_behavior_node.py        Phần 3: /detected_gesture -> máy trạng thái -> /cmd_vel (đo bằng /odom)
  launch/
    gesture_control.launch.py     Gazebo (open_arena) + 2 node trong một lệnh
  models/gesture_recognizer.task  model MediaPipe (float16, setup.sh tải về)
  scripts/
    setup.sh            cài phụ thuộc (một lần)
    make_world.py       sinh world open_arena: sân trống 7 x 7 m
    01_sim.sh           Phần 1: Gazebo + TurtleBot 4 (world open_arena), tự đặt góc camera Gazebo
    check_cmd_vel.sh    Phần 1: undock + gửi vận tốc quay qua /cmd_vel, so yaw trước/sau
    webcam_test.sh      Phần 1: đọc và hiển thị webcam bằng OpenCV
    02_gesture.sh       Phần 2: chạy gesture_recognition_node
    03_behavior.sh      Phần 3: chạy robot_behavior_node
    run_demo.sh         mở Sim, Behavior, Gesture trong các tab terminal, đúng thứ tự
    view.sh             đặt lại camera Gazebo (xiên 45°, hoặc `view.sh top` nhìn từ trên xuống)
    stop_all.sh         tắt sạch Gazebo và mọi node
```

## Cài đặt và build

Yêu cầu: Ubuntu 22.04, ROS 2 Humble, `ros-humble-turtlebot4-simulator` (môi trường của bài 04: xem
`../scripts/00_install.sh`), webcam.

```bash
git clone -b assignments_6 https://github.com/Maki1109/UR3_letter_writer.git
cd UR3_letter_writer/btth6
chmod +x scripts/*.sh scripts/*.py nodes/*.py
./scripts/setup.sh
```

Không cần `colcon build`: hai node là script Python chạy trực tiếp bằng `/usr/bin/python3` (có `rclpy` của ROS
Humble). Bước "build" duy nhất là `setup.sh`, gồm cài thư viện Python, tải model và sinh world. Lần chạy đầu,
`scripts/env.sh` còn tự sinh thư mục `../overlay/` (vài giây) từ các gói ROS đã cài.

`setup.sh` cài `mediapipe==0.10.21`, `numpy<2` và `protobuf 4` vào `btth6/.pydeps`, tải `gesture_recognizer.task`
và sinh world. Lý do dùng `.pydeps` giống bài 05: `~/.local` trên máy này có numpy 2.x, còn OpenCV hệ thống
(`python3-opencv` 4.5.4) được biên dịch với numpy 1.x. Gói mediapipe được cài `--no-deps`, vì nó kéo theo
`opencv-contrib-python` (cần numpy 2), jax và matplotlib. Node chỉ dùng MediaPipe Tasks nên không cần các gói đó.

## Chạy

Một trong hai cách (gõ từ thư mục `btth6`):

```bash
./scripts/run_demo.sh        # mở 3 tab terminal: Sim -> Behavior -> Gesture

# hoặc dùng launch file
source scripts/env.sh
ros2 launch launch/gesture_control.launch.py                       # Gazebo + 2 node
ros2 launch launch/gesture_control.launch.py sim:=false camera:=1  # sim đã chạy sẵn, webcam 1
```

Hoặc chạy từng bước, mỗi lệnh một terminal:

```bash
./scripts/01_sim.sh          # đợi robot xuất hiện trên dock (30–60 s)
./scripts/check_cmd_vel.sh   # (Phần 1) kiểm tra /cmd_vel, có thể bỏ qua
./scripts/webcam_test.sh     # (Phần 1) kiểm tra webcam, q để thoát
./scripts/03_behavior.sh     # tự undock rồi tiến ra giữa sân, trạng thái IDLE
./scripts/02_gesture.sh      # cửa sổ "Gesture Recognition - webcam"
```

Webcam khác: `./scripts/02_gesture.sh -p camera:=1`. Lần đầu khởi động, Gazebo đôi khi treo ở bước nạp world
(log dừng ở `Creating postupdate worker thread`, robot không xuất hiện). Khi đó chạy `./scripts/stop_all.sh` rồi
chạy lại.

## Phần 1 – Simulation và webcam

- World `open_arena`: sân phẳng 7 x 7 m (x ∈ [-5, 2], y ∈ [-3.5, 3.5]), chỉ có tường bao thấp. Một vòng tròn
  xanh trên sàn đánh dấu vùng hoạt động. Robot spawn trên dock tại (0, 0). Sau khi undock và tiến ra 1 m, robot
  đứng ở khoảng (-1.35, -0.2), cách tường gần nhất hơn 2.5 m.
- `check_cmd_vel.sh` undock robot, publish `angular.z = 0.5` lên `/cmd_vel` trong 6 s, rồi in yaw trước và sau.
- `webcam_test.sh` đọc webcam bằng `cv2.VideoCapture`, lật ảnh như gương và hiển thị kèm FPS.

## Phần 2 – `gesture_recognition_node`

Mỗi khung hình (khoảng 21 FPS trên máy này khi Gazebo đang chạy):

1. `cv2.VideoCapture.read()`, lật ảnh như gương, đổi BGR sang RGB.
2. `GestureRecognizer.recognize_for_video()` ở chế độ `VIDEO` (theo dõi tay giữa các khung, nhanh hơn chế độ
   `IMAGE`), `num_hands=1`.
3. Lấy cử chỉ có điểm cao nhất. Nếu `score < min_score` (0.6) thì coi là `None`.
4. **Wave**: model của MediaPipe không có lớp vẫy tay, nên node tự nhận dạng. Node theo dõi vị trí ngang trung
   bình của 5 đầu ngón tay, chia cho cỡ bàn tay (khoảng cách cổ tay tới gốc ngón giữa), nên kết quả không phụ thuộc
   khoảng cách tới camera. Dùng đầu ngón thay vì lòng bàn tay vì khi vẫy bằng cách xoay quanh cổ tay, lòng bàn tay
   gần như đứng yên. Nếu tâm đầu ngón đổi chiều trái/phải ít nhất 3 lần trong 1.5 s, mỗi lần dịch hơn 0.4 cỡ bàn tay,
   và ít nhất 60% số khung là `Open_Palm` hoặc `None`, node trả về `Wave` và giữ nhãn này 1 s. Khung `None` được tính
   vì khi vẫy nhanh, ảnh nhòe làm MediaPipe trả `None`: lúc thử thật, phần lớn khung có tay bị trả `None`, chỉ 24
   khung được nhận là `Open_Palm`. Mất tay dưới 0.4 s (`hand_lost`) không xóa lịch sử. Trong lúc tay đang dịch nhanh, `Open_Palm` bị
   bỏ qua, để động tác vẫy không bị hiểu là "dừng". Góc phải cửa sổ có chỉ báo `Wave n/3`.
5. **Chống nhiễu**: một nhãn chỉ được chấp nhận khi xuất hiện liên tục `stable_frames` = 6 khung (khoảng 0.3 s).
   Thanh tiến độ ở góc phải trên cửa sổ cho biết nhãn đang tích lũy đến đâu.
6. Publish nhãn ổn định lên `/detected_gesture` ở mỗi khung, kể cả `None` khi không có tay. Như vậy topic này
   cũng là heartbeat cho Phần 3.

Node nhận diện **8 cử chỉ**: `Open_Palm`, `Closed_Fist`, `Pointing_Up`, `Victory`, `Thumb_Up`, `Thumb_Down`,
`ILoveYou` (của MediaPipe) và `Wave` (tự nhận dạng). `Thumb_Down` và `ILoveYou` chỉ được hiển thị và publish,
không gán hành vi.

Cửa sổ hiển thị 21 landmark của bàn tay, tên cử chỉ và độ tin cậy (%) của khung hiện tại, nhãn đã publish, FPS
và trạng thái robot (lấy từ topic `/robot_behavior/state`). Phím `q`/`Esc` chỉ dùng để đóng cửa sổ, không điều
khiển robot.

| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `camera` | 0 | chỉ số webcam |
| `min_score` | 0.6 | ngưỡng độ tin cậy |
| `stable_frames` | 6 | số khung liên tiếp để chấp nhận một cử chỉ |
| `wave_window` / `wave_reversals` / `wave_swing` | 1.5 s / 3 / 0.4 | điều kiện nhận Wave |
| `still_span` | 0.3 | Open_Palm chỉ được tính khi tay dịch ít hơn mức này trong 0.5 s |
| `trace` | '' | đường dẫn CSV để ghi từng khung (nhãn, điểm, vị trí, số lần đổi chiều), dùng khi chẩn đoán |

## Phần 3 – `robot_behavior_node`

| Cử chỉ | Hành vi |
|---|---|
| Wave | xoay trái–phải (±35°, 2 lượt) rồi về hướng cũ để chào lại; chạy ngay khi robot đang IDLE |
| Pointing_Up | **chọn** tiến–lùi 4 lần (0.4 m, 0.2 m/s) |
| Victory | **chọn** quỹ đạo tròn (bán kính 0.5 m, 0.2 m/s, đủ 360°) |
| Thumb_Up | **xác nhận**: thực hiện hành vi đã chọn, hoặc chạy tiếp hành vi đang PAUSED |
| Open_Palm | **dừng ngay**: robot đứng yên, hành vi chuyển sang PAUSED |
| Closed_Fist | **hủy** hành vi hoặc lựa chọn hiện tại, về IDLE |

```
UNDOCKING -> CLEARING -> IDLE --Pointing_Up/Victory--> SELECTED --Thumb_Up--> EXECUTING --xong--> IDLE
                          |                                                    |    ^
                          +--Wave--> EXECUTING (chào)        Open_Palm /       v    | Thumb_Up
                                                             mất tín hiệu     PAUSED
Closed_Fist: SELECTED / EXECUTING / PAUSED -> IDLE
```

Cách node đáp ứng từng yêu cầu của đề:

- **Chỉ thực hiện sau khi xác nhận**: Pointing_Up và Victory chỉ chọn hành vi (trạng thái SELECTED). Robot chỉ
  chạy khi nhận Thumb_Up. Thumb_Up khi chưa chọn gì thì bị bỏ qua. Riêng Wave là lời chào đáp lại nên chạy ngay,
  và chỉ chạy khi robot đang IDLE.
- **Không dùng bàn phím**: sau khi chạy `run_demo.sh`, mọi lệnh cho robot đều đến từ `/detected_gesture`.
- **Chống kích hoạt liên tục**: ngoài bộ lọc 6 khung ở Phần 2, node chỉ phản ứng khi nhãn **đổi** sang một cử chỉ
  mới. Giữ nguyên cử chỉ thì không lặp lệnh. Cùng một cử chỉ phải cách lần trước ít nhất `cooldown` = 1.5 s, nên
  nhãn nhấp nháy kiểu `Victory → None → Victory` không kích hoạt hai lần. Open_Palm không có cooldown vì là lệnh
  an toàn. Khi đang EXECUTING, Pointing_Up, Victory và Wave bị bỏ qua. Muốn đổi hành vi thì phải hủy trước.
- **Dừng khi có lệnh dừng hoặc mất tín hiệu**: Open_Palm gửi `Twist()` = 0 ngay trong callback, rồi gửi tiếp vận
  tốc 0 trong 0.5 s. Nếu không nhận `/detected_gesture` quá 1 s (node nhận diện bị tắt, webcam lỗi) hoặc mất
  `/odom` quá 0.5 s, robot dừng và chuyển sang PAUSED. Muốn chạy tiếp phải giơ Thumb_Up.
- **Không va chạm**: sân trống 7 x 7 m. Mọi chuyển động nằm trong bán kính khoảng 1 m quanh điểm xuất phát.
  Sau khi undock, node tự tiến thêm 1 m để khi lùi robot không chạm dock.

Quãng đường và góc quay được đo bằng `/odom` (vòng kín), có giảm tốc ở cuối mỗi đoạn. Vì vậy kích thước chuyển
động vẫn đúng khi Gazebo chạy chậm hơn thời gian thực. Khi tiếp tục sau PAUSED, robot chỉ đi nốt phần còn lại của
đoạn đang dở.

**Lưu ý Create 3**: mặc định Create 3 không cho lùi quá khoảng 0.5 m. Khi vượt mức đó, hazard `BACKUP_LIMIT` được
bật và robot đứng im giữa lượt lùi (đã gặp khi thử: lần lùi đầu tiên chỉ được 0.17 m). Node đặt tham số
`/motion_control safety_override = backup_only` lúc khởi động. Sân mô phỏng phẳng, không có vực, nên tắt giới hạn
này là an toàn. Muốn giữ cấu hình gốc thì chạy `-p safety_override:=''`.

Kết quả thử trong mô phỏng (dùng một node giả publish chuỗi cử chỉ, đọc `/odom`):

| Kịch bản | Kết quả |
|---|---|
| Undock + tiến ra | x: 0 → -0.35 (undock) → -1.14 m (với `clear_dock` = 0.8), sau đó IDLE |
| Wave | yaw dao động khoảng ±35° quanh hướng ban đầu, kết thúc lệch ~3° |
| Pointing_Up → Thumb_Up, Open_Palm giữa chừng, Thumb_Up | dừng ở đoạn 3/16, chạy tiếp, xong 16/16 và về x = -1.15 (bắt đầu ở -1.14) |
| Victory → Thumb_Up, ngắt `/detected_gesture` 3 s | PAUSED sau 1 s mất tín hiệu, Thumb_Up thì chạy nốt vòng tròn |
| Closed_Fist khi đang chạy | HỦY và về IDLE |
| Vòng tròn đầy đủ | đường kính đo được khoảng 0.95 m (đặt 1.0 m) |
| Thumb_Up khi chưa chọn | bỏ qua, có log nhắc |

| Tham số | Mặc định |
|---|---|
| `linear_speed`, `fb_distance`, `fb_repeats` | 0.2 m/s, 0.4 m, 4 |
| `circle_radius`, `circle_speed` | 0.5 m, 0.2 m/s |
| `wave_angle`, `wave_speed`, `wave_repeats` | 0.6 rad, 1.2 rad/s, 2 |
| `cooldown`, `signal_timeout`, `odom_timeout` | 1.5 s, 1.0 s, 0.5 s |
| `auto_undock`, `clear_dock`, `safety_override` | true, 1.0 m, backup_only |
