# ur3_letter_writer

Gói ROS 2 điều khiển UR3/UR3e "viết" chữ cái đầu tiên trong tên sinh viên bằng
chuyển động của đầu công tác, dùng MoveIt 2 để lập kế hoạch (Cartesian path)
và thực thi.

## 1. Cấu trúc gói

```
ur3_letter_writer/
├── package.xml                              # khai báo gói + dependency
├── CMakeLists.txt                           # build node điều khiển
├── include/ur3_letter_writer/letters.hpp    # kiểu dữ liệu Stroke/Letter
├── src/letters.cpp                          # waypoint (u,v) của từng chữ cái (có chữ S)
├── src/letter_writer_node.cpp               # node viết chữ + vẽ vết quỹ đạo thật
├── config/letter_view.rviz                  # RViz: robot + chữ dự kiến + vết quỹ đạo thật
├── launch/ur3_write_s.launch.py             # launch gọi tắt: viết chữ S
└── launch/ur3_letter_writer.launch.py       # launch tổng (letter:=<chữ bất kỳ>)
```

## 2. Yêu cầu cài đặt

- ROS 2 (Humble/Iron/Jazzy).
- `Universal_Robots_ROS2_Driver` + `Universal_Robots_ROS2_Description`
  (cung cấp `ur_robot_driver`, `ur_description`).
- `ur_moveit_config` (MoveIt 2 config cho UR, có group `ur_manipulator`).
- (Tuỳ chọn) `Universal_Robots_ROS2_GZ_Simulation` nếu muốn chạy Gazebo thay
  vì chỉ RViz + fake hardware.

Chỉ cần các gói trên đã có trong workspace (`src/`) và build được bằng
`colcon build` — gói này **không** viết lại simulator/driver/MoveIt config,
chỉ include lại launch có sẵn của UR.

## 3. Cách hoạt động (giải thích chi tiết từng bước)

### Bước 1 — Thiết kế chữ cái bằng waypoint chuẩn hoá

Mỗi chữ cái được mô tả trong file `src/letters.cpp` dưới dạng danh sách các
**nét (stroke)**. Mỗi nét là một dãy điểm `(u, v)` nằm trong ô vuông đơn vị
`[0,1] x [0,1]`:

- `u = 0` → mép trái, `u = 1` → mép phải
- `v = 0` → đáy, `v = 1` → đỉnh

Ví dụ chữ "A" gồm 3 nét: 2 đường chéo tạo hình tam giác + 1 thanh ngang.
Chữ "M" chỉ cần 1 nét zig-zag liên tục. Việc tách nét cho phép nhấc đầu công
tác lên (pen up) giữa các nét — đúng yêu cầu đề bài.

Cách thiết kế này giúp:
- Dễ hình dung/chỉnh sửa hình chữ mà không cần biết trước kích thước thật.
- Node sẽ tự động **scale + dịch chuyển** (u,v) sang toạ độ Cartesian thật
  theo tham số kích thước/vị trí mặt phẳng mà bạn cấu hình.

**Thêm chữ mới:** mở `src/letters.cpp`, thêm 1 `case` mới trong hàm
`getLetterStrokes()` với danh sách `(u,v)` do bạn tự thiết kế trên giấy/Excel.

### Bước 2 — Chọn mặt phẳng Cartesian để vẽ chữ

Node chọn **mặt phẳng đứng song song với YZ**, cách gốc robot một khoảng cố
định theo trục X (`plane_x`, mặc định 0.4 m — giống như treo một tấm bảng
trước robot). Hướng đầu công tác (orientation) được giữ **cố định** trong
suốt quá trình vẽ, cấu hình bằng roll-pitch-yaw (`tool_orientation_rpy`).

Công thức chuyển đổi (trong `makePose()`):

```
x = pen_down ? plane_x : plane_x - pen_lift   # pen up: lùi ra xa mặt phẳng
y = center_y - width/2  + u * width
z = center_z - height/2 + v * height
```

`pen_lift` (mặc định 5 cm) là khoảng nhấc lên khi di chuyển giữa 2 nét, đảm
bảo đầu công tác không quệt vào "mặt bảng" khi di chuyển tự do.

### Bước 3 — Sinh waypoint Cartesian và gọi MoveIt

Với mỗi nét chữ, node thực hiện 4 bước con (khớp với luồng chương trình đề
bài yêu cầu):

1. **Di chuyển tự do đến phía trên điểm đầu nét** (pen up) — dùng
   `setPoseTarget()` + `move()` (MoveIt tự lập kế hoạch trong không gian tự
   do, tránh va chạm).
2. **Hạ đầu công tác xuống mặt phẳng** (pen down) tại điểm đầu — một đoạn
   Cartesian ngắn qua `computeCartesianPath()`.
3. **Vẽ nét**: gọi `computeCartesianPath()` với toàn bộ các điểm còn lại của
   nét (pen down liên tục) → MoveIt nội suy IK cho từng điểm nhỏ (bước
   `eef_step`, mặc định 5 mm) → trả về `moveit_msgs::RobotTrajectory` →
   `execute()`.
4. **Nhấc đầu công tác lên** (pen up) trước khi sang nét kế tiếp.

Đây chính là chuỗi: **Cartesian Waypoints → MoveIt Planning/IK → Joint
Trajectory → UR3 Simulation** mà đề bài mô tả.

### Bước 4 — Đảm bảo an toàn (không vượt giới hạn khớp / không tự va chạm)

Vì **mọi** chuyển động đều đi qua `MoveGroupInterface` (không bao giờ ghi
trực tiếp góc khớp), MoveIt 2 tự động:
- Giải IK và loại bỏ nghiệm vi phạm giới hạn khớp (theo URDF/SRDF của UR).
- Kiểm tra va chạm (kể cả tự va chạm) với Planning Scene / ACM trong SRDF.

Thêm vào đó, node kiểm tra `fraction` trả về từ `computeCartesianPath()`:
nếu < 0.95 (không đủ 95% điểm có lời giải IK hợp lệ liên tục), đoạn đó sẽ
**bị bỏ qua thay vì thực thi nửa vời**, tránh chuyển động giật/nhảy khớp.

### Bước 5 — Quan sát trên RViz + vẽ lại quỹ đạo THỰC TẾ

Launch file mở sẵn RViz với cấu hình riêng của gói (`config/letter_view.rviz`),
camera đã canh vào mặt phẳng vẽ, gồm 2 Display **Marker**:

- `/letter_preview` — hình chữ **dự kiến** (xanh dương, nét mỏng), publish một
  lần ngay khi node khởi động, trước cả khi robot di chuyển.
- `/letter_trace` — **quỹ đạo thực tế** (cyan, nét dày). Đây không phải đường
  tính trước: một thread riêng trong node đọc TF `world -> tool0` ở 30 Hz
  **trong lúc `execute()` đang chạy**, lấy vị trí thật của đầu công tác rồi nối
  dài dần một marker `LINE_STRIP`. Vì vậy nét chữ hiện ra đúng theo chuyển
  động thật của end-effector, và mọi sai lệch giữa đường dự kiến (xanh) với
  đường robot đi thật (cyan) đều nhìn thấy được.

Chi tiết cách hoạt động của phần vẽ vết (lớp `EndEffectorTracer` trong
`src/letter_writer_node.cpp`):

- Mỗi nét chữ được ghi vào một marker **id riêng** → các nét đã vẽ vẫn giữ
  nguyên trên màn hình và không bị nối với nhau bằng đoạn thẳng "ma" lúc nhấc
  bút chuyển nét.
- Chỉ lấy thêm điểm khi đầu công tác đã đi được ≥ 0.5 mm → lúc robot đứng yên
  chờ lập kế hoạch, marker không phình to vô ích.
- Mặc định `trace_pen_down_only:=true` → chỉ ghi khi bút đang chạm mặt phẳng
  (ra đúng hình chữ). Đặt `false` để vẽ cả hành trình nhấc bút, tức toàn bộ
  quỹ đạo thật của end-effector.
- Marker publish với QoS `transient_local` → mở RViz muộn vẫn nhận lại được
  các nét đã vẽ.

## 4. Cách chạy

### 4.1 Cài đặt nhanh (từ repo này)

Clone gói vào thư mục `src/` của một workspace ROS 2 rồi build:

```bash
# tạo workspace nếu chưa có
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src

# thu muc dich dat ten ur3_letter_writer cho trung ten goi ROS
git clone https://github.com/Maki1109/UR3_letter_writer.git ur3_letter_writer

cd ~/ros2_ws
source /opt/ros/humble/setup.bash          # đổi humble theo bản ROS 2 của bạn
colcon build --packages-select ur3_letter_writer --symlink-install
source install/setup.bash
```

`--symlink-install` giúp sửa launch file / file `.rviz` là có hiệu lực ngay,
không cần build lại (sửa `.cpp` thì vẫn phải build lại).

Kiểm tra nhanh các gói phụ thuộc đã có chưa:

```bash
ros2 pkg prefix ur_robot_driver ur_moveit_config ur_description
```

### 4.2 Chạy


**Viết chữ S (launch gọi tắt) — yêu cầu chính của bài:**

```bash
ros2 launch ur3_letter_writer ur3_write_s.launch.py
```

Launch này khởi động đủ chuỗi: fake hardware + `ros2_control` → RViz
(`letter_view.rviz`) → MoveIt `move_group` → `letter_writer_node` với
`letter:=S`, rồi robot vẽ chữ S trong khi quỹ đạo thật của đầu công tác được
vẽ dần trên topic `/letter_trace`. Chạy với robot thật:

```bash
ros2 launch ur3_letter_writer ur3_write_s.launch.py \
    use_fake_hardware:=false robot_ip:=192.168.1.102
```

**Viết chữ bất kỳ (launch tổng):**

```bash
ros2 launch ur3_letter_writer ur3_letter_writer.launch.py \
    ur_type:=ur3e letter:=Bao use_fake_hardware:=true
```

Chỉ chữ cái đầu của `letter` được dùng. Các chữ hỗ trợ sẵn: A, B, I, L, M, N,
**S**, T. Chữ S là một nét cong liền mạch, sinh bằng hàm `makeS()` trong
`src/letters.cpp` (2 cung elip tiếp xúc nhau tại tâm ô chữ, 35 điểm).

Các tham số có thể chỉnh khi launch (xem thêm trong launch file):

| Tham số | Ý nghĩa | Mặc định |
|---|---|---|
| `plane_x` | khoảng cách mặt phẳng vẽ tới gốc robot (m) | 0.35 |
| `center_y`, `center_z` | tâm chữ trong mặt phẳng (m) | 0.0, 0.35 |
| `letter_width`, `letter_height` | kích thước chữ (m) | 0.14, 0.18 |
| `pen_lift` | độ nâng khi di chuyển giữa các nét (m) | 0.05 |
| `velocity_scaling`, `acceleration_scaling` | tốc độ chuyển động | 0.3 (0.2 với `ur3_write_s`) |
| `publish_ee_trace` | bật/tắt vẽ vết quỹ đạo thực tế | true |
| `trace_pen_down_only` | chỉ vẽ vết khi bút chạm mặt phẳng | true |
| `ee_trace_topic` / `ee_trace_rate` / `ee_trace_line_width` | topic / tần số lấy mẫu (Hz) / độ dày nét (m) | `letter_trace` / 30.0 / 0.006 |
| `ee_trace_frame` | TF frame đầu công tác (rỗng = lấy end-effector link của group) | `""` |
| `launch_rviz` | mở RViz kèm `letter_view.rviz` | true |

Kích thước mặc định được chọn để **mọi điểm của chữ đều nằm trong tầm với
0.5 m của UR3/UR3e** (điểm xa nhất cách khớp vai ~0.46 m). Nếu kẻ chữ to hơn
hoặc xa hơn, `computeCartesianPath` sẽ trả về `fraction` thấp và nét đó bị bỏ
qua.

## 5. Gỡ lỗi thường gặp

- **`Group 'ur_manipulator' not found`**: tên planning group có thể khác tuỳ
  phiên bản `ur_moveit_config`. Kiểm tra bằng
  `ros2 launch ur_moveit_config ur_moveit.launch.py` rồi mở RViz → Motion
  Planning → xem tên group trong dropdown, sửa tham số `planning_group`.
- **`fraction` luôn thấp / robot không với tới mặt phẳng**: mặt phẳng có thể
  nằm ngoài không gian làm việc hoặc hướng đầu công tác không khả thi. Thử
  giảm `letter_width`/`letter_height`, đổi `plane_x`, `center_y`, `center_z`,
  hoặc chỉnh `tool_orientation_rpy`. Có thể dùng Interactive Marker trong
  RViz (kéo tay gắp tới vùng dự kiến) để kiểm tra khả năng với tới trước.
- **Máy có cài Anaconda/conda** — gây 2 lỗi khác nhau:
  - Khi build: `ModuleNotFoundError: No module named 'catkin_pkg'` (CMake bắt
    nhầm `~/anaconda3/bin/python3`).
  - Khi chạy: `~/anaconda3/lib/libstdc++.so.6: version 'GLIBCXX_3.4.30' not
    found` rồi node chết với exit code 1 (binary bị nhúng rpath trỏ vào thư
    viện của conda, cũ hơn thư viện hệ thống).

  Cách xử lý: loại conda khỏi môi trường **trước khi build**, và xoá cache
  build cũ (cache đã dính conda thì build lại vẫn hỏng):

  ```bash
  conda deactivate 2>/dev/null
  export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v anaconda | paste -sd:)
  export LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v anaconda | paste -sd:)

  cd ~/ros2_ws
  rm -rf build/ur3_letter_writer install/ur3_letter_writer
  source /opt/ros/humble/setup.bash
  colcon build --packages-select ur3_letter_writer --symlink-install
  ```

  Kiểm tra đã sạch: `ldd install/ur3_letter_writer/lib/ur3_letter_writer/letter_writer_node | grep anaconda`
  phải không ra dòng nào.
- **Launch báo thiếu/thừa argument của `ur_moveit_config`**: chạy
  `ros2 launch ur_moveit_config ur_moveit.launch.py --show-args` để xem danh
  sách chính xác trên máy bạn và chỉnh lại `launch_arguments` trong
  `ur3_letter_writer.launch.py`.

## 6. Mở rộng: sinh waypoint từ ảnh chữ viết tay (tuỳ chọn)

Đề bài gợi ý pipeline:

```
Ảnh chữ → Threshold/Binarization → Contour Extraction → Raw 2D Path
        → Simplify/Resample → Scale+Translate → (u,v) → Cartesian → MoveIt
```

Có thể triển khai bằng một node Python riêng (`cv2` + `moveit_py`, hoặc
Python node gọi action `/move_action` của MoveIt), với các bước OpenCV:

```python
import cv2
img = cv2.imread("letter.png", cv2.IMREAD_GRAYSCALE)
_, binary = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY_INV)
contours, _ = cv2.findContours(binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
strokes = []
for cnt in contours:
    approx = cv2.approxPolyDP(cnt, epsilon=2.0, closed=False)  # đơn giản hoá
    pts = approx.reshape(-1, 2).astype(float)
    # chuẩn hoá pixel (u,v) về [0,1] dựa theo bounding box toàn ảnh
    strokes.append(pts)
```

Kết quả `strokes` có cùng cấu trúc `Letter`/`Stroke` như trong
`letters.hpp`, nên có thể tái sử dụng gần như nguyên vẹn phần
`letter_writer_node` (chỉ cần thay nguồn dữ liệu strokes, phần MoveIt giữ
nguyên). Đây là phần mở rộng, không bắt buộc để hoàn thành yêu cầu chính.
