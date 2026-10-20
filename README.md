# Bài thực hành 05 – TurtleBot 4: Localization, Autonomous Navigation và nhận diện vật thể bằng YOLO

Dùng lại bản đồ đã quét ở bài 04 (`../maps/map.yaml`, `../maps/map.pgm`, world `room` 7 x 5 m) để
localization bằng AMCL. Một ROS 2 node Python dùng **Nav2 Simple Commander** để robot tự đi qua 3 goal. Node
thứ hai lấy ảnh camera OAK-D, chạy **YOLO11n** (ultralytics) và vẽ kết quả nhận diện.

Repo chỉ chứa phần của bài 05: thư mục `btth5/` (node, cấu hình, script), bản đồ `maps/map.yaml` + `map.pgm` và vài script dùng chung
ở `scripts/` (môi trường, overlay Gazebo, world `room`, khởi động sim). Mọi lệnh bên dưới chạy từ thư mục `btth5`.

## Cấu trúc

```
btth5/
  nodes/
    patrol_navigator.py   Phần 3: initial pose -> đợi Nav2 -> undock -> Goal 1 -> 2 -> 3 (goToPose)
    yolo_detector.py      Phần 4: /oakd/rgb/preview/image_raw -> cv_bridge -> YOLO -> OpenCV + /yolo/image_annotated
  config/
    goals.yaml            Phần 2: tọa độ (x, y, yaw) của các goal trong frame map
    nav2.yaml             tham số Nav2 của TurtleBot 4, giảm inflation để qua khe hẹp (xem bên dưới)
    nav_yolo.rviz         RViz2: map, LaserScan, particle AMCL, costmap, path + ảnh YOLO
  scripts/
    setup.sh              cài phụ thuộc (một lần)
    make_world.py         sinh world room_objects = room + người, ghế, sofa, trụ cứu hỏa
    01_sim.sh             Gazebo + TurtleBot 4 (world room_objects)
    02_localization.sh    AMCL + map_server, nạp ../maps/map.yaml
    03_nav2.sh            Nav2
    rviz.sh               RViz2 với config/nav_yolo.rviz
    set_initial_pose.sh   đặt initial pose thủ công (mặc định 0 0 0)
    get_goal_coords.sh    in tọa độ từ /clicked_point (Publish Point) hoặc /amcl_pose
    04_yolo.sh            chạy node YOLO
    05_patrol.sh          chạy node điều hướng
    run_demo.sh           mở tất cả các bước trên trong các tab terminal, đúng thứ tự phụ thuộc
    wait_for.sh           đợi robot / localization / pose / nav2 / rviz sẵn sàng (run_demo.sh dùng)
    auto_initial_pose.sh  tự đặt initial pose khi AMCL active, lặp tới khi có TF map -> base_link
    stop_all.sh           tắt sạch Gazebo, Nav2 và các node của bài 05
    test_run.sh           chạy tất cả ở nền, ghi log vào btth5/logs/ (để kiểm thử)
  models/yolo11n.pt       trọng số YOLO (setup.sh tải về)
```

## Cài đặt

Bài này là các script và node Python, không có gói colcon cần build: không chạy `colcon build`. Trước tiên cần môi
trường của bài 04 (thư mục cha `scripts/`, `maps/`: ROS 2 Humble, `ros-humble-turtlebot4-simulator`, Nav2, xem
README gốc của repo, hoặc `../scripts/00_install.sh`). Sau đó cài phần riêng của bài 05:

```bash
cd btth5
./scripts/setup.sh
```

`setup.sh` cài `ros-humble-turtlebot4-navigation`, `ros-humble-nav2-simple-commander` và `ros-humble-cv-bridge`
(cần sudo), tải `yolo11n.pt` và sinh world. Ngoài ra nó cài `ultralytics` và `numpy<2` vào thư mục `btth5/.pydeps`.
Lý do: `~/.local` trên máy này có numpy 2.x, còn `cv_bridge` của ROS Humble được biên dịch với numpy 1.x nên báo
lỗi `_ARRAY_API not found`. `scripts/env.sh` đưa `.pydeps` lên đầu `PYTHONPATH`, và chỉ làm vậy cho các node của bài
này. torch, OpenCV và scipy dùng bản đã cài sẵn. Nếu cài theo đề (`pip install ultralytics opencv-python`) trên
một máy dùng numpy 1.x thì cũng chạy được.

## Chạy demo (quay video)

Hướng dẫn chi tiết từng bước khi quay: [HUONG_DAN_QUAY_VIDEO.md](btth5/HUONG_DAN_QUAY_VIDEO.md).

Cách nhanh nhất là mở tất cả trong một lệnh:

```bash
./scripts/run_demo.sh            # Sim, Localization, Nav2, RViz, YOLO, Patrol
PATROL=0 ./scripts/run_demo.sh   # không tự chạy Patrol; tự gõ ./scripts/05_patrol.sh khi bắt đầu quay
```

Nếu muốn chạy từng bước (mỗi lệnh một terminal, gõ từ thư mục `btth5`):

### Phần 1 – Simulation và Localization

```bash
./scripts/01_sim.sh              # đợi robot xuất hiện trên dock (30–60 s)
./scripts/02_localization.sh     # AMCL + map_server với ../maps/map.yaml
./scripts/03_nav2.sh
./scripts/rviz.sh
./scripts/set_initial_pose.sh 0 0 0   # robot trên dock = gốc bản đồ (hoặc dùng 2D Pose Estimate)
```

World `room_objects` có tường, vách và hộp giống hệt world `room` dùng để quét bản đồ ở bài 04. World này có thêm
5 vật thể từ Gazebo Fuel, đặt sát tường để không chắn đường đi:

| Vật thể | Lớp COCO | Vị trí (m) |
|---|---|---|
| Standing person | person | (-1.5, 1.9), góc tây bắc phòng trái |
| Sofa | couch | (-1.55, -0.2), sát tường tây phòng trái |
| Fire hydrant | fire hydrant | (1.85, -2.1), góc tây nam hành lang giữa |
| Casual female | person | (3.0, 2.1), góc đông bắc hành lang giữa |
| Chair | chair | (1.8, 2.15), góc tây bắc hành lang giữa |

Kiểm tra trên RViz: bản đồ hiện đúng, robot nằm trên dock, các tia LaserScan (đỏ) trùng với tường đen của bản đồ
và particle AMCL co cụm quanh robot. Các vật thể mới không có trên bản đồ, nên chúng chỉ hiện trong scan và
costmap. AMCL vẫn định vị tốt vì phần lớn tia quét trúng tường.

### Phần 2 – Tọa độ goal

```bash
./scripts/get_goal_coords.sh       # bấm Publish Point trên RViz rồi click lên bản đồ -> (x, y)
./scripts/get_goal_coords.sh amcl  # pose hiện tại của robot (x, y, yaw)
```

Các goal đã chọn nằm trong `config/goals.yaml`. Hướng yaw được chọn để camera nhìn vào vật thể khi robot tới nơi:

| Goal | Khu vực | x (m) | y (m) | yaw | Camera nhìn thấy |
|---|---|---|---|---|---|
| 1 | phía bắc hành lang giữa (sau khe vách A) | 2.00 | 0.60 | 70° | người, ghế |
| 2 | phía nam hành lang giữa | 2.50 | -1.00 | -120° | trụ cứu hỏa |
| 3 | phòng trái (gần dock) | 0.60 | 0.60 | 180° | sofa, người |

Không đặt goal trong vùng phải (sau vách B): lối vào đó chỉ là một khe rộng khoảng 0.7 m giữa đầu vách B và hộp
`box_3`. Khi thử nghiệm, robot vào được, nhưng lúc quay ra thường kẹt sát đầu vách, vì DWB của TurtleBot 4 không được
phép lùi. Node vẫn hỗ trợ điểm trung gian: thêm `via: [[x, y, yaw], ...]` vào một goal thì goal đó được gửi bằng
`goThroughPoses()`.

### Phần 3 – Navigation tự động bằng code

```bash
./scripts/05_patrol.sh
```

`nodes/patrol_navigator.py` dùng `TurtleBot4Navigator` (lớp con của `nav2_simple_commander.BasicNavigator`, có
thêm undock). Các bước:

1. `setInitialPose` với pose trong `goals.yaml`, rồi `waitUntilNav2Active()`.
2. Nếu robot đang trên dock thì `undock()`.
3. Với mỗi goal: gọi `goToPose()` (action `/navigate_to_pose`). Goal nào có `via` thì gọi `goThroughPoses()`
   (action `/navigate_through_poses`). Trong lúc chờ `isTaskComplete()`, node in quãng
   đường còn lại mỗi 3 s. Nếu một goal quá 180 s thì node hủy goal đó. Tới nơi, robot đứng lại `dwell` giây để
   YOLO quan sát.
4. Cuối cùng node in bảng kết quả. Mã thoát là 0 nếu tới đủ mọi goal.

Sau khi chạy node thì không cần chuột hay teleop. Nav2 tự lập kế hoạch đường đi qua hai khe vách và tránh vật cản.

`config/nav2.yaml` là file tham số mặc định của `turtlebot4_navigation`, có 3 chỉnh sửa (tìm `# btth5` trong file):

- `inflation_radius` 0.45 → 0.35 và `cost_scaling_factor` 4 → 5. Khe giữa đầu vách B và hộp `box_3` chỉ rộng
  khoảng 0.7 m. Với inflation 0.45, vùng chi phí thấp còn khoảng 0.2 m, nên DWB liên tục báo `Failed to make progress`
  và chạy recovery khi vào hoặc ra vùng phải. Inflation nhỏ hơn cũng giúp robot đi êm hơn qua khe vách A.
- `movement_time_allowance` 10 → 20 s, vì mô phỏng trên máy này chạy chậm hơn thời gian thực.

### Phần 4 – YOLO

```bash
./scripts/04_yolo.sh                    # cửa sổ OpenCV "TurtleBot4 camera - YOLO"
./scripts/04_yolo.sh -p conf:=0.4       # đổi ngưỡng tin cậy (mặc định 0.3)
```

`nodes/yolo_detector.py` làm các bước sau:

1. Subscribe `/oakd/rgb/preview/image_raw` (320 x 240, QoS sensor data).
2. Đổi ảnh sang OpenCV bằng `CvBridge.imgmsg_to_cv2(..., 'bgr8')`.
3. Chạy `YOLO('yolo11n.pt').predict(...)`, ngưỡng `conf` 0.3, và chỉ giữ các lớp có trong world (`classes`: person, chair,
   couch, fire hydrant). Các box nhỏ hơn 150 px² (`min_area`) bị bỏ. Khi chưa lọc, ảnh mô phỏng (tường xám, sàn
   phẳng) hay bị nhận nhầm thành umbrella, traffic light, handbag, dining table. Để nhận mọi lớp COCO, chạy
   `./scripts/04_yolo.sh -p "classes:=['']"`. Node chỉ xử lý khung mới nhất, tối đa 10 Hz, và torch dùng 2 luồng
   CPU để không tranh CPU với Gazebo.
4. Vẽ bounding box, tên class và độ tự tin, sau đó:
   - hiện kết quả trong cửa sổ OpenCV (phóng to 2 lần);
   - publish lên `/yolo/image_annotated` (RViz hiện ở panel "YOLO Camera");
   - publish tóm tắt lên `/yolo/detections` (`std_msgs/String`);
   - in ra terminal mỗi khi tập vật thể thay đổi, hoặc 2 s một lần:

```
[yolo_detector]: [t=  85.3s] 2 vật thể (8 FPS)
    - person         conf=0.82  box=(205,45)-(230,75)
    - chair          conf=0.71  box=(240,60)-(270,120)
```

## Xử lý sự cố

| Triệu chứng | Cách xử lý |
|---|---|
| `AttributeError: _ARRAY_API not found` khi import cv_bridge | Chạy qua `scripts/*.sh` (nạp `.pydeps` có numpy<2), hoặc `source scripts/env.sh` trước |
| Lần đầu chạy `make_world.py` báo lỗi tải model | Lần đầu cần Internet để tải 5 model từ Gazebo Fuel về cache `~/.ignition/fuel`. Sau đó model được chép vào overlay và Gazebo chạy offline |
| Patrol đứng ở "Waiting for amcl_pose" | Localization chưa chạy, hoặc Nav2 chưa báo `Managed nodes are active` |
| YOLO FPS thấp | Máy chạy bằng CPU. Thử `-p threads:=4` hoặc `-p imgsz:=256` |
| Gazebo/Nav2 rất chậm khi chạy YOLO | Ultralytics tự đặt torch dùng 8 luồng ở lần `predict` đầu, và OpenBLAS cũng tạo luồng chờ bận. Node đặt lại `threads` sau warm-up và `04_yolo.sh` đặt `OMP/MKL/OPENBLAS_NUM_THREADS`; hãy chạy node qua script |
| Robot kẹt ở khe vào vùng phải, nhiều recovery | Kiểm tra Nav2 đang dùng `config/nav2.yaml` (chạy qua `03_nav2.sh`) |
| Còn tiến trình cũ, `/clock` có nhiều publisher, log báo "more than one action server" | `./scripts/stop_all.sh` (bản của bài 05 dọn cả node YOLO và Patrol) |
