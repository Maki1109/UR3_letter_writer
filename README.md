# ur3_llm_control — UR3e + tay gắp + camera điều khiển bằng LLM

Bài thực hành 03 (phát triển từ Bài 02) · Môn Tương tác Người – Robot · Ubuntu 22.04 · ROS 2 Humble · Gazebo Sim (Ignition Fortress) · MoveIt 2

Sinh viên: **Lương Nguyễn Việt Sang** — MSSV **23020762** (P = 62 mod 6 = 2 → Zone A = Yellow, Zone B = Red, Zone C = Blue)

---

## 1. Giới thiệu

Giữ nguyên kiến trúc của Bài 02 và bổ sung **tay gắp thật** và **camera**:

```
Natural Language Command ─► LLM Planner ─► Structured Plan (JSON) ─► Plan Validator ─► Robot Skills ─► MoveIt 2 ─► UR3e + Gripper (Gazebo)
                                ▲                                         │                  ▲
         Camera ─► WorldState ──┴──────── trạng thái khối / zone ─────────┘                  │
            ▲                                                                                 │
            └──────────── chụp lại trước mỗi pick/place và kiểm tra kết quả cuối ─────────────┘
```

Môi trường Gazebo: 1 UR3e, 1 tay gắp song song 2 ngón, 1 camera RGB nhìn xuống, 1 bàn, 3 zone (`zone_a`, `zone_b`, `zone_c`) và 5 khối (`red_cube`, `yellow_cube`, `blue_cube`, `green_cube`, `purple_cube`). Ban đầu `blue_cube` nằm ở `zone_b` và `green_cube` ở `zone_c`. Vì chỉ có 3 zone cho 5 khối nên zone đích hay bị chiếm.

Những điểm mới so với Bài 02:

| | Bài 02 | Bài 03 |
|---|---|---|
| Tay gắp | "gắp ảo": khối bám theo `tool0` bằng `set_pose` | **tay gắp 2 ngón** (`urdf/parallel_gripper.xacro`), điều khiển qua ros2_control. Vật được giữ bằng **lực kẹp + ma sát**; chương trình **không bao giờ đặt pose của vật** |
| Khối | vật kinematic, tắt trọng lực | vật động lực học (khối lượng 50 g, trọng lực, ma sát) |
| Vị trí khối | đọc từ `scene.yaml` | đo bằng **camera** (`perception.py`). `scene.yaml` chỉ chứa vị trí spawn mà launch dùng |
| Zone bị chiếm | dùng `temp_zone` cố định | **tìm chỗ trống** trên bàn (`find_free_position`) → `temporary_position` |
| Skill | home, pick, place, move_above, move_to_zone | **detect_objects, check_zone**, pick, place (zone hoặc `temporary_position`), home |
| Kiểm tra | validator mô phỏng kế hoạch | validator mô phỏng trên trạng thái camera, **tự chèn bước dọn zone** nếu LLM quên; camera **kiểm tra kết quả** sau khi chạy |

Nguyên tắc:

- **LLM chỉ hiểu câu lệnh, chọn skill, xác định tham số và sắp thứ tự.** LLM không sinh góc khớp, quỹ đạo hay toạ độ; validator chặn mọi khoá lạ như `joints` hoặc `trajectory`.
- **Kế hoạch luôn được kiểm tra trước khi thực thi** (schema + mô phỏng ngữ nghĩa trên trạng thái do camera đo).
- **Camera là nguồn duy nhất** cho vị trí khối và trạng thái zone. Robot chụp lại trước mỗi `pick`/`place` (vòng kín).

### Cấu trúc thư mục

```
ur3_llm_control/
├── launch/llm_robot.launch.py       # Gazebo + UR3e/tay gắp + bàn/5 khối/3 zone/camera + controllers + MoveIt 2 + RViz
├── urdf/
│   ├── parallel_gripper.xacro       # tay gắp 2 ngón tự xây dựng (+ ros2_control cho Gazebo)
│   └── ur_with_gripper.urdf.xacro   # UR (ur_description) + tay gắp gắn vào tool0
├── srdf/ur_with_gripper.srdf.xacro  # SRDF của UR + cặp link tay gắp bỏ qua va chạm
├── worlds/llm_table.world           # world gz-sim (physics, sensors/ogre2, ánh sáng)
├── config/
│   ├── scene.yaml                   # bàn, zone, màu khối, camera, tay gắp, chuyển động (+ vị trí spawn)
│   ├── ur_gripper_controllers.yaml  # controllers UR + gripper_controller
│   ├── student_config.yaml          # họ tên + MSSV
│   ├── llm_config.yaml              # backend LLM đang dùng (mặc định: Ollama)
│   └── llm_config_9router.yaml      # mẫu cấu hình 9Router / API kiểu OpenAI
├── prompts/system_prompt.txt        # prompt gửi LLM (tự điền skill/object/zone/trạng thái camera)
├── ur3_llm_control/
│   ├── commander_node.py            # node "llm_commander": camera → LLM → validator → executor → kiểm tra
│   ├── llm_planner.py               # gọi LLM, dựng prompt, vòng lặp LLM tự sửa
│   ├── task_validator.py            # whitelist skill + schema + mô phỏng + chèn bước dọn zone
│   ├── skill_executor.py            # chạy tuần tự kế hoạch hợp lệ
│   ├── robot_skills.py              # skill qua MoveIt 2 + điều khiển tay gắp
│   ├── perception.py                # nhận dạng khối từ ảnh (numpy): màu → blob → toạ độ bàn + yaw
│   ├── camera.py                    # lớp ROS bọc perception: ảnh mới, ảnh debug, marker RViz
│   ├── free_space.py                # find_free_position(): chỗ trống trên bàn
│   └── scene_config.py              # đọc YAML, tính P, WorldState
├── test/test_planning_pipeline.py   # 27 unit test offline (không cần ROS)
└── package.xml, setup.py, setup.cfg, resource/
```

---

## 2. Yêu cầu hệ thống

- Ubuntu 22.04 (hoặc WSL2 Ubuntu 22.04 + WSLg), ROS 2 Humble.
- Khuyến nghị ≥ 16 GB RAM. Camera Gazebo cần render bằng OpenGL; GPU tích hợp (Intel) là đủ. Không cần GPU cho LLM, nhưng chạy trên CPU sẽ chậm (xem mục 7).
- Docker (nếu dùng Ollama) hoặc Node.js 18+ (nếu dùng 9Router).

> **Anaconda/Miniconda:** đảm bảo `/usr/bin` đứng trước conda trong `$PATH` khi build/chạy
> (`export PATH=/usr/bin:$PATH`) hoặc `conda deactivate`.
> Phần nhận dạng ảnh chỉ dùng numpy (không dùng OpenCV/cv_bridge), nên vẫn chạy được khi máy có numpy 2.x cài bằng pip làm hỏng `cv2` của apt.

---

## 3. Cài đặt

### 3.1 ROS 2 Humble, MoveIt 2, Gazebo Sim, UR

```bash
sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions python3-rosdep git xterm \
    ros-humble-ur ros-humble-moveit \
    ros-humble-ros-gz-sim ros-humble-ros-gz-bridge ros-humble-gz-ros2-control \
    ros-humble-ros2-controllers ros-humble-ros-gz-image python3-requests python3-yaml python3-numpy python3-pytest

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && source ~/.bashrc
```

### 3.2 Workspace: gói mô phỏng UR, pymoveit2 và gói này

```bash
mkdir -p ~/ur3_ws/src && cd ~/ur3_ws/src

# Cấu hình UR cho Gazebo Sim (chưa có sẵn trên apt)
git clone --depth 1 -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation.git

# pymoveit2: thư viện Python gọi MoveIt 2 (không có bản apt)
git clone https://github.com/AndrejOrsula/pymoveit2.git
(cd pymoveit2 && git checkout 20e6ee5)      # phiên bản đã dùng khi phát triển

# Gói của bài (nhánh assignments_3), đặt tên thư mục là ur3_llm_control
git clone -b assignments_3 https://github.com/Maki1109/UR3_letter_writer.git ur3_llm_control

cd ~/ur3_ws
export PATH=/usr/bin:$PATH
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
echo "source ~/ur3_ws/install/setup.bash" >> ~/.bashrc && source ~/.bashrc
```

Kiểm tra mô phỏng UR gốc chạy được (Ctrl+C để tắt):

```bash
ros2 launch ur_simulation_gz ur_sim_moveit.launch.py ur_type:=ur3e
```

### 3.3 Khai báo MSSV

Sửa `config/student_config.yaml` (đường dẫn `~/ur3_ws/src/ur3_llm_control/config/`):

```yaml
student_name: "Luong Nguyen Viet Sang"
student_id: "23020762"
```

Chương trình tự tính `P = (2 số cuối) mod 6` và bảng Zone → màu:

| P | Zone A | Zone B | Zone C |
|---|--------|--------|--------|
| 0 | Red    | Yellow | Blue   |
| 1 | Red    | Blue   | Yellow |
| 2 | Yellow | Red    | Blue   |
| 3 | Yellow | Blue   | Red    |
| 4 | Blue   | Red    | Yellow |
| 5 | Blue   | Yellow | Red    |

MSSV 23020762 → `62 mod 6 = 2` → A = Yellow, B = Red, C = Blue.

### 3.4 Cài LLM (chọn một trong hai)

Cấu hình đang dùng nằm ở `config/llm_config.yaml` (mặc định là **Ollama**). Sau khi sửa file cần `colcon build` lại nếu không dùng `--symlink-install`.

**Cách A — Ollama (chạy cục bộ, miễn phí, không cần API key) — mặc định**

```bash
# cài Docker nếu chưa có: https://docs.docker.com/engine/install/ubuntu/
sudo docker run -d --name ollama -p 11434:11434 ollama/ollama:latest
sudo docker exec ollama ollama pull mistral       # ~4.4 GB
curl -s http://localhost:11434/api/tags           # phải thấy "mistral:latest"
```

Các lần sau chỉ cần `sudo docker start ollama`. `config/llm_config.yaml` mặc định:

```yaml
base_url: "http://localhost:11434"
api_type: "ollama"
model: "mistral"
timeout_sec: 180
```

**Cách B — 9Router / API kiểu OpenAI (`/v1/chat/completions`)**

```bash
curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash - && sudo apt install -y nodejs
npm install -g 9router
9router                                            # dashboard: http://localhost:20128/dashboard
# Trong dashboard: kết nối 1 provider, copy API key, chọn model
export NINEROUTER_API_KEY="<api-key>"
cp ~/ur3_ws/src/ur3_llm_control/config/llm_config_9router.yaml \
   ~/ur3_ws/src/ur3_llm_control/config/llm_config.yaml      # sửa "model" cho khớp dashboard
```

---

## 4. Chạy hệ thống

Cần 2 terminal (đều đã `source ~/ur3_ws/install/setup.bash`, và `export PATH=/usr/bin:$PATH` nếu dùng conda).

**Terminal 1 — mô phỏng + MoveIt 2:**

```bash
ros2 launch ur3_llm_control llm_robot.launch.py
```

Đợi đến khi log báo `Configured and activated gripper_controller` và `You can start planning now!` (khoảng 10–30 s).

| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `ur_type` | `ur3e` | `ur3` hoặc `ur3e` |
| `gazebo_gui` | `true` | `false` = Gazebo không giao diện (camera vẫn hoạt động; dùng khi GUI bị crash) |
| `launch_rviz` | `true` | `false` để không mở RViz |
| `start_commander` | `false` | `true` = tự mở `llm_commander` trong cửa sổ `xterm` |

**Terminal 2 — node điều khiển bằng ngôn ngữ tự nhiên:**

```bash
ros2 run ur3_llm_control llm_commander
```

Khi khởi động, node mở tay gắp, đưa robot về home (tư thế dựng đứng, không che camera), chụp ảnh và in trạng thái thế giới đo được rồi chờ lệnh ở `Command>`. Chạy một lệnh rồi thoát:

```bash
ros2 run ur3_llm_control llm_commander --ros-args -p command:="Put the red cube in Zone B."
```

**Quan sát camera:** `ros2 run rqt_image_view rqt_image_view` → chọn `/overhead_camera/image` (ảnh gốc) hoặc `/perception/debug_image` (có khung các khối đã nhận dạng). Trong RViz: *Add → MarkerArray*, topic `/perception/markers` (khối đo được; zone xanh = trống, đỏ = có vật).

> Nếu chạy lại, hãy tắt hẳn launch cũ trước (Ctrl+C). Còn sót tiến trình (`move_group`, `ign gazebo`, `parameter_bridge`) thì `pkill -9` rồi launch lại. **Khởi động lại launch để đặt lại cảnh ban đầu.**

---

## 5. Demo

### Tình huống bắt buộc: zone đích đang bị chiếm

Cảnh ban đầu: `blue_cube` nằm ở `zone_b`. Kết quả chạy thật (LLM: Ollama `mistral` 7B):

```
USER COMMAND:
Put the red cube in Zone B.

CAMERA (world state):
- red_cube: on the table (not in any zone)
- yellow_cube: on the table (not in any zone)
- blue_cube: in zone_b
- green_cube: in zone_c
- purple_cube: on the table (not in any zone)
- zone_a: free
- zone_b: occupied by blue_cube
- zone_c: occupied by green_cube
- robot is holding: nothing

LLM PLAN:
  check_zone(zone_b)
  pick(blue_cube)
  place(blue_cube, temporary_position)
  pick(red_cube)
  place(red_cube, zone_b)
  home()

VALIDATION: PASSED

EXECUTION:
check_zone(zone_b) .................... SUCCESS   [zone_b occupied by blue_cube]
pick(blue_cube) ....................... SUCCESS   [at x=0.360 y=0.000 yaw=3deg, finger gap 40 mm]
place(blue_cube, temporary_position) .. SUCCESS   [free spot x=0.250 y=0.000]
pick(red_cube) ........................ SUCCESS   [at x=0.240 y=0.120 yaw=15deg, finger gap 40 mm]
place(red_cube, zone_b) ............... SUCCESS
home() ................................ SUCCESS

TASK SUCCESS

CAMERA VERIFICATION:
  blue_cube: expected temporary_position, camera sees table ... OK
  red_cube: expected zone_b, camera sees zone_b ... OK
  zones: zone_a=free, zone_b=red_cube, zone_c=green_cube
```

Luồng: camera thấy `zone_b` bị chiếm → `find_free_position()` chọn chỗ trống (cách mọi khối ≥ 10 cm, cách mọi zone ≥ 10 cm, trong tầm với, gần vật nhất) → dời `blue_cube` ra → gắp `red_cube` → đặt vào `zone_b` → home → camera kiểm tra.

**Lưới an toàn:** nếu LLM quên bước dọn zone, validator tự chèn các bước đó và đánh dấu:

```
Command> /plan {"plan":[{"skill":"pick","object":"purple_cube"},{"skill":"place","object":"purple_cube","zone":"zone_a"},{"skill":"home"}]}
PLAN (LLM + validator):
  check_zone(zone_a)   <- added by validator (zone occupied)
  pick(yellow_cube)   <- added by validator (zone occupied)
  place(yellow_cube, temporary_position)   <- added by validator (zone occupied)
  pick(purple_cube)
  place(purple_cube, zone_a)
  home()
```

(Tắt tính năng này bằng `-p auto_resolve_occupied_zones:=false`; khi đó kế hoạch bị từ chối với lỗi `ZONE_OCCUPIED` và lỗi được gửi lại cho LLM sửa.)

### Nhiều vật, nhiều bước: sắp xếp theo MSSV

```
Command> Sắp xếp các khối theo MSSV của tôi.
```

Với MSSV 23020762 (A=yellow, B=red, C=blue) và trạng thái sau demo trên (B=red, C=green), LLM sinh kế hoạch dời `green_cube` ra chỗ trống, đặt `yellow_cube` → A, `blue_cube` → C và không động vào `red_cube` (đã đúng chỗ). Camera kiểm tra: `zone_a=yellow_cube, zone_b=red_cube, zone_c=blue_cube`.

Các câu lệnh khác (không có câu nào được hard-code): `Move the purple cube to zone A.` · `Hãy lấy khối màu xanh lá và đặt nó vào ô C.` · `Put the yellow block somewhere free.`

### Kiểm tra Plan Validator (kế hoạch sai bị từ chối)

```
Command> Put the orange cube in zone D.
Command> /plan {"plan":[{"skill":"throw","object":"red_cube"}]}
Command> /plan {"plan":[{"skill":"home","joints":[0,0,0,0,0,0]}]}
Command> /plan {"plan":[{"skill":"pick","object":"red_cube"},{"skill":"pick","object":"blue_cube"}]}
```

Kết quả: `VALIDATION: REJECTED ... TASK REJECTED (nothing was executed)`.

### Lệnh đặc biệt trong `Command>`

| Lệnh | Ý nghĩa |
|---|---|
| `/state` | chụp ảnh, in trạng thái (kèm toạ độ x, y, yaw từ camera) |
| `/student` | in MSSV, P và bố trí đích |
| `/plan <json>` | đưa kế hoạch JSON thẳng vào validator (bỏ qua LLM) |
| `/home`, `/open`, `/close` | về home / mở / đóng tay gắp |
| `/help`, `/quit` | trợ giúp / thoát |

---

## 6. Kiểm thử offline

Không cần ROS hay Gazebo:

```bash
cd ~/ur3_ws/src/ur3_llm_control && python3 -m pytest test -v      # 27 passed
```

Gồm: validator (hợp lệ, từ chối, tự chèn bước dọn zone, `temporary_position`), nhận dạng khối trên ảnh tổng hợp đúng phối cảnh của camera (sai số < 3 mm, < 3°), `find_free_position`, prompt, vòng lặp LLM tự sửa.

---

## 7. Lưu ý về LLM chạy trên CPU

- Lệnh đầu tiên sau khi khởi động lạnh phải nạp mô hình (~1 phút); `llm_commander` chạy *warm-up* nền khi khởi động.
- Mỗi lệnh mất khoảng 15–60 giây để lập kế hoạch (mô hình 7B chạy CPU). Mỗi pick/place mất khoảng 5–10 giây.
- Prompt được sắp *phần tĩnh trước, trạng thái camera sau cùng* để Ollama tái sử dụng KV-cache.

---

## 8. Xử lý sự cố

| Hiện tượng | Nguyên nhân / cách xử lý |
|---|---|
| `No camera image on /overhead_camera/image` | Camera chưa render: xem log Terminal 1 (lỗi OpenGL/ogre2). Kiểm tra `ign topic -l \| grep overhead`. |
| `OBJECT_NOT_FOUND` | Camera không thấy khối (rơi khỏi bàn, bị che). Gõ `/state` để xem; khởi động lại launch để đặt lại cảnh. |
| `GRASP_FAILED` | Ngón khép hết mà không kẹp được vật (khe hở < 30 mm). `pick` tự thử lại 1 lần sau khi chụp lại ảnh. |
| `NO_FREE_POSITION` | Bàn quá chật; nới `free_space` trong `scene.yaml`. |
| `LLM call failed: ... Connection refused` | LLM chưa chạy: `sudo docker start ollama` (hoặc chạy `9router`). |
| `LLM API HTTP 404` | `api_type` không khớp backend: `ollama` cho Ollama, `openai` cho 9Router. |
| `Waiting for MoveIt move_group` mãi | MoveIt chưa lên; xem log Terminal 1. |
| Gazebo GUI crash (không có GPU) | Chạy `gazebo_gui:=false`; camera vẫn render được, quan sát bằng RViz / rqt_image_view. |
| `PLANNING_FAILED` thỉnh thoảng | OMPL ngẫu nhiên; thử lại lệnh. |
| `ModuleNotFoundError` khi build/chạy | Đang dùng Python của conda: `export PATH=/usr/bin:$PATH`. |

---

## 9. Ghi chú thiết kế

- **Tay gắp**: hai ngón trượt trên khớp tịnh tiến, điều khiển vị trí qua `gripper_controller` (JointGroupPositionController) và `ign_ros2_control`. Ngón trái là *ngón tựa* (giới hạn lực 40 N), được đưa tới đúng nửa cạnh khối. Ngón phải là *ngón kẹp* (15 N), ra lệnh đóng hết nên bị khối chặn lại và ép khối vào ngón tựa. Nếu hai ngón cùng giới hạn lực thì khi cùng bị chặn, chúng cân bằng phiếm định: cả cặp ngón + khối trôi sang một bên khi mang đi (đo được lệch ~2 cm). Lực không đều nhau làm khối luôn nằm giữa tay. Kết quả kẹp được kiểm tra bằng khe hở ngón: ≈ 40 mm là đang giữ vật, < 30 mm là trượt.
- **Camera / nhận dạng** (`perception.py`): RGB → HSV, tách màu theo ngưỡng hue của từng khối, lấy thành phần liên thông lớn nhất, giữ mặt trên (sáng nhất), chiếu từng pixel lên mặt phẳng `z = mặt trên khối` bằng nội tham số (`camera_info`) và pose camera (ngoại tham số cố định trong `scene.yaml`). Yaw lấy từ hình chữ nhật bao nhỏ nhất. Sai số đo trong Gazebo < 1 mm và < 1°. Một khối bị che quá nửa thì giữ vị trí cũ; nếu cần gắp vật đang bị che, robot về home rồi chụp lại.
- **Zone**: vị trí zone là hạ tầng cố định (`scene.yaml`); trạng thái *trống / có vật* suy ra từ vị trí khối camera đo (tâm khối cách tâm zone < 4.5 cm).
- **MoveIt 2**: URDF có tay gắp nên MoveIt kiểm tra va chạm của cả tay gắp. Bàn và các khối (từ camera) nằm trong Planning Scene; khối đang kẹp được attach vào `tool0`. Hạ/nâng dùng đường thẳng Cartesian. Di chuyển tự do giải IK với seed cố định rồi lập kế hoạch theo không gian khớp, loại các quỹ đạo xoay cả vòng.
- Danh sách skill hợp lệ nằm ở một nơi duy nhất (`SKILL_SPECS` trong `task_validator.py`); prompt, validator và bảng điều phối của executor đều dựa vào đó.
