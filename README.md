# ur3_llm_control — Điều khiển UR3e bằng LLM và Skill-based Planning

Bài thực hành 02 · Môn Tương tác Người – Robot · Ubuntu 22.04 · ROS 2 Humble · Gazebo Sim (Ignition Fortress) · MoveIt 2

Sinh viên: **Lương Nguyễn Việt Sang** — MSSV **23020762** (P = 62 mod 6 = 2 → Zone A = Yellow, Zone B = Red, Zone C = Blue)

---

## 1. Giới thiệu

Hệ thống cho phép ra lệnh cho cánh tay robot **UR3e** bằng **ngôn ngữ tự nhiên** (tiếng Việt hoặc tiếng Anh), ví dụ:

```
Please put the red cube in zone B.
Hãy lấy khối màu vàng và đặt nó vào ô C.
Arrange all objects according to my student ID.
```

Robot gắp/đặt ba khối (`red_cube`, `yellow_cube`, `blue_cube`) giữa các vùng trên bàn (Zone A/B/C và một vùng tạm `temp_zone`) trong mô phỏng Gazebo, mọi chuyển động do MoveIt 2 lập kế hoạch.

```
Câu lệnh tự nhiên ─► LLM Planner ─► Kế hoạch JSON ─► Plan Validator ─► Skill Executor ─► Robot Skills ─► MoveIt 2 ─► UR3e (Gazebo)
                        ▲                                   │
                        └──────── lỗi: gửi lại cho LLM sửa ─┘  (repair loop)
```

Nguyên tắc chính:

- **LLM chỉ chọn và sắp xếp skill** (`home`, `pick`, `place`, `move_above`, `move_to_zone`). Nó không bao giờ sinh góc khớp, quỹ đạo hay toạ độ.
- **Plan Validator** kiểm tra kế hoạch trước khi chạy (schema + mô phỏng ngữ nghĩa): chặn skill lạ, khoá lạ như `joints`, vật/vùng không tồn tại, đặt chồng vật, gắp khi tay đang đầy… Kế hoạch sai bị từ chối, robot không chuyển động.
- **Nhiệm vụ cá nhân theo MSSV**: `P = XX mod 6` (XX là hai chữ số cuối), tính tự động từ `config/student_config.yaml`.
- Mức nâng cao có xử lý bế tắc bằng **vùng tạm** (`temp_zone`).

### Cấu trúc thư mục

```
ur3_llm_control/
├── launch/llm_robot.launch.py       # Gazebo + UR3e + bàn/khối/vùng + MoveIt 2 + RViz
├── worlds/llm_table.world           # world gz-sim tối thiểu
├── config/
│   ├── scene.yaml                   # vị trí bàn, khối, vùng, tham số chuyển động
│   ├── student_config.yaml          # họ tên + MSSV   ← sửa theo MSSV của bạn
│   ├── llm_config.yaml              # backend LLM đang dùng (mặc định: Ollama)
│   └── llm_config_9router.yaml      # mẫu cấu hình cho 9Router / API kiểu OpenAI
├── prompts/system_prompt.txt        # prompt gửi LLM (tự điền skill/object/zone/trạng thái)
├── ur3_llm_control/
│   ├── commander_node.py            # node "llm_commander": nhận lệnh, in kết quả
│   ├── llm_planner.py               # gọi LLM, dựng prompt, vòng lặp LLM tự sửa
│   ├── task_validator.py            # whitelist skill + schema + mô phỏng ngữ nghĩa
│   ├── skill_executor.py            # chạy tuần tự kế hoạch hợp lệ
│   ├── robot_skills.py              # home/pick/place/... qua MoveIt 2 (pymoveit2)
│   ├── gazebo_bridge.py             # "tay gắp ảo": khối bám theo tool0 trong Gazebo
│   └── scene_config.py              # đọc YAML, tính P, WorldState
├── test/test_planning_pipeline.py   # 19 unit test offline (không cần ROS)
├── package.xml, setup.py, setup.cfg, resource/
└── README.md
```

---

## 2. Yêu cầu hệ thống

- Ubuntu 22.04 (hoặc WSL2 Ubuntu 22.04 + WSLg), ROS 2 Humble.
- Khuyến nghị ≥ 16 GB RAM. Không cần GPU (LLM chạy được trên CPU, nhưng chậm: xem mục 7).
- Docker (nếu dùng Ollama) hoặc Node.js 18+ (nếu dùng 9Router).

> **Anaconda/Miniconda:** đảm bảo `/usr/bin` đứng trước conda trong `$PATH` khi build/chạy
> (`export PATH=/usr/bin:$PATH`) hoặc `conda deactivate`, nếu không `colcon`/`ros2 launch` sẽ lỗi `ModuleNotFoundError`.

---

## 3. Cài đặt

### 3.1 ROS 2 Humble, MoveIt 2, Gazebo Sim, UR

```bash
sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions python3-rosdep git xterm \
    ros-humble-ur ros-humble-moveit \
    ros-humble-ros-gz-sim ros-humble-ros-gz-bridge ros-humble-gz-ros2-control \
    ros-humble-ros2-controllers python3-requests python3-yaml python3-pytest

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

# Gói của bài (nhánh assignments_2), đặt tên thư mục là ur3_llm_control
git clone -b assignments_2 https://github.com/Maki1109/UR3_letter_writer.git ur3_llm_control

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

Đợi đến khi Gazebo hiện robot UR3e, bàn gỗ, 3 khối đỏ/vàng/xanh và các ô vùng, và log `move_group` báo `You can start planning now!`. Trong RViz có thể *Add → MarkerArray*, topic `/scene_markers` để xem nhãn vùng.

Tuỳ chọn launch:

| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `ur_type` | `ur3e` | `ur3` hoặc `ur3e` |
| `gazebo_gui` | `true` | `false` = chạy Gazebo không giao diện (dùng khi GUI bị crash do không có GPU) |
| `launch_rviz` | `true` | `false` để không mở RViz |
| `start_commander` | `false` | `true` = tự mở `llm_commander` trong cửa sổ `xterm` |

**Terminal 2 — node điều khiển bằng ngôn ngữ tự nhiên:**

```bash
ros2 run ur3_llm_control llm_commander
```

Node in thông tin MSSV, trạng thái thế giới, rồi chờ lệnh ở dấu nhắc `Command>`. Có thể chạy một lệnh rồi thoát:

```bash
ros2 run ur3_llm_control llm_commander --ros-args -p command:="Move the blue cube to zone C"
```

> Nếu chạy lại, hãy tắt hẳn launch cũ trước (Ctrl+C). Nếu còn tiến trình cũ (`move_group`, `ign gazebo`, `parameter_bridge`) thì `pkill -9` chúng rồi launch lại.

---

## 5. Demo

### Mức cơ bản

```
Command> Please put the red cube in zone B.
============================================================
USER COMMAND:
Please put the red cube in zone B.

LLM PLAN:
pick(red_cube)
place(red_cube, zone_b)
home()

VALIDATION: PASSED

EXECUTION:
pick(red_cube) ............ SUCCESS
place(red_cube, zone_b) ... SUCCESS
home() .................... SUCCESS

TASK SUCCESS
```

Thử thêm các cách diễn đạt khác (không có câu nào được hard-code):
`Hãy lấy khối màu vàng và đặt nó vào ô C.` · `Đưa vật màu đỏ sang vùng B.` · `Move the blue cube to zone A.`

### Mức nâng cao

```
Command> Arrange all objects according to my student ID.
```

LLM nhận trạng thái hiện tại và bố trí đích theo MSSV rồi tự sinh chuỗi `pick`/`place`. Nếu vùng đích đang bị vật khác chiếm, kế hoạch dùng `temp_zone` để dời vật chặn. Kịch bản demo hay: chạy ba lệnh `red → B`, `yellow → C`, `blue → A` (được A=blue, B=red, C=yellow), rồi ra lệnh sắp xếp. Với MSSV 23020762 (đích A=yellow, B=red, C=blue) sẽ thấy kế hoạch tối ưu 3 lần di chuyển:

```
pick(blue_cube)   → place(blue_cube, temp_zone)
pick(yellow_cube) → place(yellow_cube, zone_a)
pick(blue_cube)   → place(blue_cube, zone_c)
home()
```

### Kiểm tra Plan Validator (kế hoạch sai bị từ chối)

```
Command> Put the green cube in zone D.
Command> /plan {"plan":[{"skill":"throw","object":"red_cube"}]}
Command> /plan {"plan":[{"skill":"home","joints":[0,0,0,0,0,0]}]}
Command> /plan {"plan":[{"skill":"pick","object":"red_cube"},{"skill":"pick","object":"blue_cube"}]}
```

`/plan` đưa JSON thẳng vào validator (bỏ qua LLM). Kết quả: `VALIDATION: REJECTED ... TASK REJECTED (nothing was executed)`.

### Lệnh đặc biệt trong `Command>`

| Lệnh | Ý nghĩa |
|---|---|
| `/state` | in trạng thái thế giới (vật nào ở đâu, tay đang cầm gì) |
| `/student` | in MSSV, P và bố trí đích |
| `/plan <json>` | đưa kế hoạch JSON thẳng vào validator |
| `/home` | đưa robot về tư thế home |
| `/help`, `/quit` | trợ giúp / thoát |

---

## 6. Kiểm thử offline

Không cần ROS hay Gazebo:

```bash
cd ~/ur3_ws/src/ur3_llm_control && python3 -m pytest test -v      # 19 passed
```

---

## 7. Lưu ý về LLM chạy trên CPU

- **Lệnh đầu tiên sau khi khởi động lạnh** phải nạp mô hình vào RAM và đánh giá prompt (~1 phút). `llm_commander` chạy bước *warm-up* nền khi khởi động nên thường không phải chờ; `timeout_sec: 180` đủ cho trường hợp này.
- Mỗi lệnh sau đó mất khoảng 12–45 giây cho bước lập kế hoạch (mô hình 7B chạy CPU ~5–6 token/s). Robot chạy một skill chỉ mất vài giây.
- Prompt được sắp xếp *phần tĩnh trước, phần động (trạng thái thế giới) sau cùng* để Ollama tái sử dụng KV-cache giữa các lệnh; đừng chèn nội dung thay đổi vào đầu prompt.

---

## 8. Xử lý sự cố

| Hiện tượng | Nguyên nhân / cách xử lý |
|---|---|
| `LLM call failed: ... Connection refused` | LLM chưa chạy: `sudo docker start ollama` (hoặc chạy `9router`); kiểm tra `base_url` trong `config/llm_config.yaml`. |
| `LLM call failed: ... Read timed out` | Máy chậm/lần gọi đầu: tăng `timeout_sec`; đảm bảo không có tiến trình nặng khác. |
| `LLM API HTTP 404` | `api_type` không khớp backend: `ollama` cho Ollama, `openai` cho 9Router. |
| `HTTP 401` (9Router) | Chưa `export NINEROUTER_API_KEY=...` trong terminal chạy commander. |
| `Waiting for MoveIt move_group` mãi | MoveIt chưa lên; xem log Terminal 1 (đợi ~30–60 s). |
| Gazebo GUI tự thoát/crash (không có GPU) | Chạy `gazebo_gui:=false`; vẫn quan sát được robot bằng RViz. |
| `PLANNING_FAILED` thỉnh thoảng | OMPL ngẫu nhiên; thử lại lệnh. Nếu luôn fail ở một vùng, kiểm tra vị trí trong `config/scene.yaml`. |
| `gz-sim 'set_pose' service not found` | Bridge/world chưa lên; khối sẽ không di chuyển trong Gazebo. Khởi động lại launch. |
| Khối lệch vị trí sau khi khởi động lại commander | Vị trí ban đầu lấy từ `scene.yaml`; khởi động lại cả launch để spawn lại khối. |
| `ModuleNotFoundError` khi build/chạy | Đang dùng Python của conda: `export PATH=/usr/bin:$PATH`. |

---

## 9. Ghi chú thiết kế

- Danh sách skill hợp lệ nằm ở một nơi duy nhất (`SKILL_SPECS` trong `task_validator.py`); prompt, validator và bảng điều phối của executor đều dựa vào đó.
- Mọi chuyển động qua MoveIt 2: kiểm tra giới hạn khớp, tự va chạm và va chạm môi trường (bàn và ba khối nằm trong Planning Scene; khối đang cầm được attach vào `tool0`). Hạ/nâng dùng đường thẳng Cartesian (yêu cầu ≥ 95% điểm giải được IK).
- Bước di chuyển tự do giải IK với seed là tư thế home rồi lập kế hoạch theo không gian khớp, và loại các quỹ đạo lệch nhánh (khớp xoay quá lớn) — tránh nghiệm IK lệch $2\pi$ làm robot xoay cả vòng.
- UR3e không có tay gắp: khối là vật kinematic trong Gazebo, bám theo `tool0` qua dịch vụ `set_pose` khi "gắp" (xem `gazebo_bridge.py`).
- Vị trí ban đầu của các khối lấy từ `scene.yaml` (đề bài cho phép khai báo trước), không dùng camera.
