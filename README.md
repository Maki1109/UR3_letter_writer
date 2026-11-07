# Bài thực hành 07 – Phân vùng bản đồ, gắn nhãn khu vực và điều hướng TurtleBot 4 bằng giọng nói

Bản đồ bài 04 (`../maps/map.pgm`) → **OpenCV** (threshold, morphology, distance transform, watershed) → 4 khu vực
có tên và điểm đích (`config/zones.yaml`) → micro → **Whisper** / Vosk → `/voice_command` → phân tích câu lệnh
→ **Nav2 `NavigateToPose`** → TurtleBot 4 trong Ignition Gazebo (ROS 2 Humble).

![Phân vùng](figures/zones.png)

## Cấu trúc

```
btth7/
  tools/segment_map.py            Phần 1+2: phân vùng map.pgm, gắn nhãn, chọn điểm đích -> config/zones.yaml + hình
  nodes/
    speech_recognition_node.py    Phần 3: micro (arecord) -> VAD -> Whisper/Vosk -> /voice_command
    command_parser.py             phân tích câu lệnh tiếng Việt -> lộ trình qua các khu vực (không phụ thuộc ROS)
    voice_nav_node.py             Phần 4: /voice_command -> lộ trình nhiều điểm -> NavigateToPose; xem trước
                                  đường đi (/voice_nav/planned_path), /zones/markers cho RViz
    map_view_node.py              Mở rộng: cửa sổ bản đồ riêng (khu vực, robot, đường dự kiến, trạng thái, bấm để đi)
  config/
    segmentation.yaml             tham số phân vùng + nhãn (tên, bí danh) cho từng khu vực
    zones.yaml                    KẾT QUẢ phân vùng (sinh tự động): id, tên, tâm, điểm đích, đường bao
    voice_nav.rviz                RViz: bản đồ + khu vực tô màu, robot, LaserScan, costmap, đường đi
  figures/zones.png, zones_steps.png   bản đồ đã phân vùng và các bước xử lý ảnh
  tests/
    test_command_parser.py        26 câu lệnh mẫu (đúng, không dấu, nhiều điểm, sai khu vực, dừng, câu rác)
    send_and_wait.py              gửi câu lệnh văn bản, đợi /voice_nav/status (test_run.sh dùng)
  models/vosk-model-small-vn-0.4  model Vosk tiếng Việt (setup.sh tải về)
  scripts/
    setup.sh          cài phụ thuộc (một lần)
    segment.sh        Phần 1+2: chạy tools/segment_map.py
    01_sim.sh         Gazebo + TurtleBot 4, world room_objects (bài 05)
    02_localization.sh, 03_nav2.sh   AMCL + map_server (../maps/map.yaml), Nav2 (../btth5/config/nav2.yaml)
    rviz.sh           RViz với config/voice_nav.rviz
    04_voice_nav.sh   Phần 4: node điều hướng
    05_speech.sh      Phần 3: node nhận dạng giọng nói
    say.sh            gửi một câu lệnh dạng văn bản (bỏ qua micro)
    mic_test.sh       ghi 4 s từ micro, phát lại, in văn bản nhận dạng được và kết quả phân tích
    06_map_view.sh    Mở rộng: cửa sổ bản đồ riêng
    run_demo.sh       mở mọi bước trong các tab terminal, đúng thứ tự
    test_run.sh       chạy cả hệ thống ở nền và gửi 6 câu lệnh kiểm thử (log vào logs/)
    stop_all.sh       tắt sạch Gazebo, Nav2 và các node
```

## Cài đặt

Yêu cầu: Ubuntu 22.04, ROS 2 Humble, môi trường của bài 04 và 05 (`../scripts/00_install.sh`,
`../btth5/scripts/setup.sh`), micro.

```bash
cd btth7
./scripts/setup.sh
```

Không cần `colcon build`: các node là script Python chạy bằng `/usr/bin/python3` (có `rclpy`). `setup.sh` cài
`openai-whisper`, `vosk` và `numpy<2` vào `btth7/.pydeps`, tải model Whisper `small` và Vosk tiếng Việt, rồi chạy
phân vùng. Giống bài 05: `~/.local` trên máy này có numpy 2.x, còn OpenCV/cv2 hệ thống và numba (Whisper cần) dùng
numpy 1.x, nên `scripts/env.sh` đưa `.pydeps` lên đầu `PYTHONPATH` chỉ cho các node của bài này. torch dùng bản có
sẵn. Thu âm dùng `arecord` (gói `alsa-utils`) nên không cần PyAudio/sounddevice.

## Chạy demo

```bash
./scripts/run_demo.sh                 # Sim, Localization, Init pose, Nav2, RViz, Voice Nav, Map view, Speech
ENGINE=vosk ./scripts/run_demo.sh     # dùng Vosk
ENGINE=text ./scripts/run_demo.sh     # gõ câu lệnh trong tab Speech thay cho micro
```

Đợi 1–3 phút tới khi tab Speech báo `Đang nghe...` (giữ im lặng 1 s đầu để node đo ồn nền), rồi nói, ví dụ
"Đi đến phòng thí nghiệm". Tab Voice Nav in lộ trình, quãng đường còn lại và kết quả; RViz tô đậm khu vực đang tới.
Chạy từng bước thì mỗi lệnh một terminal (gõ từ thư mục `btth7`):

```bash
./scripts/01_sim.sh                   # đợi robot xuất hiện trên dock (30–60 s)
./scripts/02_localization.sh
../btth5/scripts/set_initial_pose.sh 0 0 0
./scripts/03_nav2.sh                  # đợi "Managed nodes are active"
./scripts/rviz.sh
./scripts/04_voice_nav.sh
./scripts/05_speech.sh                # hoặc: ./scripts/say.sh "Đi đến khu vực A"
```

## Phần 1 – Phân vùng bản đồ (`tools/segment_map.py`)

```bash
./scripts/segment.sh          # --show để mở cửa sổ xem
```

![Các bước](figures/zones_steps.png)

1. **Threshold**: pixel 254 là trống; 0 (vật cản) và 205 (chưa biết) đều coi là vật cản.
2. **Morphology**: `MORPH_CLOSE` với kernel 0.25 m bịt các lỗ nhỏ trên tường (chỗ tia laser lọt qua khi quét), rồi
   chỉ giữ thành phần trống liên thông lớn nhất, là vùng robot tới được.
3. **Bỏ đồ đạc**: các khối vật cản rời nhỏ hơn 1 m² (3 hộp) được coi là trống, chỉ trong bước tìm phòng. Phòng do
   tường tạo nên. Khi chưa bỏ đồ đạc, khe hẹp giữa hộp `box_2` và đầu vách A bị coi là "cửa", nên Phòng học bị lấn
   sang nửa trên hành lang. Sau khi bỏ, ranh giới nằm đúng trên đường kéo dài của vách.
4. **Distance transform** (`cv2.distanceTransform`, L2): mỗi pixel mang khoảng cách tới tường gần nhất (độ thoáng).
5. **Hạt giống**: hạ dần ngưỡng t từ độ thoáng lớn nhất xuống 0.35 m, mỗi bước 2.5 cm. Ở mỗi mức, thành phần liên
   thông nào của {dist > t} chưa chứa hạt và rộng từ 0.1 m² trở lên thì thành hạt mới. Mỗi phòng có một lõi thoáng
   riêng, và các lõi chỉ nối với nhau qua cửa hẹp, nên chúng tách nhau trước khi hợp lại. Kết quả là 4 hạt.
6. **Watershed**: ngập nước bằng hàng đợi ưu tiên, pixel thoáng hơn ngập trước, nên hai vùng gặp nhau ở chỗ hẹp nhất
   (cửa). Không dùng thẳng `cv2.watershed`, vì hàm này lan theo gradient màu của ảnh, không theo độ cao, và cho
   biên lởm chởm khi thử. Sau đó trả đồ đạc về là vật cản, và gộp vùng nhỏ hơn 1 m² vào vùng kề.
7. **Điểm đích hợp lệ**: trong mỗi vùng, chỉ xét pixel cách vật cản thật (có cả hộp) từ 0.55 m trở lên. Mức này
   bằng bán kính robot 0.17 m cộng inflation 0.35 m, thêm một chút dư. Pixel cũng phải nằm ngoài vùng cấm quanh dock.
   Trong các pixel đó, chọn pixel gần tâm vùng nhất.
8. **Cửa**: với mỗi cặp vùng kề nhau, chọn pixel trên biên chung có độ thoáng thật lớn nhất. Kết quả là 3 cửa:
   A–B (1.44, 1.31), B–D (2.39, -0.50) và C–D (4.19, -0.50), vẽ bằng ô vuông trắng trên hình. Đây là đồ thị khu vực
   mà node điều hướng dùng để lập lộ trình.

Đổi pixel (cột u, hàng v) sang frame `map` theo `map.yaml` (resolution 0.05, origin [-1.94, -2.47]):
`x = origin_x + (u + 0.5)·res`, `y = origin_y + (H − v − 0.5)·res` (ảnh có hàng 0 ở trên, trục y của map hướng lên).

## Phần 2 – Gắn nhãn (`config/segmentation.yaml` → `config/zones.yaml`)

Tool đánh số vùng tự động. Mỗi nhãn trong `segmentation.yaml` có một điểm `hint`, và vùng nào chứa điểm đó thì
nhận id, tên và bí danh của nhãn. Vùng không có nhãn nào trỏ tới vẫn được giữ, với tên `Khu vực <chữ cái>`.

| ID | Tên | Diện tích | Tâm (m) | Điểm đích Nav2 (m) | Cách vật cản | Bí danh |
|---|---|---|---|---|---|---|
| A | Phòng học | 15.4 m² | (-0.27, 0.09) | (-0.26, 0.71) | 1.64 m | lớp học, phòng a |
| B | Hành lang | 5.0 m² | (2.40, 0.93) | (2.39, 0.81) | 0.55 m | hành lang giữa, phòng b |
| C | Phòng thí nghiệm | 4.0 m² | (4.25, 0.94) | (4.24, 0.96) | 0.70 m | phòng lab, phòng c |
| D | Sảnh | 6.2 m² | (3.18, -1.44) | (3.19, -1.45) | 0.80 m | sảnh chính, phòng d |

Đích của A không trùng tâm. Tâm A nằm ngay trước dock, mà dock thấp hơn LiDAR nên không có trên bản đồ. Vì vậy
`segmentation.yaml` khai báo vùng cấm bán kính 0.7 m quanh dock. `zones.yaml` còn lưu `polygon` (đường bao) của
từng vùng. Node điều hướng dùng nó để biết robot đang ở khu vực nào và để vẽ trên RViz (topic `/zones/markers`:
tô màu, viền, tên, cột đỏ tại điểm đích).

## Phần 3 – Nhận dạng giọng nói (`nodes/speech_recognition_node.py`)

1. `arecord -f S16_LE -r 16000 -c 1` đọc micro thành các khung 30 ms.
2. **VAD theo năng lượng**: 1 s đầu node đo ồn nền và đặt ngưỡng = max(3 × RMS nền, 300). Khi một khung vượt ngưỡng,
   node bắt đầu ghi một câu (kèm 0.3 s trước đó để không mất phụ âm đầu). Câu kết thúc sau 0.8 s im lặng, hoặc khi dài
   tới 7 s. Câu có ít hơn 0.35 s tiếng nói bị bỏ.
3. **Whisper `small`** (mặc định), `language="vi"`, `temperature=0`. `initial_prompt` gồm câu mẫu và tên các khu
   vực trong `zones.yaml`, để Whisper viết đúng "khu vực A", "Phòng thí nghiệm". Prompt cố ý có cả "khu vực E" (không
   tồn tại). Khi thử với prompt chỉ liệt kê A–D, Whisper "sửa" câu "Đi đến khu vực E" thành "khu vực A", nên hệ thống
   không còn phát hiện được khu vực lạ. Nếu Whisper tự đánh giá `no_speech_prob > 0.6` thì câu bị bỏ.
4. **Vosk** (`-p engine:=vosk`): grammar là danh sách câu lệnh sinh từ `zones.yaml` (động từ × khu vực, cả cặp "từ X
   đến Y", có các chữ E/F/G để còn báo lỗi). Khi chỉ đưa danh sách từ đơn, Vosk ghép từ lung tung (ví dụ "đi điểm
   khu vực").
5. Publish văn bản lên `/voice_command` (`std_msgs/String`) và trạng thái (`listening`/`hearing`/`recognizing`) lên
   `/speech/state`.

Kết quả thử với 6 file giọng đọc tiếng Việt tổng hợp (16 kHz), CPU 12 nhân, torch 4 luồng:

| Câu nói | Whisper small | Vosk small-vn |
|---|---|---|
| Đi đến khu vực A. | Đi đến khu vực A. | đi đến khu vực a |
| Di chuyển từ khu vực A đến khu vực B. | Di chuyển từ khu vực A đến khu vực B. | di chuyển từ khu vực a đến khu vực bê |
| Đi đến phòng thí nghiệm. | Đi đến phòng thí nghiệm. | đi đến phòng thí nghiệm |
| Đi đến khu vực E. | Đi đến khu vực E. | đi đến khu vực xê (sai) |
| Đi đến sảnh. | Đi đến Sảnh. | đi đến sảnh |
| Dừng lại. | Dừng lại. | dừng lại |
| Thời gian / câu | 1.7–2.2 s (câu đầu 4.5 s) | < 0.1 s |

Kiểm tra micro: `./scripts/mic_test.sh` (hoặc `./scripts/mic_test.sh vosk`). Micro khác: `DEVICE=hw:1,0
./scripts/mic_test.sh`, `./scripts/05_speech.sh -p device:=hw:1,0` (danh sách: `arecord -L`). Phòng ồn thì đặt
ngưỡng tay, ví dụ `-p threshold:=1500`.

## Phần 4 – Điều hướng bằng giọng nói (`nodes/command_parser.py`, `nodes/voice_nav_node.py`)

**Phân tích câu lệnh**:

- Chuẩn hóa: chữ thường, bỏ dấu câu, bỏ dấu tiếng Việt (đ → d).
- Tìm các khu vực được nhắc tới theo thứ tự trong câu:
  - tên và bí danh (cụm dài khớp trước; cụm từ 8 ký tự trở lên cho phép sai nhẹ, ví dụ "phòng thí nghiệp");
  - "khu vực / khu / vùng / phòng / điểm" + chữ cái, kể cả cách đọc kiểu Việt ("bê", "xê", "đê");
  - chữ cái đứng sau "đến / tới / về / từ...".
- Ghép thành lộ trình. Khu vực đứng sau "từ" là điểm xuất phát.

| Câu lệnh | Kết quả |
|---|---|
| Đi đến khu vực A. | A |
| Di chuyển từ khu vực A đến khu vực B. | A → B, xuất phát từ A |
| Đi đến phòng thí nghiệm. | C |
| Đi đến hành lang rồi về phòng học | B → A |
| Đi tới khu vực E / Đi đến phòng ngủ | từ chối: "Không có khu vực 'khu vực E'. Các khu vực hiện có: A (Phòng học), …" |
| Dừng lại / Hủy | hủy goal đang chạy |
| Hãy đăng ký kênh để ủng hộ (câu Whisper hay "ảo giác" khi chỉ có ồn) | bỏ qua: không có khu vực |

`python3 tests/test_command_parser.py` chạy 26 câu mẫu, kết quả 26/26 đúng.

**Điều hướng** (`voice_nav_node.py`, action client `nav2_msgs/action/NavigateToPose` trên `/navigate_to_pose`):

- Mỗi khu vực trong lộ trình là một goal `NavigateToPose` tại điểm đích của khu vực đó, gửi lần lượt. Nếu khu
  vực đích không kề khu vực robot đang đứng, node tìm đường trên đồ thị khu vực bằng BFS và gửi thêm một goal
  `NavigateToPose` tại mỗi cửa phải đi qua. Ví dụ, từ Phòng học tới Phòng thí nghiệm, robot đi qua cửa A–B, B–D,
  C–D rồi mới tới đích. Lần thử đầu chưa có bước này: Nav2 lập một đường ôm sát đầu vách B để vào Phòng thí nghiệm,
  và robot kẹt ở (2.85, -0.60) suốt 240 s với 11 lần recovery, vì DWB của TurtleBot 4 không được lùi. Đi qua cửa
  C–D, robot vào thẳng theo hướng bắc. Hướng của goal
  là hướng từ điểm trước tới điểm đích, nên tới nơi robot không phải quay tại chỗ. Nav2 lập đường đi trên global
  costmap và tránh cả các vật thể không có trên bản đồ (người, ghế, sofa của world `room_objects`).
- **"Từ A đến B"**: node lấy vị trí robot từ `/amcl_pose` và kiểm tra điểm đó nằm trong đường bao nào. Robot đang ở
  A thì đi thẳng tới B. Đang ở chỗ khác thì goal 1 là A, goal 2 là B.
- **Khu vực không tồn tại** hoặc câu không hiểu: node từ chối, báo danh sách khu vực hợp lệ lên terminal và
  `/voice_nav/status`. Robot vẫn làm tiếp việc đang làm.
- **Lệnh mới thay lệnh cũ**: goal đang chạy bị hủy, rồi lộ trình mới bắt đầu. Kết quả của goal cũ bị bỏ qua (đếm
  `run`). "Dừng lại" chỉ hủy goal.
- Robot đang trên dock thì được undock (action `/undock`) trước goal đầu. Goal quá 240 s thì bị hủy.

## Mở rộng – Nhiều điểm đến và hiển thị đường đi dự kiến

![Map view](figures/map_view_route.png)

**Câu lệnh nhiều điểm.** Lộ trình gồm mọi khu vực được nhắc tới, theo đúng thứ tự trong câu, bao nhiêu điểm cũng
được. Các từ nối như "qua", "rồi", "sau đó", "và" chỉ ngăn cách các điểm. Tên loại phòng đi kèm chữ cái ("hành
lang C", "phòng B") thì **chữ cái quyết định khu vực**. Nhờ vậy câu "Đi từ phòng A qua hành lang C rồi đến phòng B"
cho lộ trình A → C → B, dù tên thật của C là "Phòng thí nghiệm". Chữ cái chỉ được tính khi viết hoa (Whisper)
hoặc đọc nhiều âm kiểu Vosk ("xê", "bê"), để tiểu từ "ạ" trong "đi đến phòng học ạ" không bị hiểu thành khu vực A.

| Câu lệnh | Lộ trình |
|---|---|
| Đi từ phòng A qua hành lang C rồi đến phòng B | A → C → B, xuất phát từ A |
| Đi qua sảnh rồi đến phòng thí nghiệm, sau đó về phòng học | D → C → A |
| Đi đến khu vực A rồi B rồi C | A → B → C |
| Đi từ khu vực D qua khu vực B đến khu vực A rồi đến khu vực C | D → B → A → C, xuất phát từ D |
| Đi từ phòng A qua hành lang E rồi đến phòng B | từ chối: không có "hành lang e" |

**Lập kế hoạch cả lộ trình trước khi đi.** `voice_nav_node` đổi lộ trình thành danh sách chặng: cửa giữa các
khu vực (BFS trên đồ thị khu vực) và điểm đích của từng khu vực. Sau đó nó gửi lần lượt từng chặng tới planner của
Nav2 (action `/compute_path_to_pose`, cùng planner mà `bt_navigator` dùng). Chặng thứ i bắt đầu từ điểm cuối chặng
i−1. Các đoạn được nối thành một `nav_msgs/Path` trên `/voice_nav/planned_path` (transient local), rồi robot mới
bắt đầu đi. Planner lỗi ở chặng nào thì chặng đó được vẽ bằng đoạn thẳng. Phải lập kế hoạch xong rồi mới đi, vì
planner server chỉ xử lý một yêu cầu một lúc: gọi xen giữa lúc Nav2 đang chạy sẽ hủy lượt lập đường của
`bt_navigator`. Trong lúc đi, path chỉ còn phần chưa đi.

**Hiển thị:**

- **RViz** (`config/voice_nav.rviz`):
  - display "Planned route (btth7)": đường xanh đậm của cả lộ trình;
  - display "Zones (btth7)": khu vực tô màu và các điểm của lộ trình đánh số `1. C`, `2. B`, với khối nhỏ tại
    cửa (xám = đã qua, cam = đang tới, xanh = sắp tới);
  - display Path của Nav2: đường của chặng hiện tại.
- **Cửa sổ bản đồ riêng** (`./scripts/06_map_view.sh`, OpenCV, `run_demo.sh` tự mở):
  - bản đồ có khu vực tô màu, cửa □ và điểm đích ✕;
  - robot (hình tròn kèm hướng, từ `/amcl_pose`);
  - đường dự kiến (xanh) và đường Nav2 đang bám (cam);
  - các điểm dừng đánh số theo trạng thái;
  - câu lệnh cuối, lộ trình, trạng thái micro và kết quả.

  Bấm chuột vào một khu vực để gửi "Đi đến khu vực X". Phím `s` lưu ảnh vào `figures/`.
  `-p snapshot:=file.png -p show:=false` ghi ảnh định kỳ mà không mở cửa sổ.
- **Topic `/voice_nav/route`** (String JSON: câu lệnh, lộ trình, các chặng, chặng hiện tại, trạng thái) cho giao
  diện khác dùng.

**Kết quả trong mô phỏng** (gửi câu lệnh dạng văn bản):

| Câu lệnh | Robot ở | Chặng | Kết quả |
|---|---|---|---|
| Đi từ phòng A qua hành lang C rồi đến phòng B | A | bỏ A; cửa A–B, B–D, C–D, đích C; cửa C–D, B–D, đích B (7 chặng) | tới C rồi B, 121 s |
| Đi qua sảnh rồi đến phòng thí nghiệm, sau đó về phòng học | dock (A) | 9 chặng, qua 6 cửa, đường dự kiến 15.7 m | tới D, C, A, 191 s |
| Đi từ phòng A qua hành lang E rồi đến phòng B | dock (A) | – | từ chối, robot đứng yên |

Nhận dạng câu nhiều điểm (2 file giọng đọc tổng hợp):
- **Whisper small**: đúng cả 2 câu, 5–7 s mỗi câu vì CPU đang chạy cùng Gazebo.
- **Vosk**: sai cả 2 câu ("đi từ phòng qua hành lang xê…" mất chữ A). Grammar sinh cho câu 3 điểm lên tới
  16 082 câu, khó giải mã đúng. Với câu nhiều điểm, nên dùng Whisper.

## Kết quả kiểm thử trong mô phỏng

`./scripts/test_run.sh` chạy Sim, Localization, Nav2 và Voice Nav ở nền, rồi gửi lần lượt 5 câu lệnh văn bản
(log trong `logs/`):

| Câu lệnh (văn bản) | Vị trí robot lúc ra lệnh | Robot làm gì | Kết quả | Thời gian mô phỏng |
|---|---|---|---|---|
| Đi đến khu vực E | trên dock (A) | từ chối, báo danh sách A–D, robot đứng yên | đúng | 0 s |
| Đi đến phòng thí nghiệm | trên dock (A) | undock, qua cửa A–B, B–D, C–D, tới C | tới C | 93 s |
| Di chuyển từ khu vực A đến khu vực B | C (không phải A) | về A trước (qua C–D, B–D, A–B), rồi tới B | tới A rồi B | 88 s |
| Đi đến sảnh | B | qua cửa B–D, tới D | tới D | 21 s |
| Di chuyển từ sảnh đến phòng học | D (đang ở điểm xuất phát) | bỏ qua D, đi thẳng tới A (qua B–D, A–B) | tới A | 49 s |

Lộ trình tránh được các vật thể không có trên bản đồ (ghế, người, trụ cứu hỏa của world `room_objects`) nhờ
costmap của Nav2. Giọng nói thật từ micro chưa được kiểm thử trong lần chạy này. Phần nhận dạng đã được kiểm thử
với 6 file giọng đọc tổng hợp (bảng ở Phần 3); khi quay video, hãy kiểm tra micro bằng `./scripts/mic_test.sh`.

## Xử lý sự cố

| Triệu chứng | Cách xử lý |
|---|---|
| Tab Speech không phản ứng khi nói | `./scripts/mic_test.sh`: xem RMS. Nếu gần 0, chọn đúng micro (`arecord -L`, `-p device:=...`) và bật micro trong Settings → Sound |
| Nói xong mà node nhận dạng liên tục cả tiếng ồn | Ngưỡng quá thấp: `-p threshold:=1500`, hoặc giữ im lặng 1 s lúc khởi động để node đo lại ồn nền |
| Whisper chậm | `-p model:=base` (nhanh hơn, kém chính xác hơn), hoặc dùng `-p engine:=vosk` |
| `numpy.core.multiarray failed to import` | Chạy qua `scripts/*.sh` (nạp `.pydeps` có numpy<2), hoặc `source scripts/env.sh` trước |
| Voice Nav báo "Nav2 chưa sẵn sàng" | Đợi tab Nav2 báo `Managed nodes are active` (cần initial pose: `../btth5/scripts/set_initial_pose.sh 0 0 0`) |
| RViz không thấy các khu vực | Display "Zones (btth7)" phải dùng topic `/zones/markers`, Durability = Transient Local; node Voice Nav phải đang chạy |
| Đổi bản đồ / thêm phòng | Sửa `config/segmentation.yaml` (map, nhãn và `hint`), chạy `./scripts/segment.sh`, khởi động lại Voice Nav |
| RViz không thấy đường dự kiến | Display "Planned route (btth7)": topic `/voice_nav/planned_path`, Durability = Transient Local |
| Còn tiến trình cũ, `/clock` có nhiều publisher | `./scripts/stop_all.sh` |
