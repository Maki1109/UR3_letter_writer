# Hướng dẫn quay video – Bài thực hành 07

Video cần thể hiện đủ chuỗi: **nói câu lệnh → hệ thống nhận dạng thành văn bản → xác định khu vực đích → robot tự
di chuyển, tránh vật cản**. Các cảnh cần thấy: (1) bản đồ đã phân vùng có nhãn, (2) lệnh giọng nói và văn bản nhận
dạng được, (3) Gazebo, RViz hoặc cửa sổ bản đồ khi robot di chuyển, (4) ít nhất 3 câu lệnh đúng và một câu có khu
vực không tồn tại.

Mọi lệnh gõ trong thư mục `btth7`:

```bash
cd ~/Downloads/turtlebot_simulation/btth7
```

## 0. Chuẩn bị (làm trước, không quay)

**Phần mềm quay màn hình có tiếng.** Video phải nghe được giọng bạn nói lệnh, nên cần phần mềm quay thu cả micro.
Máy này chưa có OBS. Cài một lần:

```bash
sudo apt install obs-studio
```

Trong OBS: Sources → thêm **Screen Capture (PipeWire)** (phiên Wayland) hoặc **Screen Capture (XSHM)** (phiên Xorg),
và **Audio Input Capture** chọn đúng micro. Settings → Output → Recording: định dạng `mp4`, chất lượng "Same as
stream". Settings → Video: độ phân giải 1920×1080, 30 FPS. Bấm thử quay 10 s và nghe lại để chắc chắn có tiếng.
(Quay bằng `Ctrl+Alt+Shift+R` của GNOME thì không có tiếng, nên chỉ dùng được nếu bạn thuyết minh thêm bằng
phần mềm khác rồi ghép.)

**Kiểm tra micro và nhận dạng**, ở nơi yên tĩnh:

```bash
./scripts/mic_test.sh
```

Script ghi 4 s, phát lại, rồi in văn bản Whisper nhận được và lộ trình phân tích được. Nói đúng câu sẽ dùng khi quay,
ví dụ "Đi đến khu vực A". Cần thấy dòng `RMS = ...` lớn hơn 300 khi bạn nói. Nếu RMS gần 0, chọn đúng micro
(`arecord -L`, rồi `DEVICE=hw:1,0 ./scripts/mic_test.sh`) và kiểm tra Settings → Sound. Tập nói mỗi câu 2–3 lần cho
tới khi nhận dạng đúng, rồi ghi lại những câu hay bị nhận sai để tránh khi quay.

**Cách nói để Whisper nhận đúng:**
- Nói rõ từng chữ, tốc độ vừa phải, cách micro 20–30 cm, phòng yên tĩnh (quạt, nhạc nền làm nhận sai).
- Đọc chữ cái là "A", "B", "C" (a, bê, xê). Khi quay đừng thêm từ thừa ở đầu hoặc cuối câu.
- Mỗi câu nói liền một hơi. Dừng hơn 0.8 s giữa câu thì node coi như hết câu và nhận dạng phần đầu.
- Sau khi nói, **đợi** dòng `>>> "..."` hiện ra ở tab Speech (khoảng 2–5 s) rồi mới nói câu tiếp.

**Bố cục màn hình** (nên dùng màn hình lớn hoặc 2 màn hình): cửa sổ **Gazebo** và **Map view** ở một bên, **RViz**
ở bên kia, và các tab terminal **Speech** + **Voice Nav** cạnh nhau ở phía dưới. Cần thấy rõ được chữ trong hai tab
terminal. Có thể kéo tab ra thành cửa sổ riêng, rồi bấm `Ctrl +` để chữ to hơn.

**Dọn dẹp:** tắt thông báo, đóng các ứng dụng không liên quan, và chạy `./scripts/stop_all.sh` để chắc chắn không còn
tiến trình cũ.

## 1. Cảnh mở đầu: bản đồ gốc và bản đồ phân vùng (quay trước, khoảng 1 phút)

Chỉ cho người xem 3 thứ, vừa chỉ vừa nói:

1. **Bản đồ gốc:** mở `../maps/map.pgm` (ảnh bài 04), nói rằng đây là bản đồ do SLAM Toolbox quét.
2. **Bản đồ phân vùng:**
   ```bash
   ./scripts/segment.sh --show
   ```
   Cửa sổ hiện ảnh 4 khu vực tô màu, có tên, điểm đích ✕ và cửa □. Bấm phím bất kỳ để đóng. Đọc nhanh 4 khu vực A
   Phòng học, B Hành lang, C Phòng thí nghiệm, D Sảnh. Mở `figures/zones_steps.png` để chỉ các bước: threshold,
   distance transform, hạt giống, watershed.
3. **File thông tin khu vực:** mở `config/zones.yaml` bằng trình soạn thảo, cuộn qua id, tên, `goal` (tọa độ đích
   trong frame map), `polygon` và `doors`.

## 2. Khởi động hệ thống

```bash
./scripts/run_demo.sh
```

Script mở các tab: Sim, Localization, Init pose, Nav2, RViz, Voice Nav, Map view, Speech. Đợi 1–3 phút. Hệ thống sẵn
sàng khi:

- Gazebo có robot trên dock, và RViz hiện bản đồ cùng các khu vực tô màu;
- tab **Nav2** báo `Managed nodes are active`;
- tab **Voice Nav** in `4 khu vực: A (Phòng học), ...` và dòng `Vị trí robot (...)`;
- tab **Speech** in `Ồn nền RMS ... -> ngưỡng ...` rồi `Đang nghe... hãy nói câu lệnh`. **Giữ yên lặng** trong 1 s đầu để
  node đo ồn nền.

Nếu sau 3 phút robot không xuất hiện hoặc có tab lỗi: `./scripts/stop_all.sh` rồi chạy lại `run_demo.sh`. Gazebo đôi khi
treo ở bước nạp world, chạy lại là hết.

Chưa quay vội. Hãy nói thử một câu (ví dụ "Đi đến khu vực A") để chắc chắn mọi thứ chạy, đợi robot tới nơi, rồi
bắt đầu quay từ trạng thái sạch. Cách nhanh nhất để đưa robot về dock là chạy lại `run_demo.sh`.

## 3. Kịch bản quay

Bấm **Start Recording** rồi đọc lời dẫn mở đầu: "Hệ thống nhận lệnh giọng nói, xác định khu vực đích trên bản đồ
đã phân vùng và gửi cho Nav2". Sau mỗi câu lệnh, **đợi robot tới nơi** (tab Voice Nav in `Đã đến ...`) rồi mới nói
câu tiếp. Thời gian lấy từ lần chạy thử trong mô phỏng.

| # | Bạn nói | Cần thấy | Thời gian |
|---|---|---|---|
| 1 | "Đi đến khu vực A." | Speech in văn bản; Voice Nav báo undock, rồi đi tới A (Phòng học); Map view hiện điểm số 1 | khoảng 30–40 s |
| 2 | "Đi đến khu vực E." | Voice Nav báo đỏ `Không có khu vực 'khu vực E'. Các khu vực hiện có: ...`; robot **không** di chuyển | 5 s |
| 3 | "Đi đến phòng thí nghiệm." | Lộ trình `C`, 4 chặng qua cửa A–B, B–D, C–D; đường xanh dự kiến trên Map view và RViz; robot né các vật thể, tới C | khoảng 90 s |
| 4 | "Di chuyển từ khu vực A đến khu vực B." | Robot đang ở C, không phải A. Voice Nav báo `tới điểm xuất phát A (Phòng học) trước`, đi qua A rồi mới tới B | khoảng 90 s |
| 5 | "Đi đến sảnh." | Tới D (Sảnh), gọi bằng tên chứ không phải chữ cái | khoảng 20 s |
| 6 | "Đi từ phòng A qua hành lang C rồi đến phòng B." | Câu nhiều điểm (phần mở rộng): lộ trình A → C → B, đường dự kiến cả lộ trình, các điểm dừng đánh số 1, 2 | khoảng 2 phút |
| 7 | Câu 6, khi robot đang đi, nói "Dừng lại." | Voice Nav báo `Đã dừng theo lệnh`, robot đứng lại | 10 s |

Mỗi câu nên chỉ ra ba thứ trên màn hình bằng chuột hoặc lời nói:
1. tab **Speech**: văn bản nhận dạng được (`>>> "..."`), chứng minh bước chuyển giọng nói thành văn bản;
2. tab **Voice Nav**: `Lộ trình: ...` cho thấy khu vực được xác định từ câu nói;
3. **Gazebo / Map view / RViz**: robot di chuyển theo đường xanh và tới đúng khu vực.

Đề yêu cầu tối thiểu 3 câu lệnh đúng và một câu có khu vực không tồn tại, và bảng trên đủ cả 3 loại ví dụ
("đi đến khu vực A", "từ khu vực A đến khu vực B", "đi đến phòng thí nghiệm"). Câu 5, 6, 7 là bổ sung, có thể
cắt nếu video quá dài.

**Nếu Whisper nhận sai một câu:** đừng lo, nói lại câu đó. Nếu lệnh sai nhưng hợp lệ (ví dụ nhận nhầm thành
khu vực khác), nói "Dừng lại." rồi nói lại đúng câu. Phần cắt ghép sau đó bỏ đoạn nhầm. Nếu một câu nhận sai nhiều lần
liên tiếp, đổi cách nói ("khu vực A" ↔ "phòng học"), hoặc tạm dùng `./scripts/say.sh "Đi đến khu vực A"` ở một
terminal khác (không nên dùng cho cả video vì mất ý nghĩa "bằng giọng nói").

## 4. Cảnh kết thúc

- Dừng quay trong OBS.
- Tắt mọi thứ: `./scripts/stop_all.sh`.
- Tùy chọn: cuối video chiếu lại `figures/map_view_route.png` hoặc ảnh chụp cửa sổ Map view cuối cùng.

## 5. Sau khi quay

- Xem lại toàn bộ video ở tốc độ bình thường: có nghe rõ giọng nói không, đọc được chữ ở hai tab terminal không.
- Video dài (khoảng 8–10 phút do robot chạy chậm). Nên **cắt đoạn chờ** hoặc tăng tốc 2–4× đoạn robot đang đi, nhưng giữ
  nguyên tốc độ lúc bạn nói và lúc văn bản nhận dạng hiện ra. Có thể dùng Kdenlive hoặc OpenShot (`sudo apt install kdenlive`).
- Xuất `mp4` (H.264) dưới 100 MB nếu nộp qua form, hoặc tải lên YouTube/Drive để dán đường dẫn.

## 6. Sự cố thường gặp khi quay

| Triệu chứng | Cách xử lý |
|---|---|
| Nói mà Speech không phản ứng | `./scripts/mic_test.sh` xem RMS; chọn đúng micro; hoặc `./scripts/05_speech.sh -p threshold:=500` (hạ ngưỡng) |
| Speech nhận dạng cả tiếng ồn, in câu vô nghĩa | Hệ thống tự bỏ qua câu không có khu vực (`Không tìm thấy khu vực nào ...`). Nếu bị nhiều, nâng ngưỡng `-p threshold:=1500` |
| Whisper nhận sai chữ cái (B ↔ D) | Nói chậm "khu vực... B", hoặc dùng tên ("hành lang"), vì tên dài ít bị nhầm hơn chữ cái |
| Robot không chạy dù Voice Nav báo `Đang đi tới` | Nav2 chưa active hoặc chưa có initial pose: `../btth5/scripts/set_initial_pose.sh 0 0 0` |
| Robot kẹt, `recoveries` tăng liên tục | Đợi 240 s node tự hủy goal, hoặc nói "Dừng lại." rồi đọc lại lệnh. Tránh bắt đầu quay khi robot đang sát vách |
| Cửa sổ Map view không có chữ tiếng Việt | Cần font DejaVu (`fc-list | grep DejaVu`); có sẵn trên Ubuntu |
| Âm thanh trong video có tiếng quạt, tiếng gõ | Bật noise suppression trong OBS (Mic filters → Noise Suppression), dùng tai nghe có micro |

## 7. Danh sách nộp (kiểm tra trước khi nộp)

- [ ] **Source code trên GitHub (public)**: đẩy cả thư mục `btth7/` (không đẩy `.pydeps/`, `models/vosk-*`, `logs/`;
  các thư mục này lớn và `setup.sh` tự sinh lại). Kiểm tra repo mở được khi chưa đăng nhập.
- [ ] **Bản đồ gốc** `maps/map.pgm` + `map.yaml`, **bản đồ phân vùng** `btth7/figures/zones.png`, **file thông tin
  khu vực** `btth7/config/zones.yaml`.
- [ ] **Video demo** (mục 3).
- [ ] **Báo cáo PDF 1–2 trang**: phương pháp phân vùng, nhận dạng giọng nói, kết quả thử nghiệm.
