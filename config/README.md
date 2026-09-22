# Cấu hình RViz của gói

- `letter_view.rviz` — dùng bởi `ur3_letter_writer.launch.py` / `ur3_write_s.launch.py`:
  RobotModel + TF + Grid, camera canh sẵn vào mặt phẳng vẽ, và 2 Display **Marker**:
  - `/letter_preview` — hình chữ dự kiến (xanh dương, nét mỏng)
  - `/letter_trace` — quỹ đạo **thực tế** của end-effector (cyan, nét dày)

## Lưu ý khi sửa các file này

Display Marker của **RViz2** (Humble trở đi) dùng khoá YAML `Topic:` cho topic, kèm
các khoá con `Depth / Durability Policy / Filter size / History Policy /
Reliability Policy / Value`:

```yaml
    - Class: rviz_default_plugins/Marker
      Enabled: true
      Topic:
        Depth: 50
        Durability Policy: Transient Local
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /letter_trace
      Name: Letter Trace
      Namespaces:
        {}
      Value: true
```

Viết là `Marker Topic:` (cú pháp RViz1) thì RViz2 **bỏ qua khoá đó mà không báo lỗi
nạp file**: display vẫn hiện trong danh sách nhưng giữ topic mặc định
`/visualization_marker`, không subscribe topic của mình, và hiện `Status: Error` —
nhìn như "marker không được publish" dù `ros2 topic echo` vẫn thấy dữ liệu. Khoá
`Queue Size:` cũng là của RViz1, RViz2 không dùng.

Cách kiểm tra nhanh khi marker không hiện: `ros2 topic info -v /letter_trace` lúc
node đang chạy — phải thấy `Subscription count: 1` với `Node name: rviz2_letter_view`.
