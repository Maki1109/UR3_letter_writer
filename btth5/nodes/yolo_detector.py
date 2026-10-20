#!/usr/bin/env python3
"""Phần 4 – Nhận diện vật thể từ camera RGB của TurtleBot 4 bằng YOLO.

  sub  /oakd/rgb/preview/image_raw  (sensor_msgs/Image)
  pub  /yolo/image_annotated        (sensor_msgs/Image, ảnh đã vẽ bounding box – xem được trên RViz2)
  pub  /yolo/detections             (std_msgs/String, tóm tắt "class conf" của mỗi khung)

Ảnh ROS -> OpenCV bằng cv_bridge, đưa qua YOLO (ultralytics), vẽ box + tên class + độ tự tin lên cửa sổ
OpenCV và in vật thể phát hiện được ra terminal.

Tham số ROS: image_topic, model (đường dẫn .pt), conf (ngưỡng tin cậy), imgsz, show (mở cửa sổ OpenCV),
upscale (phóng to cửa sổ hiển thị), log_period (giây, chu kỳ in khi danh sách vật thể không đổi),
threads (số luồng CPU cho torch), max_rate (Hz tối đa),
classes (chỉ giữ các lớp COCO này – các vật thể có trong world; [''] = mọi lớp),
min_area (px², bỏ box nhỏ hơn – nhận nhầm thường là box rất nhỏ ở xa).
"""
import time

import cv2
import numpy as np
import rclpy
import torch
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO


class YoloDetector(Node):

    def __init__(self):
        super().__init__('yolo_detector')
        self.declare_parameter('image_topic', '/oakd/rgb/preview/image_raw')
        self.declare_parameter('model', 'yolo11n.pt')
        self.declare_parameter('conf', 0.3)
        self.declare_parameter('imgsz', 320)
        self.declare_parameter('show', True)
        self.declare_parameter('upscale', 2.0)
        self.declare_parameter('log_period', 2.0)
        self.declare_parameter('threads', 2)       # luồng CPU cho torch; để nhiều sẽ tranh CPU với Gazebo
        self.declare_parameter('max_rate', 10.0)   # Hz, giới hạn tần số suy luận
        # Lọc lớp: world chỉ có người, ghế, sofa, trụ cứu hỏa -> các lớp khác (umbrella, traffic light...) là nhận nhầm
        self.declare_parameter('classes', ['person', 'chair', 'couch', 'fire hydrant'])
        self.declare_parameter('min_area', 150)
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.conf, self.imgsz, self.show = p('conf'), p('imgsz'), p('show')
        self.upscale, self.log_period = p('upscale'), p('log_period')
        cv2.setNumThreads(1)

        self.get_logger().info(f"Nạp mô hình YOLO: {p('model')}")
        self.model = YOLO(p('model'))
        wanted = [c for c in p('classes') if c]
        ids = {v: k for k, v in self.model.names.items()}
        unknown = [c for c in wanted if c not in ids]
        if unknown:
            self.get_logger().warn(f'Bỏ qua lớp không có trong mô hình: {unknown}')
        self.class_ids = [ids[c] for c in wanted if c in ids] or None   # None = mọi lớp
        self.min_area = p('min_area')
        self.get_logger().info(f"Lớp nhận diện: {wanted or 'tất cả'}, conf >= {p('conf')}, box >= {self.min_area} px²")
        self.model.predict(np.zeros((240, 320, 3), np.uint8), imgsz=self.imgsz, verbose=False)  # warm-up
        # ultralytics đặt lại torch.set_num_threads(8) ở lần predict đầu -> phải đặt sau warm-up
        torch.set_num_threads(p('threads'))
        self.bridge = CvBridge()
        self.latest = None      # chỉ xử lý khung mới nhất, bỏ các khung tới trong lúc đang suy luận
        self.last_summary, self.last_log = None, 0.0
        self.fps = 0.0

        topic = p('image_topic')
        self.create_subscription(Image, topic, self.on_image, qos_profile_sensor_data)
        self.pub_img = self.create_publisher(Image, '/yolo/image_annotated', 10)
        self.pub_det = self.create_publisher(String, '/yolo/detections', 10)
        self.create_timer(1.0 / p('max_rate'), self.process)
        self.get_logger().info(f'Đang chờ ảnh trên {topic} ...')

    def on_image(self, msg):
        self.latest = msg

    def process(self):
        if self.latest is None:
            return
        msg, self.latest = self.latest, None
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        t0 = time.time()
        res = self.model.predict(frame, imgsz=self.imgsz, conf=self.conf, classes=self.class_ids,
                                 verbose=False)[0]
        dt = time.time() - t0
        self.fps = 0.9 * self.fps + 0.1 / max(dt, 1e-3) if self.fps else 1.0 / max(dt, 1e-3)

        dets = []
        for box in res.boxes:
            x1, y1, x2, y2 = (int(v) for v in box.xyxy[0].tolist())
            if (x2 - x1) * (y2 - y1) < self.min_area:
                continue
            name = res.names[int(box.cls)]
            dets.append((name, float(box.conf), (x1, y1, x2, y2)))
        self.report(dets, msg)

        out = self.draw(frame, dets)
        img_msg = self.bridge.cv2_to_imgmsg(out, encoding='bgr8')
        img_msg.header = msg.header
        self.pub_img.publish(img_msg)
        if self.show:
            if self.upscale != 1.0:
                out = cv2.resize(out, None, fx=self.upscale, fy=self.upscale, interpolation=cv2.INTER_LINEAR)
            cv2.imshow('TurtleBot4 camera - YOLO', out)
            cv2.waitKey(1)

    def draw(self, frame, dets):
        out = frame.copy()
        for name, conf, (x1, y1, x2, y2) in dets:
            color = self.color(name)
            cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
            label = f'{name} {conf:.2f}'
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
            ty = max(y1, th + 4)
            cv2.rectangle(out, (x1, ty - th - 4), (x1 + tw + 2, ty), color, -1)
            cv2.putText(out, label, (x1 + 1, ty - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1,
                        cv2.LINE_AA)
        cv2.putText(out, f'YOLO {self.fps:.0f} FPS | {len(dets)} obj', (4, out.shape[0] - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1, cv2.LINE_AA)
        return out

    @staticmethod
    def color(name):
        h = sum(ord(c) * 31 ** i for i, c in enumerate(name))
        return (h % 200 + 40, (h // 200) % 200 + 40, (h // 40000) % 200 + 40)

    def report(self, dets, msg):
        summary = ', '.join(f'{n} {c:.2f}' for n, c, _ in sorted(dets, key=lambda d: -d[1])) or '(không có)'
        self.pub_det.publish(String(data=summary))
        # In khi tập class thay đổi, hoặc định kỳ log_period giây
        classes = tuple(sorted(n for n, _, _ in dets))
        now = time.time()
        if classes == self.last_summary and now - self.last_log < self.log_period:
            return
        self.last_summary, self.last_log = classes, now
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        lines = [f'[t={stamp:7.1f}s] {len(dets)} vật thể ({self.fps:.0f} FPS)']
        for n, c, (x1, y1, x2, y2) in dets:
            lines.append(f'    - {n:<14} conf={c:.2f}  box=({x1},{y1})-({x2},{y2})')
        self.get_logger().info('\n'.join(lines))


def main():
    rclpy.init()
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
