#!/usr/bin/env python3
"""Node nhận diện biểu cảm: webcam -> OpenCV -> DeepFace (hoặc FER) -> làm mượt -> /emotion.

Topic phát:
  /emotion          std_msgs/String   nhãn ổn định: happy | sad | neutral | angry | surprise | fear | none
  /emotion/scores   std_msgs/String   JSON: nhãn tức thời, điểm 6 lớp (đã làm mượt), khung mặt, fps
  /emotion/image    sensor_msgs/Image ảnh đã vẽ kết quả (bgr8) – xem bằng rqt_image_view
Topic nghe:
  /emotion_motion/status  (từ emotion_motion_node) để hiện hành động robot đang làm trên cửa sổ

Tham số: device (0), backend (deepface|fer), detector (opencv|ssd|mtcnn|retinaface), width/height, window,
         min_score, min_ratio, show (true), publish_image (true), mirror (true)
Phím trên cửa sổ: q/ESC thoát · SPACE tạm dừng (phát neutral -> robot dừng) · s lưu ảnh vào figures/ · h robot về chỗ cũ
"""
import json
import os
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from emotion_core import COLOR, EMOTIONS, VI_NAME, EmotionRecognizer, EmotionSmoother  # noqa: E402

ACTION_TEXT = {'happy': 'Cong sang TRAI', 'sad': 'Cong sang PHAI', 'neutral': 'DUNG tai cho',
               'angry': 'LUI nhe', 'surprise': 'XOAY 1 vong', 'fear': 'TIEN nhe', 'none': 'DUNG'}
FIG_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'figures')


class EmotionDetector(Node):
    def __init__(self):
        super().__init__('emotion_detector')
        P = {'device': '0', 'backend': 'deepface', 'detector': 'opencv', 'width': 640, 'height': 480,
             'window': 8, 'min_score': 0.40, 'min_ratio': 0.6, 'show': True, 'publish_image': True,
             'mirror': True}
        for k, v in P.items():
            self.declare_parameter(k, v)
        self.p = {k: self.get_parameter(k).value for k in P}

        self.pub = self.create_publisher(String, '/emotion', 10)
        self.pub_scores = self.create_publisher(String, '/emotion/scores', 10)
        self.pub_cmd = self.create_publisher(String, '/emotion_motion/command', 10)
        self.pub_img = self.create_publisher(Image, '/emotion/image', 2)
        self.create_subscription(String, '/emotion_motion/status', self.on_status, 10)
        self.status = None
        self.status_time = 0.0

        self.get_logger().info(f'Nạp model {self.p["backend"]} (detector {self.p["detector"]}) ...')
        self.rec = EmotionRecognizer(self.p['backend'], self.p['detector'])
        self.smooth = EmotionSmoother(self.p['window'], self.p['min_score'], self.p['min_ratio'])

        dev = self.p['device']
        # số -> webcam /dev/videoN (V4L2); chuỗi khác -> file video / URL luồng (để kiểm thử không cần webcam)
        self.is_file = not str(dev).isdigit()
        self.cap = cv2.VideoCapture(dev) if self.is_file else cv2.VideoCapture(int(dev), cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.p['width'])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.p['height'])
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.cap.isOpened():
            raise RuntimeError(f'Không mở được camera {dev} (thử -p device:=1, xem: v4l2-ctl --list-devices)')
        self.get_logger().info(f'Camera {dev} đã mở. Phím: q thoát, SPACE tạm dừng, s lưu ảnh.')
        self.paused = False

    def on_status(self, msg):
        try:
            self.status = json.loads(msg.data)
            self.status_time = time.time()
        except ValueError:
            pass

    # ---------- vòng chính (chạy ở luồng chính để cv2.imshow hoạt động) ----------
    def run(self):
        fps, t_prev = 0.0, time.time()
        while rclpy.ok():
            ok, frame = self.cap.read()
            if not ok and self.is_file:                   # hết video -> phát lại từ đầu
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ok, frame = self.cap.read()
            if not ok:
                self.get_logger().warn('Không đọc được frame từ camera', throttle_duration_sec=2.0)
                time.sleep(0.1)
                continue
            if self.is_file:
                time.sleep(max(0.0, 1.0 / 15 - (time.time() - t_prev)))   # phát file ở ~15 fps như webcam
            if self.p['mirror'] and not self.is_file:
                frame = cv2.flip(frame, 1)

            faces = [] if self.paused else self.rec.detect(frame)
            face = faces[0] if faces else None            # chỉ điều khiển theo mặt lớn nhất (gần camera nhất)
            label = 'neutral' if self.paused else self.smooth.update(face.scores if face else None)

            now = time.time()
            fps = 0.9 * fps + 0.1 / max(now - t_prev, 1e-3) if fps else 1.0 / max(now - t_prev, 1e-3)
            t_prev = now

            self.pub.publish(String(data=label))
            self.pub_scores.publish(String(data=json.dumps({
                'stable': label, 'instant': face.label if face else 'none',
                'scores': {e: round(v, 3) for e, v in self.smooth.mean.items()},
                'box': list(face.box) if face else None, 'faces': len(faces), 'fps': round(fps, 1)})))

            vis = self.draw(frame, faces, label, fps)
            if self.p['publish_image'] and self.pub_img.get_subscription_count() > 0:
                self.pub_img.publish(self.to_msg(vis))
            if self.p['show']:
                cv2.imshow('Nhan dien bieu cam -> TurtleBot', vis)
                k = cv2.waitKey(1) & 0xFF
                if k in (27, ord('q')):
                    break
                if k == ord(' '):
                    self.paused = not self.paused
                    self.smooth.reset()
                    self.get_logger().info('TẠM DỪNG (robot dừng)' if self.paused else 'Tiếp tục nhận diện')
                if k == ord('h'):
                    self.pub_cmd.publish(String(data='home'))
                    self.get_logger().info('Gửi lệnh VỀ CHỖ CŨ')
                if k == ord('s'):
                    os.makedirs(FIG_DIR, exist_ok=True)
                    f = os.path.join(FIG_DIR, f'snapshot_{label}_{time.strftime("%H%M%S")}.png')
                    cv2.imwrite(f, vis)
                    self.get_logger().info(f'Đã lưu {f}')
        self.pub.publish(String(data='neutral'))
        self.cap.release()
        cv2.destroyAllWindows()

    # ---------- hiển thị ----------
    def draw(self, frame, faces, label, fps):
        vis = frame.copy()
        h, w = vis.shape[:2]
        for i, f in enumerate(faces):
            x, y, bw, bh = f.box
            c = COLOR[f.label] if i == 0 else (120, 120, 120)
            cv2.rectangle(vis, (x, y), (x + bw, y + bh), c, 2 if i == 0 else 1)
            txt = f'{f.label} {f.scores[f.label] * 100:.0f}%'
            (tw, th), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            cv2.rectangle(vis, (x, y - th - 10), (x + tw + 8, y), c, -1)
            cv2.putText(vis, txt, (x + 4, y - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

        # Panel bên phải: điểm 6 lớp đã làm mượt
        pw = 230
        panel = np.full((h, pw, 3), 30, np.uint8)
        cv2.putText(panel, 'Diem (lam muot)', (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (230, 230, 230), 1)
        for i, e in enumerate(EMOTIONS):
            y0 = 45 + i * 34
            s = self.smooth.mean.get(e, 0.0)
            cv2.rectangle(panel, (95, y0), (95 + int(s * 120), y0 + 20), COLOR[e], -1)
            cv2.rectangle(panel, (95, y0), (215, y0 + 20), (90, 90, 90), 1)
            col = (255, 255, 255) if e == label else (170, 170, 170)
            cv2.putText(panel, e, (8, y0 + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)
            cv2.putText(panel, f'{s * 100:.0f}', (180, y0 + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)

        # Trạng thái robot (từ emotion_motion_node)
        y0 = 260
        cv2.putText(panel, 'TurtleBot', (10, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (230, 230, 230), 1)
        st = self.status if time.time() - self.status_time < 1.5 else None
        if st is None:
            lines = [('chua ket noi', (0, 0, 255)), ('(emotion_motion_node)', (150, 150, 150))]
        else:
            lines = [(st['action'] or '-', (0, 255, 255)),
                     ('dang chay' if st['running'] else ('dang roi dock...' if not st['ready'] else 'dung yen'),
                      (0, 255, 0) if st['running'] else (200, 200, 200)),
                     (f"v={st['v']:+.2f} m/s", (220, 220, 220)), (f"w={st['w']:+.2f} rad/s", (220, 220, 220)),
                     (f"luot: {st['count']}", (220, 220, 220))]
            if st['running']:
                cv2.rectangle(panel, (10, y0 + 140), (10 + int(st['progress'] * 205), y0 + 150), (0, 255, 0), -1)
                cv2.rectangle(panel, (10, y0 + 140), (215, y0 + 150), (90, 90, 90), 1)
        for i, (t, c) in enumerate(lines):
            cv2.putText(panel, t, (10, y0 + 28 + i * 24), cv2.FONT_HERSHEY_SIMPLEX, 0.52, c, 1)

        # Thanh trên cùng: biểu cảm ổn định -> hành động
        cv2.rectangle(vis, (0, 0), (w, 40), (0, 0, 0), -1)
        head = 'TAM DUNG' if self.paused else f'{label.upper()} ({VI_NAME[label]}) -> {ACTION_TEXT[label]}'
        cv2.putText(vis, head, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.75, COLOR.get(label, (255, 255, 255)), 2)
        cv2.putText(vis, f'{fps:4.1f} fps', (w - 95, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1)
        return np.hstack([vis, panel])

    def to_msg(self, img):
        m = Image()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'webcam'
        m.height, m.width = img.shape[:2]
        m.encoding, m.step = 'bgr8', img.shape[1] * 3
        m.data = img.tobytes()
        return m


def main():
    rclpy.init()
    node = EmotionDetector()
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.pub.publish(String(data='neutral'))
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
