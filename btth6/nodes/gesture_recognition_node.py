#!/usr/bin/env python3
"""Phần 2 – gesture_recognition_node: webcam (OpenCV) -> MediaPipe Gesture Recognizer -> /detected_gesture.

Cử chỉ:
  - 7 lớp có sẵn của model MediaPipe: Open_Palm, Closed_Fist, Pointing_Up, Victory, Thumb_Up,
    Thumb_Down, ILoveYou.
  - Wave (vẫy tay): model không có lớp này, nên node tự nhận: bàn tay mở và đầu các ngón tay đổi chiều trái/phải
    ít nhất `wave_reversals` lần trong `wave_window` giây, mỗi lần dịch quá `wave_swing` (tính theo cỡ
    bàn tay, nên không phụ thuộc khoảng cách tới camera). Trong lúc tay đang dịch nhanh, Open_Palm bị giữ lại để vẫy tay không bị hiểu thành "dừng".

Chống nhiễu: một cử chỉ chỉ được chấp nhận khi độ tin cậy >= `min_score` và xuất hiện liên tục
`stable_frames` khung. Nhãn ổn định (kể cả "None" khi không có tay) được publish ở mỗi khung (~20-30 Hz),
nên /detected_gesture còn đóng vai trò heartbeat: robot_behavior_node dừng robot nếu mất bản tin.

Cửa sổ webcam hiển thị landmark, tên cử chỉ + độ tin cậy của khung hiện tại, nhãn ổn định đã publish và
trạng thái robot (topic /robot_behavior/state).
"""
import collections
import os
import time

import cv2
import mediapipe as mp
import rclpy
from mediapipe.tasks import python as mp_tasks
from mediapipe.tasks.python import vision
from rclpy.node import Node
from std_msgs.msg import String

HAND_CONNECTIONS = [(0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8), (5, 9), (9, 10),
                    (10, 11), (11, 12), (9, 13), (13, 14), (14, 15), (15, 16), (13, 17), (0, 17),
                    (17, 18), (18, 19), (19, 20)]
TIPS = (4, 8, 12, 16, 20)         # đầu 5 ngón: dịch mạnh cả khi vẫy bằng cách xoay quanh cổ tay
DEFAULT_MODEL = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'models', 'gesture_recognizer.task')
COLORS = {'Open_Palm': (0, 0, 255), 'Closed_Fist': (0, 140, 255), 'Thumb_Up': (0, 200, 0),
          'Wave': (255, 160, 0), 'Pointing_Up': (255, 0, 200), 'Victory': (200, 0, 255)}


class WaveDetector:
    """Đếm số lần tâm bàn tay đổi chiều theo phương ngang (có trễ), trong một cửa sổ thời gian."""

    def __init__(self, window, swing, reversals, palm_ratio=0.6):
        self.window, self.swing, self.reversals, self.palm_ratio = window, swing, reversals, palm_ratio
        self.hist = collections.deque()   # (t, x chuẩn hóa theo cỡ bàn tay, khung có phải bàn tay mở/nhòe)

    def reset(self):
        self.hist.clear()

    def update(self, t, x, palm=True):
        self.hist.append((t, x, palm))
        while self.hist and t - self.hist[0][0] > self.window:
            self.hist.popleft()

    def count(self):
        """Số lần đổi chiều có biên độ >= swing trong cửa sổ."""
        n, direction, extreme = 0, 0, None
        for _, x, _ in self.hist:
            if extreme is None:
                extreme = x
            elif direction >= 0 and x < extreme - self.swing:
                n += direction > 0
                direction, extreme = -1, x
            elif direction <= 0 and x > extreme + self.swing:
                n += direction < 0
                direction, extreme = 1, x
            elif (direction > 0 and x > extreme) or (direction < 0 and x < extreme):
                extreme = x
        return n

    def span(self, last):
        """Biên độ dịch chuyển (max - min) trong `last` giây cuối."""
        if not self.hist:
            return 0.0
        t_end = self.hist[-1][0]
        xs = [x for t, x, _ in self.hist if t_end - t <= last]
        return max(xs) - min(xs)

    def detected(self):
        # vài khung bị nhận nhầm sang cử chỉ khác lúc tay đang vẫy thì vẫn chấp nhận, miễn đa số là bàn tay mở
        palm = sum(1 for _, _, ok in self.hist if ok)
        return self.count() >= self.reversals and palm >= self.palm_ratio * len(self.hist)


class GestureRecognitionNode(Node):
    def __init__(self):
        super().__init__('gesture_recognition_node')
        self.declare_parameter('camera', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('model', DEFAULT_MODEL)
        self.declare_parameter('min_score', 0.6)        # ngưỡng độ tin cậy của MediaPipe
        self.declare_parameter('stable_frames', 6)      # số khung liên tiếp để chấp nhận cử chỉ
        self.declare_parameter('wave_window', 1.5)      # s
        self.declare_parameter('wave_swing', 0.4)       # biên độ mỗi lần vẫy, đơn vị = cỡ bàn tay (cổ tay -> gốc ngón giữa)
        self.declare_parameter('hand_lost', 0.4)        # s mất tay liên tục mới xóa lịch sử vẫy
        self.declare_parameter('wave_reversals', 3)     # số lần đổi chiều
        self.declare_parameter('wave_hold', 1.0)        # s giữ nhãn Wave sau khi phát hiện
        self.declare_parameter('still_span', 0.3)       # Open_Palm chỉ tính khi tay gần như đứng yên
        self.declare_parameter('mirror', True)
        self.declare_parameter('show', True)
        self.declare_parameter('trace', '')             # đường dẫn CSV ghi từng khung (để chẩn đoán), '' = tắt
        p = lambda n: self.get_parameter(n).value   # noqa: E731
        self.min_score, self.stable_frames = p('min_score'), p('stable_frames')
        self.wave_hold, self.still_span = p('wave_hold'), p('still_span')
        self.mirror, self.show, self.hand_lost = p('mirror'), p('show'), p('hand_lost')
        self.trace = open(p('trace'), 'w', buffering=1) if p('trace') else None
        if self.trace:
            self.trace.write('t,hand,label,score,x,reversals,palm_frac,out\n')
        self.t_hand = 0.0
        self.wave = WaveDetector(p('wave_window'), p('wave_swing'), p('wave_reversals'))

        self.cap = cv2.VideoCapture(p('camera'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, p('width'))
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, p('height'))
        if not self.cap.isOpened():
            raise RuntimeError(f'Không mở được webcam {p("camera")}')
        model = os.path.normpath(p('model'))
        if not os.path.isfile(model):
            raise RuntimeError(f'Không thấy model {model} (chạy scripts/setup.sh)')
        self.recognizer = vision.GestureRecognizer.create_from_options(vision.GestureRecognizerOptions(
            base_options=mp_tasks.BaseOptions(model_asset_path=model),
            running_mode=vision.RunningMode.VIDEO, num_hands=1,
            min_hand_detection_confidence=0.6, min_hand_presence_confidence=0.6, min_tracking_confidence=0.5))

        self.pub = self.create_publisher(String, '/detected_gesture', 10)
        self.create_subscription(String, '/robot_behavior/state', self.on_state, 10)
        self.robot_state = '(robot_behavior_node chưa chạy)'
        self.candidate, self.count, self.stable = 'None', 0, 'None'
        self.wave_until = 0.0
        self.t0 = time.monotonic()
        self.last_ts = -1
        self.fps, self.t_fps, self.n_fps = 0.0, time.monotonic(), 0
        self.closed = False
        self.create_timer(1 / 30, self.step)
        self.get_logger().info(f'Webcam {p("camera")} + {os.path.basename(model)} sẵn sàng, publish /detected_gesture')

    def on_state(self, msg):
        self.robot_state = msg.data

    def classify(self, result, now):
        """-> (nhãn thô của khung, độ tin cậy, landmark hoặc None)."""
        if not result.hand_landmarks:
            if now - self.t_hand > self.hand_lost:    # vẫy nhanh hay mất tay vài khung vì ảnh nhòe
                self.wave.reset()
            return 'None', 0.0, None
        self.t_hand = now
        lm = result.hand_landmarks[0]
        top = result.gestures[0][0]
        name, score = top.category_name or 'None', top.score
        if score < self.min_score:
            name = 'None'
        # đầu các ngón, chuẩn hóa theo cỡ bàn tay (cổ tay -> gốc ngón giữa) để không phụ thuộc khoảng cách
        cx = sum(lm[i].x for i in TIPS) / len(TIPS)
        size = max(((lm[0].x - lm[9].x) ** 2 + (lm[0].y - lm[9].y) ** 2) ** 0.5, 0.03)
        self.wave.update(now, cx / size, name in ('Open_Palm', 'None'))   # None: ảnh nhòe khi vẫy nhanh
        if self.wave.detected():
            self.wave_until = now + self.wave_hold
            self.wave.reset()
        if now < self.wave_until:
            return 'Wave', 1.0, lm
        if name == 'Open_Palm' and self.wave.span(0.5) > self.still_span:
            name = 'None'                     # tay đang dịch -> có thể đang vẫy, chưa coi là "dừng"
        return name, score, lm

    def step(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn('Không đọc được khung hình từ webcam', throttle_duration_sec=2.0)
            return                             # không publish -> behavior node sẽ dừng robot (mất tín hiệu)
        if self.mirror:
            frame = cv2.flip(frame, 1)
        now = time.monotonic()
        ts = int((now - self.t0) * 1000)
        if ts <= self.last_ts:
            ts = self.last_ts + 1
        self.last_ts = ts
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = self.recognizer.recognize_for_video(mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb), ts)
        name, score, lm = self.classify(result, now)
        if self.trace:
            h = self.wave.hist
            x = f'{h[-1][1]:.3f}' if h and lm is not None else ''
            frac = sum(1 for e in h if e[2]) / len(h) if h else 0
            self.trace.write(f'{now - self.t0:.3f},{int(lm is not None)},{name},{score:.2f},{x},'
                             f'{self.wave.count()},{frac:.2f},{self.stable}\n')

        # lọc nhiễu: cùng một nhãn liên tục stable_frames khung mới được chấp nhận
        if name == self.candidate:
            self.count += 1
        else:
            self.candidate, self.count = name, 1
        if self.count >= self.stable_frames and self.stable != self.candidate:
            self.stable = self.candidate
            self.get_logger().info(f'Cử chỉ: {self.stable}' + (f' ({score:.2f})' if self.stable != 'None' else ''))
        self.pub.publish(String(data=self.stable))

        self.n_fps += 1
        if now - self.t_fps >= 1.0:
            self.fps, self.t_fps, self.n_fps = self.n_fps / (now - self.t_fps), now, 0
        if self.show:
            self.draw(frame, name, score, lm)

    def draw(self, frame, name, score, lm):
        h, w = frame.shape[:2]
        if lm is not None:
            pts = [(int(p.x * w), int(p.y * h)) for p in lm]
            for a, b in HAND_CONNECTIONS:
                cv2.line(frame, pts[a], pts[b], (255, 255, 255), 2)
            for pt in pts:
                cv2.circle(frame, pt, 4, (0, 200, 255), -1)
        cv2.rectangle(frame, (0, 0), (w, 92), (30, 30, 30), -1)
        color = COLORS.get(name, (220, 220, 220))
        cv2.putText(frame, f'{name}  {score * 100:.0f}%' if name != 'None' else 'None', (10, 32),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)
        bar = int(min(self.count, self.stable_frames) / self.stable_frames * 150)
        cv2.rectangle(frame, (w - 165, 14), (w - 15, 30), (90, 90, 90), 1)
        cv2.rectangle(frame, (w - 165, 14), (w - 165 + bar, 30), color, -1)
        cv2.putText(frame, f'{self.fps:.0f} FPS', (w - 110, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        cv2.putText(frame, f'/detected_gesture: {self.stable}', (10, 58),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, COLORS.get(self.stable, (220, 220, 220)), 2)
        n = min(self.wave.count(), self.wave.reversals)
        cv2.putText(frame, f'Wave {n}/{self.wave.reversals}', (w - 110, 74), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    COLORS['Wave'] if n else (200, 200, 200), 1)
        cv2.putText(frame, f'Robot: {self.robot_state}', (10, 82), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 1)
        cv2.imshow('Gesture Recognition - webcam', frame)
        if cv2.waitKey(1) & 0xFF in (27, ord('q')):     # chỉ để đóng cửa sổ, không điều khiển robot
            self.closed = True

    def destroy_node(self):
        self.cap.release()
        if self.trace:
            self.trace.close()
        self.recognizer.close()
        cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    node = GestureRecognitionNode()
    try:
        while rclpy.ok() and not node.closed:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
