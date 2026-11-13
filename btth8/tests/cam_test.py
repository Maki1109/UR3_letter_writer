#!/usr/bin/env python3
"""Thử webcam + DeepFace không cần ROS: hiện khung mặt, biểu cảm tức thời, nhãn đã làm mượt và FPS.
    python3 tests/cam_test.py [device] [số frame (0 = chạy tới khi bấm q)]
In ra thời gian xử lý trung bình mỗi frame khi thoát.
"""
import os
import sys
import time

import cv2

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'nodes'))
from emotion_core import COLOR, EmotionRecognizer, EmotionSmoother  # noqa: E402

dev = sys.argv[1] if len(sys.argv) > 1 else '0'
nmax = int(sys.argv[2]) if len(sys.argv) > 2 else 0
cap = cv2.VideoCapture(int(dev) if dev.isdigit() else dev, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
if not cap.isOpened():
    sys.exit(f'Không mở được camera {dev}')
t0 = time.time()
rec = EmotionRecognizer('deepface')
print(f'Nạp model: {time.time() - t0:.1f} s')
sm = EmotionSmoother()
times, n = [], 0
while True:
    ok, frame = cap.read()
    if not ok:
        break
    frame = cv2.flip(frame, 1)
    t = time.time()
    faces = rec.detect(frame)
    times.append(time.time() - t)
    lab = sm.update(faces[0].scores if faces else None)
    for f in faces:
        x, y, w, h = f.box
        cv2.rectangle(frame, (x, y), (x + w, y + h), COLOR[f.label], 2)
        cv2.putText(frame, f'{f.label} {f.scores[f.label]:.2f}', (x, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    COLOR[f.label], 2)
    cv2.putText(frame, f'on dinh: {lab}   {1 / max(times[-1], 1e-3):.1f} fps', (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    n += 1
    if n % 10 == 0:
        print(f'frame {n}: {len(faces)} mặt, tức thời={faces[0].label if faces else "-"}, ổn định={lab}')
    if os.environ.get('DISPLAY'):
        cv2.imshow('cam_test (q de thoat)', frame)
        if cv2.waitKey(1) & 0xFF in (27, ord('q')):
            break
    if nmax and n >= nmax:
        cv2.imwrite('/tmp/btth8_cam_test.png', frame)
        break
if times:
    print(f'{n} frame, xử lý trung bình {1000 * sum(times) / len(times):.0f} ms/frame '
          f'(~{len(times) / sum(times):.1f} fps)')
