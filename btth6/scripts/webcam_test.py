#!/usr/bin/env python3
"""Phần 1 – Đọc và hiển thị webcam bằng OpenCV.   Chạy: ./scripts/webcam_test.sh [camera_id]   (q / Esc: thoát)"""
import sys
import time

import cv2

cam = int(sys.argv[1]) if len(sys.argv) > 1 else 0
cap = cv2.VideoCapture(cam)
if not cap.isOpened():
    sys.exit(f'Không mở được webcam {cam}')
t, n, fps = time.monotonic(), 0, 0.0
while True:
    ok, frame = cap.read()
    if not ok:
        sys.exit('Không đọc được khung hình')
    frame = cv2.flip(frame, 1)
    n += 1
    if time.monotonic() - t >= 1.0:
        fps, t, n = n / (time.monotonic() - t), time.monotonic(), 0
    h, w = frame.shape[:2]
    cv2.putText(frame, f'Webcam {cam}: {w}x{h}  {fps:.0f} FPS', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    cv2.imshow('Webcam test (OpenCV)', frame)
    if cv2.waitKey(1) & 0xFF in (27, ord('q')):
        break
cap.release()
cv2.destroyAllWindows()
