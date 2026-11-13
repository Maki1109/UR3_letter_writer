"""Nhận diện biểu cảm khuôn mặt (không phụ thuộc ROS): backend DeepFace / FER + bộ lọc làm mượt theo thời gian.

    rec = EmotionRecognizer('deepface')
    faces = rec.detect(frame_bgr)        # [Face(box=(x,y,w,h), scores={'happy':0.9,...}), ...]
    smoother.update(faces[0].scores)     # -> nhãn ổn định ('happy', 'sad', ..., 'none')
"""
from collections import deque
from dataclasses import dataclass, field
import os

import numpy as np

# 6 trạng thái của đề bài (DeepFace/FER còn trả 'disgust' -> bỏ, chuẩn hoá lại trên 6 lớp)
EMOTIONS = ('happy', 'sad', 'neutral', 'angry', 'surprise', 'fear')

VI_NAME = {'happy': 'Vui ve', 'sad': 'Buon', 'neutral': 'Binh thuong', 'angry': 'Tuc gian',
           'surprise': 'Ngac nhien', 'fear': 'So hai', 'none': 'Khong thay mat'}

# Màu BGR cho từng biểu cảm (khung mặt, thanh điểm)
COLOR = {'happy': (0, 200, 255), 'sad': (255, 120, 0), 'neutral': (200, 200, 200),
         'angry': (0, 0, 255), 'surprise': (255, 0, 255), 'fear': (0, 180, 0), 'none': (90, 90, 90)}


@dataclass
class Face:
    box: tuple                     # (x, y, w, h) theo pixel của ảnh gốc
    scores: dict = field(default_factory=dict)   # emotion -> xác suất (tổng = 1 trên 6 lớp)

    @property
    def label(self):
        return max(self.scores, key=self.scores.get) if self.scores else 'none'

    @property
    def area(self):
        return self.box[2] * self.box[3]


def _normalize(raw):
    """dict điểm (phần trăm hoặc xác suất, có thể có 'disgust') -> xác suất trên 6 lớp."""
    s = np.array([float(raw.get(e, 0.0)) for e in EMOTIONS])
    tot = s.sum()
    s = s / tot if tot > 0 else np.full(len(EMOTIONS), 1 / len(EMOTIONS))
    return dict(zip(EMOTIONS, s.tolist()))


class EmotionRecognizer:
    """backend='deepface' (mặc định) hoặc 'fer'. Cả hai dùng CNN huấn luyện trên FER-2013 (ảnh xám 48x48).

    detector: bộ dò mặt của DeepFace ('opencv' = Haar cascade, nhanh trên CPU; 'ssd', 'mtcnn', 'retinaface'
    chính xác hơn nhưng chậm). Để có tốc độ thời gian thực, ảnh được thu nhỏ còn `max_width` trước khi dò.
    """

    def __init__(self, backend='deepface', detector='opencv', max_width=640):
        self.backend = backend
        self.detector = detector
        self.max_width = max_width
        if backend == 'deepface':
            os.environ.setdefault('TF_CPP_MIN_LOG_LEVEL', '3')
            from deepface import DeepFace
            self._df = DeepFace
            self._fix_haar_path()
            # Nạp model một lần (tải trọng số ~6 MB về ~/.deepface lần đầu) để frame đầu không bị khựng
            try:
                DeepFace.build_model(task='facial_attribute', model_name='Emotion')
            except TypeError:          # deepface < 0.0.93
                DeepFace.build_model('Emotion')
        elif backend == 'fer':
            from fer import FER
            self._fer = FER(mtcnn=(detector == 'mtcnn'))
        else:
            raise ValueError(f'backend không hỗ trợ: {backend}')

    @staticmethod
    def _fix_haar_path():
        """DeepFace tìm Haar cascade ở <thư mục cv2>/data (có trong opencv-python của pip). OpenCV bản apt
        (python3-opencv) để file ở /usr/share/opencv4/haarcascades -> trỏ DeepFace sang đó."""
        import cv2
        if os.path.isfile(os.path.join(os.path.dirname(cv2.__file__), 'data',
                                       'haarcascade_frontalface_default.xml')):
            return
        for d in ('/usr/share/opencv4/haarcascades', '/usr/share/opencv/haarcascades',
                  getattr(getattr(cv2, 'data', None), 'haarcascades', '')):
            if d and os.path.isfile(os.path.join(d, 'haarcascade_frontalface_default.xml')):
                try:
                    from deepface.models.face_detection.OpenCv import OpenCvClient
                except ImportError:    # deepface < 0.0.92
                    from deepface.detectors.OpenCv import OpenCvClient
                OpenCvClient._OpenCvClient__get_opencv_path = lambda self, d=d: d
                return

    def detect(self, frame):
        h, w = frame.shape[:2]
        k = min(1.0, self.max_width / w)
        small = frame if k == 1.0 else __import__('cv2').resize(frame, (int(w * k), int(h * k)))
        faces = self._detect_deepface(small) if self.backend == 'deepface' else self._detect_fer(small)
        for f in faces:                # đưa toạ độ về ảnh gốc
            f.box = tuple(int(round(v / k)) for v in f.box)
        return sorted(faces, key=lambda f: f.area, reverse=True)

    def _detect_deepface(self, img):
        try:
            res = self._df.analyze(img, actions=['emotion'], detector_backend=self.detector,
                                   enforce_detection=True, silent=True)
        except ValueError as e:        # chỉ nuốt lỗi "Face could not be detected", lỗi khác (thiếu model...) báo ra
            if 'could not be detected' in str(e) or 'Face' in str(e):
                return []
            raise
        out = []
        for r in (res if isinstance(res, list) else [res]):
            reg = r.get('region', {})
            box = (reg.get('x', 0), reg.get('y', 0), reg.get('w', 0), reg.get('h', 0))
            # Khi dò mặt thất bại một phần, DeepFace có thể trả cả khung hình -> bỏ
            if box[2] <= 0 or box[2] >= img.shape[1] - 2:
                continue
            out.append(Face(box, _normalize(r['emotion'])))
        return out

    def _detect_fer(self, img):
        return [Face(tuple(r['box']), _normalize(r['emotions'])) for r in self._fer.detect_emotions(img)]


class EmotionSmoother:
    """Làm mượt nhãn theo thời gian để robot không giật theo từng frame nhiễu.

    Giữ `window` vector điểm gần nhất (None = frame không thấy mặt) và lấy trung bình. Nhãn mới chỉ được chấp
    nhận khi: lớp có điểm trung bình cao nhất >= `min_score` và thắng ở >= `min_ratio` số frame trong cửa sổ.
    Không đạt -> giữ nhãn cũ. Không thấy mặt ở quá nửa cửa sổ -> 'none'.
    """

    def __init__(self, window=8, min_score=0.40, min_ratio=0.6):
        self.buf = deque(maxlen=window)
        self.min_score = min_score
        self.min_ratio = min_ratio
        self.label = 'none'
        self.mean = {e: 0.0 for e in EMOTIONS}

    def update(self, scores):
        self.buf.append(scores)
        seen = [s for s in self.buf if s]
        if len(seen) * 2 <= len(self.buf):
            self.label = 'none'
            self.mean = {e: 0.0 for e in EMOTIONS}
            return self.label
        self.mean = {e: float(np.mean([s[e] for s in seen])) for e in EMOTIONS}
        best = max(self.mean, key=self.mean.get)
        wins = sum(1 for s in seen if max(s, key=s.get) == best)
        if self.mean[best] >= self.min_score and wins >= self.min_ratio * len(seen):
            self.label = best
        elif self.label == 'none':     # vừa thấy mặt lại nhưng chưa rõ biểu cảm -> coi như bình thường (dừng)
            self.label = 'neutral'
        return self.label

    def reset(self):
        self.buf.clear()
        self.label = 'none'
