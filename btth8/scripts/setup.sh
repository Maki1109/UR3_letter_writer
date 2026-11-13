#!/usr/bin/env bash
# Cài phụ thuộc cho bài 08 (chạy một lần, cần mạng):
#   - numpy<2, tensorflow-cpu 2.15, deepface (+ fer tuỳ chọn) vào btth8/.pydeps – không đụng tới Python hệ thống
#   - trọng số model Emotion của DeepFace (~6 MB, cache ~/.deepface)
#   - world open_arena (bài 06)
# OpenCV dùng bản hệ thống (python3-opencv 4.5.4) nên deepface/fer được cài --no-deps để pip không kéo
# opencv-python về đè lên.
set -e
D="$(cd "$(dirname "$0")/.." && pwd)"
dpkg -s python3-opencv v4l-utils >/dev/null 2>&1 || { sudo apt update; sudo apt install -y python3-opencv v4l-utils; }
PIP="/usr/bin/python3 -m pip install --no-cache-dir --target $D/.pydeps --upgrade"
$PIP "numpy<2" tensorflow-cpu==2.15.1
# setuptools mới bị kéo theo làm hỏng ros2/colcon (cần setuptools<80) -> gỡ khỏi .pydeps
rm -rf "$D"/.pydeps/{setuptools*,pkg_resources,_distutils_hack,distutils-precedence.pth,wheel,wheel-*}
# tf-keras: deepface đặt TF_USE_LEGACY_KERAS=1 nên TF 2.15 cần gói này để có tensorflow.keras
$PIP --no-deps tf-keras==2.15.1 deepface==0.0.93 mtcnn==0.1.1 retina-face==0.0.17 gdown==5.2.0 lz4 joblib \
  pandas==2.2.3 pytz tzdata python-dateutil six requests urllib3 idna certifi charset-normalizer \
  tqdm Pillow beautifulsoup4 soupsieve filelock PySocks
[ "$WITH_FER" = 1 ] && $PIP --no-deps fer==22.5.1 facenet-pytorch moviepy || true
source "$D/scripts/env.sh"
$PY - <<'PY'
import sys; sys.path.insert(0, __import__('os').environ['B8_DIR'] + '/nodes')
from emotion_core import EmotionRecognizer
EmotionRecognizer('deepface'); print('DeepFace OK – model Emotion đã sẵn sàng')
PY
echo "Xong. Kiểm tra: ./scripts/cam_test.sh   và   $PY tests/test_motion_plans.py"
