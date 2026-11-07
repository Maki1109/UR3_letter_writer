#!/usr/bin/env bash
# Cài phụ thuộc cho bài 07 (chạy một lần).
#   - gói ROS: Nav2, TurtleBot 4 navigation (cần sudo); world room_objects của bài 05
#   - openai-whisper, vosk, numpy<2 vào btth7/.pydeps (torch dùng bản đã cài sẵn)
#   - model Whisper "small" (~460 MB, cache ~/.cache/whisper) và Vosk tiếng Việt (~50 MB, btth7/models)
#   - phân vùng bản đồ -> config/zones.yaml
set -e
D="$(cd "$(dirname "$0")/.." && pwd)"
if ! dpkg -s ros-humble-turtlebot4-navigation ros-humble-nav2-bringup alsa-utils >/dev/null 2>&1; then
  sudo apt update
  sudo apt install -y ros-humble-turtlebot4-navigation ros-humble-nav2-bringup alsa-utils
fi
/usr/bin/python3 -m pip install --target "$D/.pydeps" --no-deps "numpy<2" openai-whisper==20240930 \
  numba==0.60.0 llvmlite==0.43.0 tiktoken regex tqdm vosk==0.3.45 srt cffi pycparser websockets
/usr/bin/python3 -c "import torch" 2>/dev/null || /usr/bin/python3 -m pip install --user torch \
  --index-url https://download.pytorch.org/whl/cpu
mkdir -p "$D/models"
if [ ! -d "$D/models/vosk-model-small-vn-0.4" ]; then
  curl -L -o /tmp/vosk-vn.zip https://alphacephei.com/vosk/models/vosk-model-small-vn-0.4.zip
  unzip -q -o /tmp/vosk-vn.zip -d "$D/models" && rm /tmp/vosk-vn.zip
fi
source "$D/scripts/env.sh"
$PY -c "import whisper; whisper.load_model('small')"
$PY "$B5_DIR/scripts/make_world.py"
$PY "$D/tools/segment_map.py"
echo "Xong. Kiểm tra: ./scripts/mic_test.sh  và  python3 tests/test_command_parser.py"
