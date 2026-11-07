#!/usr/bin/env bash
# Kiểm tra micro + nhận dạng: ghi 4 s, phát lại, rồi in văn bản Whisper nhận được (không cần ROS).
#   ./mic_test.sh [vosk]
source "$(dirname "$0")/env.sh"
F=/tmp/btth7_mic.wav
echo "Nói một câu lệnh trong 4 giây, ví dụ: \"Đi đến khu vực A\" ..."
arecord -q -D "${DEVICE:-default}" -f S16_LE -r 16000 -c 1 -d 4 "$F" || exit 1
echo "Phát lại ..."; aplay -q "$F"
$PY - "$F" "${1:-whisper}" <<'PY'
import sys, wave, numpy as np
sys.path.insert(0, __import__('os').environ['B7_DIR'] + '/nodes')
from speech_recognition_node import Recognizer
from command_parser import CommandParser, load_zones
import logging; logging.basicConfig(level=logging.INFO, format='%(message)s')
zones = load_zones(__import__('os').environ['B7_DIR'] + '/config/zones.yaml')
w = wave.open(sys.argv[1]); pcm = np.frombuffer(w.readframes(w.getnframes()), np.int16)
print('RMS =', int(np.sqrt(np.mean(pcm.astype(float) ** 2))), '(im lặng thường < 300)')
text = Recognizer(sys.argv[2], 'small', 'vi', zones, logging.getLogger()).transcribe(pcm)
p = CommandParser(zones)
print(f'Văn bản: "{text}"\n-> {p.parse(text).describe(p.zones)}')
PY
