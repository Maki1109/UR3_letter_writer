#!/usr/bin/env python3
"""Phần 3 – Nhận dạng giọng nói: micro -> phát hiện câu nói (VAD) -> Whisper / Vosk -> /voice_command.

Thu âm bằng `arecord` (ALSA/PulseAudio, 16 kHz mono, không cần thêm thư viện âm thanh). Âm thanh được cắt thành
khung 30 ms; năng lượng (RMS) của khung so với mức ồn nền (đo 1 s lúc khởi động) để biết đang có tiếng nói hay
không. Một câu kết thúc khi im lặng `silence` giây (mặc định 0.8 s) hoặc dài quá `max_seconds`. Mỗi câu được
chuyển thành văn bản bằng:
  - engine:=whisper  (mặc định) OpenAI Whisper `small`, language=vi. `initial_prompt` chứa sẵn tên các khu vực
    trong zones.yaml và câu mẫu, để Whisper viết đúng "khu vực A", "Phòng thí nghiệm"...
  - engine:=vosk     Vosk vosk-model-small-vn-0.4 (nhẹ, offline, nhanh). Bộ giải mã bị giới hạn trong danh sách câu
    lệnh sinh từ zones.yaml (grammar) nên ít nhận nhầm, nhưng không nhận được câu nằm ngoài danh sách đó.
  - engine:=text     gõ câu lệnh từ bàn phím (thử hệ thống khi không có micro).
Văn bản được publish lên /voice_command (std_msgs/String). Node điều hướng phân tích câu đó.

    python3 nodes/speech_recognition_node.py --ros-args -p engine:=whisper -p model:=small
    python3 nodes/speech_recognition_node.py --ros-args -p wav:=lenh.wav    # nhận dạng file thay cho micro
"""
import audioop
import json
import os
import queue
import subprocess
import sys
import threading
import time
import wave

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

HERE = os.path.dirname(os.path.abspath(__file__))
B7_DIR = os.path.dirname(HERE)
sys.path.insert(0, HERE)
from command_parser import load_zones  # noqa: E402

RATE = 16000
FRAME = 480                     # 30 ms
# Câu mẫu cho Whisper. Có cả một khu vực không tồn tại (E): nếu prompt chỉ liệt kê các khu vực có thật, Whisper
# hay "sửa" câu "khu vực E" thành "khu vực A" và hệ thống không còn phát hiện được khu vực lạ.
PROMPT = ('Đi đến khu vực A. Di chuyển từ khu vực C đến khu vực B. Đi tới khu vực E. '
          'Đi từ phòng A qua hành lang C rồi đến phòng B. ')
# Vosk: grammar dạng danh sách câu (chỉ liệt kê từ đơn thì Vosk ghép từ lung tung, ví dụ "đi điểm khu vực").
# Chữ cái đọc kiểu Việt; gồm cả E, F, G (không tồn tại) để còn báo lỗi được.
VOSK_LETTERS = ['a', 'bê', 'xê', 'đê', 'e', 'ép', 'gờ']
VOSK_VERBS = ['đi đến', 'đi tới', 'đến', 'tới', 'về', 'quay về', 'di chuyển đến', 'di chuyển tới']


def vosk_phrases(zones):
    places = [f'khu vực {c}' for c in VOSK_LETTERS] + [n.lower() for z in zones
                                                     for n in [z['name'], *z.get('aliases', [])]]
    out = [f'{v} {p}' for v in VOSK_VERBS for p in places]
    out += [f'{v} từ {a} đến {b}' for v in ('di chuyển', 'đi') for a in places for b in places if a != b]
    out += [f'{v} {a} rồi đến {b}' for v in ('đi đến',) for a in places for b in places if a != b]
    # 3 điểm "đi từ X qua Y rồi đến Z": chỉ dùng "khu vực <chữ>" và tên chính (đủ mọi tổ hợp bí danh thì >10^4 câu)
    short = [f'khu vực {c}' for c in VOSK_LETTERS] + [z['name'].lower() for z in zones]
    short += [f'{t} {c}' for t in ('phòng', 'hành lang') for c in VOSK_LETTERS]          # "hành lang xê"
    out += [f'đi từ {a} qua {b} rồi đến {c}' for a in short for b in short for c in short
            if a != b and b != c]
    return out + ['dừng lại', 'đứng lại', 'hủy', '[unk]']


class Recognizer:
    """Bọc Whisper / Vosk: transcribe(np.int16 mảng 16 kHz) -> str."""

    def __init__(self, engine, model, language, zones, log):
        self.engine = engine
        if engine == 'whisper':
            import torch
            import whisper
            torch.set_num_threads(int(os.environ.get('WHISPER_THREADS', '4')))   # chừa CPU cho Gazebo/Nav2
            t = time.time()
            self.model = whisper.load_model(model, device='cpu')
            log.info(f'Whisper "{model}" nạp xong sau {time.time() - t:.1f}s')
            self.language = language
            self.prompt = PROMPT + ', '.join(z['name'] for z in zones) + '.'
        elif engine == 'vosk':
            import vosk
            vosk.SetLogLevel(-1)
            path = model if os.path.isdir(model) else os.path.join(B7_DIR, 'models', 'vosk-model-small-vn-0.4')
            self.model = vosk.Model(path)
            phrases = vosk_phrases(zones)
            self.grammar = json.dumps(phrases, ensure_ascii=False)
            log.info(f'Vosk {os.path.basename(path)} nạp xong, grammar {len(phrases)} câu')

    def transcribe(self, pcm):
        if self.engine == 'whisper':
            audio = pcm.astype(np.float32) / 32768.0
            audio -= audio.mean()                       # bỏ lệch DC
            peak = float(np.percentile(np.abs(audio), 99.5))
            if peak > 1e-4:                             # micro laptop thường rất nhỏ: kéo biên độ lên ~0.5
                audio = np.clip(audio * min(0.5 / peak, 30.0), -1.0, 1.0)
            r = self.model.transcribe(audio, language=self.language, task='transcribe', fp16=False,
                                      temperature=0.0, initial_prompt=self.prompt,
                                      condition_on_previous_text=False)
            segs = r.get('segments', [])
            if segs and np.mean([s['no_speech_prob'] for s in segs]) > 0.6 \
                    and np.mean([s['avg_logprob'] for s in segs]) < -1.0:
                return ''           # Whisper tự đánh giá là không có tiếng nói (và cũng không tự tin vào chữ nghe được)
            return r['text'].strip()
        import vosk
        rec = vosk.KaldiRecognizer(self.model, RATE, self.grammar)
        rec.AcceptWaveform(pcm.tobytes())
        text = json.loads(rec.FinalResult()).get('text', '')
        return ' '.join(w for w in text.split() if w != '[unk]')


class SpeechNode(Node):
    def __init__(self):
        super().__init__('speech_recognition')
        p = self.declare_parameter
        self.engine = p('engine', 'whisper').value
        model = p('model', 'small').value
        language = p('language', 'vi').value
        self.device = p('device', 'default').value          # arecord -D ...
        self.wav = p('wav', '').value                       # nhận dạng file thay cho micro
        self.silence = p('silence', 0.8).value
        self.max_seconds = p('max_seconds', 7.0).value
        self.min_speech = p('min_speech', 0.35).value
        self.threshold = p('threshold', 0.0).value          # RMS; 0 = tự đo ồn nền
        zones_file = p('zones_file', os.path.join(B7_DIR, 'config', 'zones.yaml')).value
        topic = p('topic', '/voice_command').value

        self.pub = self.create_publisher(String, topic, 10)
        self.state_pub = self.create_publisher(String, '/speech/state', 10)
        self.utterances = queue.Queue()
        zones = load_zones(zones_file)
        self.rec = None if self.engine == 'text' else Recognizer(self.engine, model, language, zones, self.get_logger())
        threading.Thread(target=self.worker, daemon=True).start()
        if self.engine == 'text':
            threading.Thread(target=self.keyboard, daemon=True).start()
        elif self.wav:
            threading.Thread(target=self.from_files, daemon=True).start()
        else:
            threading.Thread(target=self.listen, daemon=True).start()

    def state(self, s):
        self.state_pub.publish(String(data=s))

    def publish(self, text, dt=None):
        text = text.strip()
        if not text:
            self.get_logger().info('(không nghe rõ – bỏ qua)')
            return
        self.pub.publish(String(data=text))
        extra = f'  [{dt:.1f}s]' if dt is not None else ''
        self.get_logger().info(f'>>> "{text}"{extra}  -> {self.pub.topic_name}')

    # --- nguồn âm thanh --------------------------------------------------------------------------------------
    def listen(self):
        cmd = ['arecord', '-q', '-D', self.device, '-f', 'S16_LE', '-r', str(RATE), '-c', '1', '-t', 'raw']
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE)
        read = lambda: proc.stdout.read(FRAME * 2)
        thr = self.threshold
        if thr <= 0:                                   # đo ồn nền 1 s (đừng nói lúc này)
            levels = [audioop.rms(read(), 2) for _ in range(int(1.5 * RATE / FRAME))]
            noise = float(np.median(levels))            # trung vị: bỏ qua các tiếng động ngắn lúc đo
            thr = max(2.2 * noise, noise + 250.0, 300.0)
            self.get_logger().info(f'Ồn nền RMS {noise:.0f} -> ngưỡng tiếng nói {thr:.0f} (tự cập nhật; cố định: -p threshold:=...)')
        self.get_logger().info('Đang nghe... hãy nói câu lệnh, ví dụ "Đi đến khu vực A"')
        self.state('listening')
        pre = []                                       # giữ 0.3 s trước khi bắt đầu nói
        voiced, silent, frames = 0, 0, []
        while rclpy.ok():
            buf = read()
            if len(buf) < FRAME * 2:
                self.get_logger().error(f'arecord dừng (thiết bị "{self.device}"?)')
                return
            lv = audioop.rms(buf, 2)
            loud = lv > thr
            if self.threshold <= 0 and not loud:        # theo dõi ồn nền (quạt máy tăng/giảm theo tải CPU)
                noise = 0.97 * noise + 0.03 * lv
                thr = max(2.2 * noise, noise + 250.0, 300.0)
            if not frames:
                pre = (pre + [buf])[-10:]
                if loud:
                    frames, voiced, silent = list(pre), 1, 0
                    self.state('hearing')
                continue
            frames.append(buf)
            voiced += loud
            silent = 0 if loud else silent + 1
            dur = len(frames) * FRAME / RATE
            if silent * FRAME / RATE >= self.silence or dur >= self.max_seconds:
                if voiced * FRAME / RATE >= self.min_speech:
                    self.utterances.put(np.frombuffer(b''.join(frames), np.int16))
                frames = []
                self.state('listening')

    def from_files(self):
        for path in self.wav.split(','):
            if not os.path.isfile(path):
                self.get_logger().error(f'Không thấy file {path}')
                continue
            with wave.open(path) as w:
                assert w.getframerate() == RATE and w.getnchannels() == 1 and w.getsampwidth() == 2, \
                    f'{path}: cần WAV 16 kHz mono 16-bit (ffmpeg -i in -ar 16000 -ac 1 out.wav)'
                self.get_logger().info(f'File {path}')
                self.utterances.put(np.frombuffer(w.readframes(w.getnframes()), np.int16))
        self.utterances.put(None)

    def keyboard(self):
        self.get_logger().info('Chế độ text: gõ câu lệnh rồi Enter (Ctrl+D để thoát)')
        for line in sys.stdin:
            self.publish(line)
        self.utterances.put(None)

    # --- nhận dạng -------------------------------------------------------------------------------------------
    def worker(self):
        while rclpy.ok():
            pcm = self.utterances.get()
            if pcm is None:
                time.sleep(0.5)       # cho message cuối kịp gửi đi
                rclpy.try_shutdown()
                return
            self.state('recognizing')
            self.get_logger().info(f'Nhận dạng câu {len(pcm) / RATE:.1f}s ...')
            t = time.time()
            text = self.rec.transcribe(pcm)
            self.publish(text, time.time() - t)
            self.state('listening')


def main():
    rclpy.init(args=sys.argv)
    node = SpeechNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
