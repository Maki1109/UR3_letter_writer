#!/usr/bin/env python3
"""Kiểm thử bộ phân tích câu lệnh với config/zones.yaml:  python3 tests/test_command_parser.py"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'nodes'))
from command_parser import CommandParser, load_zones  # noqa: E402

CASES = [
    # (câu lệnh, intent, route, origin)
    ('Đi đến khu vực A.', 'navigate', ['A'], None),
    ('Di chuyển từ khu vực A đến khu vực B.', 'navigate', ['A', 'B'], 'A'),
    ('Đi đến phòng thí nghiệm.', 'navigate', ['C'], None),
    ('đi đến khu vực bê', 'navigate', ['B'], None),             # Vosk: chữ cái đọc kiểu Việt
    ('đi đến khu vực xê', 'navigate', ['C'], None),
    ('Đi đến D', 'navigate', ['D'], None),
    ('đi đến khu vực hành lang', 'navigate', ['B'], None),
    ('Đi đến hành lang rồi về phòng học', 'navigate', ['B', 'A'], None),
    ('di chuyen tu phong hoc den sanh', 'navigate', ['A', 'D'], 'A'),   # không dấu
    ('Từ sảnh đi đến phòng lab', 'navigate', ['D', 'C'], 'D'),
    ('đi đến phòng thí nghiệp', 'navigate', ['C'], None),       # lỗi nhận dạng 1 âm
    ('Quay về sảnh nhé', 'navigate', ['D'], None),
    ('Đi tới khu vực E', 'unknown_zone', [], None),
    ('Đi đến phòng ngủ', 'unknown_zone', [], None),
    ('Đi đến khu vực A, sau đó đến khu vực Z', 'unknown_zone', [], None),
    # nhiều điểm đến
    ('Đi từ phòng A qua hành lang C rồi đến phòng B', 'navigate', ['A', 'C', 'B'], 'A'),
    ('đi từ phòng a qua hành lang xê rồi đến phòng bê', 'navigate', ['A', 'C', 'B'], 'A'),   # kiểu Vosk
    ('Đi qua sảnh rồi đến phòng thí nghiệm, sau đó về phòng học', 'navigate', ['D', 'C', 'A'], None),
    ('Đi đến khu vực A rồi B rồi C', 'navigate', ['A', 'B', 'C'], None),
    ('Đi từ khu vực D qua khu vực B đến khu vực A rồi đến khu vực C', 'navigate', ['D', 'B', 'A', 'C'], 'D'),
    ('Đi đến phòng học ạ', 'navigate', ['A'], None),             # "ạ" là tiểu từ, không phải chữ A
    ('Đi từ phòng A qua hành lang E rồi đến phòng B', 'unknown_zone', [], None),
    ('Dừng lại!', 'stop', [], None),
    ('Hủy', 'stop', [], None),
    ('Hãy đăng ký kênh để ủng hộ', 'not_understood', [], None),  # câu Whisper hay "ảo giác" khi chỉ có ồn
    ('', 'not_understood', [], None),
]


def main():
    p = CommandParser(load_zones(os.path.join(HERE, '..', 'config', 'zones.yaml')))
    fails = 0
    for text, intent, route, origin in CASES:
        c = p.parse(text)
        ok = (c.intent, c.route, c.origin) == (intent, route, origin)
        fails += not ok
        print(f"{'OK ' if ok else 'SAI'} {text!r:45} -> {c.describe(p.zones)}")
    print(f'{len(CASES) - fails}/{len(CASES)} đúng')
    sys.exit(1 if fails else 0)


if __name__ == '__main__':
    main()
