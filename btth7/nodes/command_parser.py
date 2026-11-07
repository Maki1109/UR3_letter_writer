#!/usr/bin/env python3
"""Phân tích câu lệnh điều hướng tiếng Việt (văn bản từ Whisper/Vosk) thành lộ trình qua các khu vực.

Không phụ thuộc ROS, chạy thử trực tiếp:
    python3 nodes/command_parser.py "Di chuyển từ khu vực A đến khu vực B"

Cách làm:
  1. Chuẩn hóa: chữ thường, bỏ dấu câu, bỏ dấu tiếng Việt (đ -> d), nên "Phòng Thí Nghiệm", "phong thi nghiem"
     và "phòng thí nghiệm." là như nhau.
  2. Tìm mọi lần nhắc tới khu vực theo thứ tự xuất hiện:
     - tên/bí danh trong zones.yaml (ưu tiên cụm dài nhất; cụm >= 8 ký tự cho phép sai nhẹ, ví dụ lỗi nhận dạng
       "phòng thí nghiệp");
     - "khu vực / khu / vùng / phòng / điểm" + chữ cái, đọc kiểu Việt cũng được ("bê", "xê", "đê"...);
     - chữ cái đứng một mình sau "đến / tới / sang / qua / về / từ / rồi / và" ("đi đến A", "rồi B");
     - tên loại phòng + chữ cái ("hành lang C", "phòng B"): CHỮ CÁI quyết định khu vực, tên chỉ là cách gọi. Chỉ
       áp dụng khi chữ cái viết hoa (Whisper) hoặc đọc kiểu Việt nhiều âm ("bê", "xê" – Vosk), để tiểu từ "ạ"/"à"
       ở cuối câu ("đi đến phòng học ạ") không bị hiểu thành khu vực A.
     Cụm "khu vực X" hoặc "phòng X" mà X không khớp khu vực nào thì được ghi lại là khu vực không tồn tại.
  3. Ý định:
     - có khu vực không tồn tại -> từ chối cả câu, báo các khu vực hợp lệ;
     - có ít nhất một khu vực -> lộ trình theo thứ tự nhắc tới, bao nhiêu điểm cũng được ("Đi từ phòng A qua
       hành lang C rồi đến phòng B" -> A, C, B). Khu vực đứng sau "từ" là điểm xuất phát: node điều hướng bỏ qua
       nó nếu robot đang ở sẵn trong đó;
     - "dừng", "đứng lại", "hủy" (không kèm khu vực) -> dừng;
     - còn lại -> không hiểu (ví dụ Whisper "ảo giác" ra câu vô nghĩa khi chỉ có tiếng ồn).
"""
import difflib
import re
import sys
import unicodedata
from dataclasses import dataclass, field

# Cách đọc chữ cái trong tiếng Việt (đã bỏ dấu) -> chữ cái. Không có "di"/"dê" cho D vì trùng với "đi" (bỏ dấu)
# và "dê"; D được nhận qua "d", "de" (đê), "dzi".
LETTER_WORDS = {
    'a': 'A', 'ay': 'A', 'b': 'B', 'be': 'B', 'bo': 'B', 'bi': 'B', 'c': 'C', 'xe': 'C', 'ce': 'C', 'xi': 'C',
    'si': 'C', 'co': 'C', 'd': 'D', 'de': 'D', 'dzi': 'D', 'e': 'E', 'f': 'F', 'ep': 'F', 'g': 'G', 'gio': 'G',
    'h': 'H', 'hat': 'H', 'i': 'I', 'j': 'J', 'k': 'K', 'ca': 'K', 'l': 'L', 'm': 'M', 'n': 'N', 'o': 'O',
    'p': 'P', 'q': 'Q', 'r': 'R', 's': 'S', 't': 'T', 'u': 'U', 'v': 'V', 'w': 'W', 'x': 'X', 'y': 'Y', 'z': 'Z',
}
AREA_WORDS = [('khu', 'vuc'), ('khu',), ('vung',), ('phong',), ('diem',)]   # "khu vực" phải thử trước "khu"
TO_WORDS = {'den', 'toi', 'sang', 'qua', 've', 'vao', 'tu', 'roi', 'va'}
STOP_WORDS = {'dung', 'huy', 'stop', 'thoi'}
# từ nối: kết thúc tên một khu vực lạ ("phòng ngủ rồi đến ...")
CONNECTORS = TO_WORDS | {'roi', 'sau', 'do', 'va', 'di', 'chuyen', 'nhe', 'nha', 'giup', 'toi', 'minh', 'cua',
                         'ha', 'a', 'khu', 'vuc', 'phong'}


def strip_accents(text):
    text = text.lower().replace('đ', 'd')
    text = ''.join(c for c in unicodedata.normalize('NFD', text) if unicodedata.category(c) != 'Mn')
    return re.sub(r'[^a-z0-9 ]+', ' ', text)


def words(text):
    return strip_accents(text).split()


@dataclass
class Mention:
    pos: int            # vị trí từ (token) trong câu
    end: int
    zone: str = None    # id khu vực nếu tồn tại
    spoken: str = ''    # cách nói trong câu (khi khu vực không tồn tại)


@dataclass
class Command:
    intent: str                     # navigate | stop | unknown_zone | not_understood
    route: list = field(default_factory=list)   # id khu vực theo thứ tự đi
    origin: str = None              # khu vực xuất phát ("từ A ..."), có thể bỏ qua nếu robot đang ở đó
    unknown: list = field(default_factory=list)  # tên khu vực không tồn tại
    text: str = ''

    def describe(self, zones):
        name = lambda z: f"{z} ({zones[z]['name']})" if z in zones else z
        if self.intent == 'navigate':
            s = ' -> '.join(name(z) for z in self.route)
            return f'Đi: {s}' + (f' (xuất phát từ {name(self.origin)})' if self.origin else '')
        if self.intent == 'stop':
            return 'Dừng'
        if self.intent == 'unknown_zone':
            return (f"Không có khu vực {', '.join(repr(u) for u in self.unknown)}. "
                    f"Các khu vực hiện có: {', '.join(name(z) for z in zones)}")
        return f'Không hiểu câu lệnh: "{self.text}"'


class CommandParser:
    def __init__(self, zones):
        """zones: danh sách dict có 'id', 'name', 'aliases' (như trong config/zones.yaml)."""
        self.zones = {str(z['id']): z for z in zones}
        self.phrases = []       # (tokens, zone_id)
        for zid, z in self.zones.items():
            for p in [z['name'], *z.get('aliases', [])]:
                toks = tuple(words(p))
                if toks:
                    self.phrases.append((toks, zid))
        self.phrases.sort(key=lambda p: -len(p[0]))      # cụm dài khớp trước

    # --- tìm khu vực -------------------------------------------------------------------------------------
    def _match_phrase(self, toks, i):
        for p, zid in self.phrases:
            n = len(p)
            cand = toks[i:i + n]
            if tuple(cand) == p:
                return n, zid
            a, b = ' '.join(p), ' '.join(cand)
            if len(cand) == n and len(a) >= 8 and difflib.SequenceMatcher(None, a, b).ratio() >= 0.85:
                return n, zid
        return None

    def _letter_after(self, toks, cased, k):
        """Chữ cái chỉ định khu vực ở vị trí k ("hành lang C", "phòng bê"), hoặc None."""
        if k >= len(toks) or toks[k] not in LETTER_WORDS:
            return None
        if (len(cased[k]) == 1 and cased[k].isupper()) or len(toks[k]) >= 2:
            return LETTER_WORDS[toks[k]]
        return None

    def _mentions(self, toks, orig, cased):
        out, i = [], 0
        while i < len(toks):
            m = self._match_phrase(toks, i)
            if m:
                letter = self._letter_after(toks, cased, i + m[0])
                if letter:                                                  # "hành lang C" -> C
                    out.append(Mention(i, i + m[0] + 1, zone=letter if letter in self.zones else None,
                                       spoken=' '.join(orig[i:i + m[0] + 1])))
                    i += m[0] + 1
                    continue
                out.append(Mention(i, i + m[0], zone=m[1]))
                i += m[0]
                continue
            area = next((a for a in AREA_WORDS if tuple(toks[i:i + len(a)]) == a), None)
            if area:
                j = i + len(area)
                m = self._match_phrase(toks, j)
                if m:                                                       # "khu vực hành lang"
                    out.append(Mention(i, j + m[0], zone=m[1]))
                    i = j + m[0]
                    continue
                if j < len(toks) and toks[j] in LETTER_WORDS:              # "khu vực bê"
                    letter = LETTER_WORDS[toks[j]]
                    out.append(Mention(i, j + 1, zone=letter if letter in self.zones else None,
                                       spoken=f'khu vực {letter}'))
                    i = j + 1
                    continue
                k = j                                                       # "phòng ngủ", "khu vực bếp"
                while k < len(toks) and k - j < 3 and toks[k] not in CONNECTORS:
                    k += 1
                if k > j:
                    out.append(Mention(i, k, spoken=' '.join(orig[i:k])))
                    i = k
                    continue
            elif (toks[i] in LETTER_WORDS and len(toks[i]) <= 2 and i > 0 and toks[i - 1] in TO_WORDS):
                letter = LETTER_WORDS[toks[i]]                              # "đi đến A"
                out.append(Mention(i, i + 1, zone=letter if letter in self.zones else None,
                                   spoken=f'khu vực {letter}'))
            i += 1
        return out

    # --- phân tích -----------------------------------------------------------------------------------------
    def parse(self, text):
        toks = words(text)
        cased = re.sub(r'[^\w ]+', ' ', text).split()           # cùng cách tách từ, còn dấu và chữ hoa
        if len(cased) != len(toks):
            cased = toks
        ms = self._mentions(toks, [w.lower() for w in cased], cased)
        unknown = [m.spoken for m in ms if m.zone is None]
        if unknown:
            return Command('unknown_zone', unknown=unknown, text=text)
        route = []
        for m in ms:
            if not route or route[-1] != m.zone:
                route.append(m.zone)
        if route:
            origin = ms[0].zone if ms[0].pos > 0 and toks[ms[0].pos - 1] == 'tu' and len(route) > 1 else None
            return Command('navigate', route=route, origin=origin, text=text)
        if STOP_WORDS & set(toks):          # "dừng", "đứng lại" (bỏ dấu đều thành "dung"), "hủy"
            return Command('stop', text=text)
        return Command('not_understood', text=text)


def load_zones(path):
    import yaml
    with open(path) as f:
        return yaml.safe_load(f)['zones']


if __name__ == '__main__':
    import os
    zones = load_zones(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config', 'zones.yaml'))
    p = CommandParser(zones)
    for line in (sys.argv[1:] or sys.stdin):
        c = p.parse(line.strip())
        print(f'{line.strip()!r:55} -> {c.describe(p.zones)}')
