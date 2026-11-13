#!/usr/bin/env python3
"""Kiểm thử không cần ROS/camera: quỹ đạo của từng biểu cảm (mô phỏng động học vi sai) và bộ làm mượt nhãn.

    /usr/bin/python3 tests/test_motion_plans.py        (hoặc qua env.sh để có numpy<2)
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'nodes'))
from motion_plans import default_actions, Executor, GoHome, wrap  # noqa: E402
from emotion_core import EMOTIONS, EmotionSmoother, _normalize  # noqa: E402


def simulate(action, dt=0.05, use_odom=True):
    """Tích phân động học robot vi sai; trả về (x, y, yaw) cuối, quãng đường, thời gian."""
    x = y = th = 0.0
    ex = Executor(action, (x, y, th) if use_odom else None)
    t = 0.0
    while True:
        v, w, done = ex.step(dt, (x, y, th) if use_odom else None)
        if done:
            return x, y, th, ex.dist, t
        x += v * math.cos(th) * dt
        y += v * math.sin(th) * dt
        th = wrap(th + w * dt)
        t += dt


fails = 0


def check(name, cond, info=''):
    global fails
    fails += not cond
    print(f'  [{"OK " if cond else "SAI"}] {name} {info}')


A = default_actions()
print('Quỹ đạo (mô phỏng động học, dt = 50 ms):')
for e in EMOTIONS:
    x, y, th, d, t = simulate(A[e])
    print(f'  {e:9s} -> {A[e].text:16s} x={x:+.2f} y={y:+.2f} yaw={math.degrees(th):+7.1f}° t={t:4.1f}s')

x, y, th, *_ = simulate(A['happy'])
check('happy: cong sang trái', x > 0.2 and y > 0.1 and th > 0.5, f'(y={y:+.2f}, yaw={math.degrees(th):+.0f}°)')
x2, y2, th2, *_ = simulate(A['sad'])
check('sad: đối xứng happy qua trục x', abs(x2 - x) < 1e-6 and abs(y2 + y) < 1e-6 and abs(th2 + th) < 1e-6)
R = A['happy'].v / A['happy'].w
check('bán kính cung', abs(math.hypot(x, y - R) - R) < 0.01, f'R = {R:.2f} m')
x, y, th, *_ = simulate(A['neutral'])
check('neutral: đứng yên', x == y == th == 0.0)
x, y, th, d, _ = simulate(A['angry'])
check('angry: lùi ~0.2 m, không xoay', abs(x + 0.20) < 0.02 and abs(y) < 1e-9 and th == 0.0, f'(x={x:+.3f})')
x, y, th, d, _ = simulate(A['fear'])
check('fear: tiến ~0.2 m', abs(x - 0.20) < 0.02 and abs(y) < 1e-9, f'(x={x:+.3f})')
ex = Executor(A['surprise'], (0, 0, 0))
x = y = th = 0.0
while not ex.done:
    v, w, _ = ex.step(0.05, (x, y, th))
    th = wrap(th + w * 0.05)
check('surprise: xoay đúng 1 vòng', abs(ex.angle - 2 * math.pi) < math.radians(4) and abs(th) < math.radians(4),
      f'(tổng {math.degrees(ex.angle):.1f}°)')
x, y, th, d, _ = simulate(A['angry'], use_odom=False)
check('không có odom: quy ra thời gian', abs(x + 0.20) < 0.03, f'(x={x:+.3f})')

print('Về chỗ cũ:')
for start in [(1.2, 0.6, 2.0), (-0.8, -0.5, -1.0), (0.3, 0.0, 3.0)]:
    x, y, th = start
    gh, t = GoHome((0.0, 0.0, 0.0)), 0.0
    while not gh.done and t < 60:
        v, w, _ = gh.step((x, y, th))
        x += v * math.cos(th) * 0.05; y += v * math.sin(th) * 0.05; th = wrap(th + w * 0.05); t += 0.05
    check(f'từ {start}', gh.done and math.hypot(x, y) < 0.08 and abs(th) < 0.08,
          f'(sai số {math.hypot(x, y):.2f} m, {math.degrees(th):+.1f}°, {t:.0f}s)')

print('Bộ làm mượt:')
one = lambda e, p=0.8: _normalize({k: (p if k == e else (1 - p) / 5) for k in EMOTIONS})
s = EmotionSmoother(window=8)
check('ban đầu: none', s.label == 'none')
for _ in range(8):
    s.update(one('happy'))
check('8 frame happy -> happy', s.label == 'happy')
s.update(one('angry')); s.update(one('angry'))
check('2 frame nhiễu angry -> vẫn happy', s.label == 'happy')
for _ in range(8):
    s.update(one('angry'))
check('angry kéo dài -> angry', s.label == 'angry')
for _ in range(5):
    s.update(None)
check('mất mặt > nửa cửa sổ -> none', s.label == 'none')
s.reset()
for _ in range(6):
    s.update(_normalize({e: 1 for e in EMOTIONS}))
check('điểm đều nhau (không rõ) -> neutral (dừng)', s.label == 'neutral')
d = _normalize({'happy': 50, 'disgust': 50})
check('bỏ disgust, chuẩn hoá về 6 lớp', abs(sum(d.values()) - 1) < 1e-9 and d['happy'] == 1.0)

print('KẾT QUẢ:', 'tất cả đạt' if not fails else f'{fails} lỗi')
sys.exit(1 if fails else 0)
