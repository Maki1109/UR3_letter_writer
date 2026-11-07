#!/usr/bin/env python3
"""Phần 1+2 – Phân vùng bản đồ Occupancy Grid và gắn nhãn ngữ nghĩa cho từng khu vực.

Các bước (OpenCV, mọi kích thước tính bằng mét rồi đổi sang pixel theo `resolution` của map.yaml):
  1. Threshold: pixel trắng (254) là không gian trống; đen (0) và xám chưa biết (205) đều là vật cản.
  2. Morphology: đóng (close) vật cản bằng kernel 0.25 m để bịt các lỗ nhỏ trên tường, rồi chỉ giữ thành phần
     trống liên thông lớn nhất (vùng robot đi tới được).
  3. Bản đồ cấu trúc: bỏ các khối vật cản rời, nhỏ hơn `furniture_max_area_m2` (hộp, bàn ghế...). Phòng được
     định nghĩa bởi tường; nếu giữ đồ đạc, khe hẹp giữa hộp và tường cũng bị coi là "cửa" và cắt phòng sai chỗ.
     Distance transform trên bản đồ cấu trúc: mỗi pixel mang khoảng cách tới tường gần nhất (độ "thoáng").
  4. Hạt giống: hạ dần ngưỡng t từ độ thoáng lớn nhất xuống `seed_min_clearance_m`. Ở mỗi mức, một thành phần
     liên thông của {dist > t} chưa chứa hạt nào và đủ rộng (`seed_min_area_m2`) trở thành hạt mới. Mỗi phòng
     cho một lõi thoáng riêng; các lõi chỉ nối với nhau qua cửa/khe hẹp nên tách ra trước khi hợp lại.
  5. Watershed (ngập nước theo độ thoáng, hàng đợi ưu tiên): từ các hạt, lan dần sang pixel thoáng hơn trước.
     Hai vùng gặp nhau ở chỗ hẹp nhất – tức là ở cửa/khe giữa các vách. Cuối cùng cắt bỏ phần đồ đạc.
  6. Gộp vùng nhỏ hơn `min_zone_area_m2`, đặt chữ A, B, C... theo thứ tự trái -> phải, rồi gắn tên theo `labels`.
  7. Điểm đích hợp lệ: trong vùng, chỉ xét pixel cách vật cản >= `goal.min_clearance_m` và ngoài vùng `avoid`,
     chọn pixel gần tâm vùng nhất (độ thoáng lúc này tính trên bản đồ thật, có cả đồ đạc).

Tọa độ pixel (cột u, hàng v) -> map: x = origin_x + (u + 0.5) * res, y = origin_y + (H - v - 0.5) * res
(ảnh .pgm có hàng 0 ở trên, trục y của map hướng lên).

    python3 tools/segment_map.py [config/segmentation.yaml] [-o config/zones.yaml] [--show]
"""
import argparse
import heapq
import os
import sys

import cv2
import numpy as np
import yaml
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
B7_DIR = os.path.dirname(HERE)
FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'   # có đủ dấu tiếng Việt (cv2.putText thì không)
COLORS = [(231, 76, 60), (46, 204, 113), (52, 152, 219), (241, 196, 15), (155, 89, 182), (26, 188, 156),
          (230, 126, 34), (149, 165, 166)]                  # RGB


class MapGeometry:
    """Đổi qua lại giữa pixel ảnh bản đồ và tọa độ frame map."""

    def __init__(self, yaml_path):
        with open(yaml_path) as f:
            meta = yaml.safe_load(f)
        img_path = os.path.join(os.path.dirname(yaml_path), meta['image'])
        self.image = cv2.imread(img_path, cv2.IMREAD_UNCHANGED)
        if self.image is None:
            raise FileNotFoundError(img_path)
        if meta.get('negate', 0):
            self.image = 255 - self.image
        self.res = float(meta['resolution'])
        self.ox, self.oy = float(meta['origin'][0]), float(meta['origin'][1])
        self.h, self.w = self.image.shape[:2]

    def to_world(self, u, v):
        return float(self.ox + (u + 0.5) * self.res), float(self.oy + (self.h - v - 0.5) * self.res)

    def to_pixel(self, x, y):
        return int((x - self.ox) / self.res), int(self.h - (y - self.oy) / self.res)

    def px(self, meters):
        return max(1, int(round(meters / self.res)))


def free_space(geo, close_m):
    obstacle = (geo.image < 250).astype(np.uint8)              # 0 = occupied, 205 = unknown
    k = geo.px(close_m) | 1
    obstacle = cv2.morphologyEx(obstacle, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (k, k)))
    free = 1 - obstacle
    n, lab, stats, _ = cv2.connectedComponentsWithStats(free, connectivity=4)
    if n < 2:
        raise RuntimeError('bản đồ không có vùng trống')
    biggest = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    return (lab == biggest).astype(np.uint8)


def remove_furniture(free, max_area_px):
    """Coi các khối vật cản rời nhỏ (không nối với tường) là trống, chỉ để tìm cấu trúc phòng."""
    obstacle = 1 - free
    n, lab, stats, _ = cv2.connectedComponentsWithStats(obstacle, connectivity=8)
    struct = free.copy()
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] < max_area_px:
            struct[lab == i] = 1
    return struct


def find_seeds(dist, res, min_clear_m, min_area_m2):
    """Hạt giống = các lõi thoáng tách rời nhau khi hạ dần ngưỡng độ thoáng (giống h-maxima có điều kiện diện tích)."""
    seeds = np.zeros(dist.shape, np.int32)
    min_area = min_area_m2 / (res * res)
    n_seed = 0
    for t in np.arange(dist.max(), min_clear_m / res, -0.5):     # bước 0.5 px = 2.5 cm
        n, lab, stats, _ = cv2.connectedComponentsWithStats((dist > t).astype(np.uint8), connectivity=8)
        for i in range(1, n):
            comp = lab == i
            if stats[i, cv2.CC_STAT_AREA] >= min_area and not seeds[comp].any():
                n_seed += 1
                seeds[comp] = n_seed
    return seeds


def watershed_flood(free, dist, seeds):
    """Watershed bằng hàng đợi ưu tiên: pixel thoáng hơn được ngập trước, nên biên vùng rơi vào chỗ hẹp nhất.
    (cv2.watershed lan theo gradient ảnh màu chứ không theo độ cao, nên không dùng trực tiếp được ở đây.)"""
    lab = seeds.copy()
    h, w = lab.shape
    heap = [(-dist[v, u], v, u) for v, u in zip(*np.nonzero(lab))]
    heapq.heapify(heap)
    while heap:
        _, v, u = heapq.heappop(heap)
        for dv, du in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            vv, uu = v + dv, u + du
            if 0 <= vv < h and 0 <= uu < w and free[vv, uu] and not lab[vv, uu]:
                lab[vv, uu] = lab[v, u]
                heapq.heappush(heap, (-dist[vv, uu], vv, uu))
    return lab


def merge_small(lab, min_area):
    """Gộp vùng nhỏ vào vùng kề có biên chung dài nhất."""
    kernel = np.ones((3, 3), np.uint8)
    while True:
        ids, counts = np.unique(lab[lab > 0], return_counts=True)
        small = [(c, i) for i, c in zip(ids, counts) if c < min_area]
        if not small or len(ids) == 1:
            return lab
        _, i = min(small)
        ring = cv2.dilate((lab == i).astype(np.uint8), kernel).astype(bool) & (lab > 0) & (lab != i)
        if not ring.any():
            lab[lab == i] = 0
            continue
        nb, cnt = np.unique(lab[ring], return_counts=True)
        lab[lab == i] = nb[np.argmax(cnt)]


def find_doors(geo, lab, dist, zones):
    """Cửa giữa hai khu vực kề nhau = pixel trên biên chung có độ thoáng thật lớn nhất (giữa khe/cửa)."""
    ids = {z['label']: z['id'] for z in zones}
    best = {}
    h, w = lab.shape
    for dv, du in ((0, 1), (1, 0)):
        a = lab[:h - dv, :w - du]
        b = lab[dv:, du:]
        vs, us = np.nonzero((a > 0) & (b > 0) & (a != b))
        for v, u in zip(vs, us):
            key = tuple(sorted((ids[a[v, u]], ids[b[v, u]])))
            d = float(dist[v, u])
            if d > best.get(key, (-1,))[0]:
                best[key] = (d, u, v)
    doors = []
    for key, (d, u, v) in sorted(best.items()):
        x, y = geo.to_world(u, v)
        doors.append({'zones': list(key), 'point': [round(x, 2), round(y, 2)], 'clearance_m': round(d * geo.res, 2)})
    return doors


def pick_goal(geo, mask, dist, min_clear_m, avoid):
    """Pixel trong vùng, đủ thoáng, ngoài vùng cấm, gần tâm vùng nhất. Trả về (u, v) hoặc None."""
    vs, us = np.nonzero(mask)
    cu, cv = us.mean(), vs.mean()
    ok = mask & (dist * geo.res >= min_clear_m)
    for x, y, r in avoid:
        au, av = geo.to_pixel(x, y)
        yy, xx = np.ogrid[:geo.h, :geo.w]
        ok &= (xx - au) ** 2 + (yy - av) ** 2 > (r / geo.res) ** 2
    if not ok.any():       # vùng quá hẹp: lấy điểm thoáng nhất
        ok = mask & (dist == dist[mask].max())
    vs, us = np.nonzero(ok)
    k = int(np.argmin((us - cu) ** 2 + (vs - cv) ** 2))
    return (cu, cv), (int(us[k]), int(vs[k]))


def segment(cfg_path):
    with open(cfg_path) as f:
        cfg = yaml.safe_load(f)
    map_yaml = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(cfg_path)), cfg['map']))
    geo = MapGeometry(map_yaml)
    sc, gc = cfg['segmentation'], cfg['goal']

    free = free_space(geo, sc['close_kernel_m'])
    struct = remove_furniture(free, sc['furniture_max_area_m2'] / geo.res ** 2)
    sdist = cv2.distanceTransform(struct, cv2.DIST_L2, 5)
    seeds = find_seeds(sdist, geo.res, sc['seed_min_clearance_m'], sc['seed_min_area_m2'])
    lab = watershed_flood(struct, sdist, seeds)
    lab[free == 0] = 0                                       # trả đồ đạc về là vật cản
    lab = merge_small(lab, sc['min_zone_area_m2'] / geo.res ** 2)
    dist = cv2.distanceTransform(free, cv2.DIST_L2, 5)       # độ thoáng thật, dùng chọn điểm đích

    # đánh số lại theo tâm vùng: trái -> phải, rồi trên -> dưới
    ids = [i for i in np.unique(lab) if i > 0]
    cents = {i: np.argwhere(lab == i).mean(axis=0) for i in ids}          # (v, u)
    order = sorted(ids, key=lambda i: (round(cents[i][1] / geo.px(1.0)), cents[i][0]))
    relab = np.zeros_like(lab)
    for k, i in enumerate(order, 1):
        relab[lab == i] = k

    zones = []
    labels = cfg.get('labels', [])
    for k in range(1, len(order) + 1):
        mask = relab == k
        zone = {'id': None, 'name': None, 'aliases': []}
        for lb in labels:
            u, v = geo.to_pixel(*lb['hint'])
            if 0 <= v < geo.h and 0 <= u < geo.w and relab[v, u] == k:
                zone.update(id=str(lb['id']), name=lb['name'], aliases=list(lb.get('aliases', [])))
        (cu, cv), (gu, gv) = pick_goal(geo, mask, dist, gc['min_clearance_m'], gc.get('avoid', []))
        cnts, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnt = cv2.approxPolyDP(max(cnts, key=cv2.contourArea), 1.0, True)[:, 0, :]
        gx, gy = geo.to_world(gu, gv)
        zone.update(
            label=k,
            area_m2=round(float(mask.sum()) * geo.res ** 2, 2),
            center=[round(c, 2) for c in geo.to_world(cu, cv)],
            goal=[round(gx, 2), round(gy, 2)],
            goal_clearance_m=round(float(dist[gv, gu]) * geo.res, 2),
            polygon=[[round(c, 3) for c in geo.to_world(u, v)] for u, v in cnt])
        zones.append(zone)
    # vùng không có nhãn: chữ cái kế tiếp chưa bị nhãn nào dùng
    letters = (chr(c) for c in range(ord('A'), ord('Z') + 1) if chr(c) not in [z['id'] for z in zones])
    for z in zones:
        if z['id'] is None:
            z['id'] = next(letters)
            z['name'] = f"Khu vực {z['id']}"
    zones.sort(key=lambda z: z['id'])
    doors = find_doors(geo, relab, dist, zones)
    missing = [lb['id'] for lb in labels if lb['id'] not in [z['id'] for z in zones]]
    return geo, map_yaml, struct, sdist, seeds, relab, zones, doors, missing


def draw(geo, free, dist, seeds, lab, zones, doors, path, scale=6):
    """figures/zones.png: bản đồ gốc phủ màu các vùng, viền, tên, tâm (o) và điểm đích (x) + hình các bước."""
    base = cv2.cvtColor(geo.image, cv2.COLOR_GRAY2RGB)
    over = base.copy()
    for z in zones:
        over[lab == z['label']] = COLORS[(z['label'] - 1) % len(COLORS)]
    over = cv2.addWeighted(base, 0.45, over, 0.55, 0)
    over[geo.image == 0] = 0
    big = cv2.resize(over, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
    # biên giữa các vùng
    edge = cv2.morphologyEx(lab.astype(np.uint8), cv2.MORPH_GRADIENT, np.ones((3, 3), np.uint8)) > 0
    edge = cv2.resize((edge & (lab > 0)).astype(np.uint8), None, fx=scale, fy=scale,
                      interpolation=cv2.INTER_NEAREST).astype(bool)
    big[edge] = (40, 40, 40)
    img = Image.new('RGB', (big.shape[1], big.shape[0] + 28), (255, 255, 255))
    img.paste(Image.fromarray(big), (0, 0))
    d = ImageDraw.Draw(img)
    f_big, f_small = ImageFont.truetype(FONT, 22), ImageFont.truetype(FONT, 14)

    def to_img(x, y):
        u = (x - geo.ox) / geo.res
        v = geo.h - (y - geo.oy) / geo.res
        return u * scale, v * scale

    for dr in doors:
        x, y = to_img(*dr['point'])
        d.rectangle([x - 6, y - 6, x + 6, y + 6], fill=(255, 255, 255), outline=(0, 0, 0), width=2)
    for z in zones:
        cx, cy = to_img(*z['center'])
        gx, gy = to_img(*z['goal'])
        d.ellipse([cx - 5, cy - 5, cx + 5, cy + 5], outline=(0, 0, 0), width=2)
        d.line([gx - 8, gy - 8, gx + 8, gy + 8], fill=(200, 0, 0), width=4)
        d.line([gx - 8, gy + 8, gx + 8, gy - 8], fill=(200, 0, 0), width=4)
        title = f"{z['id']}: {z['name']}"
        sub = f"đích ({z['goal'][0]:.2f}, {z['goal'][1]:.2f})  {z['area_m2']:.1f} m²"
        tw = max(d.textlength(title, font=f_big), d.textlength(sub, font=f_small))
        tx, ty = min(max(cx - tw / 2, 4), img.width - tw - 12), max(cy, gy) + 14
        d.rectangle([tx - 4, ty - 2, tx + tw + 4, ty + 46], fill=(255, 255, 255), outline=(0, 0, 0))
        d.text((tx, ty), title, font=f_big, fill=(0, 0, 0))
        d.text((tx, ty + 27), sub, font=f_small, fill=(60, 60, 60))
    legend = 'o tâm vùng   x điểm đích Nav2   □ cửa giữa hai vùng   (frame map, m)'
    d.text((8, img.height - 22), legend, font=f_small, fill=(0, 0, 0))
    img.save(path)

    # hình các bước: threshold+morphology | distance transform | hạt giống | watershed
    steps = [cv2.cvtColor(free * 255, cv2.COLOR_GRAY2RGB),
             cv2.applyColorMap((255 * dist / dist.max()).astype(np.uint8), cv2.COLORMAP_JET)[:, :, ::-1],
             cv2.cvtColor(free * 90, cv2.COLOR_GRAY2RGB), over]
    for i in range(1, seeds.max() + 1):
        steps[2][seeds == i] = COLORS[(i - 1) % len(COLORS)]
    steps[1][free == 0] = 0
    names = ['1. Threshold + morphology, bỏ đồ đạc', '2. Distance transform', '3. Hạt giống (lõi thoáng)', '4. Watershed + nhãn']
    tiles = []
    for im, name in zip(steps, names):
        t = Image.fromarray(cv2.resize(im, None, fx=3, fy=3, interpolation=cv2.INTER_NEAREST))
        canvas = Image.new('RGB', (t.width, t.height + 26), (255, 255, 255))
        canvas.paste(t, (0, 26))
        ImageDraw.Draw(canvas).text((4, 3), name, font=f_small, fill=(0, 0, 0))
        tiles.append(canvas)
    sheet = Image.new('RGB', (tiles[0].width * 2 + 10, tiles[0].height * 2 + 10), (255, 255, 255))
    for i, t in enumerate(tiles):
        sheet.paste(t, ((i % 2) * (t.width + 10), (i // 2) * (t.height + 10)))
    sheet.save(path.replace('.png', '_steps.png'))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('config', nargs='?', default=os.path.join(B7_DIR, 'config', 'segmentation.yaml'))
    ap.add_argument('-o', '--output', default=os.path.join(B7_DIR, 'config', 'zones.yaml'))
    ap.add_argument('--figure', default=os.path.join(B7_DIR, 'figures', 'zones.png'))
    ap.add_argument('--show', action='store_true', help='mở cửa sổ xem hình kết quả')
    a = ap.parse_args()

    geo, map_yaml, struct, sdist, seeds, lab, zones, doors, missing = segment(a.config)
    out = {'frame_id': 'map', 'map': os.path.relpath(map_yaml, os.path.dirname(os.path.abspath(a.output))),
           'zones': [{k: z[k] for k in ('id', 'name', 'aliases', 'center', 'goal', 'goal_clearance_m',
                                        'area_m2', 'polygon')} for z in zones],
           'doors': doors}
    with open(a.output, 'w') as f:
        f.write('# Sinh tự động bởi tools/segment_map.py từ config/segmentation.yaml – đừng sửa tay, hãy chạy lại tool.\n')
        f.write('# center: tâm hình học của vùng; goal: điểm đích hợp lệ gửi cho Nav2 (frame map, m);\n')
        f.write('# polygon: đường bao vùng, dùng để biết robot đang ở khu vực nào và vẽ trên RViz;\n')
        f.write('# doors: điểm thoáng nhất trên biên chung của hai vùng kề nhau – robot đi qua đó khi chuyển vùng.\n')
        yaml.safe_dump(out, f, allow_unicode=True, sort_keys=False, default_flow_style=None, width=120)
    os.makedirs(os.path.dirname(a.figure), exist_ok=True)
    draw(geo, struct, sdist, seeds, lab, zones, doors, a.figure)

    print(f'Bản đồ {map_yaml}: {geo.w}x{geo.h} px, {geo.res} m/px; {seeds.max()} hạt giống -> {len(zones)} khu vực')
    for z in zones:
        print(f"  {z['id']}  {z['name']:<18} {z['area_m2']:5.1f} m²  tâm ({z['center'][0]:5.2f}, {z['center'][1]:5.2f})"
              f"  đích ({z['goal'][0]:5.2f}, {z['goal'][1]:5.2f}), cách vật cản {z['goal_clearance_m']:.2f} m")
    if missing:
        print(f'  CẢNH BÁO: nhãn {missing} không rơi vào khu vực nào (hint nằm ngoài vùng trống?)', file=sys.stderr)
    for dr in doors:
        print(f"  cửa {dr['zones'][0]}–{dr['zones'][1]}  ({dr['point'][0]:5.2f}, {dr['point'][1]:5.2f}), "
              f"cách vật cản {dr['clearance_m']:.2f} m")
    print(f'-> {a.output}\n-> {a.figure}, {a.figure.replace(".png", "_steps.png")}')
    if a.show:
        cv2.imshow('Map segmentation', cv2.imread(a.figure))
        cv2.waitKey(0)


if __name__ == '__main__':
    main()
