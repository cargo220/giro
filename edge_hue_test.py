# 초점거리 2.1 mm 렌즈의 색 어긋남을 재는 테스트.
# 가운데는 R, G, B가 한자리에 겹치고, 가장자리로 갈수록 짧은 파장인 파랑이 바깥으로 밀린다.
# 색상각으로 이름을 정하지 않는다. camera.py의 각 되돌림도 쓰지 않는다.
# OpenMV IDE에서 실행한다. 블럭을 화면 가운데에 두었다가 가장자리로 옮긴다.
# 노출은 7500 µs로 고정한다.
#
# 한 줄의 뜻:
#   r     광학 중심 (160, 120)에서의 거리
#   edge  프레임 테두리까지 더 가까운 쪽 거리. 0이면 화면 끝
#   BR    파랑 중심이 빨강 중심보다 중심에서 바깥으로 몇 픽셀인지. 양수면 파랑이 더 바깥
#   GR    초록 중심이 빨강 중심보다 바깥으로 몇 픽셀인지
#   dBR   가운데에서 잠근 BR과의 차. 가장자리에서 파랑이 더 날아간 양
#   dGR   가운데에서 잠근 GR과의 차

import sensor
import time
from math import sqrt

GATES = (
    ("파랑", "BLUE", 18, 100, -55, 22, -100, -20),
    ("초록", "GREEN", 18, 100, -100, -12, 8, 100),
    ("빨강", "RED", 18, 100, 14, 100, 0, 80),
    ("보라", "PURPLE", 10, 100, 4, 90, -95, -8),
    ("노랑", "YELLOW", 18, 100, -28, 28, 18, 110),
)

DRAW = {
    "파랑": (40, 80, 255),
    "초록": (0, 255, 80),
    "빨강": (255, 40, 40),
    "보라": (180, 60, 255),
    "노랑": (255, 220, 0),
}

OPT_X = 160.0
OPT_Y = 120.0
GAIN_DB = 24.0
RGB_BASE_DB = (64.4, 60.2, 63.7)
GREEN_GAIN_CUT = 0.2
EXPO_START = 7500
CENTER_LOCK = 40.0
# 이보다 어두운 픽셀은 천이라 중심에 넣지 않는다.
PIX_SUM_MIN = 80

_text_split = True


def blob_value(blob, name):
    value = getattr(blob, name)
    if callable(value):
        return value()
    return value


def draw_text(img, x, y, text):
    global _text_split
    if _text_split:
        try:
            img.draw_string(x, y, text, color=(255, 255, 255), scale=2)
            return
        except TypeError:
            _text_split = False
    img.draw_string((x, y), text, color=(255, 255, 255), scale=2)


def gate_name(code):
    i = 0
    for row in GATES:
        if code & (1 << i):
            return row[0], row[1]
        i += 1
    return None, None


def pixel_rgb(img, x, y):
    pix = img.get_pixel(x, y)
    if isinstance(pix, tuple):
        return pix[0], pix[1], pix[2]
    # RGB565 정수. R은 상위, B는 하위가 아니라 센서가 돌려주는 튜플을 우선한다.
    r = (pix >> 11) & 0x1F
    g = (pix >> 5) & 0x3F
    b = pix & 0x1F
    return r << 3, g << 2, b << 3


def channel_centroids(img, rect):
    # 블럭 안에서 R, G, B 각각의 무게중심. 어두운 천은 빠진다.
    x0, y0, w, h = int(rect[0]), int(rect[1]), int(rect[2]), int(rect[3])
    sxr = syr = sr = 0
    sxg = syg = sg = 0
    sxb = syb = sb = 0
    yy = y0
    while yy < y0 + h:
        xx = x0
        while xx < x0 + w:
            r, g, b = pixel_rgb(img, xx, yy)
            if r + g + b >= PIX_SUM_MIN:
                sxr += xx * r
                syr += yy * r
                sr += r
                sxg += xx * g
                syg += yy * g
                sg += g
                sxb += xx * b
                syb += yy * b
                sb += b
            xx += 1
        yy += 1
    if sr < 1 or sg < 1 or sb < 1:
        return None
    return (sxr / float(sr), syr / float(sr), sxg / float(sg), syg / float(sg), sxb / float(sb), syb / float(sb))


def radial(px, py, ux, uy):
    # 광학 중심에서 블럭 쪽으로 가는 축 위의 위치. 클수록 바깥이다.
    return (px - OPT_X) * ux + (py - OPT_Y) * uy


def edge_gap(cx, cy, iw, ih):
    left = cx
    right = iw - 1 - cx
    top = cy
    bottom = ih - 1 - cy
    gap = left
    if right < gap:
        gap = right
    if top < gap:
        gap = top
    if bottom < gap:
        gap = bottom
    return gap


sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
r_db, g_db, b_db = RGB_BASE_DB
g_db -= GREEN_GAIN_CUT
sensor.set_auto_gain(False, gain_db=GAIN_DB)
sensor.skip_frames(time=200)
try:
    sensor.set_auto_whitebal(False, rgb_gain_db=(r_db, g_db, b_db))
except Exception:
    sensor.set_auto_whitebal(False)
    print("채널 게인을 쓰지 못함")
sensor.skip_frames(time=200)
sensor.set_auto_exposure(False, exposure_us=EXPO_START)
sensor.skip_frames(time=300)

thresholds = [row[2:] for row in GATES]
clock = time.clock()
# 색별 가운데 기준 (BR, GR). 중심 40픽셀 안에서 처음 잡힌 채널 차이다.
locked = {}
last_key = None
print("채널 어긋남 테스트. 노출 %d us" % EXPO_START)
print("r edge BR GR dBR dGR  — 단위는 픽셀. 양수면 그 채널이 빨강보다 바깥")

while True:
    clock.tick()
    img = sensor.snapshot()
    iw = img.width()
    ih = img.height()
    img.draw_cross((int(OPT_X), int(OPT_Y)), color=(80, 80, 80), size=8)
    blobs = img.find_blobs(
        thresholds,
        pixels_threshold=80,
        area_threshold=120,
        merge=False,
        x_stride=2,
        y_stride=2,
    )
    blob = None
    best_px = 0
    if blobs:
        for item in blobs:
            pixels = blob_value(item, "pixels")
            if pixels > best_px:
                best_px = pixels
                blob = item
    if blob is None:
        draw_text(img, 2, 2, "no block")
        continue

    rect = blob_value(blob, "rect")
    cx = blob_value(blob, "cx")
    cy = blob_value(blob, "cy")
    name, label = gate_name(blob_value(blob, "code"))
    if name is None:
        continue
    centers = channel_centroids(img, rect)
    if centers is None:
        draw_text(img, 2, 2, "flat")
        continue
    rx, ry, gx, gy, bx, by = centers
    dx = cx - OPT_X
    dy = cy - OPT_Y
    radius = sqrt(dx * dx + dy * dy)
    if radius < 1.0:
        ux = 1.0
        uy = 0.0
    else:
        ux = dx / radius
        uy = dy / radius
    br = radial(bx, by, ux, uy) - radial(rx, ry, ux, uy)
    gr = radial(gx, gy, ux, uy) - radial(rx, ry, ux, uy)
    gap = edge_gap(cx, cy, iw, ih)

    if name not in locked and radius <= CENTER_LOCK:
        locked[name] = (br, gr)
        print("기준 %s BR%+.1f GR%+.1f" % (name, br, gr))
    base = locked.get(name)
    if base is None:
        dbr = 0.0
        dgr = 0.0
    else:
        dbr = br - base[0]
        dgr = gr - base[1]

    color = DRAW[name]
    img.draw_rectangle(rect, color=color, thickness=2)
    img.draw_cross((int(rx), int(ry)), color=(255, 0, 0), size=3)
    img.draw_cross((int(gx), int(gy)), color=(0, 255, 0), size=3)
    img.draw_cross((int(bx), int(by)), color=(0, 0, 255), size=3)
    draw_text(img, 2, 2, "%s r%.0f e%.0f" % (label, radius, gap))
    draw_text(img, 2, 18, "B%+.1f G%+.1f" % (dbr, dgr))

    bucket = int(radius / 10.0)
    key = (name, bucket)
    if key != last_key:
        print("%s r%.0f edge%.0f BR%+.1f GR%+.1f dBR%+.1f dGR%+.1f" % (name, radius, gap, br, gr, dbr, dgr))
        last_key = key
