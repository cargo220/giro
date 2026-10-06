# OpenMV IDE에서 이 파일을 열고 실행한다.
# 프레임 화면과 USB 시리얼은 IDE가 이미 붙여 둔다.
# snapshot()은 IDE 화면으로, print()는 IDE 터미널로 간다.
# PC에서 이 파일을 PC 파이썬으로 돌리지 않는다.
# 로봇으로 결과를 넘기는 핀 통신은 2차에서 따로 둔다.
#
# 대상 보드는 OpenMV Cam H7이다. 연습은 RT1062에서 해도
# RGB565 320×240 안에서만 돈다. H7 내장 RAM 1 MB가 이 해상도의 한도다.
#
# 한 프레임의 순서:
# 1. LAB 상자로 덩어리를 찾는다. 상자는 자리만 찾고, 다른 색끼리는 합치지 않는다.
# 2. 한 덩어리 안에서 이름이 여럿이면 각 색의 칸을 감싼다. 이름이 같으면 한 면이다.
#    사이가 비어 밀도가 낮으면 넓은 파랑 칸만 남긴다.
# 3. 면 가운데의 각도로 이름을 붙인다. 가장자리면 각을 되돌린다.
#    테두리는 그 각의 부채꼴만 다시 찾아 회전 꼭짓점으로 그린다.
# 4. 보라 면 위에 절반 넘게 겹친 파랑만 버린다. 파란 면 위의 보라도 버린다.
#    보라 상자가 빠져도 직전 자리를 다시 재고, 아니면 최대 여섯 프레임 남긴다.
# 5. 첫 면으로 노출 창을 한 번 고른 뒤, 면 밝기만 보고 노출을 움직인다.

import sensor
import time
from math import atan2, sqrt, cos, sin

# 검출용 LAB 상자. 이름을 정하는 기준이 아니다.
# L의 아래를 막아 둔다. 0부터 열면 긴 노출의 검은 바닥이 파란 잡음으로 상자에 들어온다.
# 파랑은 B를 -20보다 푸르게 두어 바닥을 빼고, A 상한은 22까지 연다.
# 이 카메라에서 파란 블럭 A가 6을 넘나들면 상자 밖으로 나가 이름이 빠졌고,
# 그때 보라 상자만 남아 파랑이 보라로 바뀌었다.
# (이름, 화면 글자, Lmin, Lmax, Amin, Amax, Bmin, Bmax)
GATES = (
    ("파랑", "BLUE", 18, 100, -55, 22, -100, -20),
    ("초록", "GREEN", 18, 100, -100, -12, 8, 100),
    ("빨강", "RED", 18, 100, 14, 100, 0, 80),
    ("보라", "PURPLE", 10, 100, 4, 90, -95, -8),
    ("노랑", "YELLOW", 18, 100, -28, 28, 18, 110),
)

# 각 색의 기준 각도(도). atan2(B, A). 0°는 +A(빨강), 90°는 +B(노랑).
# 예전 상자 중심에서 온 값이다. 밝기가 바뀌면 A·B의 길이가 변하지, 이 각이 변하지 않는다.
# 이름은 측정 각과 여기 다섯 각의 원형 거리가 가장 짧은 색이다.
REFERENCES = (
    ("파랑", "BLUE", -96.0),
    ("보라", "PURPLE", -43.0),
    ("빨강", "RED", 33.0),
    ("노랑", "YELLOW", 90.0),
    ("초록", "GREEN", 142.0),
)

# 화면 글자색. 프레임 버퍼에는 한글이 없어 영문 이름을 이 색으로 그린다.
DRAW = {
    "파랑": (40, 80, 255),
    "초록": (0, 255, 80),
    "빨강": (255, 40, 40),
    "보라": (180, 60, 255),
    "노랑": (255, 220, 0),
}

# 원형 거리가 이 값보다 크면 다섯 색의 사이(청록 가장자리 등)로 보고 버린다.
MAX_HUE_DEG = 50.0
# 반지름 16은 노출이 길어진 검은 천도 통과해 사각형이 됐다.
# 블럭 면은 이보다 진하다. 5000 µs 보라의 유효 반지름은 21이다.
CHROMA_MIN = 20
# 이보다 어두운 면은 바닥이다. 이름은 밝기로 정하지 않고, 바닥만 거른다.
# 22면 노출이 내려간 보라 면(L 20 근처)이 이름에서 빠졌다.
MIN_FACE_L = 16
# 이보다 짧은 변은 천의 잡음 덩어리다. 블럭 면은 더 길다.
MIN_SIDE = 20
# 외접 상자 대비 채운 비율. 바닥 잡음이 뭉친 상자는 속이 비어 있다.
MIN_DENSITY = 0.4
# 테두리를 다시 맞출 때의 각 반폭과 반지름 비율.
# 같은 페인트는 각이 남고 반지름만 밝기에 따라 변한다.
# 파랑과 보라는 각이 가까워 반폭을 줄이고, 반지름도 그 면 근처에만 연다.
TIGHT_HALF_DEG = 16.0
TIGHT_HALF_NEAR = 8.0
TIGHT_C0 = 0.45
TIGHT_C1 = 1.35
TIGHT_C0_NEAR = 0.82
TIGHT_C1_NEAR = 1.20
# 파란 블럭은 밝을 때 -89°, 어두우면 -74°이고 반지름은 42보다 크다.
# 보라 블럭은 -69°~-61°이고 반지름은 그 아래다.
# -76° 이하는 파랑이다. -68° 이상은 보리다.
# 그 사이는 반지름이 42 이상일 때만 파랑이다. 보라의 어두운 면이 여기 걸린다.
# 화면 끝에서 5°를 되돌리면 보라가 -75°까지 내려온다. -75°를 파랑으로 두면 그 자리가 파랑이 된다.
BLUE_MAX_ANGLE = -76.0
PURPLE_MIN_ANGLE = -68.0
BLUE_MIN_CHROMA = 42.0
# 면의 L 상위 사분위가 여기 이상이면 하이라이트가 잘리고 각도가 돌아간다.
CLIP_L = 92
# 이름으로 받은 면이 전부 이보다 어두우면 노출을 늘린다.
DARK_L = 28
# 보라 면이 이보다 어두우면, 다른 면이 잘리지 않은 한 노출을 올린다.
# 노란 면이 같이 밝아도 보라는 L 20 근처에서 이름이 빠졌다.
PURPLE_L_AIM = 36
# 창의 폭은 5000 µs. 7500에서 본 첫 면으로 둘 중 하나를 고르고 다시 옮기지 않는다.
# 잘림만 있으면 조명이 밝아 낮은 창. 그 밖은 높은 창.
# 10000 µs를 넘기면 바닥 잡음이 파란 블럭이 되었다.
EXPO_LOW_MIN = 2500
EXPO_LOW_MAX = 7500
EXPO_HIGH_MIN = 5000
EXPO_HIGH_MAX = 10000
EXPO_MIN = EXPO_HIGH_MIN
EXPO_MAX = EXPO_HIGH_MAX
# 켜질 때의 노출. 낮은 창의 상한이자 높은 창의 한가운데다.
EXPO_START = 7500
# 한 번에 움직이는 노출 시간. 배율로 곱하지 않는다.
EXPO_STEP = 150
EXPO_UP = 300
# 켜질 때 장면 밝기로 게인을 고르지 않는다.
# 어두운 블럭을 보고 잠긴 값이 23.8 dB였고, 그때 노출은 8000 µs에서 아래로 내려갔다.
GAIN_DB = 24.0
# OV5640 초기 레지스터 0x3400. dB = 20*log10(레지스터).
# 빨강 0x0680 → 64.4, 초록 0x0400 → 60.2, 파랑 0x0600 → 63.7.
# 0 dB는 레지스터 1이라 화면이 검게 나간다.
RGB_BASE_DB = (64.4, 60.2, 63.7)
# 위 기본에서 초록만 이만큼 낮춘다. 장면이 옮긴 자동 비율에서 빼지 않는다.
GREEN_GAIN_CUT = 0.2
EXPO_SETTLE = 5
# QVGA 광학 중심. 색수차는 중심에서 멀수록 커진다.
OPT_X = 160.0
OPT_Y = 120.0
RAD2DEG = 57.2957795
# 화면 가로 끝(중심에서 160픽셀)에서 측정각을 되돌리는 양.
# 파란 채널이 바깥으로 밀려 면 안쪽의 각이 보라 쪽으로 커진다.
# 6°를 넘기면 모서리의 흐린 보라(-69°, 반지름 35 이하)가 파랑이 된다.
EDGE_RADIUS = 160.0
EDGE_HUE_PULL = 5.0

# 새 프레임 비중. 높을수록 블럭을 빨리 따라가고, 나머지는 한 프레임 떨림을 누른다.
FOLLOW = 0.8
# 붙은 두 블럭. 같은 페인트는 밝기가 달라도 각이 유지된다.
# 다른 페인트가 한 덩어리에 있으면 긴 변을 따라 각이 한 곳에서 뛴다.
# 12°보다 작은 차이는 색수차와 그림자라 새로 자르지 않는다.
# 이미 자른 경계는 8° 아래로 떨어지기 전에는 유지한다. 한 프레임의 흔들림으로 다시 붙지 않는다.
# 이름이 같으면 한 면이다. 맞닿은 가장자리가 기울어도 그 사각형은 유지한다.
# 한 덩어리에 이름이 여럿이면 각 색이 있는 칸을 감싸 상자를 나눈다.
# 가로 또는 세로가 한 칸인 색은 맞닿아 스민 가장자리라 떼지 않는다.
# ㄱ자의 가장자리가 다른 색이면 그 줄만 뺀다. 속을 빈칸 없는 직사각형으로 줄이지 않는다.
# 중간에 걸린 칸은 이웃 색이 12° 더 가까울 때만 그 색으로 넘긴다.
# 같은 색 두 블럭은 이름이 같아서 남지 않는다.
SPLIT_HUE = 12.0
SPLIT_HOLD = 8.0
SPLIT_CORNER = 24.0
SPLIT_SIDE = 10.0
SPLIT_MIN = 32
SPLIT_PART = 14
SPLIT_MISS_MAX = 3
SPLIT_FOLLOW = 0.6
SPLIT_MATCH = 36


def read_stat(stats, name):
    # H7 펌웨어는 속성, 일부 새 펌웨어는 메서드로 같은 값을 준다.
    value = getattr(stats, name)
    if callable(value):
        return value()
    return value


def hue_deg(a, b):
    # A, B를 색상 각도(도)로 바꾼다. 0°는 빨강, 90°는 노랑, -90°는 파랑이다.
    return atan2(b, a) * RAD2DEG


def circ_abs(delta):
    # 각도 차를 -180~180으로 접은 뒤 거리만 남긴다.
    if delta > 180.0:
        delta -= 360.0
    elif delta < -180.0:
        delta += 360.0
    if delta < 0.0:
        return -delta
    return delta


def covers(rect, cx, cy, pad):
    # 점이 사각형을 pad만큼 키운 안에 있는지. 같은 블럭의 두 덩어리를 가릴 때 쓴다.
    x, y, w, h = int(rect[0]), int(rect[1]), int(rect[2]), int(rect[3])
    return (x - pad) <= cx <= (x + w + pad) and (y - pad) <= cy <= (y + h + pad)


def overlap_frac(inner, outer):
    # inner 넓이 가운데 outer 안에 들어가는 비율. 맞닿은 가장자리는 작다.
    x, y, w, h = inner
    ox, oy, ow, oh = outer
    ix0 = x if x > ox else ox
    iy0 = y if y > oy else oy
    ix1 = x + w
    if ox + ow < ix1:
        ix1 = ox + ow
    iy1 = y + h
    if oy + oh < iy1:
        iy1 = oy + oh
    if ix1 <= ix0 or iy1 <= iy0 or w < 1 or h < 1:
        return 0.0
    return ((ix1 - ix0) * (iy1 - iy0)) / float(w * h)


PURPLE_CODE = 1 << 3  # GATES에서 보라가 네 번째 상자다. find_blobs 코드 비트.
PURPLE_MISS_MAX = 6  # 검출이 빠져도 면이 보라이면 이 프레임 수만큼 이름을 유지한다.


def draw_face(img, name, label, quad, cx, cy):
    # 면의 테두리만 그린다. 십자와 영문 이름은 프레임에 남지 않는다.
    img.draw_edges(quad, color=DRAW[name], thickness=2)


def blob_value(blob, name):
    # 펌웨어에 따라 blob 값이 속성일 수도, 메서드일 수도 있다. 둘 다 읽는다.
    value = getattr(blob, name)
    if callable(value):
        return value()
    return value


def classify_angle(a, b, l_med, purple_soft=False):
    # 면의 색 이름을 정한다. 상자에 들어간 것만으로는 이름이 아니다.
    # purple_soft는 이미 그린 보라의 직전 자리를 다시 잴 때만 쓴다.
    # 새 덩어리에 쓰면 천의 옅은 보라 잡음이 사각형이 된다.
    # 바닥은 반지름이 작거나 L이 낮다. 밝기가 달라져도 이름은 각도로 정한다.
    min_l = 12 if purple_soft else MIN_FACE_L
    min_c = 16 if purple_soft else CHROMA_MIN
    if l_med < min_l:
        return None
    if a * a + b * b < min_c * min_c:
        return None
    angle = hue_deg(a, b)
    chroma = sqrt(a * a + b * b)
    # 파랑과 보라는 둘 다 B가 음수다. 어두운 파랑의 각이 보라 쪽으로 다가온다.
    if b < -8 and -105.0 < angle < -15.0:
        if angle <= BLUE_MAX_ANGLE or (
            angle < PURPLE_MIN_ANGLE and chroma >= BLUE_MIN_CHROMA
        ):
            return ("파랑", "BLUE", angle, circ_abs(angle + 96.0))
        return ("보라", "PURPLE", angle, circ_abs(angle + 43.0))
    best = None
    best_d = None
    for row in REFERENCES:
        dist = circ_abs(angle - row[2])
        if best_d is None or dist < best_d:
            best_d = dist
            best = row
    if best_d is None or best_d > MAX_HUE_DEG:
        return None
    # 위 비율에 못 미친 B 음수 각은 파랑이다. 노랑·초록 기준에 떨어지지 않게 한다.
    if b < 0 and best[0] != "파랑" and best[0] != "보라":
        if circ_abs(angle + 96.0) <= MAX_HUE_DEG:
            return ("파랑", "BLUE", angle, circ_abs(angle + 96.0))
        return None
    return best[0], best[1], angle, best_d


def edge_weight(cx, cy):
    # 광학 중심에서 멀수록 커지는 보정 세기. 가운데는 0, 가로 끝은 1이다.
    # 0은 화면 중심, 1은 가로 끝과 그 바깥의 모서리다. 가운데는 제곱이라 거의 0이다.
    dx = cx - OPT_X
    dy = cy - OPT_Y
    radius = sqrt(dx * dx + dy * dy)
    u = radius / EDGE_RADIUS
    if u > 1.0:
        u = 1.0
    return u * u


def correct_edge(a, b, cx, cy):
    # 가장자리에서 커진 색상각을 되돌린 A, B를 돌려준다. 채도 반지름은 유지한다.
    # 어느 색이든 같은 렌즈 보정이다. 각만 되돌리고 반지름은 유지한다.
    # 반지름을 키우면 흐린 보라가 파랑 기준을 넘는다.
    w = edge_weight(cx, cy)
    if w <= 0.0:
        return a, b
    shift = EDGE_HUE_PULL * w / RAD2DEG
    c = cos(shift)
    s = sin(shift)
    return a * c + b * s, -a * s + b * c


def fringe_px(cx, cy):
    # 통계에서 빼 둘 테두리 두께(픽셀). 가장자리로 갈수록 두꺼워진다.
    # 횡색수차. 렌즈 중심에서는 거의 없고, 화면 가장자리에서 픽셀 테두리가 색으로 샌다.
    # 중심 2픽셀, 모서리(반지름 약 200)에서 6픽셀. 실측 렌즈 모델이 아니라 장의 방향만 따른다.
    dx = cx - OPT_X
    dy = cy - OPT_Y
    radius = sqrt(dx * dx + dy * dy)
    return int(2 + radius / 50.0)


def face_roi(rect):
    # 덩어리 전체 대신, 색을 잴 면 가운데 사각형을 만든다.
    # 외접 사각형의 가장자리는 바닥, 그림자, 색수차 테두리다.
    # 통계는 면의 가운데만 쓴다. 검출 상자로 거르지 않아야
    # 바닥의 파란 픽셀이 보라 면의 A를 0 쪽으로 끌어가지 않는다.
    x, y, w, h = int(rect[0]), int(rect[1]), int(rect[2]), int(rect[3])
    margin = fringe_px(x + w // 2, y + h // 2)
    if w // 5 > margin:
        margin = w // 5
    if h // 5 > margin:
        margin = h // 5
    limit = w // 3
    if h // 3 < limit:
        limit = h // 3
    if limit < 1:
        limit = 1
    if margin > limit:
        margin = limit
    nw = w - 2 * margin
    nh = h - 2 * margin
    if nw < 4 or nh < 4:
        return (x, y, w, h)
    return (x + margin, y + margin, nw, nh)


def roi_medians(img, roi):
    # 사각형 안의 A, B, L 중앙값. 평균은 테두리 한 줄에 끌린다.
    stats = img.get_statistics(roi=roi, l_bins=32, a_bins=64, b_bins=64)
    return (
        read_stat(stats, "a_median"),
        read_stat(stats, "b_median"),
        read_stat(stats, "l_median"),
    )


def strip_hue(img, roi):
    # 칸의 색상각. 바닥처럼 흐리거나 어두우면 경계로 세지 않는다.
    a, b, l_med = roi_medians(img, roi)
    if l_med < 8:
        return None
    if a * a + b * b < 12 * 12:
        return None
    return hue_deg(a, b)


def side_hue(hues):
    # 경계 한쪽에 각이 하나인지 본다. 한 칸만 튀면 자른 경계가 아니다.
    base = None
    for hue in hues:
        if hue is None:
            continue
        if base is None:
            base = hue
        elif circ_abs(hue - base) > SPLIT_SIDE:
            return None
    return base


def axis_measure(img, rect, horizontal):
    # 한 축을 네 칸으로 나눠, 가장 크게 벌어진 각과 그 경계 비율을 돌려준다.
    x, y, w, h = rect
    length = w if horizontal else h
    if length < SPLIT_MIN:
        return None, None
    n = 4
    step = length // n
    if step < 8:
        return None, None
    hues = []
    i = 0
    while i < n:
        origin = i * step
        size = step if i < n - 1 else length - origin
        if horizontal:
            roi = (x + origin, y, size, h)
        else:
            roi = (x, y + origin, w, size)
        hues.append(strip_hue(img, roi))
        i += 1
    best_i = None
    best_gap = 0.0
    i = 0
    while i < n - 1:
        left = side_hue(hues[: i + 1])
        right = side_hue(hues[i + 1 :])
        if left is not None and right is not None:
            gap = circ_abs(left - right)
            if gap > best_gap:
                best_gap = gap
                best_i = i
        i += 1
    if best_i is None:
        return None, None
    left = side_hue(hues[: best_i + 1])
    right = side_hue(hues[best_i + 1 :])
    # 각이 달라도 이름이 같으면 한 면이다. 가장자리 기울기로 블럭을 자르지 않는다.
    # 이웃 색에 12° 더 가깝지 않은 쪽도 그 면으로 남긴다.
    if hue_name(left) is None or hue_name(right) is None:
        return None, None
    if hue_name(left) == hue_name(right) or stays_home(right, left) or stays_home(left, right):
        return None, None
    cut = (best_i + 1) * step
    if cut < SPLIT_PART or length - cut < SPLIT_PART:
        return None, None
    return best_gap, cut / float(length)


# 큰 덩어리를 가로세로 네 칸으로 나눈다. 두 칸이면 경계가 면 한가운데로 붙는다.
SPLIT_GRID = 4


def hue_name(angle, chroma=None):
    # 칸의 이름. 반지름이 있으면 면과 같은 규칙으로 파랑과 보라를 가른다.
    # 반지름이 없으면 각만 본다. 같은 이름이면 각이 달라도 한 면이다.
    if angle is None:
        return None
    if -105.0 < angle < -15.0:
        if chroma is not None and (
            angle <= BLUE_MAX_ANGLE
            or (angle < PURPLE_MIN_ANGLE and chroma >= BLUE_MIN_CHROMA)
        ):
            return "파랑"
        if chroma is not None:
            return "보라"
        if angle < PURPLE_MIN_ANGLE:
            return "파랑"
        return "보라"
    best = None
    best_d = None
    for row in REFERENCES:
        dist = circ_abs(angle - row[2])
        if best_d is None or dist < best_d:
            best_d = dist
            best = row[0]
    if best_d is None or best_d > MAX_HUE_DEG:
        return None
    return best


def has_index(indices, index):
    for item in indices:
        if item == index:
            return True
    return False


def cell_hue_chroma(img, roi, cx, cy):
    # 칸의 각과 반지름. 화면 끝은 면과 같이 되돌린 뒤의 값이다.
    a, b, l_med = roi_medians(img, roi)
    if l_med < 8:
        return None, None
    if a * a + b * b < 12 * 12:
        return None, None
    a, b = correct_edge(a, b, cx, cy)
    return hue_deg(a, b), sqrt(a * a + b * b)


def grid_sample(img, rect):
    # 덩어리를 칸으로 나눠 각 칸의 이름과 반지름을 읽는다. 검은 칸은 상자에 넣지 않는다.
    x, y, w, h = rect
    if w < SPLIT_MIN or h < SPLIT_MIN:
        return None
    n = SPLIT_GRID
    cw = w // n
    ch = h // n
    if cw < 8 or ch < 8:
        n = 2
        cw = w // n
        ch = h // n
        if cw < 8 or ch < 8:
            return None
    hues = []
    chromas = []
    boxes = []
    row = 0
    while row < n:
        col = 0
        while col < n:
            qx = x + col * cw
            qy = y + row * ch
            qw = cw if col < n - 1 else x + w - qx
            qh = ch if row < n - 1 else y + h - qy
            inset = 1 if qw < 14 or qh < 14 else 2
            rw = qw - 2 * inset
            rh = qh - 2 * inset
            boxes.append((qx, qy, qw, qh))
            if rw < 6 or rh < 6:
                hues.append(None)
                chromas.append(None)
            else:
                hue, chroma = cell_hue_chroma(
                    img,
                    (qx + inset, qy + inset, rw, rh),
                    qx + qw // 2,
                    qy + qh // 2,
                )
                hues.append(hue)
                chromas.append(chroma)
            col += 1
        row += 1
    return hues, chromas, boxes, n


def reference_angle(name):
    for row in REFERENCES:
        if row[0] == name:
            return row[2]
    return None


def blue_purple_pair(left, right):
    # 붙어 있는 파랑과 보라는 각이 가깝다. 더 많은 색에 붙이면 한 덩어리가 된다.
    if left == "파랑" and right == "보라":
        return True
    if left == "보라" and right == "파랑":
        return True
    return False


def majority_hue(hues, names):
    # 이 덩어리에서 칸이 가장 많은 이름의 평균 각. 맞닿은 칸을 그 면에 붙일 때 쓴다.
    best_name = None
    best_n = 0
    best_sum = 0.0
    seen = []
    i = 0
    while i < len(hues):
        name = names[i]
        if name is not None:
            found = False
            j = 0
            while j < len(seen):
                if seen[j][0] == name:
                    seen[j][1] += 1
                    seen[j][2] += hues[i]
                    if seen[j][1] > best_n:
                        best_n = seen[j][1]
                        best_name = name
                        best_sum = seen[j][2]
                    found = True
                    break
                j += 1
            if not found:
                seen.append([name, 1, hues[i]])
                if best_n < 1:
                    best_n = 1
                    best_name = name
                    best_sum = hues[i]
        i += 1
    if best_name is None or best_n < 1:
        return None, None
    return best_sum / best_n, best_name


def stays_home(angle, home, name=None, home_name=None):
    # 이웃 기준색이 이 면의 각보다 12° 더 가까울 때만 그 색으로 넘긴다.
    # 파랑과 보라는 기준각이 실제 면보다 멀다. 반지름으로 갈라진 칸은 붙이지 않는다.
    if angle is None or home is None:
        return False
    if name is None:
        name = hue_name(angle)
    if home_name is None:
        home_name = hue_name(home)
    if name is None:
        return False
    if blue_purple_pair(name, home_name):
        return False
    if name == home_name:
        return True
    ref = reference_angle(name)
    if ref is None:
        return False
    return circ_abs(angle - ref) + SPLIT_HUE >= circ_abs(angle - home)


def name_groups(hues, chromas=None):
    # 같은 이름의 칸을 모은다. 노랑 90°와 노랑 70°는 한 면이다.
    # 중간에 걸린 칸은 이 덩어리에서 가장 많은 색으로 남긴다.
    names = []
    i = 0
    while i < len(hues):
        chroma = None
        if chromas is not None:
            chroma = chromas[i]
        names.append(hue_name(hues[i], chroma))
        i += 1
    home, home_name = majority_hue(hues, names)
    groups = []
    i = 0
    while i < len(hues):
        name = names[i]
        if (
            name is not None
            and home is not None
            and home_name is not None
            and stays_home(hues[i], home, name, home_name)
        ):
            name = home_name
        if name is not None:
            placed = False
            for group in groups:
                if group[0] == name:
                    group[2].append(i)
                    placed = True
                    break
            if not placed:
                groups.append([name, hues[i], [i]])
        i += 1
    groups.sort(key=lambda group: len(group[2]), reverse=True)
    return groups


def cells_bbox(boxes, indices):
    x0 = None
    y0 = None
    x1 = None
    y1 = None
    for index in indices:
        bx, by, bw, bh = boxes[index]
        if x0 is None or bx < x0:
            x0 = bx
        if y0 is None or by < y0:
            y0 = by
        if x1 is None or bx + bw > x1:
            x1 = bx + bw
        if y1 is None or by + bh > y1:
            y1 = by + bh
    if x0 is None:
        return None
    return (x0, y0, x1 - x0, y1 - y0)


def group_wide(indices, n):
    # 4×4에서 가로 또는 세로가 한 칸이면 맞닿은 가장자리다. 블럭으로 나누지 않는다.
    if n < 4:
        return True
    seen_row = []
    seen_col = []
    for index in indices:
        row = index // n
        col = index % n
        if not has_index(seen_row, row):
            seen_row.append(row)
        if not has_index(seen_col, col):
            seen_col.append(col)
    return len(seen_row) >= 2 and len(seen_col) >= 2


def count_border(have, other, n, fixed, a0, a1, along_row):
    # 이 가장자리에서 자기 색 칸과 다른 색 칸의 수.
    own_n = 0
    other_n = 0
    i = a0
    while i <= a1:
        if along_row:
            index = fixed * n + i
        else:
            index = i * n + fixed
        if have[index]:
            own_n += 1
        elif other[index]:
            other_n += 1
        i += 1
    return own_n, other_n


def cover_cells(boxes, n, indices, other):
    # 자기 색 칸을 감싼다. 가장자리가 다른 색이면 그 줄만 뺀다.
    # 빈칸 없는 가장 큰 직사각형만 남기면 대각의 ㄱ자가 한 줄이 된다.
    have = []
    them = []
    i = 0
    while i < n * n:
        have.append(False)
        them.append(False)
        i += 1
    for index in indices:
        have[index] = True
    for index in other:
        them[index] = True
    r0 = None
    r1 = None
    c0 = None
    c1 = None
    for index in indices:
        row = index // n
        col = index % n
        if r0 is None or row < r0:
            r0 = row
        if r1 is None or row > r1:
            r1 = row
        if c0 is None or col < c0:
            c0 = col
        if c1 is None or col > c1:
            c1 = col
    if r0 is None:
        return None
    guard = 0
    while guard < n * 4:
        guard += 1
        trimmed = False
        if r1 - r0 >= 2:
            own_n, other_n = count_border(have, them, n, r1, c0, c1, True)
            if other_n > own_n:
                r1 -= 1
                trimmed = True
        if not trimmed and r1 - r0 >= 2:
            own_n, other_n = count_border(have, them, n, r0, c0, c1, True)
            if other_n > own_n:
                r0 += 1
                trimmed = True
        if not trimmed and c1 - c0 >= 2:
            own_n, other_n = count_border(have, them, n, c1, r0, r1, False)
            if other_n > own_n:
                c1 -= 1
                trimmed = True
        if not trimmed and c1 - c0 >= 2:
            own_n, other_n = count_border(have, them, n, c0, r0, r1, False)
            if other_n > own_n:
                c0 += 1
                trimmed = True
        if not trimmed:
            break
    kept = []
    row = r0
    while row <= r1:
        col = c0
        while col <= c1:
            index = row * n + col
            if have[index]:
                kept.append(index)
            col += 1
        row += 1
    if len(kept) < 1:
        return cells_bbox(boxes, indices)
    return cells_bbox(boxes, kept)


def group_gap(hues, left, right):
    # 두 이름 칸의 평균 각 차이. 같은 이름끼리는 부르지 않는다.
    def mean(indices):
        total = 0.0
        count = 0
        for index in indices:
            total += hues[index]
            count += 1
        if count < 1:
            return 0.0
        return total / count

    return circ_abs(mean(left) - mean(right))


def diagonal_parts(img, rect):
    # 넓은 이름이 하나면 None. 호출한 쪽이 블럭 사각형을 그대로 쓴다.
    # 넓은 이름이 여럿이면 각 색의 칸을 감싼 사각형을 돌려준다.
    sampled = grid_sample(img, rect)
    if sampled is None:
        return None
    hues, chromas, boxes, n = sampled
    groups = name_groups(hues, chromas)
    if len(groups) < 1:
        return None
    need = n
    kept = []
    for group in groups:
        if len(group[2]) >= need and group_wide(group[2], n):
            kept.append(group)
    if len(kept) < 1:
        # 색이 있는 칸이 한 줄보다 적으면 빈 상자를 그 칸으로 줄인다.
        cells = groups[0][2]
        bbox = cells_bbox(boxes, cells)
        if bbox is None:
            return None
        if bbox[2] * bbox[3] >= rect[2] * rect[3]:
            return None
        return ([bbox], None, len(cells))
    if len(kept) == 1:
        return None
    gap = group_gap(hues, kept[0][2], kept[1][2])
    # 파랑과 보라는 맞닿아도 각이 12°보다 작다. 반지름으로 갈라졌으면 그 칸으로 나눈다.
    if blue_purple_pair(kept[0][0], kept[1][0]) and gap < SPLIT_HUE:
        gap = SPLIT_HUE
    small_n = len(kept[1][2])
    parts = []
    i = 0
    while i < len(kept):
        other = []
        j = 0
        while j < len(kept):
            if j != i:
                for cell in kept[j][2]:
                    other.append(cell)
            j += 1
        piece = cover_cells(boxes, n, kept[i][2], other)
        if piece is not None and (piece[2] < rect[2] or piece[3] < rect[3]):
            parts.append(piece)
        i += 1
    if len(parts) < 2:
        return (None, gap, small_n)
    return (parts, gap, small_n)


def shift_rects(rects, dx, dy):
    moved = []
    for item in rects:
        moved.append((item[0] + dx, item[1] + dy, item[2], item[3]))
    return moved


def blend_rects(prev_rects, new_rects):
    # 직전 사각형과 지금 귀퉁이를 섞어, 칸 하나가 튀어도 상자가 점프하지 않게 한다.
    used = []
    blended = []
    for nr in new_rects:
        ncx = nr[0] + nr[2] // 2
        ncy = nr[1] + nr[3] // 2
        best = None
        best_d = None
        best_i = None
        i = 0
        while i < len(prev_rects):
            if has_index(used, i):
                i += 1
                continue
            pr = prev_rects[i]
            pcx = pr[0] + pr[2] // 2
            pcy = pr[1] + pr[3] // 2
            d = (pcx - ncx) * (pcx - ncx) + (pcy - ncy) * (pcy - ncy)
            if best_d is None or d < best_d:
                best_d = d
                best = pr
                best_i = i
            i += 1
        if best is None or best_d > SPLIT_MATCH * SPLIT_MATCH:
            blended.append(nr)
        else:
            used.append(best_i)
            k = SPLIT_FOLLOW
            x = int(best[0] * (1.0 - k) + nr[0] * k + 0.5)
            y = int(best[1] * (1.0 - k) + nr[1] * k + 0.5)
            w = int(best[2] * (1.0 - k) + nr[2] * k + 0.5)
            h = int(best[3] * (1.0 - k) + nr[3] * k + 0.5)
            if w < 1:
                w = 1
            if h < 1:
                h = 1
            blended.append((x, y, w, h))
    return blended


def parts_at(rect, horizontal, ratio):
    # 경계 비율로 두 사각형을 만든다. 한쪽이 너무 얇으면 자르지 않는다.
    x, y, w, h = rect
    length = w if horizontal else h
    cut = int(length * ratio + 0.5)
    if cut < SPLIT_PART:
        cut = SPLIT_PART
    if length - cut < SPLIT_PART:
        cut = length - SPLIT_PART
    if cut < SPLIT_PART or length - cut < SPLIT_PART:
        return None
    if horizontal:
        return ((x, y, cut, h), (x + cut, y, w - cut, h))
    return ((x, y, w, cut), (x, y + cut, w, h - cut))


def choose_fresh(img, rect):
    # 새로 자를 축. 가로와 세로 중 각 차이가 큰 쪽이다.
    best = None
    horizontal = True
    while True:
        gap, ratio = axis_measure(img, rect, horizontal)
        if gap is not None and (best is None or gap > best[1]):
            best = (horizontal, gap, ratio)
        if not horizontal:
            break
        horizontal = False
    return best


def take_split_memory(mem, cx, cy, mode):
    # 직전 프레임에서 이 덩어리의 같은 방식 경계를 찾는다. 한 번만 쓴다.
    best_i = None
    best_d = None
    i = 0
    while i < len(mem):
        item = mem[i]
        if item[2] != mode:
            i += 1
            continue
        d = (item[0] - cx) * (item[0] - cx) + (item[1] - cy) * (item[1] - cy)
        if best_d is None or d < best_d:
            best_d = d
            best_i = i
        i += 1
    if best_d is None or best_d > SPLIT_MATCH * SPLIT_MATCH:
        return None
    return mem.pop(best_i)


def has_split_memory(mem, cx, cy, mode):
    # 빼지 않고 직전 경계가 있는지만 본다. 대각 유지와 가로 자르기를 고를 때 쓴다.
    i = 0
    while i < len(mem):
        item = mem[i]
        if item[2] == mode:
            d = (item[0] - cx) * (item[0] - cx) + (item[1] - cy) * (item[1] - cy)
            if d <= SPLIT_MATCH * SPLIT_MATCH:
                return True
        i += 1
    return False


def quad_need(small_n):
    # 칸 하나만 다른 색이면 24°. 반반이면 나란한 경계와 같은 12°다.
    if small_n < 2:
        return SPLIT_CORNER
    return SPLIT_HUE


def split_by_axis(img, rect, cx, cy, split_prev, split_next, record):
    # 한 줄로 붙은 덩어리. 긴 변의 각이 뛰는 곳에서 자른다.
    horizontal = None
    ratio = None
    miss = 0
    prev = None
    if record:
        prev = take_split_memory(split_prev, cx, cy, "a")
    if prev is not None:
        gap, raw = axis_measure(img, rect, prev[3][0])
        if gap is not None and gap >= SPLIT_HOLD:
            ratio = prev[3][1] * (1.0 - SPLIT_FOLLOW) + raw * SPLIT_FOLLOW
            horizontal = prev[3][0]
        elif prev[4] + 1 <= SPLIT_MISS_MAX:
            ratio = prev[3][1]
            horizontal = prev[3][0]
            miss = prev[4] + 1
    if ratio is None:
        fresh = choose_fresh(img, rect)
        if fresh is None or fresh[1] < SPLIT_HUE:
            return [rect]
        horizontal = fresh[0]
        ratio = fresh[2]
        miss = 0
    parts = parts_at(rect, horizontal, ratio)
    if parts is None:
        return [rect]
    if record:
        split_next.append((cx, cy, "a", (horizontal, ratio), miss))
    return [parts[0], parts[1]]


def split_by_quad(img, rect, cx, cy, split_prev, split_next, record, found=None):
    # 이름이 여럿인 덩어리. 각 색의 칸으로 만든 사각형을 유지한다.
    if found is None:
        found = diagonal_parts(img, rect)
    prev = None
    if record:
        prev = take_split_memory(split_prev, cx, cy, "q")
    parts = None
    miss = 0
    gap = None
    small_n = 2
    new_parts = None
    if found is not None:
        new_parts = found[0]
        gap = found[1]
        small_n = found[2]
    open_gap = quad_need(small_n)
    if new_parts is not None and gap is not None and gap >= open_gap:
        parts = new_parts
        if prev is not None and len(prev[3]) >= 2:
            dx = cx - prev[0]
            dy = cy - prev[1]
            parts = blend_rects(shift_rects(prev[3], dx, dy), parts)
    elif (
        new_parts is not None
        and gap is not None
        and gap >= SPLIT_HOLD
        and small_n >= 2
        and prev is not None
        and len(prev[3]) >= 2
    ):
        dx = cx - prev[0]
        dy = cy - prev[1]
        parts = blend_rects(shift_rects(prev[3], dx, dy), new_parts)
    elif prev is not None and len(prev[3]) >= 2 and prev[4] + 1 <= SPLIT_MISS_MAX:
        dx = cx - prev[0]
        dy = cy - prev[1]
        parts = shift_rects(prev[3], dx, dy)
        miss = prev[4] + 1
    if parts is None:
        if new_parts is not None and gap is None:
            parts = new_parts
        else:
            return [rect]
    if record:
        split_next.append((cx, cy, "q", tuple(parts), miss))
    return parts


def split_rect(img, rect, split_prev, split_next, record):
    # 한 덩어리를 그대로 둘지, 맞닿은 두 면으로 나눌지 정한다.
    # 이름이 같으면 블럭 사각형을 유지하고, 이름이 여럿이면 각 색의 칸을 감싼다.
    x, y, w, h = int(rect[0]), int(rect[1]), int(rect[2]), int(rect[3])
    rect = (x, y, w, h)
    if w < SPLIT_MIN and h < SPLIT_MIN:
        return [rect]
    cx = x + w // 2
    cy = y + h // 2
    if w >= SPLIT_MIN and h >= SPLIT_MIN:
        found = diagonal_parts(img, rect)
        confident = False
        if found is not None and found[0] is not None and found[1] is not None:
            confident = found[1] >= quad_need(found[2])
        hold_q = record and has_split_memory(split_prev, cx, cy, "q")
        if confident or hold_q:
            return split_by_quad(
                img, rect, cx, cy, split_prev, split_next, record, found
            )
        if found is not None and found[1] is None and found[0]:
            # 색이 있는 칸만 남긴다. 빈 칸까지 감싼 큰 상자를 줄인다.
            if record:
                split_next.append((cx, cy, "q", tuple(found[0]), 0))
            return found[0]
        return [rect]
    return split_by_axis(img, rect, cx, cy, split_prev, split_next, record)


def rect_quad(rect):
    # 잘라 낸 면은 회전 꼭짓점이 없다. 그 사각형의 네 귀퉁이를 쓴다.
    x, y, w, h = rect
    return (
        (x, y),
        (x + w - 1, y),
        (x + w - 1, y + h - 1),
        (x, y + h - 1),
    )


def same_color_merge(blob0, blob1):
    # 붙어 있어도 상자 비트가 다르면 다른 페인트다. 같은 비트만 한 블럭의 조각으로 합친다.
    c0 = blob_value(blob0, "code")
    c1 = blob_value(blob1, "code")
    if c0 != c1 or c0 == 0:
        return False
    return (c0 & (c0 - 1)) == 0


_merge_cb_ok = True


def find_gated(img, gates, pixels, area):
    # 같은 색의 조각만 합친다. 오래된 펌웨어에 merge_cb가 없으면 합친 뒤를 각도로 자른다.
    global _merge_cb_ok
    if _merge_cb_ok:
        try:
            return img.find_blobs(
                gates,
                pixels_threshold=pixels,
                area_threshold=area,
                merge=True,
                margin=2,
                x_stride=4,
                y_stride=2,
                merge_cb=same_color_merge,
            )
        except TypeError:
            _merge_cb_ok = False
    return img.find_blobs(
        gates,
        pixels_threshold=pixels,
        area_threshold=area,
        merge=True,
        margin=2,
        x_stride=4,
        y_stride=2,
    )


def center_in_faces(faces, cx, cy, pad):
    # 이미 잡은 면 안의 중심이면 같은 블럭을 두 번 그리지 않는다.
    for face in faces:
        if covers(face[8], cx, cy, pad):
            return True
    return False


def overlaps_existing(faces, rect, cx, cy, pad):
    # 새 면의 중심이 이미 잡은 사각형 안이면 같은 블럭이다.
    # 쌓인 블럭은 이웃 중심이 이 사각형 가장자리에 걸린다. 그 중심 때문에 빼지 않는다.
    # 보라만 다시 찾을 때는 기존 중심을 품으면 넣지 않아 파란 면을 보라로 복제하지 않는다.
    for face in faces:
        if covers(face[8], cx, cy, pad):
            return True
        if pad != 0 and covers(rect, face[6], face[7], pad):
            return True
    return False


def hide_as_shadow(faces):
    # 면의 절반 이상이 다른 색 사각형 안이면 그 블럭의 그림자다.
    # 파랑 위의 작은 보라, 보라 위의 작은 파랑만 뺀다. 옆에 쌓인 블럭은 남긴다.
    hide = []
    i = 0
    while i < len(faces):
        hide.append(False)
        i += 1
    i = 0
    while i < len(faces):
        j = 0
        while j < len(faces):
            if i != j:
                left = faces[i][0]
                right = faces[j][0]
                if (
                    (left == "파랑" and right == "보라")
                    or (left == "보라" and right == "파랑")
                ) and overlap_frac(faces[i][8], faces[j][8]) >= 0.5:
                    hide[i] = True
            j += 1
        i += 1
    return hide


def push_face(img, faces, seen_l, seen_uq, rect, corners, cx, cy, purple_soft, purple_only):
    # 한 면의 가운데를 재고, 가장자리 보정 뒤의 이름으로 후보에 넣는다.
    pad = 16 if purple_only else 0
    if overlaps_existing(faces, rect, cx, cy, pad):
        return
    roi = face_roi(rect)
    stats = img.get_statistics(roi=roi, l_bins=32, a_bins=64, b_bins=64)
    a_raw = read_stat(stats, "a_median")
    b_raw = read_stat(stats, "b_median")
    l_med = read_stat(stats, "l_median")
    l_uq = read_stat(stats, "l_uq")
    a_med, b_med = correct_edge(a_raw, b_raw, cx, cy)
    seen_l.append(l_med)
    seen_uq.append(l_uq)
    named = classify_angle(a_med, b_med, l_med, purple_soft=purple_soft)
    if named is None:
        return
    name, label, angle, _dist = named
    if purple_only and name != "보라":
        return
    fitted = tight_corners(img, rect, a_raw, b_raw, name)
    if fitted is not None:
        corners, cx, cy, rect = fitted
    quad = order_quad(corners)
    quad = smooth_quad(nearest_held(held, quad), quad)
    if is_ribbon(quad):
        return
    if int(rect[2]) < MIN_SIDE or int(rect[3]) < MIN_SIDE:
        return
    chroma = sqrt(a_med * a_med + b_med * b_med)
    faces.append((name, label, angle, l_med, chroma, quad, cx, cy, rect, l_uq))


def sparse_blue_parts(img, rect):
    # 파랑 상자는 어두운 보라의 A, B도 받는다. 쌓이면 두 외접 상자가 겹쳐 한 덩어리가 된다.
    # 사이가 비어 밀도가 0.4 아래로 떨어지고, 덩어리를 버리면 파란 면이 통째로 빠진다.
    # 칸에 다른 이름이 있고 파랑 칸이 가로세로로 넓을 때만 그 칸을 남긴다.
    # 보라 칸까지 여기서 그리면 그 상자가 파랑 중심을 먹어 파랑이 다시 빠진다.
    x, y, w, h = int(rect[0]), int(rect[1]), int(rect[2]), int(rect[3])
    if w < SPLIT_MIN or h < SPLIT_MIN:
        return None
    sampled = grid_sample(img, (x, y, w, h))
    if sampled is None:
        return None
    hues, chromas, boxes, n = sampled
    groups = name_groups(hues, chromas)
    blue = None
    other = False
    i = 0
    while i < len(groups):
        group = groups[i]
        if group[0] == "파랑" and len(group[2]) >= n and group_wide(group[2], n):
            blue = group
        elif group[0] != "파랑":
            other = True
        i += 1
    if blue is None or not other:
        return None
    them = []
    i = 0
    while i < len(groups):
        group = groups[i]
        if group[0] != "파랑":
            j = 0
            while j < len(group[2]):
                them.append(group[2][j])
                j += 1
        i += 1
    piece = cover_cells(boxes, n, blue[2], them)
    if piece is None:
        return None
    cx = piece[0] + piece[2] // 2
    cy = piece[1] + piece[3] // 2
    roi = face_roi(piece)
    stats = img.get_statistics(roi=roi, l_bins=32, a_bins=64, b_bins=64)
    a_med = read_stat(stats, "a_median")
    b_med = read_stat(stats, "b_median")
    l_med = read_stat(stats, "l_median")
    a_med, b_med = correct_edge(a_med, b_med, cx, cy)
    named = classify_angle(a_med, b_med, l_med, purple_soft=False)
    if named is None or named[0] != "파랑":
        return None
    return (piece,)


def _ring_box(angle, c0, c1, half):
    # 각·반지름 부채꼴의 A, B 극값. 축을 지나면 그 방향의 끝도 넣는다.
    samples = []
    offs = (-half, -half * 0.5, 0.0, half * 0.5, half)
    for off in offs:
        rad = (angle + off) / RAD2DEG
        ca = cos(rad)
        sa = sin(rad)
        samples.append((c0 * ca, c0 * sa))
        samples.append((c1 * ca, c1 * sa))
    for axis in (0.0, 90.0, -90.0, 180.0):
        delta = axis - angle
        if delta > 180.0:
            delta -= 360.0
        elif delta < -180.0:
            delta += 360.0
        if delta < -half or delta > half:
            continue
        rad = axis / RAD2DEG
        ca = cos(rad)
        sa = sin(rad)
        samples.append((c0 * ca, c0 * sa))
        samples.append((c1 * ca, c1 * sa))
    amin = samples[0][0]
    amax = amin
    bmin = samples[0][1]
    bmax = bmin
    for a, b in samples:
        if a < amin:
            amin = a
        if a > amax:
            amax = a
        if b < bmin:
            bmin = b
        if b > bmax:
            bmax = b
    return amin, amax, bmin, bmax


def paint_gate(a_raw, b_raw, name):
    # 이 면의 각 근처만 연다. 검출 상자는 이웃 페인트의 A, B도 통과시킨다.
    # 밝기가 변하면 반지름이 변하고 각은 남으므로, 각은 좁히고 반지름은 따라간다.
    # 파랑과 보라는 각이 겹쳐 반지름으로 갈리므로 그 폭을 더 좁힌다.
    # 가장자리 보정은 넣지 않는다. find_blobs가 보는 값은 보정 전 A, B다.
    row = None
    for gate in GATES:
        if gate[0] == name:
            row = gate
            break
    if row is None:
        return None
    c = sqrt(a_raw * a_raw + b_raw * b_raw)
    if c < 12.0:
        return None
    near = name == "파랑" or name == "보라"
    if near:
        half = TIGHT_HALF_NEAR
        c0 = c * TIGHT_C0_NEAR
        c1 = c * TIGHT_C1_NEAR
    else:
        half = TIGHT_HALF_DEG
        c0 = c * TIGHT_C0
        c1 = c * TIGHT_C1
    if c0 < 10.0:
        c0 = 10.0
    if c1 > 120.0:
        c1 = 120.0
    if c1 < c0 + 6.0:
        c1 = c0 + 6.0
    angle = atan2(b_raw, a_raw) * RAD2DEG
    amin, amax, bmin, bmax = _ring_box(angle, c0, c1, half)
    amin = int(amin) - 1
    amax = int(amax) + 1
    bmin = int(bmin) - 1
    bmax = int(bmax) + 1
    if amin < row[4]:
        amin = row[4]
    if amax > row[5]:
        amax = row[5]
    if bmin < row[6]:
        bmin = row[6]
    if bmax > row[7]:
        bmax = row[7]
    if amin < -128:
        amin = -128
    if amax > 127:
        amax = 127
    if bmin < -128:
        bmin = -128
    if bmax > 127:
        bmax = 127
    if amax - amin < 4 or bmax - bmin < 4:
        return None
    return (row[2], row[3], amin, amax, bmin, bmax)


def tight_corners(img, rect, a_raw, b_raw, name):
    # 이 면의 각 근처만 다시 찾아 회전 꼭짓점을 그 페인트로 줄인다.
    # 넓은 색 상자는 맞닿은 이웃까지 통과시켜 테두리가 옆 블럭을 덮는다.
    gate = paint_gate(a_raw, b_raw, name)
    if gate is None:
        return None
    x, y, w, h = int(rect[0]), int(rect[1]), int(rect[2]), int(rect[3])
    x -= 2
    y -= 2
    w += 4
    h += 4
    if x < 0:
        w += x
        x = 0
    if y < 0:
        h += y
        y = 0
    iw = img.width()
    ih = img.height()
    if x + w > iw:
        w = iw - x
    if y + h > ih:
        h = ih - y
    if w < 8 or h < 8:
        return None
    blobs = img.find_blobs(
        [gate],
        roi=(x, y, w, h),
        pixels_threshold=30,
        area_threshold=50,
        merge=True,
        margin=3,
        x_stride=1,
        y_stride=1,
    )
    if not blobs:
        return None
    blob = None
    best_px = 0
    for item in blobs:
        pixels = blob_value(item, "pixels")
        if pixels > best_px:
            best_px = pixels
            blob = item
    if blob is None:
        return None
    # 45도로 선 정사각형은 축에 맞춘 상자의 절반쯤만 채운다.
    # 여기서 거르면 이웃까지 감싼 넓은 테두리로 돌아간다.
    if blob_value(blob, "density") < 0.2:
        return None
    bw = blob_value(blob, "w")
    bh = blob_value(blob, "h")
    if bw < 10 or bh < 10:
        return None
    cx = blob_value(blob, "cx")
    cy = blob_value(blob, "cy")
    if not covers(rect, cx, cy, 4):
        return None
    corners = blob_value(blob, "min_corners")
    if corners is None:
        return None
    raw = blob_value(blob, "rect")
    fitted = (int(raw[0]), int(raw[1]), int(raw[2]), int(raw[3]))
    return corners, cx, cy, fitted


def take_blob(img, blob, faces, seen_l, seen_uq, purple_only, split_prev, split_next):
    # 밀도를 통과한 덩어리를 색 경계에서 나누고, 각 면을 이름 후보로 올린다.
    # 밀도가 낮아도 넓은 파랑 칸이 따로 있으면 그 면만 올린다.
    code = blob_value(blob, "code")
    purple_blob = purple_only or (code & PURPLE_CODE) != 0
    density = blob_value(blob, "density")
    rect = blob_value(blob, "rect")
    floor = 0.2 if purple_blob else MIN_DENSITY
    if density < floor:
        if purple_blob:
            return
        parts = sparse_blue_parts(img, rect)
        if parts is None:
            return
        for part in parts:
            push_face(
                img,
                faces,
                seen_l,
                seen_uq,
                part,
                rect_quad(part),
                part[0] + part[2] // 2,
                part[1] + part[3] // 2,
                False,
                False,
            )
        return
    parts = split_rect(img, rect, split_prev, split_next, not purple_only)
    full = (int(rect[0]), int(rect[1]), int(rect[2]), int(rect[3]))
    one = parts[0]
    kept = (
        len(parts) == 1
        and int(one[0]) == full[0]
        and int(one[1]) == full[1]
        and int(one[2]) == full[2]
        and int(one[3]) == full[3]
    )
    if kept:
        pieces = (
            (
                parts[0],
                blob_value(blob, "min_corners"),
                blob_value(blob, "cx"),
                blob_value(blob, "cy"),
            ),
        )
    else:
        pieces = []
        for part in parts:
            pieces.append(
                (
                    part,
                    rect_quad(part),
                    part[0] + part[2] // 2,
                    part[1] + part[3] // 2,
                )
            )
    for part, corners, cx, cy in pieces:
        push_face(
            img,
            faces,
            seen_l,
            seen_uq,
            part,
            corners,
            cx,
            cy,
            False,
            purple_only,
        )


def order_quad(corners):
    # 네 꼭짓점을 중심 기준 각도로 정렬해, 변이 대각선으로 이어지지 않게 한다.
    pts = [(p[0], p[1]) for p in corners]
    cx = 0
    cy = 0
    for p in pts:
        cx += p[0]
        cy += p[1]
    cx /= 4.0
    cy /= 4.0
    # 중심에서 본 각도로 정렬해야 변 순서가 되고, 대각선으로 잇지 않음
    pts.sort(key=lambda p: atan2(p[1] - cy, p[0] - cx))
    return pts


def _cross(ax, ay, bx, by):
    # 외적. 두 선분이 서로 다른 쪽으로 갈라지는지 볼 때 부호만 쓴다.
    return ax * by - ay * bx


def _segments_cross(a, b, c, d):
    # 선분 AB와 CD가 서로의 가운데를 지나면 True다.
    abx = b[0] - a[0]
    aby = b[1] - a[1]
    c1 = _cross(abx, aby, c[0] - a[0], c[1] - a[1])
    c2 = _cross(abx, aby, d[0] - a[0], d[1] - a[1])
    cdx = d[0] - c[0]
    cdy = d[1] - c[1]
    c3 = _cross(cdx, cdy, a[0] - c[0], a[1] - c[1])
    c4 = _cross(cdx, cdy, b[0] - c[0], b[1] - c[1])
    if c1 == 0 or c2 == 0 or c3 == 0 or c4 == 0:
        return False
    return (c1 > 0) != (c2 > 0) and (c3 > 0) != (c4 > 0)


def is_ribbon(pts):
    # 마주 보는 변이 교차하면 블럭이 아니라 꼬인 검출이라 그리지 않는다.
    return _segments_cross(pts[0], pts[1], pts[2], pts[3]) or _segments_cross(
        pts[1], pts[2], pts[3], pts[0]
    )


def smooth_quad(prev, corners):
    # 직전 사각형과 이번 사각형을 섞어 한 프레임 떨림을 줄인다.
    # 새 위치가 80%, 이전 위치가 20%다. 중심이 멀면 다른 블럭으로 보고 섞지 않는다.
    current = [(p[0], p[1]) for p in corners]
    if prev is None:
        return current
    cx = cy = px = py = 0
    for p in current:
        cx += p[0]
        cy += p[1]
    for p in prev:
        px += p[0]
        py += p[1]
    # 합의 차이가 160이면 평균으로는 약 40픽셀. 그 이상이면 다른 블럭
    if (cx - px) ** 2 + (cy - py) ** 2 > 160 * 160:
        return current
    pool = current
    ordered = []
    for anchor in prev:
        best_i = 0
        best_d = None
        for i, p in enumerate(pool):
            d = (p[0] - anchor[0]) ** 2 + (p[1] - anchor[1]) ** 2
            if best_d is None or d < best_d:
                best_d = d
                best_i = i
        ordered.append(pool.pop(best_i))
    keep = 1.0 - FOLLOW
    blended = []
    for i in range(4):
        x = prev[i][0] * keep + ordered[i][0] * FOLLOW
        y = prev[i][1] * keep + ordered[i][1] * FOLLOW
        blended.append((int(x), int(y)))
    return blended


def nearest_held(held, quad):
    # 직전 프레임에서 중심이 80픽셀 안인 사각형을 고른다. 없으면 새로 그린다.
    cx = (quad[0][0] + quad[1][0] + quad[2][0] + quad[3][0]) // 4
    cy = (quad[0][1] + quad[1][1] + quad[2][1] + quad[3][1]) // 4
    best = None
    best_d = None
    for prev in held:
        px = (prev[0][0] + prev[1][0] + prev[2][0] + prev[3][0]) // 4
        py = (prev[0][1] + prev[1][1] + prev[2][1] + prev[3][1]) // 4
        d = (cx - px) ** 2 + (cy - py) ** 2
        if best_d is None or d < best_d:
            best_d = d
            best = prev
    if best_d is None or best_d > 80 * 80:
        return None
    return best


def lock_window(face_clip, dark_face, purple_dark):
    # 7500 µs에서 이름이 붙은 첫 면으로 창을 한 번 고른다.
    # 잘림이고 어두운 면과 어두운 보라가 없으면 조명이 밝다.
    global EXPO_MIN, EXPO_MAX
    if face_clip and not dark_face and not purple_dark:
        EXPO_MIN = EXPO_LOW_MIN
        EXPO_MAX = EXPO_LOW_MAX
    else:
        EXPO_MIN = EXPO_HIGH_MIN
        EXPO_MAX = EXPO_HIGH_MAX
    print("노출창 %d-%d us" % (EXPO_MIN, EXPO_MAX))


def step_exposure(delta):
    # 노출 시간만 delta µs만큼 바꾼다. 게인과 화이트밸런스는 건드리지 않는다.
    # 게인을 그대로 두고 노출만 움직인다.
    # 게인을 올리면 빛과 읽기 잡음이 같이 커지고, 노출을 늘리면 광자가 더 쌓인다.
    global exposure_us, settle, expo_state
    prev = exposure_us
    nxt = prev + delta
    note = ""
    if nxt < EXPO_MIN:
        nxt = EXPO_MIN
        note = " 하한맞춤"
    elif nxt > EXPO_MAX:
        nxt = EXPO_MAX
        note = " 상한맞춤"
    if nxt == prev:
        expo_state = "하한" if delta < 0 else "상한"
        print("노출 한계 %s %d us  %+d%s" % (expo_state, prev, delta, note))
        return
    exposure_us = nxt
    sensor.set_auto_exposure(False, exposure_us=exposure_us)
    settle = EXPO_SETTLE
    if exposure_us <= EXPO_MIN:
        expo_state = "하한"
    elif exposure_us >= EXPO_MAX:
        expo_state = "상한"
    elif delta > 0:
        expo_state = "증가"
    else:
        expo_state = "감소"
    print("노출 %s %d→%d us  %+d%s" % (expo_state, prev, exposure_us, delta, note))


# 시작 2초 동안 장면이 옮긴 자동 화이트밸런스는 쓰지 않는다. 그때 읽힌 게인과 노출도 쓰지 않는다.
# 그 값을 잠그면 형광등을 본 실행은 어둡게, 어두운 블럭을 본 실행은 밝게 고정된다.
sensor.reset()
sensor.set_pixformat(sensor.RGB565)  # H7에서 컬러 프레임 버퍼 한도인 320×240
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)

# 채널은 센서 초기값이다. 시작 2초에 장면이 옮긴 자동 비율은 쓰지 않는다.
# 0 dB를 쓰면 레지스터가 1이 되어 화면이 검게 나간다.
r_db, g_db, b_db = RGB_BASE_DB
print("기본 채널 R %.1f  G %.1f  B %.1f dB" % (r_db, g_db, b_db))
g_db -= GREEN_GAIN_CUT
rgb_gain = (r_db, g_db, b_db)
print("잠금 채널 R %.1f  G %.1f  B %.1f dB" % rgb_gain)

gain_db = GAIN_DB
sensor.set_auto_gain(False, gain_db=gain_db)
sensor.skip_frames(time=200)
try:
    sensor.set_auto_whitebal(False, rgb_gain_db=rgb_gain)
except Exception:
    sensor.set_auto_whitebal(False)
    print("채널 게인을 쓰지 못함")
sensor.skip_frames(time=200)
exposure_us = EXPO_START
sensor.set_auto_exposure(False, exposure_us=exposure_us)
sensor.skip_frames(time=300)

thresholds = [row[2:] for row in GATES]
clock = time.clock()
last_names = None
held = []
split_prev = []
settle = 0
window_wait = EXPO_SETTLE
window_ready = False
expo_state = "유지"
last_expo_why = None
purple_memory = None  # (rect, quad, cx, cy, miss)
print("잠금 노출 %d us 게인 %.1f dB" % (exposure_us, gain_db))

EXPO_MARK = {"유지": "=", "증가": "+", "감소": "-", "상한": "MAX", "하한": "MIN"}

while True:
    # 프레임을 찍고, 다섯 LAB 상자에 들어오는 덩어리를 찾는다.
    # 다른 색 상자는 맞닿아도 한 덩어리로 합치지 않는다.
    clock.tick()
    img = sensor.snapshot()
    blobs = find_gated(img, thresholds, 220, 250)

    names = []
    details = []
    next_held = []
    accepted_l = []
    accepted_uq = []
    purple_l = []
    seen_l = []
    seen_uq = []
    faces = []
    split_next = []

    # 각 덩어리를 색 경계에서 나누고, 면 가운데 각도로 이름을 붙인다.
    for blob in blobs:
        take_blob(img, blob, faces, seen_l, seen_uq, False, split_prev, split_next)

    # x 보폭 4면 흐린 보라 면의 표본이 220 근처에서 들락거려 상자만 깜빡인다.
    # 같은 보폭으로 보라 상자만 픽셀 수를 낮춰 한 번 더 찾는다.
    # 이미 잡은 면과 겹치면 넣지 않아 파란 블럭을 보라로 복제하지 않는다.
    for blob in find_gated(img, [GATES[3][2:]], 80, 120):
        take_blob(img, blob, faces, seen_l, seen_uq, True, split_prev, split_next)

    # 이름을 화면에 그린다. 절반 넘게 겹친 파랑·보라만 같은 블럭의 그림자로 뺀다.
    # 아래에 쌓인 파란 블럭은 윗면의 보라 상자와 가장자리만 닿으므로 남긴다.
    hidden = hide_as_shadow(faces)
    blue_rects = []
    i = 0
    while i < len(faces):
        name, label, angle, l_med, chroma, quad, cx, cy, _rect, l_uq = faces[i]
        if hidden[i]:
            if (
                name == "보라"
                and purple_memory is not None
                and overlap_frac(purple_memory[0], _rect) >= 0.5
            ):
                purple_memory = None
            i += 1
            continue
        if name == "파랑":
            blue_rects.append(_rect)
        accepted_l.append(l_med)
        accepted_uq.append(l_uq)
        next_held.append(quad)
        names.append(name)
        details.append((name, l_med, angle, chroma))
        draw_face(img, name, label, quad, cx, cy)
        if name == "보라":
            purple_memory = (_rect, quad, cx, cy, 0)
            purple_l.append(l_med)
        i += 1

    # 보라 검출이 한 프레임 빠져도, 직전 자리를 다시 재서 아직 보라이면 이름을 유지한다.
    # 그 자리가 파란 면 위면 유지하지 않는다. 쌓인 파랑을 보라로 붙들어 두지 않기 위해서다.
    if purple_memory is not None:
        for rect in blue_rects:
            if overlap_frac(purple_memory[0], rect) >= 0.5:
                purple_memory = None
                break
    if "보라" not in names and purple_memory is not None:
        rect, quad, cx, cy, miss = purple_memory
        roi = face_roi(rect)
        stats = img.get_statistics(roi=roi, l_bins=32, a_bins=64, b_bins=64)
        a_med = read_stat(stats, "a_median")
        b_med = read_stat(stats, "b_median")
        l_med = read_stat(stats, "l_median")
        l_uq = read_stat(stats, "l_uq")
        a_med, b_med = correct_edge(a_med, b_med, cx, cy)
        named = classify_angle(a_med, b_med, l_med, purple_soft=True)
        if named is not None and named[0] == "보라":
            miss = 0
            chroma = sqrt(a_med * a_med + b_med * b_med)
            angle = named[2]
            # 검출이 빠져도 이 면이 어두우면 노출을 줄이지 않는다.
            # 노란 면만 보고 줄이면 보라가 더 자주 빠진다.
            accepted_l.append(l_med)
            accepted_uq.append(l_uq)
            purple_l.append(l_med)
        else:
            miss += 1
            chroma = 0
            angle = 0
            l_med = 0
        if miss <= PURPLE_MISS_MAX:
            names.append("보라")
            details.append(("보라", l_med, angle, chroma))
            draw_face(img, "보라", "PURPLE", quad, cx, cy)
            next_held.append(quad)
            purple_memory = (rect, quad, cx, cy, miss)
        else:
            purple_memory = None

    held = next_held
    split_prev = split_next

    # 이번 프레임의 면 밝기로 다음 노출을 정한다. 화면 평균은 쓰지 않는다.
    # 노출은 블럭 면만 본다. 천이 검다고 화면 평균에 맞추면
    # 노출이 길어지고, 바닥의 파란 잡음이 블럭으로 잡힌다.
    clipping = seen_uq and max(seen_uq) >= CLIP_L
    purple_dark = purple_l and min(purple_l) < PURPLE_L_AIM
    dark_face = accepted_l and min(accepted_l) <= DARK_L
    face_clip = bool(accepted_uq) and max(accepted_uq) >= CLIP_L
    if window_wait > 0:
        # 7500 µs가 센서에 반영된 뒤의 면만 창을 고르는 데 쓴다.
        why = "준비%d" % window_wait
        window_wait -= 1
    elif not window_ready and not accepted_l:
        why = "창 대기"
        expo_state = "유지"
    else:
        if not window_ready:
            lock_window(face_clip, bool(dark_face), bool(purple_dark))
            window_ready = True
        if settle > 0:
            why = "대기%d" % settle
            settle -= 1
        elif clipping and not dark_face and not purple_dark:
            # 노란 면만 밝고 파랑·보라가 이미 어두우면 노출을 더 줄이지 않는다.
            # 줄이면 파랑 각이 보라 쪽으로 붙고, 보라 면은 이름에서 빠진다.
            why = "감소 잘림"
            step_exposure(-EXPO_STEP)
        elif purple_dark and not clipping:
            # 보라만 어둡고 다른 면은 아직 안 잘렸으면 노출을 올린다.
            why = "증가 보라"
            step_exposure(EXPO_UP)
        elif accepted_l and max(accepted_l) <= DARK_L:
            why = "증가 전부어두움"
            step_exposure(EXPO_UP)
        elif clipping and dark_face:
            why = "유지 잘림+어두운면"
            expo_state = "유지"
        elif clipping and purple_dark:
            why = "유지 잘림+보라"
            expo_state = "유지"
        else:
            why = "유지 중간"
            expo_state = "유지"
    # 같은 분기가 매 프레임 반복되지 않게, 분기가 바뀔 때만 남긴다.
    # 실제로 시간이 바뀌는 줄은 step_exposure가 따로 남긴다.
    if why != last_expo_why:
        uq = max(seen_uq) if seen_uq else -1
        lo = min(accepted_l) if accepted_l else -1
        hi = max(accepted_l) if accepted_l else -1
        pl = min(purple_l) if purple_l else -1
        print("노출판단 %s %d us  uq%d  면%d-%d  보라%d" % (why, exposure_us, uq, lo, hi, pl))
        last_expo_why = why

    # 색 집합이 바뀔 때만 터미널에 남긴다. 화면에는 프레임 속도와 노출을 그린다.
    if names:
        key = tuple(sorted(names))
        if key != last_names:
            parts = []
            for name, l_med, angle, chroma in details:
                parts.append("%s L%d h%.0f c%.0f" % (name, l_med, angle, chroma))
            print(" ".join(parts))
            last_names = key
    elif last_names is not None:
        print("없음")
        last_names = None

    img.draw_string((2, 2), "%.0f fps" % clock.fps(), color=(255, 255, 255), scale=2)
    img.draw_string(
        (2, 20),
        "E %d %s" % (exposure_us, EXPO_MARK[expo_state]),
        color=(255, 255, 255),
        scale=2,
    )
