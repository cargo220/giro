# OpenMV IDE에서 이 파일을 열고 실행한다.
# 프레임 화면과 USB 시리얼은 IDE가 이미 붙여 둔다.
# snapshot()은 IDE 화면으로, print()는 IDE 터미널로 간다.
# PC에서 COM 포트를 열거나 이 파일을 PC 파이썬으로 돌리지 않는다.
# 로봇으로 결과를 넘기는 핀 통신은 2차에서 따로 둔다.

import sensor  # 보드의 카메라 센서를 제어하는 모듈
import time  # 프레임 사이 시간과 fps 계산
from math import atan2  # 꼭짓점을 중심 기준 각도로 정렬할 때 사용

# LAB. L은 0~100, A·B는 -128~127.
# OpenMV IDE Threshold Editor로 현장 조명에 맞게 바꾼다.
# 파랑/보라, 초록/노랑은 범위가 겹칠 수 있다. 겹치면 덩어리의 A·B 평균이
# 더 가까운 색을 고른다.
# (이름, 화면 글자, Lmin, Lmax, Amin, Amax, Bmin, Bmax)
COLORS = (
    ("파랑", "BLUE", 0, 100, -35, 0, -80, -15),  # 파랑은 A가 0 이하. 붉은 기(보라)를 넘기지 않음
    ("초록", "GREEN", 0, 100, -80, -15, 5, 70),  # 초록 LAB 초깃값
    ("빨강", "RED", 0, 100, 20, 80, 5, 60),  # 빨강 LAB 초깃값
    ("보라", "PURPLE", 0, 100, 0, 70, -70, 8),  # 옅은 보라도 포함. B가 0 근처여도 A가 파랑보다 붉으면 보라
    ("노랑", "YELLOW", 0, 100, -20, 20, 25, 90),  # 노랑 LAB 초깃값
)

DRAW = {
    "파랑": (40, 80, 255),  # 파랑 블럭에 그을 선 색
    "초록": (0, 255, 80),  # 초록 블럭에 그을 선 색
    "빨강": (255, 40, 40),  # 빨강 블럭에 그을 선 색
    "보라": (180, 60, 255),  # 보라 블럭에 그을 선 색
    "노랑": (255, 220, 0),  # 노랑 블럭에 그을 선 색
}


def ab_center(row):
    # A 범위 중심과 B 범위 중심. 겹친 색 중 어디가 더 가까운지 재는 기준점
    return (row[4] + row[5]) / 2, (row[6] + row[7]) / 2


def classify(code, a_mean, b_mean):
    best = None  # 지금까지 가장 가까운 색 행
    best_d = None  # 그 색까지의 A·B 거리 제곱
    for i, row in enumerate(COLORS):  # 다섯 색을 순서대로 검사
        if not code & (1 << i):  # 이 색 비트가 꺼져 있으면 후보가 아님
            continue  # 다음 색으로
        a_ref, b_ref = ab_center(row)  # 이 색 범위의 A·B 중심
        d = (a_mean - a_ref) ** 2 + (b_mean - b_ref) ** 2  # 덩어리 평균과의 거리 제곱
        if best_d is None or d < best_d:  # 처음이거나 더 가까우면 교체
            best_d = d  # 최단 거리 갱신
            best = row  # 그 색 행을 기억
    return best  # 겹친 색 중 가장 가까운 색


def matched_color(code):
    found = None  # 비트 하나로만 맞은 색. 둘 이상이면 통계로 넘겨야 함
    for i, row in enumerate(COLORS):  # 다섯 색의 비트를 확인
        if code & (1 << i):  # 이 색 범위에 걸림
            if found is not None:  # 이미 다른 색도 걸려 있음
                return None  # 한 색으로 확정할 수 없음
            found = row  # 처음 걸린 색을 저장
    return found  # 색이 하나면 그 행, 없으면 None


# 새 프레임 비중. 높을수록 블럭을 빨리 따라가고, 나머지는 한 프레임 떨림을 누른다.
FOLLOW = 0.8
# A·B가 이 거리보다 0에 가까우면 색이 없는 회색으로 보고 그리지 않음
# 보라는 면이 옅어 12에서는 같이 있을 때 통째로 버려졌다
CHROMA_MIN = 6


def has_chroma(a_mean, b_mean):
    # 원점에서의 거리 제곱. 어두워 색이 죽은 픽셀은 여기에 걸림
    return a_mean * a_mean + b_mean * b_mean >= CHROMA_MIN * CHROMA_MIN


def row_named(name):
    for row in COLORS:
        if row[0] == name:
            return row
    return None


def order_quad(corners):
    pts = [(p[0], p[1]) for p in corners]  # 네 꼭짓점을 (x, y) 리스트로 복사
    cx = 0  # 네 점 중심의 x 합
    cy = 0  # 네 점 중심의 y 합
    for p in pts:  # 네 점을 더해 중심을 구함
        cx += p[0]  # x 누적
        cy += p[1]  # y 누적
    cx /= 4.0  # 평균 x
    cy /= 4.0  # 평균 y
    # 중심에서 본 각도로 정렬해야 변 순서가 되고, 대각선으로 잇지 않음
    pts.sort(key=lambda p: atan2(p[1] - cy, p[0] - cx))
    return pts  # 시계 또는 반시계 방향으로 정렬된 네 점


def _cross(ax, ay, bx, by):
    # 두 벡터의 외적. 부호가 어느 쪽에 있는지를 알려 줌
    return ax * by - ay * bx


def _segments_cross(a, b, c, d):
    abx = b[0] - a[0]  # 선분 AB의 x 성분
    aby = b[1] - a[1]  # 선분 AB의 y 성분
    c1 = _cross(abx, aby, c[0] - a[0], c[1] - a[1])  # C가 AB의 어느 쪽인지
    c2 = _cross(abx, aby, d[0] - a[0], d[1] - a[1])  # D가 AB의 어느 쪽인지
    cdx = d[0] - c[0]  # 선분 CD의 x 성분
    cdy = d[1] - c[1]  # 선분 CD의 y 성분
    c3 = _cross(cdx, cdy, a[0] - c[0], a[1] - c[1])  # A가 CD의 어느 쪽인지
    c4 = _cross(cdx, cdy, b[0] - c[0], b[1] - c[1])  # B가 CD의 어느 쪽인지
    if c1 == 0 or c2 == 0 or c3 == 0 or c4 == 0:  # 끝에서 닿거나 한 직선 위
        return False  # 리본(완전히 가로지름)으로 보지 않음
    # 양쪽 끝점이 서로 반대편에 있어야 두 변이 중간에서 교차
    return (c1 > 0) != (c2 > 0) and (c3 > 0) != (c4 > 0)


def is_ribbon(pts):
    # 마주 보는 변 0-1과 2-3, 또는 1-2와 3-0이 교차하면 리본
    return _segments_cross(pts[0], pts[1], pts[2], pts[3]) or _segments_cross(
        pts[1], pts[2], pts[3], pts[0]
    )


def smooth_quad(prev, corners):
    current = [(p[0], p[1]) for p in corners]  # 이번 프레임 네 점
    if prev is None:  # 근처에 직전 사각형이 없음
        return current  # 섞지 않고 이번 위치를 그대로 사용
    cx = cy = px = py = 0  # 이번 중심 합과 직전 중심 합
    for p in current:  # 이번 네 점의 합
        cx += p[0]
        cy += p[1]
    for p in prev:  # 직전 네 점의 합
        px += p[0]
        py += p[1]
    # 합의 차이가 160이면 평균으로는 약 40픽셀. 그 이상이면 다른 블럭
    if (cx - px) ** 2 + (cy - py) ** 2 > 160 * 160:
        return current  # 멀리 움직였으므로 이전 위치와 섞지 않음
    pool = current  # 아직 짝이 정해지지 않은 이번 점
    ordered = []  # 직전 점 순서에 맞춘 이번 점
    for anchor in prev:  # 직전 사각형의 각 꼭짓점
        best_i = 0  # 가장 가까운 이번 점의 인덱스
        best_d = None  # 그 거리 제곱
        for i, p in enumerate(pool):  # 남은 이번 점과 거리 비교
            d = (p[0] - anchor[0]) ** 2 + (p[1] - anchor[1]) ** 2
            if best_d is None or d < best_d:  # 더 가까운 점
                best_d = d
                best_i = i
        ordered.append(pool.pop(best_i))  # 그 점을 직전 꼭짓점과 짝지음
    keep = 1.0 - FOLLOW  # 직전 위치 비중 0.2
    blended = []  # 섞인 네 점
    for i in range(4):  # 꼭짓점 네 개
        x = prev[i][0] * keep + ordered[i][0] * FOLLOW  # x를 80% 새 위치로
        y = prev[i][1] * keep + ordered[i][1] * FOLLOW  # y를 80% 새 위치로
        blended.append((int(x), int(y)))  # 그리려면 정수 좌표
    return blended


def nearest_held(held, quad):
    # 이번 사각형 중심
    cx = (quad[0][0] + quad[1][0] + quad[2][0] + quad[3][0]) // 4
    cy = (quad[0][1] + quad[1][1] + quad[2][1] + quad[3][1]) // 4
    best = None  # 가장 가까운 직전 사각형
    best_d = None  # 그 중심까지 거리 제곱
    for prev in held:  # 직전 프레임에 그렸던 사각형들
        px = (prev[0][0] + prev[1][0] + prev[2][0] + prev[3][0]) // 4
        py = (prev[0][1] + prev[1][1] + prev[2][1] + prev[3][1]) // 4
        d = (cx - px) ** 2 + (cy - py) ** 2  # 중심 사이 거리 제곱
        if best_d is None or d < best_d:  # 더 가까운 직전 사각형
            best_d = d
            best = prev
    if best_d is None or best_d > 80 * 80:  # 없거나 80픽셀보다 멀면 같은 블럭이 아님
        return None
    return best  # 이 블럭의 직전 사각형


sensor.reset()  # 센서를 초기 상태로
sensor.set_pixformat(sensor.RGB565)  # 색 판별용 컬러. H7은 이 형식이 320×240까지
sensor.set_framesize(sensor.QVGA)  # 320×240
sensor.skip_frames(time=2000)  # 노출이 자리 잡을 때까지 2초분 프레임을 버림
sensor.set_auto_gain(False)  # 게인을 고정해 색이 밝기에 따라 변하지 않게 함
sensor.set_auto_whitebal(False)  # 화이트밸런스를 고정해 색 축이 흔들리지 않게 함
# 바닥이 아니라 블럭 면이 보이도록 노출을 고정. 보라가 회색이면 이 값을 올림
sensor.set_auto_exposure(False, exposure_us=5000)
sensor.skip_frames(time=300)  # 고정 직후 몇 프레임은 아직 불안정해서 버림

thresholds = [row[2:] for row in COLORS]  # 이름·글자를 뺀 LAB 여섯 숫자만 모음
clock = time.clock()  # 프레임 간격으로 fps를 재는 시계
last_names = None  # 직전에 터미널에 낸 색 목록. 같을 때는 다시 안 냄
held = []  # 직전 프레임에서 실제로 그린 사각형들

while True:  # 보드가 켜져 있는 동안 매 프레임 반복
    clock.tick()  # 이번 프레임 시작 시각을 기록
    img = sensor.snapshot()  # 한 장을 찍고 IDE 화면에 올릴 이미지를 받음
    blobs = img.find_blobs(  # 다섯 색에 해당하는 덩어리를 모두 찾음
        thresholds,  # 색마다 LAB 범위
        pixels_threshold=200,  # 픽셀이 200개보다 적으면 노이즈로 버림
        area_threshold=200,  # 가로×세로 면적이 200보다 작으면 버림
        merge=True,  # 가까이 붙은 같은 색 조각은 한 블럭으로 합침
        margin=2,  # 2픽셀 이내면 붙은 것으로 봄
        x_stride=4,  # 가로로 4픽셀씩 건너뛰며 찾아 주기를 줄임
        y_stride=2,  # 세로로 2픽셀씩 건너뛰며 찾음
    )

    names = []  # 이번에 그린 블럭들의 한글 색 이름
    next_held = []  # 이번에 그린 사각형. 다음 프레임의 흔들림 보정에 씀
    for blob in blobs:  # 잡힌 블럭을 하나씩 따로 처리
        # 칸 수를 줄인 히스토그램으로 A·B 평균만 빨리 구함
        stats = img.get_statistics(roi=blob.rect, l_bins=4, a_bins=8, b_bins=8)
        a_mean = stats.a_mean
        b_mean = stats.b_mean
        if not has_chroma(a_mean, b_mean):  # 색이 없는 어두운 회색
            continue
        blue_purple = (1 << 0) | (1 << 3)  # 파랑 비트와 보라 비트
        if blob.code & blue_purple and not (blob.code & ~blue_purple):
            # B가 음수인 쪽. A가 양수면 보라, 0 이하면 파랑
            row = row_named("보라" if a_mean > 0 else "파랑")
        else:
            row = matched_color(blob.code)  # 색 비트가 하나면 그 색으로 확정
            if row is None and blob.code:  # 두 색 이상이면 평균에 더 가까운 색
                row = classify(blob.code, a_mean, b_mean)
        if row is None:  # 어떤 색으로도 정하지 못함
            continue  # 이 덩어리는 그리지 않음
        quad = order_quad(blob.min_corners)  # 블럭에 맞춘 네 점을 변 순서로
        quad = smooth_quad(nearest_held(held, quad), quad)  # 같은 블럭의 직전 위치와 섞음
        if is_ribbon(quad):  # 마주 보는 변이 교차하면 리본
            continue  # 선·글자·십자를 그리지 않음
        name, label = row[0], row[1]  # 터미널용 한글 이름, 화면용 영어 이름
        color = DRAW[name]  # 그 색으로 그릴 RGB
        next_held.append(quad)  # 다음 프레임 보정용으로 남김
        names.append(name)  # 출력 목록에 색 이름 추가
        img.draw_edges(quad, color=color, thickness=2)  # 네 변을 블럭 모양대로 그림
        img.draw_cross((blob.cx, blob.cy), color=color)  # 덩어리 중심에 작은 십자
        top = quad[0]  # 글자를 올릴 기준점. 아래에서 가장 위 점으로 바꿈
        for point in quad:  # 네 점 중
            if point[1] < top[1]:  # 화면에서 더 위에 있는 점
                top = point  # 글자 위치로 선택
        y = top[1] - 16  # 점 바로 위에 글자가 오도록 올림
        if y < 0:  # 화면 위로 나가면
            y = 0  # 맨 위에 붙임
        # 펌웨어 5는 글자 위치를 (x, y) 한 묶음으로 받음
        img.draw_string((top[0], y), label, color=color, scale=2)
    held = next_held  # 이번에 그린 것만 다음 프레임의 직전 사각형으로 넘김

    if names:  # 하나 이상 그렸으면
        key = tuple(sorted(names))  # 순서가 바뀌어도 같은 목록이면 같은 키
        if key != last_names:  # 색 구성이 바뀔 때만
            print(" ".join(key))  # IDE 터미널에 한글 이름을 한 줄로
            last_names = key  # 같은 구성은 다음 프레임에 다시 안 냄
    elif last_names is not None:  # 직전에는 있었는데 지금은 없음
        print("없음")  # 사라진 것을 한 번만 알림
        last_names = None  # 없는 상태가 계속돼도 반복 출력하지 않음

    # 왼쪽 위에 이 루프의 처리 속도. 사각형을 얼마나 자주 다시 그리는지
    img.draw_string((2, 2), "%.0f fps" % clock.fps(), color=(255, 255, 255), scale=2)
