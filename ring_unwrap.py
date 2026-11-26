#!/usr/bin/env python3
# 카메라 화면에 비친 링을 찾아 띠로 편다. 원은 곧은 가로 띠가 되고, 꺾인 자리는 혹으로 남는다.
# ring_judge.py가 이 띠와 꺾임으로 불량을 가린다.
#
#   python3 ring_unwrap.py dataset/frames/P003/P003_000012_dent_d0.6_w10_e7498.png
#   python3 ring_unwrap.py a.png --out strip.png --debug circle.png
#   python3 ring_unwrap.py a.png --center 160 120 --radius 52
#
# 순서
#   1. 세 채널이 모두 밝은 픽셀을 띠로 본다. 도장 색은 한 채널이 낮아 빠지고 하얀 반사만 남는다.
#   2. 그 픽셀에 원을 맞춘다. 원에서 먼 픽셀을 빼고 세 번 다시 맞춘다.
#   3. 원을 따라 각도마다 반지름 방향으로 샘플해 (반지름 × 각도) 띠를 만든다. 위가 안쪽이다.
#   4. 각도마다 띠 밝기의 가운데를 잰다. 곡면이 만든 완만한 휨은 넓은 창의 중앙값으로 빼고,
#      남은 짧은 휨을 꺾임으로 본다.
# 각도 0°는 화면 오른쪽이다. 화면 좌표는 y가 아래라 각도는 시계 방향으로 는다.
# 흰색·노랑처럼 밝은 도장은 1번에서 도장까지 띠로 잡힌다. 그때는 --center, --radius를 준다.

import argparse
import sys
import warnings

import cv2
import numpy as np

# 띠로 볼 밝기. 세 채널 가운데 가장 낮은 값이 이 값과 화면 상위 3% 가운데 큰 쪽을 넘어야 한다.
BAND_FLOOR = 200
BAND_TOP = 97.0
# 원에서 이만큼 안의 픽셀만 남겨 다시 맞춘다. 320×240에서 띠 폭은 약 8 px이다.
KEEP_PX = 6.0
# 띠를 펼 때 원 안팎으로 볼 폭. 곡면에서 원이 휘어도 띠가 이 안에 남게 넉넉히 둔다.
HALF_PX = 12
ANGLES = 360
# 완만한 휨을 빼는 창(각도 칸 수). 31칸이면 31°다. 얕은 함몰의 꺾임은 이보다 짧다.
SMOOTH = 31
# 이보다 크게 벗어나면 꺾임으로 적는다(px).
KINK_PX = 1.5


def band_pixels(rgb):
    low = rgb.min(axis=2)
    thr = max(BAND_FLOOR, float(np.percentile(low, BAND_TOP)))
    ys, xs = np.nonzero(low >= thr)
    return xs.astype(float), ys.astype(float)


def fit_circle(xs, ys, rounds=3):
    """(cx, cy, r, 쓴 픽셀 수)를 돌려준다. 픽셀이 모자라면 None이다."""
    keep = np.ones(len(xs), bool)
    for _ in range(rounds + 1):
        if keep.sum() < 20:
            return None
        x, y = xs[keep], ys[keep]
        a = np.column_stack([x, y, np.ones_like(x)])
        (c0, c1, c2), *_ = np.linalg.lstsq(a, x * x + y * y, rcond=None)
        cx, cy = c0 / 2, c1 / 2
        r = float(np.sqrt(c2 + cx * cx + cy * cy))
        keep = np.abs(np.hypot(xs - cx, ys - cy) - r) < KEEP_PX
    return float(cx), float(cy), r, int(keep.sum())


def unwrap(img, cx, cy, r, half=HALF_PX, angles=ANGLES):
    t = np.arange(angles) * 2 * np.pi / angles
    rr = r + np.arange(-half, half + 1)[:, None]
    map_x = (cx + rr * np.cos(t)[None, :]).astype(np.float32)
    map_y = (cy + rr * np.sin(t)[None, :]).astype(np.float32)
    return cv2.remap(img, map_x, map_y, cv2.INTER_LINEAR,
                     borderMode=cv2.BORDER_CONSTANT, borderValue=0)


def band_center(strip):
    """각도마다 띠 밝기의 가운데가 원에서 몇 px 벗어났는지. 띠가 안 보이면 nan이다."""
    g = strip.min(axis=2).astype(float) if strip.ndim == 3 else strip.astype(float)
    half = (g.shape[0] - 1) // 2
    # 각도마다 바탕(가운데값)을 빼고 남은 밝기로 무게중심을 잰다.
    w = np.clip(g - np.median(g, axis=0)[None, :], 0, None)
    total = w.sum(axis=0)
    if not (total > 0).any():
        return np.full(len(total), np.nan)
    k = np.arange(-half, half + 1)[:, None]
    center = (w * k).sum(axis=0) / np.where(total > 0, total, 1)
    # 밝기 합이 다른 각도의 1/5도 안 되면 띠가 끊긴 것으로 본다. 부품 밖이나 무광 자리다.
    center[total < 0.2 * np.median(total[total > 0])] = np.nan
    return center


def kinks(center, smooth=SMOOTH):
    """완만한 휨을 뺀 짧은 휨(px). 창은 원을 따라 이어 붙인다."""
    h = smooth // 2
    with warnings.catch_warnings():
        # 창 전체가 띠 없는 각도면 nanmedian이 경고를 내고 nan을 돌려준다. nan이 맞는 값이다.
        warnings.simplefilter("ignore", RuntimeWarning)
        base = np.nanmedian(np.stack([np.roll(center, s) for s in range(-h, h + 1)]), axis=0)
    return center - base


def spans(mask):
    """참인 각도 칸을 이어진 구간 [(시작, 끝)]으로 묶는다. 0°와 359°도 잇는다."""
    idx = np.flatnonzero(mask)
    if len(idx) == 0:
        return []
    groups = [[idx[0], idx[0]]]
    for i in idx[1:]:
        if i == groups[-1][1] + 1:
            groups[-1][1] = i
        else:
            groups.append([i, i])
    if len(groups) > 1 and groups[0][0] == 0 and groups[-1][1] == len(mask) - 1:
        groups[0][0] = groups.pop()[0]
    return [tuple(g) for g in groups]


def analyze(rgb, center=None, radius=None, half=HALF_PX):
    """원을 찾고(주면 그 값을 쓰고) 띠와 꺾임을 돌려준다. 원을 못 찾으면 None이다."""
    used = None
    if center is None or radius is None:
        found = fit_circle(*band_pixels(rgb))
        if found is None:
            return None
        fx, fy, fr, used = found
        center = (fx, fy) if center is None else center
        radius = fr if radius is None else radius
    (cx, cy), r = center, radius
    strip = unwrap(rgb, cx, cy, r, half)
    offset = band_center(strip)
    dev = kinks(offset)
    return {"cx": cx, "cy": cy, "r": r, "used": used, "strip": strip,
            "offset": offset, "dev": dev}


def main():
    ap = argparse.ArgumentParser(description="링 반사를 띠로 펴고 꺾임을 잰다")
    ap.add_argument("image")
    ap.add_argument("--center", type=float, nargs=2, metavar=("X", "Y"))
    ap.add_argument("--radius", type=float)
    ap.add_argument("--half", type=int, default=HALF_PX, help="원 안팎으로 볼 폭(px)")
    ap.add_argument("--out", help="편 띠를 PNG로 저장")
    ap.add_argument("--debug", help="맞춘 원과 꺾인 자리를 그린 화면을 저장")
    args = ap.parse_args()

    bgr = cv2.imread(args.image, cv2.IMREAD_COLOR)
    if bgr is None:
        print("그림을 읽지 못했다: %s" % args.image, file=sys.stderr)
        return 1
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    res = analyze(rgb, tuple(args.center) if args.center else None, args.radius, args.half)
    if res is None:
        print("띠로 볼 밝은 픽셀이 모자라 원을 맞추지 못했다. --center, --radius를 준다", file=sys.stderr)
        return 1
    dev = res["dev"]
    seen = np.isfinite(res["offset"])
    print("원 중심 (%.1f, %.1f) 반지름 %.1f px%s" % (
        res["cx"], res["cy"], res["r"], "" if res["used"] is None else ", 띠 픽셀 %d개" % res["used"]))
    print("띠가 보이는 각도 %d/%d" % (seen.sum(), len(seen)))
    if np.isfinite(dev).any():
        i = int(np.nanargmax(np.abs(dev)))
        print("가장 큰 꺾임 %.1f px, %d°" % (dev[i], i * 360 // len(dev)))
        found = spans(np.nan_to_num(np.abs(dev)) > KINK_PX)
        if found:
            print("꺾임 %.1f px 넘는 구간: %s" % (KINK_PX, ", ".join(
                "%d–%d°" % (a * 360 // len(dev), b * 360 // len(dev)) for a, b in found)))
        else:
            print("꺾임 %.1f px 넘는 구간 없음" % KINK_PX)
    if args.out:
        cv2.imwrite(args.out, cv2.cvtColor(res["strip"], cv2.COLOR_RGB2BGR))
        print("띠 저장: %s (반지름 %d × 각도 %d)" % (args.out, res["strip"].shape[0], res["strip"].shape[1]))
    if args.debug:
        view = bgr.copy()
        cv2.circle(view, (round(res["cx"]), round(res["cy"])), round(res["r"]), (240, 196, 124), 1)
        for a, b in spans(np.nan_to_num(np.abs(dev)) > KINK_PX):
            t = np.deg2rad((a + b) / 2 * 360 / len(dev))
            p = (round(res["cx"] + res["r"] * np.cos(t)), round(res["cy"] + res["r"] * np.sin(t)))
            cv2.circle(view, p, 6, (26, 66, 194), 2)
        cv2.imwrite(args.debug, view)
        print("확인 화면 저장: %s" % args.debug)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
