#!/usr/bin/env python3
# 링 반사가 형상에 비해 더 깨졌는지 보고 불량을 가린다. 기준은 정상 프레임만으로 잡는다.
# 비친 빛은 늘 같은 링이라, 정상 부품이면 형상이 정한 만큼만 휜다. 그보다 더 깨지면 불량이다.
#
#   python3 ring_judge.py calibrate          # 학습 부품의 정상 프레임으로 기준을 잡아 dataset/judge.json에 쓴다
#   python3 ring_judge.py judge P003         # 부품 하나의 프레임을 모두 판정
#   python3 ring_judge.py judge a.png --type flat
#   python3 ring_judge.py evaluate           # 검증·시험 부품으로 헛경보와 깊이별 검출을 본다
#
# 정상의 기준은 부품 종류마다 다르다.
#   flat     매끄러운 휨은 형상, 짧은 꺾임은 결함. ring_unwrap.py의 꺾임 그대로다
#   cowl     정상 카울의 기준 띠와 견준다. 모서리처럼 정상에서도 깨지는 자리는 기준에 이미 들어 있다
#            기준값도 각도마다 따로 둔다. 모서리 근처는 부품이 1 px만 비켜 놓여도 차이가 커서,
#            한 값으로 두면 그 자리 때문에 나머지 자리까지 느슨해진다
#            라인이 부품을 측정 범위 가운데에 세운다는 전제다. 자리가 크게 다르면 기준과 어긋난다
#   muffler  같은 부품의 다른 각도끼리 견준다. 원통이라 돌려도 같은 모양이 비쳐야 한다
#            기준 부품이 필요 없는 대신, 한 부품을 세 각도 이상 찍어야 판정한다
# 두 값을 잰다.
#   꺾임  기준(flat은 매끄러운 휨)에서 짧게 벗어난 가장 큰 값, px
#   끊김  기준에는 띠가 있는데 이 프레임에는 없는 가장 긴 구간, ° (cowl, muffler)
# 기준값은 정상 프레임에서 나온 가장 큰 값에 여유를 더한 것이다. 둘 중 하나라도 넘으면 불량이다.
# 기준을 잡을 때 cowl·muffler는 자기 자신을 뺀 나머지로 기준 띠를 만든다. 자기를 넣으면 늘 작게 나온다.
#
# 깊이·폭을 넣고 찍은 결함 프레임은 기준을 잡는 데 쓰지 않는다. evaluate에서 몇 mm부터 잡히는지 보는 데 쓴다.
# 얼룩과 색상 불균일은 링 모양을 바꾸지 않아 여기서 잡지 않는다. 색 판별로 따로 본다.

import argparse
import warnings
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np

import dataset_meta as dm
import ring_unwrap as ru

MODES = {"flat": "smooth", "cowl": "reference", "muffler": "angles"}
# 정상에서 나온 가장 큰 꺾임에 곱하는 여유. 정상끼리도 잡음만큼은 흔들린다.
MARGIN = 1.25
# 꺾임 기준값의 바닥(px). 정상이 너무 고르게 나와도 픽셀 잡음보다 작게 잡지 않는다.
KINK_FLOOR = 1.0
# 끊김 기준값에 더하는 여유(°).
GAP_MARGIN = 3
MIN_ANGLES = 3
# cowl의 각도별 기준값은 이웃 각도까지 넓혀 잡는다(±칸). 정상 프레임 수가 적어 한 각도만 보면 들쭉날쭉하다.
LIMIT_SPREAD = 3


def judge_path():
    return dm.DATASET / "judge.json"


def load_rgb(path):
    bgr = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if bgr is None:
        raise SystemExit("그림을 읽지 못했다: %s" % path)
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def part_frames(part_id):
    """[(png 경로, label)]. label이 없는 프레임은 판정은 하되 정상·결함 어느 쪽으로도 세지 않는다."""
    out = []
    for png in dm.frame_files(part_id):
        meta = png.with_suffix(".json")
        label = dm.load_json(meta).get("label") if meta.exists() else None
        out.append((png, label))
    return out


def part_type(part_id):
    path = dm.part_path(part_id)
    return dm.load_json(path).get("type") if path.exists() else None


def is_normal(label):
    return label is not None and label.get("kind") == "none"


def nanmedian(stack, axis=0):
    with warnings.catch_warnings():
        # 모든 프레임에서 띠가 없는 각도는 nan이 맞다.
        warnings.simplefilter("ignore", RuntimeWarning)
        return np.nanmedian(stack, axis=axis)


def longest_run(mask):
    """참이 이어진 가장 긴 구간 길이. 0°와 359°도 잇는다."""
    if not mask.any():
        return 0
    if mask.all():
        return len(mask)
    m = np.roll(mask, -int(np.argmin(mask)))
    best = run = 0
    for v in m:
        run = run + 1 if v else 0
        best = max(best, run)
    return best


def peak(resid, limit=1.0, gap=0):
    """기준값에 비해 가장 많이 벗어난 각도의 점수. limit은 한 값이거나 각도마다의 배열이다.
    {"kink": 그 각도의 꺾임, "angle": 각도, "limit": 그 각도의 기준값, "gap": 끊김}"""
    limit = np.broadcast_to(np.asarray(limit, float), resid.shape)
    if not np.isfinite(resid).any():
        return {"kink": 0.0, "angle": None, "limit": float(limit[0]), "gap": gap}
    i = int(np.nanargmax(np.abs(resid) / limit))
    return {"kink": float(abs(resid[i])), "angle": i * 360 // len(resid), "limit": float(limit[i]),
            "gap": gap}


def spread_max(values, half=LIMIT_SPREAD):
    """각도마다 이웃 ±half칸 가운데 가장 큰 값. 0°와 359°도 잇는다."""
    return np.max(np.stack([np.roll(values, s) for s in range(-half, half + 1)]), axis=0)


def common_circle(images):
    """여러 프레임에 맞춘 원의 가운데값. 같은 원으로 펴야 각도별로 견줄 수 있다."""
    fits = [f for f in (ru.fit_circle(*ru.band_pixels(rgb)) for rgb in images) if f is not None]
    if not fits:
        return None
    cx, cy, r, _ = np.median(np.array(fits), axis=0)
    return float(cx), float(cy), float(r)


def offsets_at(images, circle):
    cx, cy, r = circle
    return np.stack([ru.band_center(ru.unwrap(rgb, cx, cy, r)) for rgb in images])


def residual(offset, ref_offset):
    """기준 띠와의 차이에서 천천히 변하는 부분을 뺀 것. 천천히 변하는 차이는 놓인 자리가 조금 다른 탓이다."""
    return ru.kinks(offset - ref_offset)


def gap_of(offset, ref_visible):
    return longest_run(ref_visible & ~np.isfinite(offset))


def smooth_scores(images, limit=1.0):
    """flat: 프레임마다 점수. 원을 못 맞춘 프레임은 None."""
    out = []
    for rgb in images:
        res = ru.analyze(rgb)
        out.append(None if res is None else peak(res["dev"], limit))
    return out


def leave_one_out(offsets):
    """cowl·muffler 기준 잡기: 프레임마다 자기를 뺀 나머지의 가운데값을 기준 띠로 견준다.
    (차이 배열 목록, 끊김 목록)을 돌려준다."""
    resids, gaps = [], []
    for i in range(len(offsets)):
        rest = np.delete(offsets, i, axis=0)
        resids.append(residual(offsets[i], nanmedian(rest)))
        gaps.append(gap_of(offsets[i], np.mean(np.isfinite(rest), axis=0) >= 0.5))
    return resids, gaps


def angle_scores(images, limit=1.0):
    """muffler: 한 부품의 각도별 프레임을 서로 견준다. 각도가 모자라면 None."""
    if len(images) < MIN_ANGLES:
        return None
    circle = common_circle(images)
    if circle is None:
        return [None] * len(images)
    resids, gaps = leave_one_out(offsets_at(images, circle))
    return [peak(r, limit, g) for r, g in zip(resids, gaps)]


def reference_scores(images, ref, limit):
    circle = (ref["cx"], ref["cy"], ref["r"])
    offset = from_json_list(ref["offset"])
    visible = np.array(ref["visible"], bool)
    return [peak(residual(o, offset), limit, gap_of(o, visible)) for o in offsets_at(images, circle)]


def to_json_list(values):
    return [None if not np.isfinite(v) else round(float(v), 3) for v in values]


def from_json_list(values):
    return np.array([np.nan if v is None else v for v in values], float)


def kink_limit(cfg):
    """한 값(flat·muffler)이거나 각도마다의 배열(cowl)."""
    return from_json_list(cfg["kink_limit"]) if "kink_limit" in cfg else cfg["kink_px"]


def verdict(score, cfg):
    """(불량인지, 이유)."""
    if score is None:
        return None, "원을 맞추지 못함"
    why = []
    if score["kink"] > score["limit"]:
        why.append("꺾임 %.1f > %.1f px" % (score["kink"], score["limit"]))
    if cfg.get("gap_deg") is not None and score["gap"] > cfg["gap_deg"]:
        why.append("끊김 %d > %d°" % (score["gap"], cfg["gap_deg"]))
    return bool(why), ", ".join(why)


def score_part(ptype, images, cfg):
    mode = MODES[ptype]
    limit = kink_limit(cfg)
    if mode == "smooth":
        return smooth_scores(images, limit)
    if mode == "reference":
        return reference_scores(images, cfg["reference"], limit)
    return angle_scores(images, limit)


def split_parts(names, use_all):
    """splits.json에서 고른 나눔의 부품. 없으면 모든 부품을 쓰고 알린다."""
    parts = dm.list_parts()
    if use_all or not dm.SPLITS.exists():
        if not use_all:
            print("splits.json이 없어 모든 부품을 쓴다. 기준과 평가가 같은 부품을 보게 된다")
        return parts
    splits = dm.load_json(dm.SPLITS)
    chosen = {p for name in names for p in splits.get(name, [])}
    return [p for p in parts if p in chosen]


def calibrate(args):
    parts = split_parts(("train",), args.all)
    out = {}
    for ptype in dm.PART_TYPES:
        mode = MODES[ptype]
        groups = []   # 부품마다 정상 프레임 그림
        for pid in parts:
            if part_type(pid) != ptype:
                continue
            images = [load_rgb(p) for p, label in part_frames(pid) if is_normal(label)]
            if images:
                groups.append((pid, images))
        if not groups:
            continue
        frames = sum(len(images) for _, images in groups)
        if mode == "reference":
            images = [rgb for _, g in groups for rgb in g]
            circle = common_circle(images)
            if circle is None:
                print("%s: 정상 프레임에서 원을 맞추지 못했다" % ptype)
                continue
            offsets = offsets_at(images, circle)
            resids, gaps = leave_one_out(offsets)
            # 각도마다 정상이 벗어난 가장 큰 값. 정상에서 띠가 늘 없는 각도는 0으로 두고 바닥값을 쓴다.
            with warnings.catch_warnings():
                warnings.simplefilter("ignore", RuntimeWarning)
                worst = np.nan_to_num(np.nanmax(np.abs(np.stack(resids)), axis=0))
            limit = np.maximum(KINK_FLOOR, MARGIN * spread_max(worst))
            kink_max = float(worst.max())
            good = [s for s in offsets if np.isfinite(s).any()]
        else:
            if mode == "smooth":
                scores = smooth_scores([rgb for _, images in groups for rgb in images])
            else:
                scores = []
                for pid, images in groups:
                    got = angle_scores(images)
                    if got is None:
                        print("%s %s: 각도가 %d개라 견주지 못한다. %d개 이상 찍는다" % (
                            ptype, pid, len(images), MIN_ANGLES))
                        continue
                    scores += got
            good = [s for s in scores if s is not None]
            if good:
                kink_max = max(s["kink"] for s in good)
                gaps = [s["gap"] for s in good]
        if not good:
            print("%s: 원을 맞춘 정상 프레임이 없다" % ptype)
            continue
        cfg = {"mode": mode, "parts": len(groups), "frames": frames, "fitted": len(good),
               "kink_max": round(kink_max, 3), "kink_px": round(max(KINK_FLOOR, kink_max * MARGIN), 3)}
        if mode != "smooth":
            cfg.update(gap_max=int(max(gaps)), gap_deg=int(max(gaps) + GAP_MARGIN))
        if mode == "reference":
            cfg["kink_limit"] = to_json_list(limit)
            cfg["reference"] = {"cx": round(circle[0], 2), "cy": round(circle[1], 2), "r": round(circle[2], 2),
                                "offset": to_json_list(nanmedian(offsets)),
                                "visible": (np.mean(np.isfinite(offsets), axis=0) >= 0.5).tolist()}
        out[ptype] = cfg
    if not out:
        print("기준을 잡을 정상 프레임이 없다. 결함 종류 none으로 찍은 프레임이 필요하다")
        return 1
    dm.write_json(judge_path(), out)
    print("%-8s %-10s %4s %5s %10s %14s" % ("종류", "방식", "부품", "프레임", "정상 최대", "기준값"))
    for ptype, cfg in out.items():
        gap = "" if cfg["mode"] == "smooth" else "  끊김 %d°→%d°" % (cfg["gap_max"], cfg["gap_deg"])
        if "kink_limit" in cfg:
            lim = from_json_list(cfg["kink_limit"])
            limit = "%.2f–%.2fpx" % (np.nanmin(lim), np.nanmax(lim))
        else:
            limit = "%.2fpx" % cfg["kink_px"]
        print("%-8s %-10s %4d %5d %8.2fpx %14s%s" % (
            ptype, cfg["mode"], cfg["parts"], cfg["frames"], cfg["kink_max"], limit, gap))
        if "kink_limit" in cfg:
            loose = np.flatnonzero(lim > max(KINK_FLOOR, np.nanmedian(lim)) * 1.5)
            if len(loose):
                print("  기준값이 큰 각도: %s (정상에서도 크게 흔들리는 자리)" % ", ".join(
                    "%d–%d°" % (a, b) for a, b in ru.spans(np.isin(np.arange(len(lim)), loose))))
        if cfg["fitted"] < cfg["frames"]:
            print("  원을 못 맞춘 프레임 %d장은 뺐다" % (cfg["frames"] - cfg["fitted"]))
    print("→ %s" % judge_path())
    return 0


def load_config():
    if not judge_path().exists():
        raise SystemExit("기준이 없다. 먼저 python3 ring_judge.py calibrate")
    return dm.load_json(judge_path())


def judge(args):
    config = load_config()
    target = args.target
    if target.endswith(".png"):
        path = Path(target)
        ptype = args.type or part_type(path.parent.name)
        if ptype is None:
            raise SystemExit("부품 종류를 모른다. --type을 준다")
        if MODES[ptype] == "angles":
            raise SystemExit("머플러는 한 부품의 여러 각도를 견주므로 부품 ID로 판정한다")
        items = [(path, None)]
    else:
        ptype = args.type or part_type(target)
        items = part_frames(target)
        if ptype is None or not items:
            raise SystemExit("부품 %s의 기록이나 프레임이 없다" % target)
    if ptype not in config:
        raise SystemExit("%s 기준이 없다. 그 종류의 정상 프레임으로 calibrate한다" % ptype)
    cfg = config[ptype]
    scores = score_part(ptype, [load_rgb(p) for p, _ in items], cfg)
    if scores is None:
        raise SystemExit("각도가 %d개라 견주지 못한다. %d개 이상 찍는다" % (len(items), MIN_ANGLES))
    bad = 0
    for (path, _), score in zip(items, scores):
        flag, why = verdict(score, cfg)
        bad += bool(flag)
        if score is None:
            print("%s  %s" % (path.name, why))
            continue
        where = "" if score["angle"] is None else " @ %d°" % score["angle"]
        print("%s  꺾임 %.2f/%.2f px%s  끊김 %d°  → %s" % (
            path.name, score["kink"], score["limit"], where, score["gap"],
            "불량 (%s)" % why if flag else "정상"))
    limit = "각도마다" if "kink_limit" in cfg else "%.2f px" % cfg["kink_px"]
    print("%s 기준: 꺾임 %s%s" % (ptype, limit,
                               "" if cfg.get("gap_deg") is None else ", 끊김 %d°" % cfg["gap_deg"]))
    print("불량으로 본 프레임 %d/%d → %s" % (bad, len(items), "불량" if bad else "정상"))
    return 0


def depth_key(ptype, label):
    return (ptype, label.get("kind"), label.get("depth_mm"), label.get("width_mm"), label.get("length_mm"))


def evaluate(args):
    config = load_config()
    parts = split_parts(("val", "test"), args.all)
    by_label = defaultdict(lambda: [0, 0])    # (부품 종류, 결함 종류, 깊이, 폭, 길이) → [불량으로 본 수, 전체]
    part_rows = []
    for pid in parts:
        ptype = part_type(pid)
        if ptype not in config:
            continue
        items = part_frames(pid)
        if not items:
            continue
        scores = score_part(ptype, [load_rgb(p) for p, _ in items], config[ptype])
        if scores is None:
            continue
        flags = [verdict(s, config[ptype])[0] for s in scores]
        labels = [label for _, label in items]
        truth = any(l is not None and l.get("kind") != "none" for l in labels)
        part_rows.append((pid, ptype, truth, any(f for f in flags)))
        # 머플러는 결함이 보이는 각도가 일부라 프레임 단위로 세지 않는다. 부품 단위만 본다.
        if MODES[ptype] == "angles":
            continue
        for label, flag in zip(labels, flags):
            if label is None or flag is None:
                continue
            row = by_label[depth_key(ptype, label)]
            row[0] += bool(flag)
            row[1] += 1
    if not part_rows:
        print("평가할 부품이 없다. 검증·시험 부품의 프레임과 기준이 필요하다")
        return 1

    show = lambda v: "-" if v is None else "%g" % v
    for ptype in sorted({k[0] for k in by_label}):
        print("프레임 %s" % ptype)
        normal = by_label.get((ptype, "none", None, None, None))
        if normal:
            print("  정상 %d장 가운데 불량으로 본 것 %d장 (헛경보 %.0f%%)" % (
                normal[1], normal[0], 100.0 * normal[0] / normal[1]))
        rows = sorted((k for k in by_label if k[0] == ptype and k[1] != "none"),
                      key=lambda k: tuple((v is None, v) for v in k[1:]))
        if rows:
            print("  %-8s %6s %6s %6s %6s %6s" % ("결함", "깊이", "폭", "길이", "잡음", "비율"))
        for key in rows:
            hit, total = by_label[key]
            _, kind, depth, width, length = key
            print("  %-8s %6s %6s %6s %3d/%-2d %5.0f%%" % (
                kind, show(depth), show(width), show(length), hit, total, 100.0 * hit / total))
    print("부품")
    for truth in (False, True):
        rows = [r for r in part_rows if r[2] == truth]
        if rows:
            hit = sum(1 for r in rows if r[3])
            print("  %s 부품 %d개 가운데 불량으로 본 것 %d개" % ("결함" if truth else "정상", len(rows), hit))
    miss = [r[0] for r in part_rows if r[2] and not r[3]]
    false = [r[0] for r in part_rows if not r[2] and r[3]]
    if miss:
        print("  놓친 결함 부품: %s" % ", ".join(miss))
    if false:
        print("  헛경보 부품: %s" % ", ".join(false))
    return 0


def main():
    ap = argparse.ArgumentParser(description="링 반사가 형상에 비해 더 깨졌는지로 불량 판정")
    sub = ap.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("calibrate", help="정상 프레임으로 기준 잡기")
    c.add_argument("--all", action="store_true", help="splits.json 대신 모든 부품을 쓴다")
    j = sub.add_parser("judge", help="부품 ID나 PNG 하나 판정")
    j.add_argument("target")
    j.add_argument("--type", choices=dm.PART_TYPES)
    e = sub.add_parser("evaluate", help="검증·시험 부품으로 헛경보와 검출 한계 보기")
    e.add_argument("--all", action="store_true", help="splits.json 대신 모든 부품을 쓴다")
    args = ap.parse_args()
    return {"calibrate": calibrate, "judge": judge, "evaluate": evaluate}[args.cmd](args)


if __name__ == "__main__":
    raise SystemExit(main())
