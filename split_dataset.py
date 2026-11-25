#!/usr/bin/env python3
# 부품 단위로 학습·검증·시험을 나눈다. 같은 부품에서 찍은 프레임은 한쪽에만 들어간다.
# 프레임 단위로 섞으면 거의 같은 사진이 양쪽에 들어가 시험 성적이 실제보다 좋게 나온다.
#
#   python3 split_dataset.py                       # dataset/splits.json을 만들거나 새 부품만 더한다
#   python3 split_dataset.py --ratio 0.7 0.15 0.15
#   python3 split_dataset.py --reshuffle           # 이미 나눈 부품도 다시 나눈다
#
# 한번 정한 부품의 자리는 바꾸지 않는다. 시험 부품이 실행마다 바뀌면 성적을 견줄 수 없다.
# 새 부품은 (부품 종류, 결함 유무) 묶음마다 목표 비율에서 가장 모자란 쪽에 넣는다.
# 기록이 덜 찼거나 프레임이 없는 부품은 나누지 않고 알려 준다.

import argparse
import random
from collections import Counter

from dataset_meta import SPLITS, check_part, frame_files, list_parts, load_json, part_path, write_json

NAMES = ("train", "val", "test")


def group_key(part):
    return part["type"], "defect" if part["defects"] else "normal"


def main():
    ap = argparse.ArgumentParser(description="부품 단위 학습·검증·시험 나누기")
    ap.add_argument("--ratio", type=float, nargs=3, default=(0.7, 0.15, 0.15), metavar=("TRAIN", "VAL", "TEST"))
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--reshuffle", action="store_true", help="이미 나눈 부품도 다시 나눈다")
    args = ap.parse_args()
    total = sum(args.ratio)
    ratio = [r / total for r in args.ratio]

    parts, skipped = {}, []
    for pid in list_parts():
        part = load_json(part_path(pid))
        problems = check_part(part, pid)
        if problems:
            skipped.append("%s: %s" % (pid, problems[0]))
        elif not frame_files(pid):
            skipped.append("%s: 프레임이 없다" % pid)
        else:
            parts[pid] = part

    old = {}
    if SPLITS.exists() and not args.reshuffle:
        saved = load_json(SPLITS)
        old = {pid: name for name in NAMES for pid in saved.get(name, []) if pid in parts}
    where = dict(old)
    new = [pid for pid in sorted(parts) if pid not in where]
    random.Random(args.seed).shuffle(new)
    for pid in new:
        key = group_key(parts[pid])
        count = Counter(where[p] for p in where if group_key(parts[p]) == key)
        n = sum(count.values()) + 1
        # 비율로 받아야 할 수에서 이미 받은 수를 뺀 값이 가장 큰 쪽에 넣는다. 같으면 train부터.
        where[pid] = max(NAMES, key=lambda s: (ratio[NAMES.index(s)] * n - count[s], -NAMES.index(s)))

    out = {"ratio": ratio, "seed": args.seed}
    for name in NAMES:
        out[name] = sorted(p for p in where if where[p] == name)
    write_json(SPLITS, out)

    print("%-6s %4s %6s %8s" % ("", "부품", "프레임", "결함부품"))
    for name in NAMES:
        ids = out[name]
        frames = sum(len(frame_files(p)) for p in ids)
        defect = sum(1 for p in ids if parts[p]["defects"])
        print("%-6s %4d %6d %8d" % (name, len(ids), frames, defect))
    print("새로 나눈 부품 %d개, 그대로 둔 부품 %d개 → %s" % (len(new), len(old), SPLITS))
    for name in ("val", "test"):
        if not out[name]:
            print("%s에 부품이 없다. 부품을 더 만들어야 성적을 믿을 수 있다" % name)
    for s in skipped:
        print("나누지 않음 %s" % s)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
