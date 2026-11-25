# 학습 데이터 폴더의 기록 형식이다. capture_dataset.py와 split_dataset.py가 같이 쓴다.
# 각 칸의 뜻과 예시는 dataset.md에 있다.
#
#   dataset/rig.json                 장비 기록. 카메라·링 높이, 조명. 바꾸면 고친다
#   dataset/parts/<부품 ID>.json      부품 기록. 사람이 만들 때 쓴다. 결함의 깊이·폭이 여기 있다
#   dataset/frames/<부품 ID>/NNNNNN.png   원본 프레임 (git에 넣지 않는다)
#   dataset/frames/<부품 ID>/NNNNNN.json  프레임 기록. capture_dataset.py가 쓴다
#   dataset/splits.json              부품 단위 학습·검증·시험 나눔. split_dataset.py가 쓴다

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent
DATASET = ROOT / "dataset"
PARTS = DATASET / "parts"
FRAMES = DATASET / "frames"
RIG = DATASET / "rig.json"
SPLITS = DATASET / "splits.json"

PART_TYPES = ("flat", "cowl", "muffler")
FINISHES = ("gloss", "semi", "matte")
DEFECT_KINDS = ("dent", "scratch", "stain", "uneven")
SOURCES = ("design", "measured")

# 처음 쓰는 부품은 이 틀로 파일을 만든다. null을 채우고, 결함이 없으면 defects를 []로 둔다.
PART_TEMPLATE = {
    "part_id": None,
    "type": None,
    "color": None,
    "finish": None,
    "made": "",
    "defects": [
        {"id": "D1", "kind": "dent", "depth_mm": None, "width_mm": None, "length_mm": None,
         "x_mm": None, "y_mm": None, "source": "design", "note": ""},
    ],
    "note": "",
}

RIG_TEMPLATE = {
    "camera_height_mm": None,
    "ring_height_mm": None,
    "ring_model": None,
    "ring_diameter_mm": None,
    "room_light": None,
    "board": None,
    "note": "",
}


def load_json(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def write_json(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
        f.write("\n")


def part_path(part_id):
    return PARTS / ("%s.json" % part_id)


def list_parts():
    return sorted(p.stem for p in PARTS.glob("*.json"))


def frame_files(part_id):
    return sorted((FRAMES / part_id).glob("*.png"))


def _number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and value > 0


def check_part(part, part_id):
    """부품 기록에서 고칠 것을 글로 돌려준다. 빈 목록이면 쓸 수 있다."""
    problems = []
    if part.get("part_id") != part_id:
        problems.append("part_id가 파일 이름(%s)과 다르다" % part_id)
    if part.get("type") not in PART_TYPES:
        problems.append("type은 %s 가운데 하나다" % ", ".join(PART_TYPES))
    if not isinstance(part.get("color"), str) or not part["color"]:
        problems.append("color가 비어 있다")
    if part.get("finish") not in FINISHES:
        problems.append("finish는 %s 가운데 하나다" % ", ".join(FINISHES))
    defects = part.get("defects")
    if not isinstance(defects, list):
        return problems + ["defects는 목록이다. 결함이 없으면 []"]
    seen = set()
    for d in defects:
        name = d.get("id") or "?"
        if not d.get("id") or d["id"] in seen:
            problems.append("결함 %s: id가 비었거나 겹친다" % name)
        seen.add(d.get("id"))
        kind = d.get("kind")
        if kind not in DEFECT_KINDS:
            problems.append("결함 %s: kind는 %s 가운데 하나다" % (name, ", ".join(DEFECT_KINDS)))
        # 링은 깊이가 아니라 기울기를 본다. 같은 깊이도 폭이 넓으면 덜 꺾이니 함몰은 둘 다 적는다.
        if kind == "dent" and not (_number(d.get("depth_mm")) and _number(d.get("width_mm"))):
            problems.append("결함 %s: 함몰은 depth_mm와 width_mm를 둘 다 적는다" % name)
        if kind == "scratch" and not _number(d.get("length_mm")):
            problems.append("결함 %s: 긁힘은 length_mm를 적는다" % name)
        if d.get("source") not in SOURCES:
            problems.append("결함 %s: source는 design(설계값) 또는 measured(실측)다" % name)
    return problems


def check_rig(rig):
    return ["rig.json의 %s가 비어 있다" % k for k, v in rig.items() if v is None]
