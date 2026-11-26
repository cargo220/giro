#!/usr/bin/env python3
# 판정용 원본 프레임을 모은다. 카메라에 camera-capture.py를 올리고, 받은 프레임을
# dataset/frames/<부품 ID>/ 아래에 PNG와 기록 JSON으로 남긴다.
# 키는 터미널에서 받는다. tmux에서는 capture.sh로 띄운다.
# IDE가 카메라 포트를 잡고 있으면 연결하지 못하니 IDE 연결을 끊고 실행한다.
# ROS는 쓰지 않는다. 포트가 하나라 라인(block_bridge_node.py)과 동시에 돌리지 않는다.
#
#   python3 capture_dataset.py --part P003
#   python3 capture_dataset.py --part P003 --view                  # 미리보기 창도 띄운다
#   python3 capture_dataset.py --part P003 --exposure 3000 7500    # 한 번 저장에 노출마다 한 장
#   python3 capture_dataset.py --part P003 --kind dent --depth 0.6 --width 10
#   python3 capture_dataset.py --part C001 --auto 0.5 --count 40   # 0.5초마다 저장. 벨트로 훑을 때
#
# 키 (터미널에서. --view면 창에서도 받는다)
#   스페이스, s  다음 프레임 저장       a  자동 저장 켜고 끄기
#   k            결함 종류 바꾸기 none → dent → scratch → stain → uneven
#   d  w  l      깊이·폭·길이(mm) 입력  g  각도(°) 입력, [ ]는 한 칸씩
#   p            부품 바꾸기             n  메모 입력
#   x            값 모두 지우기          h  도움말   q  끝
#
# 파일 이름에 그때의 값이 들어간다. 같은 값은 프레임 기록 JSON의 label에도 남는다.
#   부품_번호_종류[_d깊이][_w폭][_l길이][_a각도]_e노출     예: P003_000012_dent_d0.6_w10_e7500.png
# 함몰은 깊이와 폭, 긁힘은 길이가 있어야 저장한다. 링은 깊이가 아니라 기울기를 보기 때문이다.
# 저장은 키를 누른 뒤 새로 찍힌 프레임으로 한다. 손이 화면에 있던 프레임은 들어가지 않는다.
# 처음 쓰는 부품이면 종류·색·광택을 물어 dataset/parts/<부품 ID>.json을 만든다.
# 장비 기록 dataset/rig.json은 없으면 틀을 만들고 멈춘다. 채운 뒤 다시 실행한다.
#
# 프레임은 RGB565 그대로 받아 RGB888 PNG로 남긴다. 5·6비트를 8비트로 늘린 것이라 잃는 값이 없다.
# 미리보기 창의 글자는 창에만 그리고 저장하는 프레임에는 넣지 않는다.

import argparse
import base64
import os
import re
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

from board_script import strip_comments
from term_keys import Terminal
from dataset_meta import (DEFECT_KINDS, FINISHES, FRAMES, PART_ID_RE, PART_TEMPLATE, PART_TYPES, RIG,
                          RIG_TEMPLATE, check_rig, load_json, part_path, write_json)

BOARD_SCRIPT = Path(__file__).with_name("camera-capture.py")
# 표본 픽셀과 이만큼 넘게 다르면 바이트 순서를 믿지 않는다. 5비트를 8비트로 늘리는 반올림 차이보다 크다.
SAMPLE_TOL = 8.0
WB_FAIL = "채널 게인을 쓰지 못함"
KINDS = ("none",) + DEFECT_KINDS
# 파일 이름에 넣는 값과 그 앞글자
NAME_CODES = (("depth_mm", "d"), ("width_mm", "w"), ("length_mm", "l"), ("angle_deg", "a"))
HELP = ("스페이스 저장  a 자동  k 종류  d 깊이  w 폭  l 길이  g 각도  [ ] 각도±  "
        "p 부품  n 메모  x 지움  h 도움말  q 끝")


class FrameParser:
    """camera-capture.py의 print() 줄을 모아 한 프레임씩 내놓는다."""

    def __init__(self):
        self.buf = ""
        self.cur = None
        self.broken = 0

    def feed(self, text):
        """(완성된 프레임 목록, 프레임이 아닌 줄 목록)을 돌려준다."""
        frames, messages = [], []
        self.buf += text
        # 줄 가운데서 끊긴 마지막 조각은 다음 읽기까지 남긴다.
        *lines, self.buf = self.buf.split("\n")
        for line in lines:
            line = line.strip()
            if not line:
                continue
            head, _, rest = line.partition(" ")
            if head == "FRAME":
                if self.cur is not None:
                    self.broken += 1
                seq, w, h, expo, gain = rest.split()
                # 보드는 찍자마자 FRAME 줄을 낸다. 이 줄이 온 때를 찍힌 때로 본다.
                self.cur = {"seq": int(seq), "width": int(w), "height": int(h),
                            "exposure_us": int(expo), "gain_db": float(gain), "t": time.time(),
                            "samples": [], "rows": [None] * int(h)}
            elif head == "PX" and self.cur is not None:
                self.cur["samples"].append(tuple(int(v) for v in rest.split()))
            elif head == "ROW" and self.cur is not None:
                y, data = rest.split(" ", 1)
                row = base64.b64decode(data)
                y = int(y)
                if len(row) == self.cur["width"] * 2 and 0 <= y < self.cur["height"]:
                    self.cur["rows"][y] = row
            elif head == "END":
                cur, self.cur = self.cur, None
                if cur is None or cur["seq"] != int(rest) or any(r is None for r in cur["rows"]):
                    self.broken += 1
                    continue
                cur["raw"] = b"".join(cur.pop("rows"))
                frames.append(cur)
            else:
                messages.append(line)
        return frames, messages


def rgb565_to_rgb(raw, width, height, order):
    px = np.frombuffer(raw, "<u2" if order == "little" else ">u2").astype(np.uint32)
    r = ((px >> 11) & 0x1F) * 255 // 31
    g = ((px >> 5) & 0x3F) * 255 // 63
    b = (px & 0x1F) * 255 // 31
    return np.stack([r, g, b], axis=-1).astype(np.uint8).reshape(height, width, 3)


def decode(frame):
    """바이트 순서 둘을 다 풀어 보고, 보드가 get_pixel()로 읽은 표본에 맞는 쪽을 고른다."""
    best = None
    for order in ("little", "big"):
        rgb = rgb565_to_rgb(frame["raw"], frame["width"], frame["height"], order)
        if frame["samples"]:
            err = float(np.mean([np.abs(rgb[y, x].astype(int) - (r, g, b)).mean()
                                 for x, y, r, g, b in frame["samples"]]))
        else:
            err = float("nan")
        if best is None or err < best[2]:
            best = (rgb, order, err)
    return best


def frame_index(path):
    """이름에서 여섯 자리 번호를 읽는다. 000001.png도, P003_000001_dent_….png도 읽는다."""
    for token in path.stem.split("_"):
        if len(token) == 6 and token.isdigit():
            return int(token)
    return 0


def next_index(folder):
    return max((frame_index(p) for p in folder.glob("*.png")), default=0) + 1


def frame_name(part_id, index, label, exposure_us):
    tokens = [part_id, "%06d" % index, label["kind"]]
    tokens += ["%s%g" % (code, label[key]) for key, code in NAME_CODES if label[key] is not None]
    tokens.append("e%d" % exposure_us)
    return "_".join(tokens)


def label_problem(label):
    if label["kind"] == "dent" and (label["depth_mm"] is None or label["width_mm"] is None):
        return "함몰은 깊이(d)와 폭(w)을 넣어야 저장한다"
    if label["kind"] == "scratch" and label["length_mm"] is None:
        return "긁힘은 길이(l)를 넣어야 저장한다"
    return None


def board_source(exposures):
    """보드 스크립트의 노출 줄을 바꾸고 주석을 뺀다. 잠그는 채널 값도 함께 돌려준다.
    게인과 노출은 보드가 프레임마다 실제 값을 알려 주므로 여기서 읽지 않는다."""
    text = BOARD_SCRIPT.read_text(encoding="utf-8")
    value = "(%s,)" % ", ".join(str(int(e)) for e in exposures)
    text, n = re.subn(r"^EXPOSURES_US = .*$", "EXPOSURES_US = %s" % value, text, flags=re.M)
    if n != 1:
        raise SystemExit("%s에서 EXPOSURES_US 줄을 찾지 못했다" % BOARD_SCRIPT.name)
    rgb = [float(v) for v in re.search(r"^RGB_GAIN_DB = \((.+)\)$", text, re.M).group(1).split(",")]
    return strip_comments(text), rgb


def ready_rig():
    """장비 기록을 읽는다. 없으면 틀을 만들고, 빈 칸이 있으면 알려 주고 None을 돌려준다."""
    if not RIG.exists():
        write_json(RIG, RIG_TEMPLATE)
        print("장비 기록 틀을 만들었다: %s  채운 뒤 다시 실행한다" % RIG)
        return None
    rig = load_json(RIG)
    problems = check_rig(rig)
    for p in problems:
        print(p)
    return None if problems else rig


def pick(answer, choices):
    """앞 몇 글자만 쳐도 하나로 정해지면 그 값을 돌려준다. c → cowl, g → gloss."""
    answer = answer.strip().lower()
    found = [c for c in choices if answer and c.startswith(answer)]
    return found[0] if len(found) == 1 else None


def ensure_part(part_id, ask):
    """부품 기록을 읽고, 없거나 종류·색·광택이 비었으면 물어 채운다. (됐는지, 알릴 말)을 돌려준다.
    ask가 None이면 묻지 않는다(터미널이 없을 때)."""
    if not PART_ID_RE.match(part_id):
        return False, "부품 ID는 영문·숫자·하이픈만 쓴다: %s" % part_id
    path = part_path(part_id)
    part = load_json(path) if path.exists() else dict(PART_TEMPLATE, part_id=part_id)
    questions = (("type", "종류 %s" % "/".join(PART_TYPES), PART_TYPES),
                 ("color", "색 (예: red)", None),
                 ("finish", "광택 %s" % "/".join(FINISHES), FINISHES))
    changed = not path.exists()
    for key, prompt, choices in questions:
        while not part.get(key) or (choices and part[key] not in choices):
            if ask is None:
                return False, "부품 기록 %s의 %s가 비었다. 터미널에서 실행하거나 파일을 채운다" % (path, key)
            answer = ask("%s %s: " % (part_id, prompt))
            if answer is None:
                return False, "부품 바꾸기 취소"
            part[key] = pick(answer, choices) if choices else answer.strip()
            changed = True
    if changed:
        write_json(path, part)
        return True, "부품 기록 저장: %s (%s, %s, %s)" % (path.name, part["type"], part["color"], part["finish"])
    return True, "부품 %s (%s, %s, %s)" % (part_id, part["type"], part["color"], part["finish"])


def plain_ask(prompt):
    """키 입력 모드에 들어가기 전에 쓰는 줄 입력. Ctrl+D, Ctrl+C는 취소로 본다."""
    try:
        return input(prompt)
    except (EOFError, KeyboardInterrupt):
        print()
        return None


class Headless:
    """터미널이 없을 때. 키를 받지 않고 --auto로만 저장한다."""

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        pass

    def say(self, msg):
        print(msg, flush=True)

    def show(self, status):
        pass

    def ask(self, prompt):
        return None

    def key(self, timeout):
        time.sleep(timeout)
        return None


class Capture:
    def __init__(self, args, term, rig, rgb_gain, fw):
        self.args = args
        self.term = term
        self.rig = rig
        self.rgb_gain = rgb_gain
        self.fw = fw
        self.part = args.part
        self.label = {"kind": args.kind, "depth_mm": args.depth, "width_mm": args.width,
                      "length_mm": args.length, "angle_deg": args.angle}
        self.note = args.note
        self.auto = args.auto > 0
        self.interval = args.auto if args.auto > 0 else 1.0
        self.next_auto = 0.0
        self.pending = None   # 저장 요청. 키를 누른 뒤 찍힌 프레임을 노출마다 한 장씩 받는다
        self.last = None      # 마지막으로 받은 (rgb, frame)
        self.fresh = False
        self.saved = 0
        self.wb_ok = True
        self.arrivals = []

    def request(self):
        """다음 프레임 저장을 건다. 값이 모자라 걸지 못하면 False."""
        problem = label_problem(self.label)
        if problem:
            self.term.say(problem)
            return False
        if self.pending is not None:
            return True
        self.pending = {"after": time.time(), "need": set(self.args.exposure),
                        "part": self.part, "label": dict(self.label), "note": self.note}
        return True

    def on_frame(self, frame):
        rgb, order, err = decode(frame)
        if not err <= SAMPLE_TOL:
            self.term.say("프레임 %d: 표본 픽셀과 %.1f 다르다. 바이트 순서를 믿지 않는다" % (frame["seq"], err))
        self.last = (rgb, frame)
        self.fresh = True
        now = time.time()
        self.arrivals = [t for t in self.arrivals if t > now - 5.0] + [now]
        req = self.pending
        if req is None or frame["t"] <= req["after"]:
            return
        # 보드가 알려 준 노출은 센서가 맞춘 값이라 요청과 조금 다를 수 있다. 가장 가까운 요청에 넣는다.
        want = min(self.args.exposure, key=lambda e: abs(e - frame["exposure_us"]))
        if want in req["need"]:
            req["need"].discard(want)
            self.save(rgb, frame, order, err, req)
        if not req["need"]:
            self.pending = None

    def save(self, rgb, frame, order, err, req):
        out_dir = FRAMES / req["part"]
        out_dir.mkdir(parents=True, exist_ok=True)
        name = frame_name(req["part"], next_index(out_dir), req["label"], frame["exposure_us"])
        cv2.imwrite(str(out_dir / (name + ".png")), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        write_json(out_dir / (name + ".json"), {
            "part_id": req["part"],
            "file": name + ".png",
            "time": datetime.now().astimezone().isoformat(timespec="seconds"),
            "board_seq": frame["seq"],
            "width": frame["width"],
            "height": frame["height"],
            "pixformat": "RGB565",
            "byte_order": order,
            "sample_error": None if err != err else round(err, 2),
            "exposure_us": frame["exposure_us"],
            "gain_db": frame["gain_db"],
            "rgb_gain_db": self.rgb_gain if self.wb_ok else None,
            "firmware": list(self.fw) if self.fw else None,
            "label": req["label"],
            "belt_mm": self.args.belt_mm,
            "rig": self.rig,
            "note": req["note"],
        })
        self.saved += 1
        self.term.say("저장 %s.png" % name)

    def ask_mm(self, key, prompt):
        text = self.term.ask("%s (빈칸이면 지움): " % prompt)
        if text is None:
            return
        text = text.strip()
        if not text:
            self.label[key] = None
            return
        try:
            value = float(text)
        except ValueError:
            self.term.say("숫자가 아니다: %s" % text)
            return
        if key != "angle_deg" and value <= 0:
            self.term.say("0보다 커야 한다: %s" % text)
            return
        self.label[key] = value

    def clear_values(self):
        self.label.update(kind="none", depth_mm=None, width_mm=None, length_mm=None)

    def on_key(self, key):
        """키 하나를 처리한다. 끝내야 하면 False."""
        if key in (" ", "s"):
            self.request()
        elif key == "a":
            self.auto = not self.auto
            self.next_auto = 0.0
            if self.auto and label_problem(self.label):
                self.auto = False
                self.term.say(label_problem(self.label))
        elif key == "k":
            self.label["kind"] = KINDS[(KINDS.index(self.label["kind"]) + 1) % len(KINDS)]
            if self.label["kind"] == "none":
                self.clear_values()
        elif key == "d":
            self.ask_mm("depth_mm", "깊이 mm")
        elif key == "w":
            self.ask_mm("width_mm", "폭(지름) mm")
        elif key == "l":
            self.ask_mm("length_mm", "길이 mm")
        elif key == "g":
            self.ask_mm("angle_deg", "각도 °")
        elif key in ("[", "]"):
            step = self.args.angle_step if key == "]" else -self.args.angle_step
            self.label["angle_deg"] = (self.label["angle_deg"] or 0.0) + step
        elif key == "n":
            text = self.term.ask("메모 (빈칸이면 지움): ")
            if text is not None:
                self.note = text.strip()
        elif key == "x":
            self.clear_values()
            self.label["angle_deg"] = None
            self.note = ""
        elif key == "p":
            self.change_part()
        elif key in ("h", "?"):
            self.term.say(HELP)
        elif key == "q":
            return False
        return True

    def change_part(self):
        text = self.term.ask("부품 ID (지금 %s): " % self.part)
        if not text or not text.strip() or text.strip() == self.part:
            return
        ok, msg = ensure_part(text.strip(), self.term.ask)
        self.term.say(msg)
        if ok:
            # 부품이 바뀌면 결함도 바뀐다. 앞 부품의 값이 새 부품 이름에 붙지 않게 지운다.
            self.part = text.strip()
            self.clear_values()
            self.label["angle_deg"] = self.args.angle
            self.pending = None

    def values_text(self):
        parts = [self.label["kind"]]
        parts += ["%s%g" % (code, self.label[key]) for key, code in NAME_CODES
                  if self.label[key] is not None]
        return " ".join(parts)

    def status(self):
        fps = 0.0
        if len(self.arrivals) > 1:
            fps = (len(self.arrivals) - 1) / (self.arrivals[-1] - self.arrivals[0])
        expo = self.last[1]["exposure_us"] if self.last else 0
        text = "%s | %s | E%d %.1ffps | 저장 %d | 자동 %s" % (
            self.part, self.values_text(), expo, fps, self.saved,
            "%.1fs" % self.interval if self.auto else "끔")
        if self.note:
            text += " | 메모 있음"
        if self.pending is not None:
            text += " | 저장 대기"
        return text

    def preview(self):
        rgb, frame = self.last
        view = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        view = cv2.resize(view, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST)
        # Hershey 글꼴은 한글을 못 그려서 창 글자는 영문으로 둔다.
        text = "%s  %s  E%d  saved %d%s" % (self.part, self.values_text(), frame["exposure_us"],
                                            self.saved, "  AUTO" if self.auto else "")
        for color, thick in (((0, 0, 0), 3), ((255, 255, 255), 1)):
            cv2.putText(view, text, (8, view.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        color, thick, cv2.LINE_AA)
        return view


def main():
    ap = argparse.ArgumentParser(description="판정용 원본 프레임 수집")
    ap.add_argument("--part", required=True, help="부품 ID. 영문·숫자·하이픈")
    ap.add_argument("--port", default="auto")
    ap.add_argument("--protocol", default="auto", choices=("auto", "legacy", "new"))
    ap.add_argument("--exposure", type=int, nargs="+", default=[7500], help="노출 µs, 여럿이면 번갈아")
    ap.add_argument("--auto", type=float, default=0.0, help="이 초마다 저장. 0이면 키로만")
    ap.add_argument("--count", type=int, default=0, help="이만큼 저장하면 끝. 0이면 끝없이")
    ap.add_argument("--kind", default="none", choices=KINDS, help="결함 종류 첫 값")
    ap.add_argument("--depth", type=float, help="깊이 mm 첫 값")
    ap.add_argument("--width", type=float, help="폭(지름) mm 첫 값")
    ap.add_argument("--length", type=float, help="길이 mm 첫 값")
    ap.add_argument("--angle", type=float, help="각도(°) 첫 값. 머플러")
    ap.add_argument("--angle-step", type=float, default=15.0, help="[ ] 키 한 칸")
    ap.add_argument("--belt-mm", type=float, help="벨트 위치를 손으로 적을 때")
    ap.add_argument("--note", default="", help="첫 메모")
    ap.add_argument("--view", action="store_true", help="미리보기 창도 띄운다")
    args = ap.parse_args()

    interactive = sys.stdin.isatty()
    if not interactive and args.auto <= 0:
        print("키를 받을 터미널이 없다. 터미널에서 실행하거나 --auto 초 를 준다")
        return 2
    if args.view and not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
        print("화면이 없어 --view를 쓸 수 없다")
        return 2
    if args.kind == "none" and (args.depth or args.width or args.length):
        print("--kind 없이 준 깊이·폭·길이는 쓰지 않는다")
        args.depth = args.width = args.length = None
    problem = label_problem({"kind": args.kind, "depth_mm": args.depth, "width_mm": args.width,
                             "length_mm": args.length})
    if problem and not interactive:
        # 키로 값을 고칠 수 없으니 한 장도 저장하지 못한 채 돌게 된다. 시작 전에 멈춘다.
        print(problem.replace("(d)", "(--depth)").replace("(w)", "(--width)").replace("(l)", "(--length)"))
        return 2
    rig = ready_rig()
    if rig is None:
        return 1
    ok, msg = ensure_part(args.part, plain_ask if interactive else None)
    print(msg)
    if not ok:
        return 1
    source, rgb_gain = board_source(args.exposure)

    # 카메라 모듈은 여기서 불러온다. 기록이 덜 찼을 때는 openmv·pygame을 불러오지 않는다.
    from camera_link import REBOOT_WAIT, find_camera_port, open_camera
    port = find_camera_port() if args.port in ("", "auto") else args.port

    class Log:
        info = staticmethod(print)
    try:
        cam, fw = open_camera(port, args.protocol, Log)
    except Exception as exc:
        hint = " (OpenMV IDE가 포트를 쓰고 있으면 연결을 끊는다)" if "busy" in str(exc) else ""
        print("카메라를 열지 못했다: %s%s" % (exc, hint))
        return 1

    parser = FrameParser()
    last_error = 0.0
    with (Terminal() if interactive else Headless()) as term:
        cap = Capture(args, term, rig, rgb_gain, fw)
        try:
            cam.stop()
            deadline = time.time() + REBOOT_WAIT
            while time.time() < deadline:
                cam.read_stdout()
                time.sleep(0.05)
            cam.exec(source)
            # IDE 화면 전송은 끈다. 켜 두면 보드가 프레임마다 JPEG를 만드느라 느려진다.
            cam.streaming(False)
            term.say("카메라 %s에서 %s 실행, 노출 %s µs" % (
                port, BOARD_SCRIPT.name, " / ".join(map(str, args.exposure))))
            if interactive:
                term.say(HELP)
            while True:
                try:
                    text = cam.read_stdout()
                except IOError as exc:
                    if time.time() - last_error > 1.0:
                        last_error = time.time()
                        term.say("카메라 응답 오류: %s" % exc)
                    text = ""
                if text:
                    frames, messages = parser.feed(text)
                    for m in messages:
                        if m == WB_FAIL:
                            cap.wb_ok = False
                        term.say("카메라: %s" % m)
                    for frame in frames:
                        cap.on_frame(frame)
                if args.count and cap.saved >= args.count:
                    break
                now = time.time()
                if cap.auto and cap.pending is None and now >= cap.next_auto:
                    if cap.request():
                        cap.next_auto = now + cap.interval
                    else:
                        cap.auto = False
                keys = []
                key = term.key(0 if text else 0.005)
                if key:
                    keys.append(key)
                if args.view and cap.last is not None:
                    if cap.fresh:
                        cv2.imshow("capture", cap.preview())
                        cap.fresh = False
                    code = cv2.waitKey(1)
                    if code != -1:
                        keys.append(chr(code & 0xFF))
                if not all(cap.on_key(k) for k in keys):
                    break
                term.show(cap.status())
        except KeyboardInterrupt:
            pass
        finally:
            try:
                cam.stop()
            except Exception as exc:
                print("카메라 정리 중 오류: %s" % exc, file=sys.stderr)
            finally:
                cam.disconnect()
            if args.view:
                cv2.destroyAllWindows()
    if parser.broken:
        print("깨져서 버린 프레임 %d개" % parser.broken)
    print("모두 %d장 저장" % cap.saved)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
