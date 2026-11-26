#!/usr/bin/env python3
# WLKATA Mirobot을 다기능 컨트롤러 USB로 움직인다. 벨트(D축)와 같은 포트다.
# 포트는 한 프로그램만 열 수 있다. 라인(conveyor_node.py, giro_line.sh)이 돌 때는 쓰지 않는다.
#
#   python3 mirobot.py probe              # 어느 주소의 팔이 답하는지 본다. 팔을 움직이지 않는다
#   python3 mirobot.py status             # config/arms.json의 팔마다 상태 한 줄
#
# 명령은 WLKATA 공식 SDK(wlkatapython, MIT)의 Mirobot_UART를 따른다.
#   상태          ?
#   원점 복귀     $H                          conveyor.py가 이 셀에서 써 온 명령이다
#   흡착          M3 S1000, 불기 M3 S500, 끄기 M3 S0
#   직교 이동     M20 G90|G91 G00|G01 X Y Z A B C F    X Y Z는 mm, A B C는 끝단 회전(°)
#   7축(벨트)     M21 G91 G01 D F
# 다기능 컨트롤러 USB(115200)로 붙은 팔은 접두어 없이 부른다. 여러 팔을 RS485로 부를 때는
# SDK처럼 "@주소"를 앞에 붙인다. 두 번째 팔이 어느 주소로 답하는지는 probe로 확인해 arms.json에 적는다.
#
# 상태 줄의 Angle(ABCDXYZ)는 A B C D X Y Z 순서다. 관절로는 J4 J5 J6, 7축, J1 J2 J3이다.

import argparse
import json
import re
import sys
import threading
import time
from pathlib import Path

import serial

from conveyor import EXBOX_BAUD, EXBOX_MOVE_FEED, find_belt_port

ROOT = Path(__file__).resolve().parent
ARMS_CONFIG = ROOT / "config" / "arms.json"
NUM = r"([-\d.]+)"
STATUS_RE = re.compile(
    r"<(\w+),Angle\(ABCDXYZ\):" + ",".join([NUM] * 7) +
    r",Cartesian coordinate\(XYZ RxRyRz\):" + ",".join([NUM] * 6) +
    r",Pump PWM:" + NUM + r",Valve PWM:" + NUM)
PUMP = {"on": "M3 S1000", "blow": "M3 S500", "off": "M3 S0"}
# 원점 복귀 전(Alarm), 일시 정지(Hold), 원점 복귀 중(Home)에는 이동을 보내지 않는다.
BLOCKED = ("Alarm", "Hold", "Home")
# 팔 설정 파일이 없을 때 쓰는 값. 접두어 없이 부르는 팔 하나다.
DEFAULT_CONFIG = {
    "port": "auto",
    "baud": EXBOX_BAUD,
    "arms": {"arm1": {"address": None, "label": "접두어 없이 답하는 팔 (벨트 D축이 붙은 팔)"}},
    "belt_arm": "arm1",
    "feed_mm_min": 2000,
    "z_min_mm": None,
    "safe_z_mm": None,
}


def parse_status(line):
    """상태 줄을 사전으로 푼다. 상태 줄이 아니면 None이다."""
    m = STATUS_RE.match(line)
    if not m:
        return None
    v = [float(x) for x in m.groups()[1:]]
    a, b, c, d, x, y, z = v[0:7]
    return {"state": m.group(1),
            "joints": {"J1": x, "J2": y, "J3": z, "J4": a, "J5": b, "J6": c},
            "d": d,
            "pose": dict(zip(("x", "y", "z", "a", "b", "c"), v[7:13])),
            "pump": int(v[13])}


def load_config(path=None):
    path = path or ARMS_CONFIG
    if not path.exists():
        return dict(DEFAULT_CONFIG)
    with open(path, encoding="utf-8") as f:
        return dict(DEFAULT_CONFIG, **json.load(f))


def save_config(cfg, path=None):
    path = path or ARMS_CONFIG
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(cfg, f, ensure_ascii=False, indent=2)
        f.write("\n")


def open_serial(port, baud):
    return serial.Serial(port, baud, timeout=0.02)


class Controller:
    """다기능 컨트롤러 포트 하나. 여러 팔이 이 포트를 같이 쓴다.
    답에는 어느 팔의 답인지 적혀 있지 않아서, 상태는 한 팔씩 묻고 그 답을 기다린다."""

    def __init__(self, ser):
        self.ser = ser
        self.lock = threading.Lock()
        self.buf = b""
        self.messages = []   # 상태 줄과 ok가 아닌 줄. 오류나 원점 복귀 안내가 여기 쌓인다

    def send(self, address, command):
        prefix = "" if address is None else "@%d" % address
        with self.lock:
            self.ser.write((prefix + command + "\r\n").encode("utf-8"))

    def _lines(self):
        self.buf += self.ser.read(4096)
        *lines, self.buf = self.buf.split(b"\n")
        return [raw.decode("utf-8", "replace").strip() for raw in lines]

    def query(self, address, timeout=0.5):
        """상태를 묻고 답을 사전으로 돌려준다. 답이 없으면 None이다."""
        self.drain()
        self.send(address, "?")
        deadline = time.time() + timeout
        while time.time() < deadline:
            for line in self._lines():
                status = parse_status(line)
                if status is not None:
                    return status
                if line and line != "ok":
                    self.messages.append(line)
        return None

    def drain(self):
        for line in self._lines():
            if line and line != "ok" and parse_status(line) is None:
                self.messages.append(line)

    def pop_messages(self):
        out, self.messages = self.messages, []
        return out


class Arm:
    def __init__(self, ctrl, name, address, label=""):
        self.ctrl = ctrl
        self.name = name
        self.address = address
        self.label = label

    def send(self, command):
        self.ctrl.send(self.address, command)

    def status(self):
        return self.ctrl.query(self.address)

    def jog(self, dx=0.0, dy=0.0, dz=0.0, dc=0.0, feed=2000):
        # 상대 이동이다. A, B는 0으로 두어 흡착컵이 아래를 본 채로 움직인다.
        self.send("M20 G91 G01 X%.2f Y%.2f Z%.2f A0 B0 C%.2f F%d" % (dx, dy, dz, dc, feed))

    def move_to(self, pose, feed=2000):
        self.send("M20 G90 G01 X%.2f Y%.2f Z%.2f A%.2f B%.2f C%.2f F%d" % (
            pose["x"], pose["y"], pose["z"], pose["a"], pose["b"], pose["c"], feed))

    def pump(self, mode):
        self.send(PUMP[mode])

    def home(self):
        self.send("$H")

    def belt(self, mm, feed=EXBOX_MOVE_FEED):
        self.send("M21 G91 G01 D%.2f F%d" % (mm, feed))


def open_arms(cfg):
    """(컨트롤러, {이름: Arm})을 돌려준다."""
    port = find_belt_port() if cfg["port"] in ("", "auto") else cfg["port"]
    ctrl = Controller(open_serial(port, cfg["baud"]))
    arms = {name: Arm(ctrl, name, a.get("address"), a.get("label", ""))
            for name, a in cfg["arms"].items()}
    return ctrl, arms, port


def describe(status):
    p = status["pose"]
    return "%-5s X %7.2f Y %7.2f Z %7.2f C %7.2f  흡착 %s" % (
        status["state"], p["x"], p["y"], p["z"], p["c"], "켬" if status["pump"] else "끔")


def probe(args, cfg):
    port = find_belt_port() if args.port in ("", "auto") else args.port
    ctrl = Controller(open_serial(port, args.baud))
    print("포트 %s, %d bps. 상태(?)만 묻고 팔은 움직이지 않는다" % (port, args.baud))
    found = []
    states = set()
    for address in [None] + list(range(args.first, args.last + 1)):
        status = ctrl.query(address, timeout=args.timeout)
        name = "접두어 없음" if address is None else "@%d" % address
        if status is None:
            print("  %-10s 답 없음" % name)
        else:
            found.append(address)
            states.add(status["state"])
            print("  %-10s %s" % (name, describe(status)))
    for line in ctrl.pop_messages():
        print("  컨트롤러: %s" % line)
    ctrl.ser.close()
    if not found:
        print("답한 팔이 없다. 컨트롤러 전원과 USB, --baud를 확인한다")
        return 1
    # 접두어 없이 보낸 ?에 다기능 컨트롤러가 한 팔 대신 답하는지, 주소마다 다른 팔이 답하는지는
    # 위치 값으로 가린다. 두 팔을 다른 자세로 둔 뒤 돌리면 구별된다.
    print("답한 주소: %s" % ", ".join("접두어 없음" if a is None else "@%d" % a for a in found))
    print("config/arms.json의 arms에 팔마다 address를 적는다. 접두어 없음은 null이다")
    if "Hold" in states:
        print("Hold(일시 정지)다. 이때 보낸 명령은 쌓였다가 풀리는 순간 실행된다."
              " 컨트롤러에서 일시 정지를 풀고 다시 probe 한다. 그 전에는 명령을 보내지 않는다")
    if "Alarm" in states:
        print("Alarm이다. 팔 주변을 비우고 모듈 OK 키를 2초 눌러 원점 복귀를 한다")
    return 0


def show_status(args, cfg):
    ctrl, arms, port = open_arms(cfg)
    for name, arm in arms.items():
        status = arm.status()
        print("%-6s %s" % (name, "답 없음" if status is None else describe(status)))
    for line in ctrl.pop_messages():
        print("컨트롤러: %s" % line)
    ctrl.ser.close()
    return 0


def main():
    ap = argparse.ArgumentParser(description="WLKATA Mirobot 상태 확인")
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("probe", help="주소마다 상태를 물어 답하는 팔을 찾는다")
    p.add_argument("--port", default="auto")
    p.add_argument("--baud", type=int, default=EXBOX_BAUD)
    p.add_argument("--first", type=int, default=1)
    p.add_argument("--last", type=int, default=4)
    p.add_argument("--timeout", type=float, default=0.5)
    sub.add_parser("status", help="arms.json의 팔마다 상태 한 줄")
    args = ap.parse_args()
    cfg = load_config()
    try:
        return probe(args, cfg) if args.cmd == "probe" else show_status(args, cfg)
    except serial.SerialException as exc:
        hint = " (라인이나 다른 프로그램이 포트를 쓰고 있으면 끈다)" if "busy" in str(exc).lower() else ""
        print("포트를 열지 못했다: %s%s" % (exc, hint), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
