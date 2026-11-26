#!/usr/bin/env python3
# 터미널에서 키로 Mirobot을 움직이고, 집는 자리·놓는 자리를 이름 붙여 저장한다.
# tmux에서는 arm.sh로 띄운다. 라인(giro_line.sh)과 같은 포트를 쓰니 동시에 돌리지 않는다.
#
#   python3 arm_teleop.py
#   python3 arm_teleop.py --arm arm2
#
# 키
#   1 2        조종할 팔 고르기 (config/arms.json에 적힌 순서)
#   w s        X +/-                 a d   Y +/-
#   r f        Z +/- (위·아래)        z c   끝단 회전 C +/-
#   + -        한 칸 크기 바꾸기 1 → 5 → 10 → 20 (mm, °)
#   p          흡착 켜고 끄기         b     불어서 놓기 (0.5초 불고 끈다)
#   , .        벨트 -/+ 10 mm. - 가 카메라 쪽이다
#   스페이스   지금 자리를 이름 붙여 저장       g  저장한 자리로 가기     l  저장한 자리 목록
#   t          지금 Z를 안전 높이로 정하기. 저장한 자리로 갈 때 이 높이로 올라 지나간다
#   h → y      원점 복귀. 팔 전체가 움직여 y로 한 번 더 확인받는다
#   q          끝. 흡착은 켜 둔 그대로 둔다. 부품을 들고 있으면 먼저 내려놓는다
#
# 움직이는 키는 팔이 Idle일 때만 받는다. 키를 누르고 있어도 이동이 쌓이지 않는다.
# 저장한 자리로 갈 때는 안전 높이로 올라 → 목표 위로 옮기고 → 내려간다. 안전 높이가 없으면 가지 않는다.
# 자리는 config/arm_poses.json에 팔마다 저장한다. 직교 좌표와 관절 각도를 같이 남긴다.
# 비상 정지는 컨트롤러의 정지 버튼으로 한다. 이 도구는 정지 명령을 따로 보내지 않는다.
# 컨트롤러가 명령에 SDing으로 답하면 명령을 쌓아 둔 것이라, 그 뒤로는 아무 명령도 보내지 않는다.

import argparse
import json
import sys
import time
from datetime import datetime

import serial

import mirobot as mb
from term_keys import Terminal, width

POSES = mb.ROOT / "config" / "arm_poses.json"
STEPS = (1.0, 5.0, 10.0, 20.0)
BELT_STEP = 10.0
POLL_SEC = 0.2
# 이동을 보낸 직후에는 상태가 아직 Idle로 올 수 있다. 이 시간 동안은 Idle이어도 움직이는 중으로 본다.
GRACE_SEC = 0.4
BLOW_SEC = 0.5
# 이동·흡착을 보내고 이만큼 지나도 상태가 그대로면 받지 않은 것으로 알린다.
CHECK_SEC = 1.5
# 컨트롤러가 명령에 이 말로 답하면 바로 실행하지 않고 쌓아 둔 것이다. 2026-11-26 실물에서 흡착 켜기가
# 나중에 실행되고 끄기는 먹지 않아 전원을 꺼야 했다. 이 답을 받으면 그 뒤로는 아무 명령도 보내지 않는다.
HALT_REPLIES = ("SDing",)
JOG = {"w": (1, 0, 0, 0), "s": (-1, 0, 0, 0), "a": (0, 1, 0, 0), "d": (0, -1, 0, 0),
       "r": (0, 0, 1, 0), "f": (0, 0, -1, 0), "z": (0, 0, 0, 1), "c": (0, 0, 0, -1)}
# 키 설명은 여기 한 곳에만 둔다. (키, 짧은 이름, 설명). 짧은 이름은 조종 칸의 한 줄 도움말,
# 설명은 arm.sh 설명 칸(--keys)이 쓴다.
KEY_HELP = (
    ("1 2", "팔", "조종할 팔 고르기"),
    ("w s", "X", "X +/-"),
    ("a d", "Y", "Y +/-"),
    ("r f", "Z", "Z +/- (위·아래)"),
    ("z c", "회전", "끝단 회전 C +/-"),
    ("+ -", "칸", "한 칸 크기 1 → 5 → 10 → 20"),
    ("p", "흡착", "흡착 켜기/끄기"),
    ("b", "놓기", "0.5초 불어서 놓기"),
    (", .", "벨트", "벨트 -/+ 10 mm (- 가 카메라 쪽)"),
    ("스페이스", "저장", "지금 자리를 이름 붙여 저장"),
    ("g", "가기", "저장한 자리로 가기 (올라 → 옮기고 → 내려감)"),
    ("l", "목록", "저장한 자리 목록"),
    ("t", "안전높이", "지금 Z를 안전 높이로 정하기"),
    ("h → y", "원점", "원점 복귀 (y로 한 번 더 확인)"),
    ("?", "도움말", "이 목록을 조종 칸에 한 줄로"),
    ("q", "끝", "끝내기"),
)
HELP = "  ".join("%s %s" % (k, short) for k, short, _ in KEY_HELP)


def load_poses():
    if not POSES.exists():
        return {}
    with open(POSES, encoding="utf-8") as f:
        return json.load(f)


def save_poses(poses):
    POSES.parent.mkdir(parents=True, exist_ok=True)
    with open(POSES, "w", encoding="utf-8") as f:
        json.dump(poses, f, ensure_ascii=False, indent=2)
        f.write("\n")


class Teleop:
    def __init__(self, cfg, ctrl, arms, start, term):
        self.cfg = cfg
        self.ctrl = ctrl
        self.arms = arms
        self.order = list(arms)
        self.arm = arms[start]
        self.term = term
        self.step = 5.0
        self.status = None
        self.busy_until = 0.0
        self.blow_off_at = None
        self.asking_home = False
        self.check = None     # 보낸 명령이 상태에 반영됐는지 볼 것
        self.halted = False   # 컨트롤러가 명령을 쌓아 두는 답을 했다. 다시 띄울 때까지 보내지 않는다
        self.poses = load_poses()

    def poll(self):
        self.status = self.arm.status()
        for line in self.ctrl.pop_messages():
            self.term.say("컨트롤러: %s" % line)
            if line in HALT_REPLIES and not self.halted:
                self.halted = True
                self.term.say("컨트롤러가 %s로 답했다. 명령을 바로 실행하지 않고 쌓아 뒀다가 나중에 실행할 수 있어"
                              " 이제부터 아무 명령도 보내지 않는다. 컨트롤러 전원을 껐다 켜고 다시 띄운다 (arm.md)" % line)
        self.verify()

    def stopped(self):
        """명령을 막아 둔 상태면 알리고 True."""
        if self.halted:
            self.term.say("명령을 막아 두었다. 컨트롤러 전원을 껐다 켜고 다시 띄운다")
        return self.halted

    def verify(self):
        """컨트롤러는 명령에 ok도 답하지 않는다. 그래서 상태의 자리·흡착 값이 바뀌었는지로 받았는지 본다."""
        c = self.check
        if c is None or self.status is None or c["arm"] != self.arm.name:
            return
        if self.status["state"] in ("Run", "Home"):
            self.check = None   # 움직이기 시작했다
            return
        if time.time() < c["at"]:
            return
        self.check = None
        now = self.status[c["field"]]
        if isinstance(now, dict):
            same = max(abs(now[k] - c["before"][k]) for k in now) < 0.05
        else:
            same = abs(now - c["before"]) < 0.05
        if same:
            self.term.say("%s: 보냈는데 상태가 그대로다. 컨트롤러가 명령을 받지 않았거나 이 팔이 이 포트에 없다"
                          " (arm.md의 명령이 안 먹을 때)" % c["what"])

    def ready(self, arm=None):
        """이동을 보내도 되면 None, 아니면 이유."""
        status = self.status if arm is None else arm.status()
        if status is None:
            return "팔이 답하지 않는다"
        if status["state"] == "Alarm":
            return "Alarm이다. 팔 주변을 비우고 h → y로 원점 복귀부터 한다"
        if status["state"] == "Hold":
            return ("Hold(일시 정지)다. 이때 보낸 명령은 쌓였다가 풀리는 순간 실행되니 보내지 않는다."
                    " 컨트롤러에서 일시 정지를 푼다")
        if status["state"] in mb.BLOCKED:
            return "%s 중이라 받지 않는다" % status["state"]
        if status["state"] != "Idle" or time.time() < self.busy_until:
            return ""   # 움직이는 중. 키를 누르고 있을 때마다 알리지 않는다
        return None

    def sent(self, what, field=None, moving=True):
        """보냈다고 알리고, field(pose·d·pump)가 바뀌는지 CHECK_SEC 뒤에 본다."""
        if moving:
            self.busy_until = time.time() + GRACE_SEC
        if field is not None and self.status is not None:
            self.check = {"at": time.time() + CHECK_SEC, "what": what, "field": field,
                          "before": self.status[field], "arm": self.arm.name}
        if moving and self.status is not None:
            self.status = dict(self.status, state="Run")
        self.term.say("보냄 %s" % what)

    def jog(self, key):
        if self.stopped():
            return
        why = self.ready()
        if why is not None:
            if why:
                self.term.say(why)
            return
        sx, sy, sz, sc = JOG[key]
        dz = sz * self.step
        z_min = self.cfg.get("z_min_mm")
        if z_min is not None and dz < 0 and self.status["pose"]["z"] + dz < z_min:
            self.term.say("Z가 하한 %.1f mm 아래로 간다. config/arms.json의 z_min_mm" % z_min)
            return
        self.arm.jog(sx * self.step, sy * self.step, dz, sc * self.step, self.cfg["feed_mm_min"])
        axis = "XYZC"[[sx, sy, sz, sc].index(next(v for v in (sx, sy, sz, sc) if v))]
        self.sent("%s %s%g" % (axis, "+" if sx + sy + sz + sc > 0 else "-", self.step), "pose")

    def pump_blocked(self):
        """흡착을 보내면 안 되면 알리고 True. 흡착도 이동처럼 Hold 중에는 쌓였다가 늦게 실행된다."""
        if self.stopped():
            return True
        if self.status is None:
            self.term.say("팔이 답하지 않는다")
            return True
        if self.status["state"] in mb.BLOCKED:
            self.term.say(self.ready())
            return True
        return False

    def toggle_pump(self):
        if self.pump_blocked():
            return
        mode = "off" if self.status["pump"] else "on"
        self.arm.pump(mode)
        self.blow_off_at = None
        self.sent("흡착 %s" % ("끄기" if mode == "off" else "켜기"), "pump", moving=False)

    def blow(self):
        if self.pump_blocked():
            return
        self.arm.pump("blow")
        self.blow_off_at = time.time() + BLOW_SEC
        self.term.say("불어서 놓는다")

    def belt(self, mm):
        if self.stopped():
            return
        belt_arm = self.arms.get(self.cfg.get("belt_arm"))
        if belt_arm is None:
            self.term.say("config/arms.json의 belt_arm이 팔 이름과 맞지 않는다")
            return
        why = self.ready(belt_arm)
        if why is not None:
            if why:
                self.term.say("벨트: %s" % why)
            return
        belt_arm.belt(mm)
        # 벨트 팔이 지금 고른 팔이면 D가 바뀌는지 본다.
        self.sent("벨트 %+g mm" % mm, "d" if belt_arm is self.arm else None)

    def save_here(self):
        if self.status is None:
            self.term.say("팔이 답하지 않아 자리를 읽지 못했다")
            return
        name = self.term.ask("자리 이름 (예: pick, rework, ship-red): ")
        if not name or not name.strip():
            return
        name = name.strip()
        mine = self.poses.setdefault(self.arm.name, {})
        if name in mine:
            answer = self.term.ask("%s는 이미 있다. 덮어쓰려면 y: " % name)
            if (answer or "").strip().lower() != "y":
                return
        mine[name] = {"pose": self.status["pose"], "joints": self.status["joints"], "d": self.status["d"],
                      "time": datetime.now().astimezone().isoformat(timespec="seconds")}
        save_poses(self.poses)
        p = self.status["pose"]
        self.term.say("저장 %s/%s  X %.2f Y %.2f Z %.2f C %.2f" % (self.arm.name, name, p["x"], p["y"], p["z"], p["c"]))

    def list_poses(self):
        mine = self.poses.get(self.arm.name, {})
        if not mine:
            self.term.say("%s에 저장한 자리가 없다" % self.arm.name)
            return
        for name, rec in mine.items():
            p = rec["pose"]
            self.term.say("  %-12s X %7.2f Y %7.2f Z %7.2f C %7.2f" % (name, p["x"], p["y"], p["z"], p["c"]))

    def goto(self):
        if self.stopped():
            return
        safe_z = self.cfg.get("safe_z_mm")
        if safe_z is None:
            self.term.say("안전 높이가 없다. 부품과 벨트 위로 지나갈 높이에 팔을 두고 t를 누른다")
            return
        why = self.ready()
        if why is not None:
            self.term.say(why or "팔이 움직이는 중이다")
            return
        mine = self.poses.get(self.arm.name, {})
        if not mine:
            self.term.say("%s에 저장한 자리가 없다" % self.arm.name)
            return
        name = self.term.ask("갈 자리 (%s): " % ", ".join(mine))
        if not name or name.strip() not in mine:
            if name and name.strip():
                self.term.say("그런 자리가 없다: %s" % name.strip())
            return
        target = mine[name.strip()]["pose"]
        here = self.status["pose"]
        feed = self.cfg["feed_mm_min"]
        # 올라가서 → 목표 위로 옮기고 → 내려간다. 목표가 안전 높이보다 높으면 그 높이로 지나간다.
        self.arm.move_to(dict(here, z=max(here["z"], safe_z)), feed)
        self.arm.move_to(dict(target, z=max(target["z"], safe_z)), feed)
        self.arm.move_to(target, feed)
        # 이미 그 자리에 있으면 자리가 안 바뀌는 게 맞으니 반영 여부를 보지 않는다.
        far = max(abs(target[k] - here[k]) for k in target) >= 0.05
        self.sent("%s로 가기" % name.strip(), "pose" if far else None)

    def set_safe_z(self):
        if self.status is None:
            self.term.say("팔이 답하지 않는다")
            return
        self.cfg["safe_z_mm"] = round(self.status["pose"]["z"], 1)
        mb.save_config(self.cfg)
        self.term.say("안전 높이 Z %.1f mm로 정했다 (config/arms.json)" % self.cfg["safe_z_mm"])

    def on_key(self, key):
        """키 하나를 처리한다. 끝내야 하면 False."""
        if self.asking_home:
            self.asking_home = False
            if key == "y":
                if not self.stopped():
                    self.arm.home()
                    self.sent("원점 복귀")
            else:
                self.term.say("원점 복귀 취소")
            return True
        if key in JOG:
            self.jog(key)
        elif key in ("+", "="):
            self.step = STEPS[min(STEPS.index(self.step) + 1, len(STEPS) - 1)]
        elif key == "-":
            self.step = STEPS[max(STEPS.index(self.step) - 1, 0)]
        elif key.isdigit() and 1 <= int(key) <= len(self.order):
            self.arm = self.arms[self.order[int(key) - 1]]
            self.status = None
            self.term.say("%s: %s" % (self.arm.name, self.arm.label))
        elif key == "p":
            self.toggle_pump()
        elif key == "b":
            self.blow()
        elif key == ",":
            self.belt(-BELT_STEP)
        elif key == ".":
            self.belt(BELT_STEP)
        elif key == " ":
            self.save_here()
        elif key == "g":
            self.goto()
        elif key == "l":
            self.list_poses()
        elif key == "t":
            self.set_safe_z()
        elif key == "h":
            self.asking_home = True
            self.term.say("원점 복귀: 팔 전체가 움직인다. 주변이 비었으면 y, 아니면 아무 키")
        elif key == "?":
            self.term.say(HELP)
        elif key == "q":
            return False
        return True

    def status_line(self):
        safe = self.cfg.get("safe_z_mm")
        head = "%s | " % self.arm.name
        tail = " | 칸 %g | 안전 Z %s" % (self.step, "-" if safe is None else "%g" % safe)
        if self.halted:
            tail += " | 명령 막음(전원 껐다 켜기)"
        if self.status is None:
            return head + "답 없음" + tail
        return head + mb.describe(self.status) + tail


def print_keys():
    """키 설명, 안전 높이, 저장한 자리를 보인다. 포트를 열지 않는다. arm.sh 설명 칸이 1초마다 부른다."""
    print("키 (한/영은 영문으로, 키는 조종 칸에서 누른다)")
    for keys, _, text in KEY_HELP:
        # 한글 "스페이스"는 두 칸씩 차지해서 글자 수로 맞추면 줄이 어긋난다. 보이는 폭으로 맞춘다.
        print("  %s%s%s" % (keys, " " * max(10 - width(keys), 1), text))
    cfg = mb.load_config()
    show = lambda v: "-" if v is None else "%g mm" % v
    print()
    print("안전 높이 %s   하한 Z %s   속도 %s mm/min" % (
        show(cfg.get("safe_z_mm")), show(cfg.get("z_min_mm")), cfg.get("feed_mm_min")))
    print()
    poses = load_poses()
    if not poses:
        print("저장한 자리 없음")
    for arm, mine in poses.items():
        print("저장한 자리 %s" % arm)
        for name, rec in mine.items():
            p = rec["pose"]
            print("  %-10s X %7.2f Y %7.2f Z %7.2f C %7.2f" % (name, p["x"], p["y"], p["z"], p["c"]))
    return 0


def main():
    ap = argparse.ArgumentParser(description="키보드로 Mirobot 조종·자리 저장")
    ap.add_argument("--arm", help="처음 조종할 팔 이름. 없으면 config/arms.json의 첫 팔")
    ap.add_argument("--keys", action="store_true", help="키 설명과 저장한 자리만 보이고 끝낸다")
    args = ap.parse_args()
    if args.keys:
        return print_keys()
    if not sys.stdin.isatty():
        print("키 입력을 받을 터미널이 없다. 터미널에서 직접 실행한다")
        return 1
    cfg = mb.load_config()
    if not mb.ARMS_CONFIG.exists():
        mb.save_config(cfg)
        print("팔 설정을 만들었다: %s  두 번째 팔은 python3 mirobot.py probe로 주소를 찾아 더한다" % mb.ARMS_CONFIG)
    start = args.arm or next(iter(cfg["arms"]))
    if start not in cfg["arms"]:
        print("그런 팔이 없다: %s (%s)" % (start, ", ".join(cfg["arms"])))
        return 1
    try:
        ctrl, arms, port = mb.open_arms(cfg)
    except serial.SerialException as exc:
        hint = " (라인이나 다른 프로그램이 포트를 쓰고 있으면 끈다)" if "busy" in str(exc).lower() else ""
        print("포트를 열지 못했다: %s%s" % (exc, hint))
        return 1
    with Terminal() as term:
        tele = Teleop(cfg, ctrl, arms, start, term)
        term.say("컨트롤러 %s, 팔 %s" % (port, ", ".join(arms)))
        term.say(HELP)
        next_poll = 0.0
        try:
            while True:
                now = time.time()
                if now >= next_poll:
                    tele.poll()
                    next_poll = now + POLL_SEC
                if tele.blow_off_at is not None and now >= tele.blow_off_at:
                    tele.arm.pump("off")
                    tele.blow_off_at = None
                term.show(tele.status_line())
                key = term.key(0.02)
                if key and not tele.on_key(key):
                    break
        except KeyboardInterrupt:
            pass
        except serial.SerialException as exc:
            # 같은 포트를 다른 프로그램(라인, probe 등)이 함께 열면 읽기가 끊긴다.
            term.say("포트 오류로 끝낸다: %s" % exc)
            term.say("USB가 빠졌거나 다른 프로그램이 같은 포트를 열었다. 그 프로그램을 끄고 다시 띄운다")
        finally:
            if tele.status is not None and tele.status["pump"]:
                term.say("흡착이 켜진 채로 끝낸다. 부품을 들고 있으면 조심한다")
            ctrl.ser.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
