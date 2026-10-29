#!/usr/bin/env python3
# WLKATA 컨베이어 벨트를 돌린다. 연결 방식이 둘이라 클래스도 둘이다.
# 속도는 둘 다 -100~100이며 부호가 방향, 0이 정지다.
#
# exbox (기본) — 실습 셀 구성. 벨트 모터가 다기능 컨트롤러(EXbox)의 STEPPER MOTOR 포트에 있고,
#   PC는 컨트롤러 USB(CH340, 115200)에 붙는다. 벨트는 Mirobot의 D축이라 mm 단위로만 움직인다.
#   그래서 짧은 상대 이동을 이어 보내 계속 돌리고, 보내기를 멈추면 0.5초 안에 선다.
#   전원을 켠 뒤에는 Mirobot이 Alarm이라 이동을 받지 않는다. 원점 복귀($H)를 먼저 한다.
#   원점 복귀는 home()으로 보낸다. 팔이 움직이니 키보드 창은 한 번 더 확인받는다.
# ms4220 — MS4220 보드에 USB–RS485 변환기를 바로 잇는 경우.
#   명령은 wlkatapython SDK와 같은 "@<주소>G6 F<속도>\r\n"이고, 38400 bps다.
#
#   python3 conveyor.py speed 60            # exbox는 Ctrl+C까지, ms4220은 정하고 끝낸다
#   python3 conveyor.py run 60 --seconds 3  # 3초 돌고 멈춘다
#   python3 conveyor.py stop
#   python3 conveyor.py --mode ms4220 speed 60

import argparse
import re
import sys
import threading
import time
from pathlib import Path

import serial

BAUD = 38400
SPEED_LIMIT = 100

EXBOX_BAUD = 115200
# 속도 100이 벨트 최고 속도 2400 mm/min(40 mm/s)이다.
EXBOX_MAX_FEED = 2400
# 한 번에 보내는 이동은 이 시간 분량이다. 남은 이동이 두 칸보다 적을 때만 다음 칸을 보내,
# 컨트롤러에 쌓인 이동이 0.5초를 넘지 않게 한다. 실제 속도가 명령보다 조금 느려서
# 시간만 보고 보내면 이동이 쌓여 정지가 점점 늦어진다.
EXBOX_CHUNK = 0.25
EXBOX_POLL = 0.05
STATUS_RE = re.compile(r"<(\w+),Angle\(ABCDXYZ\):[^,]+,[^,]+,[^,]+,([-\d.]+),")


class Conveyor:
    def __init__(self, port, address=10, timeout=0.5):
        self.address = address
        self.serial = serial.Serial(port, BAUD, timeout=timeout)

    def send(self, command):
        # 주소가 -1이면 SDK처럼 "@주소"를 붙이지 않는다.
        if self.address == -1:
            line = command + "\r\n"
        else:
            line = "@%d%s\r\n" % (self.address, command)
        self.serial.reset_input_buffer()
        self.serial.write(line.encode("utf-8"))
        reply = self.serial.readline().decode("utf-8", "replace").strip()
        return reply

    def speed(self, value):
        # SDK의 speed()는 범위 제한이 비교문(==)이라 실제로 자르지 않는다. 여기서 자른다.
        value = max(-SPEED_LIMIT, min(SPEED_LIMIT, int(value)))
        return self.send("G6 F%d" % value)

    def stop(self):
        return self.speed(0)

    def close(self):
        self.serial.close()


class ExboxConveyor:
    """다기능 컨트롤러의 D축 벨트. 시리얼은 안의 스레드 하나만 만진다."""

    def __init__(self, port):
        self.serial = serial.Serial(port, EXBOX_BAUD, timeout=EXBOX_POLL)
        self.value = 0
        self.state = None
        self.position = None
        self.last_error = None
        self._buf = b""
        self._target = None
        self._direction = 0
        self._home_requested = False
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        # 첫 상태를 받을 때까지 기다린다. Alarm인지 알아야 speed()가 답할 수 있다.
        deadline = time.time() + 2.0
        while self.state is None and time.time() < deadline:
            time.sleep(EXBOX_POLL)

    def speed(self, value):
        self.value = max(-SPEED_LIMIT, min(SPEED_LIMIT, int(value)))
        if self.state is None:
            return ""
        if self.state in ("Alarm", "Hold", "Home"):
            # Alarm은 원점 복귀 전, Hold는 일시 정지, Home은 원점 복귀 중이다. 모두 이동을 받지 않는다.
            # 풀면 팔이 움직일 수 있어 여기서 대신 풀지 않는다. 원점 복귀는 home()으로 따로 부른다.
            return self.state.lower()
        return "ok"

    def stop(self):
        return self.speed(0)

    def home(self):
        """Mirobot 원점 복귀($H)를 보낸다. 팔 전체가 움직이니 부르는 쪽에서 확인을 받는다."""
        if self.state == "Hold":
            # Hold 중에 보낸 $H는 쌓였다가 해제되는 순간 실행된다. 보내지 않는다.
            return "hold"
        if self.state == "Home":
            return "home"
        self.value = 0
        self._home_requested = True
        return "ok"

    def close(self):
        self.value = 0
        self._running = False
        self._thread.join(timeout=2.0)
        self.serial.close()

    def _read_lines(self):
        # 상태 줄이 두 번에 나뉘어 올 수 있어 마지막 조각은 다음 읽기까지 남긴다.
        self._buf += self.serial.read(4096)
        *lines, self._buf = self._buf.split(b"\n")
        for raw in lines:
            line = raw.decode("utf-8", "replace").strip()
            m = STATUS_RE.match(line)
            if m:
                self.state = m.group(1)
                self.position = float(m.group(2))
            elif line and line != "ok":
                self.last_error = line

    def _loop(self):
        while self._running:
            self.serial.write(b"?\r\n")
            self._read_lines()
            if self._home_requested:
                self._home_requested = False
                self._target = None
                self.serial.write(b"$H\r\n")
                # 다음 상태 줄이 Home으로 오기 전까지 벨트 이동을 보내지 않는다.
                self.state = "Home"
            self._step()
            time.sleep(EXBOX_POLL)

    def _step(self):
        value = self.value
        if value == 0 or self.state in ("Alarm", "Hold", "Home") or self.position is None:
            # 이미 보낸 이동은 0.5초 안에 끝난다. 다음 출발은 그 자리에서 다시 잰다.
            self._target = None
            return
        direction = 1 if value > 0 else -1
        if direction != self._direction:
            # 반대 방향 이동이 남아 있으면 끝날 때까지 기다린다.
            if self.state != "Idle":
                return
            self._direction = direction
            self._target = None
        if self._target is None:
            self._target = self.position
        feed = abs(value) * EXBOX_MAX_FEED / SPEED_LIMIT
        step = feed / 60.0 * EXBOX_CHUNK
        if direction * (self._target - self.position) >= 2 * step:
            return
        self.serial.write(b"M21 G91 G01 D%.2f F%d\r\n" % (direction * step, feed))
        self._target += direction * step


def find_belt_port():
    # 다기능 컨트롤러는 CH340(1a86)으로 잡힌다. USB를 다시 꽂으면 ttyUSB 번호가 바뀌어
    # by-id 이름으로 찾는다.
    by_id = Path("/dev/serial/by-id")
    if by_id.is_dir():
        found = sorted(by_id.glob("*1a86*"))
        if found:
            return str(found[0])
    return "/dev/ttyUSB0"


def open_belt(mode, port, address=10):
    if port in ("", "auto"):
        port = find_belt_port()
    if mode == "exbox":
        return ExboxConveyor(port)
    return Conveyor(port, address)


def main():
    parser = argparse.ArgumentParser(description="WLKATA 컨베이어 제어")
    parser.add_argument("--mode", choices=("exbox", "ms4220"), default="exbox")
    parser.add_argument("--port", default="auto",
                        help="auto면 /dev/serial/by-id에서 CH340 컨트롤러를 찾는다")
    parser.add_argument("--address", type=int, default=10,
                        help="ms4220 RS485 주소, 주소 없이 보내려면 -1")
    sub = parser.add_subparsers(dest="cmd", required=True)
    p_speed = sub.add_parser("speed", help="속도를 정하고 바로 끝낸다")
    p_speed.add_argument("value", type=int)
    p_run = sub.add_parser("run", help="정한 시간만 돌고 멈춘다")
    p_run.add_argument("value", type=int)
    p_run.add_argument("--seconds", type=float, required=True)
    sub.add_parser("stop")
    args = parser.parse_args()

    try:
        belt = open_belt(args.mode, args.port, args.address)
    except serial.SerialException as exc:
        sys.exit("포트를 열지 못했다: %s" % exc)

    try:
        if args.cmd == "stop":
            reply = belt.stop()
        elif args.cmd == "speed" and args.mode == "exbox":
            # exbox는 이 프로그램이 이동을 이어 보내야 돈다. Ctrl+C까지 돌린다.
            reply = belt.speed(args.value)
            print("응답:", reply or "(없음)", " Ctrl+C로 멈춘다")
            try:
                while True:
                    time.sleep(1.0)
            except KeyboardInterrupt:
                reply = belt.stop()
        elif args.cmd == "speed":
            reply = belt.speed(args.value)
        else:
            reply = belt.speed(args.value)
            print("응답:", reply or "(없음)")
            try:
                time.sleep(args.seconds)
            finally:
                # Ctrl+C로 끊어도 벨트는 세운다.
                reply = belt.stop()
        print("응답:", reply or "(없음)")
    finally:
        belt.close()


if __name__ == "__main__":
    main()
