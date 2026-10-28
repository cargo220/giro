#!/usr/bin/env python3
# WLKATA 컨베이어 벨트의 MS4220 스테퍼 보드를 RS485로 돌린다.
# PC에는 USB–RS485 변환기를 꽂고, A/B 선을 보드의 초록 터미널에 잇는다.
# 명령은 wlkatapython SDK와 같은 "@<주소>G6 F<속도>\r\n"이고, 38400 bps다.
# 속도는 -100~100이며 부호가 방향, 0이 정지다.
#
#   python3 conveyor.py speed 60            # 계속 돈다
#   python3 conveyor.py run 60 --seconds 3  # 3초 돌고 멈춘다
#   python3 conveyor.py stop

import argparse
import sys
import time

import serial

BAUD = 38400
SPEED_LIMIT = 100


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


def main():
    parser = argparse.ArgumentParser(description="WLKATA 컨베이어(MS4220) 제어")
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--address", type=int, default=10,
                        help="보드 RS485 주소, 주소 없이 보내려면 -1")
    sub = parser.add_subparsers(dest="cmd", required=True)
    p_speed = sub.add_parser("speed", help="속도를 정하고 바로 끝낸다")
    p_speed.add_argument("value", type=int)
    p_run = sub.add_parser("run", help="정한 시간만 돌고 멈춘다")
    p_run.add_argument("value", type=int)
    p_run.add_argument("--seconds", type=float, required=True)
    sub.add_parser("stop")
    args = parser.parse_args()

    try:
        belt = Conveyor(args.port, args.address)
    except serial.SerialException as exc:
        sys.exit("포트를 열지 못했다: %s" % exc)

    try:
        if args.cmd == "stop":
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
