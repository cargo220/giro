#!/usr/bin/env python3
# 터미널에서 키를 눌러 컨베이어를 움직인다. conveyor_node.py에 토픽만 보낸다.
# 키 입력을 받아야 하므로 터미널 창에서 직접 돌린다. launch에서는 새 창으로 띄운다.
#
#   스페이스   정지 / 마지막 속도로 다시 출발
#   ↑ / w      속도 +STEP (정방향 쪽)
#   ↓ / s      속도 -STEP (역방향 쪽)
#   r          방향 반전
#   0          정지하고 설정 속도를 0으로
#   a          자동 모드 켜기/끄기. 켜면 line_controller_node.py가 블럭을 중앙에 세운다
#              자동 중에는 벨트를 직접 움직이는 키(스페이스·↑↓·r·0)를 받지 않는다
#   h → y      벨트를 세우고 Mirobot 원점 복귀. 팔 전체가 움직여 y로 한 번 더 확인받는다
#              h 다음에 y가 아닌 키를 누르면 취소하고, 그 키는 실행하지 않는다
#   q / Ctrl+C 정지하고 끝낸다
#
# 발행  /conveyor/speed std_msgs/Int32, /conveyor/stop std_msgs/Empty, /conveyor/home std_msgs/Empty
#       /line/auto std_msgs/Bool
# 구독  /conveyor/state std_msgs/Int32  보드가 받아들인 속도를 화면에 보인다
#       /line/state std_msgs/String   자동 라인 상태(feed, align, hold 2 등)를 화면에 보인다

import os
import select
import sys
import termios
import tty

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, Int32, String

from conveyor import SPEED_LIMIT

STEP = 10
HELP = "스페이스 정지/출발  ↑w 빠르게  ↓s 느리게  r 반전  0 리셋  a 자동  h 원점복귀  q 끝"

KEYS = {
    " ": "toggle",
    "w": "up", "\x1b[A": "up",
    "s": "down", "\x1b[B": "down",
    "r": "reverse",
    "0": "zero",
    "a": "auto",
    "h": "home",
    "y": "yes",
    "q": "quit", "\x03": "quit",
}


class Teleop(Node):
    def __init__(self, default_speed):
        super().__init__("conveyor_teleop")
        self.target = int(self.declare_parameter("speed", default_speed).value)
        self.running = False
        self.state = None
        self.asking_home = False
        self.auto = False
        self.line_state = None
        self.speed_pub = self.create_publisher(Int32, "conveyor/speed", 10)
        self.stop_pub = self.create_publisher(Empty, "conveyor/stop", 10)
        self.home_pub = self.create_publisher(Empty, "conveyor/home", 10)
        self.auto_pub = self.create_publisher(Bool, "line/auto", 10)
        self.create_subscription(Int32, "conveyor/state", self.on_state, 10)
        self.create_subscription(String, "line/state", self.on_line_state, 10)

    def on_state(self, msg):
        if msg.data != self.state:
            self.state = msg.data
            self.show()

    def on_line_state(self, msg):
        if msg.data != self.line_state:
            self.line_state = msg.data
            self.show()

    def set_auto(self, on):
        self.auto = on
        self.running = False
        self.auto_pub.publish(Bool(data=on))

    def send(self):
        if self.running and self.target != 0:
            self.speed_pub.publish(Int32(data=self.target))
        else:
            self.stop_pub.publish(Empty())

    def act(self, action):
        if self.asking_home:
            # 확인을 묻는 중에는 y만 받는다. 다른 키는 취소로 보고 그 키도 실행하지 않는다.
            self.asking_home = False
            if action == "yes":
                self.running = False
                self.home_pub.publish(Empty())
                sys.stdout.write("\r\033[K원점 복귀를 보냈다. 끝나면 스페이스로 출발한다\n")
            else:
                sys.stdout.write("\r\033[K원점 복귀 취소\n")
            self.show()
            return True
        if action == "auto":
            self.set_auto(not self.auto)
            self.show()
            return True
        if action == "home":
            if self.auto:
                # 원점 복귀 중에 자동 라인이 벨트를 돌리지 않게 먼저 끈다.
                self.set_auto(False)
            self.asking_home = True
            sys.stdout.write("\r\033[K원점 복귀: 팔 전체가 움직인다. 주변이 비었으면 y, 아니면 아무 키\n")
            sys.stdout.flush()
            return True
        if action == "yes":
            return True
        if self.auto and action in ("toggle", "up", "down", "reverse", "zero"):
            sys.stdout.write("\r\033[K자동 중에는 a로 끈 뒤 직접 움직인다\n")
            self.show()
            return True
        if action == "toggle":
            self.running = not self.running
        elif action == "up":
            self.target = min(SPEED_LIMIT, self.target + STEP)
        elif action == "down":
            self.target = max(-SPEED_LIMIT, self.target - STEP)
        elif action == "reverse":
            self.target = -self.target
        elif action == "zero":
            self.target = 0
            self.running = False
        elif action == "quit":
            if self.auto:
                self.set_auto(False)
            self.running = False
            self.send()
            return False
        # 서 있을 때 속도만 바꾸면 보드에는 보내지 않는다. 출발할 때 그 속도로 간다.
        if self.running or action in ("toggle", "zero"):
            self.send()
        self.show()
        return True

    def show(self):
        if self.auto:
            mode = "자동 %s" % (self.line_state or "-")
        else:
            mode = "운전" if self.running else "정지"
        board = "-" if self.state is None else str(self.state)
        # 같은 줄을 덮어써서 상태 한 줄만 보인다.
        sys.stdout.write("\r\033[K%s  설정 %+d  보드 %s" % (mode, self.target, board))
        sys.stdout.flush()


def read_key(timeout):
    # sys.stdin.read는 남은 글자를 파이썬 버퍼로 미리 가져가 select가 못 본다.
    # 그러면 화살표(ESC [ A)의 뒤 두 글자를 놓치므로 fd에서 바로 읽는다.
    fd = sys.stdin.fileno()
    ready, _, _ = select.select([fd], [], [], timeout)
    if not ready:
        return None
    ch = os.read(fd, 1)
    if ch == b"\x1b":
        ready, _, _ = select.select([fd], [], [], 0.05)
        if ready:
            ch += os.read(fd, 2)
    return ch.decode("utf-8", "replace")


def main():
    if not sys.stdin.isatty():
        print("키 입력을 받을 터미널이 없다. 터미널 창에서 직접 실행한다.")
        return 1
    rclpy.init()
    node = Teleop(40)
    saved = termios.tcgetattr(sys.stdin)
    print(HELP)
    node.show()
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0)
            key = read_key(0.05)
            if key is None:
                continue
            action = KEYS.get(key)
            if action is None:
                # 원점 복귀를 묻는 중이면 모르는 키도 취소로 받는다.
                if not node.asking_home:
                    continue
                action = "cancel"
            if not node.act(action):
                break
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, saved)
        if rclpy.ok():
            # 끝날 때 한 번 더 세운다. 발행이 나가도록 잠깐 돌린다.
            node.stop_pub.publish(Empty())
            rclpy.spin_once(node, timeout_sec=0.2)
        print()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
