#!/usr/bin/env python3
# 벨트 방향이 화면의 어느 축인지, D 1 mm에 몇 px 움직이는지 재서 config/line.yaml에 쓴다.
# 측정 범위 중앙에서 SEARCH_ROI 가운데도 읽어 함께 쓴다.
#
# 준비: launch를 띄워 둔다(conveyor_node와 block_bridge_node가 떠 있어야 한다).
#       벨트 위 측정 범위 안에 블럭을 하나만 둔다. Mirobot은 Idle이어야 한다.
#   python3 calibrate_belt.py
#   python3 calibrate_belt.py --mm 30 --dry-run
#
# 블럭이 카메라 끝으로 밀려나지 않게 반대(+) 방향으로 --mm만큼 옮겨 재고, 다시 되돌린다.

import argparse
import re
import sys
import time
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

ROOT = Path(__file__).resolve().parent
CONFIG = ROOT / "config" / "line.yaml"
BLOCK_RE = re.compile(r"([A-Z]+)@(-?\d+),(-?\d+)")
ROI_RE = re.compile(r"^SEARCH_ROI\s*=\s*\((\d+),\s*(\d+),\s*(\d+),\s*(\d+)\)", re.M)


class Calibrator(Node):
    def __init__(self):
        super().__init__("calibrate_belt")
        self.blocks = []
        self.position = None
        self.moving = True
        self.create_subscription(String, "blocks", self.on_blocks, 10)
        self.create_subscription(Float32, "conveyor/position", self.on_position, 10)
        self.create_subscription(Bool, "conveyor/moving", self.on_moving, 10)
        self.move_pub = self.create_publisher(Float32, "conveyor/move", 10)

    def on_blocks(self, msg):
        self.blocks.append((time.time(), BLOCK_RE.findall(msg.data)))

    def on_position(self, msg):
        self.position = msg.data

    def on_moving(self, msg):
        self.moving = msg.data

    def spin_for(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_still(self, timeout=15.0):
        end = time.time() + timeout
        while time.time() < end:
            self.spin_for(0.1)
            if not self.moving and self.position is not None:
                self.spin_for(0.5)
                if not self.moving:
                    return True
        return False

    def measure(self, sec=1.5):
        # 블럭이 정확히 하나 보인 프레임만 평균한다.
        start = time.time()
        self.spin_for(sec)
        xs, ys = [], []
        for t, found in self.blocks:
            if t >= start and len(found) == 1:
                xs.append(float(found[0][1]))
                ys.append(float(found[0][2]))
        if len(xs) < 5:
            return None
        xs.sort()
        ys.sort()
        return xs[len(xs) // 2], ys[len(ys) // 2], len(xs)

    def move(self, mm):
        self.move_pub.publish(Float32(data=float(mm)))
        self.spin_for(0.3)
        return self.wait_still()


def main():
    parser = argparse.ArgumentParser(description="벨트 방향·축척 측정")
    parser.add_argument("--mm", type=float, default=30.0, help="잴 때 옮기는 거리(카메라 반대쪽)")
    parser.add_argument("--dry-run", action="store_true", help="결과만 보이고 파일은 쓰지 않는다")
    args = parser.parse_args()

    rclpy.init()
    node = Calibrator()
    try:
        if not node.wait_still():
            sys.exit("벨트 위치를 받지 못했다. launch의 conveyor_node가 떠 있는지 본다")
        first = node.measure()
        if first is None:
            sys.exit("블럭 하나가 안정적으로 보이지 않는다. 측정 범위 안에 블럭을 하나만 둔다")
        d0 = node.position
        print("처음  D %.2f mm  블럭 (%.1f, %.1f)  %d프레임" % (d0, *first))
        # 카메라 반대쪽(+)으로 옮긴다. 음수가 카메라 쪽이다.
        if not node.move(abs(args.mm)):
            sys.exit("벨트가 멈추지 않았다")
        second = node.measure()
        d1 = node.position
        node.move(d0 - d1)
        if second is None:
            sys.exit("옮긴 뒤 블럭을 놓쳤다. --mm을 줄여 다시 한다")
        print("옮김  D %.2f mm  블럭 (%.1f, %.1f)  %d프레임" % (d1, *second))
        dd = d1 - d0
        dx, dy = second[0] - first[0], second[1] - first[1]
        axis = "x" if abs(dx) >= abs(dy) else "y"
        px_per_mm = (dx if axis == "x" else dy) / dd
        cross = abs(dy if axis == "x" else dx)
        print("결과  축 %s, %.3f px/mm (D %+.1f mm에 %+.1f px, 다른 축 %.1f px)"
              % (axis, px_per_mm, dd, dx if axis == "x" else dy, cross))
        if cross > 0.3 * max(abs(dx), abs(dy)):
            print("경고: 벨트가 화면 축과 꽤 비스듬하다. 카메라 방향을 확인한다")

        roi = ROI_RE.search((ROOT / "camera-ros.py").read_text(encoding="utf-8"))
        data = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
        params = data["line_controller"]["ros__parameters"]
        params["axis"] = axis
        params["px_per_mm"] = round(px_per_mm, 4)
        if roi:
            x, y, w, h = map(int, roi.groups())
            params["center_x"] = float(x + w / 2)
            params["center_y"] = float(y + h / 2)
            print("중앙  SEARCH_ROI %s → (%.0f, %.0f)" % (roi.groups(), params["center_x"], params["center_y"]))
        if args.dry_run:
            print("--dry-run이라 파일은 그대로 둔다")
            return 0
        text = CONFIG.read_text(encoding="utf-8")
        # 주석을 남기려고 값만 바꾼다.
        for key in ("axis", "px_per_mm", "center_x", "center_y"):
            # axis는 따옴표로 쓴다. YAML은 맨 y를 참(yes)으로 읽는다.
            value = '"%s"' % params[key] if key == "axis" else str(params[key])
            text = re.sub(r"(^\s*%s:\s*)([^\s#]+)" % key, lambda m: m.group(1) + value,
                          text, count=1, flags=re.M)
        CONFIG.write_text(text, encoding="utf-8")
        print("%s에 썼다" % CONFIG)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
