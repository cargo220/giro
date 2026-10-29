#!/usr/bin/env python3
# 블럭 인식과 벨트를 잇는 자동 라인 노드다. 벨트는 conveyor_node.py에 토픽으로만 시킨다.
#
#   python3 line_controller_node.py --ros-args --params-file config/line.yaml
#
# 원리
#   - 블럭마다 ID를 붙여 프레임을 넘어 따라간다. 벨트가 움직인 거리(mm × px_per_mm)로
#     다음 위치를 예측해 이어 붙인다.
#   - 서로 cluster_gap_px 안에 있는 블럭은 한 덩어리다. 붙어서 온 블럭은 덩어리째 다룬다.
#   - 카메라 쪽 끝에 가장 가까운 덩어리를 목표로 삼아 그 중심을 측정 범위 중앙에 세운다.
#   - 세운 뒤에는 덩어리 안 블럭마다 ID·색·좌표를 낸다. 하나를 치워도 남은 블럭이 있으면
#     그대로 서 있고, 중앙 구역이 다 비면 다음 블럭을 받으러 간다.
#   - 벨트 끝 너머에는 아무것도 없다. 치우지 않은 블럭을 내보내지 않는다.
#
# 상태  manual → feed → align → hold → feed ...
#   manual  아무것도 시키지 않는다. 키보드로 직접 움직인다
#   feed    벨트를 카메라 쪽으로 돌린다. 덩어리가 중앙 approach_px 앞에 오면 세운다
#   align   선 뒤 덩어리 중심과 중앙의 차이를 mm로 바꿔 한 번에 옮긴다. tolerance_px 안이면 hold
#   hold    중앙 구역(hold_half_px)에 블럭이 남아 있는 동안 서 있는다
#
# 구독  /blocks String, /conveyor/position Float32, /conveyor/moving Bool, /line/auto Bool
# 발행  /conveyor/speed Int32, /conveyor/move Float32
#       /line/state   String  "manual" "feed" "align" "hold 2" 같은 상태, 바뀔 때와 1초마다
#       /line/targets String  hold 중 블럭 목록 JSON [{"id":3,"color":"RED","x":180,"y":130}], 바뀔 때와 1초마다

import json
import re
import signal
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import Bool, Float32, Int32, String

BLOCK_RE = re.compile(r"([A-Z]+)@(-?\d+),(-?\d+)")


class Track:
    def __init__(self, track_id, color, x, y, now, belt_d):
        self.id = track_id
        self.color = color
        self.x = x
        self.y = y
        self.seen = now
        self.belt_d = belt_d


class LineController(Node):
    def __init__(self):
        super().__init__("line_controller")
        # 30과 30.0처럼 정수·실수가 섞여 와도 받도록 형을 고정하지 않는다.
        p = lambda name, value: self.declare_parameter(
            name, value, ParameterDescriptor(dynamic_typing=True)).value
        # 1단계 측정값. 벨트 방향이 화면의 어느 축인지와, D 1 mm에 그 축이 몇 px 움직이는지(부호 포함).
        self.axis = p("axis", "x")
        if self.axis not in ("x", "y"):
            # 따옴표 없는 y는 YAML에서 참이 된다. 잘못 읽히면 바로 멈춘다.
            raise ValueError("axis는 \"x\" 또는 \"y\"여야 한다: %r" % (self.axis,))
        self.px_per_mm = float(p("px_per_mm", 1.0))
        # 카메라 쪽으로 보내는 D 방향. 키보드로 보니 음수가 카메라 쪽이다.
        self.camera_dir = -1 if int(p("camera_direction", -1)) < 0 else 1
        self.center = (float(p("center_x", 189.0)), float(p("center_y", 134.0)))
        self.feed_speed = abs(int(p("feed_speed", 30)))
        self.approach_px = float(p("approach_px", 40.0))
        self.tolerance_px = float(p("tolerance_px", 5.0))
        self.max_align = int(p("max_align", 3))
        self.cluster_gap_px = float(p("cluster_gap_px", 45.0))
        self.match_px = float(p("match_px", 30.0))
        self.lost_sec = float(p("lost_sec", 1.0))
        self.hold_half_px = float(p("hold_half_px", 60.0))
        self.clear_sec = float(p("clear_sec", 1.0))
        self.settle_sec = float(p("settle_sec", 0.4))
        # 공급 중 이 시간 동안 블럭이 하나도 안 보이면 벨트를 세운다. 못 본 블럭을 끝으로 밀어내지 않는다.
        self.feed_timeout_sec = float(p("feed_timeout_sec", 30.0))
        start_auto = bool(p("auto", False))

        self.tracks = {}
        self.next_id = 1
        self.belt_d = None
        self.belt_moving = False
        self.last_blocks = 0.0
        self.state = "manual"
        self.align_tries = 0
        self.align_wait_since = None
        self.empty_since = None
        self.hold_ids = set()
        self.last_targets = None
        self.last_target_key = None
        self.last_report = 0.0
        self.belt_speed = None
        self.want_speed = None
        self.last_speed_send = 0.0
        self.feed_since = None
        # auto:=true여도 벨트 노드가 위치를 내기 시작한 뒤에 켠다. 먼저 보낸 명령은 연결 전이라 사라진다.
        self.pending_auto = start_auto

        self.speed_pub = self.create_publisher(Int32, "conveyor/speed", 10)
        self.move_pub = self.create_publisher(Float32, "conveyor/move", 10)
        self.state_pub = self.create_publisher(String, "line/state", 10)
        self.targets_pub = self.create_publisher(String, "line/targets", 10)
        self.create_subscription(String, "blocks", self.on_blocks, 10)
        self.create_subscription(Float32, "conveyor/position", self.on_position, 10)
        self.create_subscription(Bool, "conveyor/moving", self.on_moving, 10)
        self.create_subscription(Int32, "conveyor/state", self.on_belt_speed, 10)
        self.create_subscription(Bool, "line/auto", self.on_auto, 10)
        self.create_timer(0.05, self.tick)
        self.get_logger().info(
            "라인 제어: 축 %s, %.2f px/mm, 중앙 (%.0f, %.0f), 카메라 쪽 D%s"
            % (self.axis, self.px_per_mm, self.center[0], self.center[1],
               "-" if self.camera_dir < 0 else "+"))

    # ── 입력 ──────────────────────────────────────────────

    def on_position(self, msg):
        self.belt_d = msg.data

    def on_moving(self, msg):
        self.belt_moving = msg.data

    def on_belt_speed(self, msg):
        self.belt_speed = msg.data

    def on_auto(self, msg):
        self.set_auto(msg.data)

    def on_blocks(self, msg):
        now = time.time()
        self.last_blocks = now
        found = [(c, float(x), float(y)) for c, x, y in BLOCK_RE.findall(msg.data)]
        self.match(found, now)

    # ── 추적 ──────────────────────────────────────────────

    def along(self, x, y):
        return x if self.axis == "x" else y

    def predicted(self, track):
        # 마지막으로 본 뒤 벨트가 움직인 만큼 벨트 방향 축으로 옮겨 본다.
        if self.belt_d is None or track.belt_d is None:
            return track.x, track.y
        shift = self.px_per_mm * (self.belt_d - track.belt_d)
        if self.axis == "x":
            return track.x + shift, track.y
        return track.x, track.y + shift

    def match(self, found, now):
        pairs = []
        for i, (color, x, y) in enumerate(found):
            for track in self.tracks.values():
                px, py = self.predicted(track)
                cost = ((px - x) ** 2 + (py - y) ** 2) ** 0.5
                if color != track.color:
                    # 색 이름이 한두 프레임 흔들려도 같은 블럭으로 잇되, 같은 색을 먼저 고른다.
                    cost += self.match_px / 3
                if cost < self.match_px:
                    pairs.append((cost, i, track.id))
        pairs.sort()
        used_found, used_track = set(), set()
        for _cost, i, track_id in pairs:
            if i in used_found or track_id in used_track:
                continue
            used_found.add(i)
            used_track.add(track_id)
            color, x, y = found[i]
            track = self.tracks[track_id]
            track.color, track.x, track.y = color, x, y
            track.seen, track.belt_d = now, self.belt_d
        for i, (color, x, y) in enumerate(found):
            if i not in used_found:
                self.tracks[self.next_id] = Track(self.next_id, color, x, y, now, self.belt_d)
                self.next_id += 1
        for track_id in [t.id for t in self.tracks.values() if now - t.seen > self.lost_sec]:
            track = self.tracks.pop(track_id)
            self.get_logger().info("블럭 #%d %s 사라짐" % (track.id, track.color))

    def remaining(self, a):
        # 중앙까지 남은 px. 카메라 쪽으로 오는 방향이 +다. 음수면 중앙을 지나쳤다.
        travel = 1 if self.px_per_mm * self.camera_dir > 0 else -1
        return (self.along(*self.center) - a) * travel

    def clusters(self):
        tracks = list(self.tracks.values())
        groups = []
        for track in tracks:
            px, py = self.predicted(track)
            near = [g for g in groups
                    if any(((px - qx) ** 2 + (py - qy) ** 2) ** 0.5 < self.cluster_gap_px
                           for _t, qx, qy in g)]
            merged = [(track, px, py)]
            for g in near:
                merged += g
                groups.remove(g)
            groups.append(merged)
        return groups

    def cluster_center(self, group):
        # 덩어리 양 끝의 가운데. 블럭 수가 아니라 덩어리 전체가 중앙에 오게 한다.
        values = [self.along(px, py) for _t, px, py in group]
        return (min(values) + max(values)) / 2

    def lead_cluster(self):
        groups = self.clusters()
        if not groups:
            return None
        # 카메라 쪽 끝에 가장 가까운(남은 거리가 가장 작은) 덩어리다.
        return min(groups, key=lambda g: self.remaining(self.cluster_center(g)))

    # ── 상태 ──────────────────────────────────────────────

    def set_auto(self, on):
        if on and self.state == "manual":
            self.get_logger().info("자동 시작")
            self.enter("align" if self.lead_in_zone() else "feed")
        elif not on and self.state != "manual":
            self.send_speed(0)
            self.get_logger().info("자동 끝, 벨트 정지")
            self.enter("manual")

    def lead_in_zone(self):
        lead = self.lead_cluster()
        return lead is not None and self.remaining(self.cluster_center(lead)) <= self.approach_px

    def send_speed(self, value):
        self.want_speed = value
        self.last_speed_send = time.time()
        self.speed_pub.publish(Int32(data=value))

    def keep_speed(self, now):
        # 벨트 노드가 받아들인 속도가 원하는 값과 다르면 1초마다 다시 보낸다. 시작 직후 연결 전에 보낸
        # 명령은 사라지고, Alarm이던 벨트는 원점 복귀 뒤에야 받는다.
        if self.want_speed is None or self.belt_speed == self.want_speed:
            return
        if now - self.last_speed_send >= 1.0:
            self.send_speed(self.want_speed)

    def enter(self, state):
        self.state = state
        if state == "feed":
            self.feed_since = time.time()
            self.send_speed(self.camera_dir * self.feed_speed)
        elif state == "manual":
            self.want_speed = None
        elif state == "align":
            self.send_speed(0)
            self.align_tries = 0
            self.align_wait_since = None
        elif state == "hold":
            self.empty_since = None
            self.hold_ids = set()
            self.last_targets = None
            self.last_target_key = None
            # 들어오자마자 구역 안 블럭을 세어 "hold 0"이 잠깐 나가지 않게 한다. 그 안에서 report한다.
            self.step_hold(time.time())
            return
        self.report(force=True)

    def tick(self):
        now = time.time()
        if self.pending_auto and self.belt_d is not None:
            self.pending_auto = False
            self.set_auto(True)
        if self.state != "manual":
            self.keep_speed(now)
        if self.state == "feed":
            if self.tracks:
                self.feed_since = now
            if self.lead_in_zone():
                self.enter("align")
            elif now - self.feed_since > self.feed_timeout_sec:
                self.get_logger().warn("%.0f초 동안 블럭이 안 보여 벨트를 세운다. a로 다시 켠다" % self.feed_timeout_sec)
                self.send_speed(0)
                self.enter("manual")
        elif self.state == "align":
            self.step_align(now)
        elif self.state == "hold":
            self.step_hold(now)
        self.report()

    def step_align(self, now):
        # 벨트가 다 서고 settle_sec 지난 뒤의 새 검출로만 잰다.
        if self.belt_moving:
            self.align_wait_since = None
            return
        if self.align_wait_since is None:
            self.align_wait_since = now
            return
        if now - self.align_wait_since < self.settle_sec or self.last_blocks < self.align_wait_since:
            return
        lead = self.lead_cluster()
        if lead is None:
            self.get_logger().warn("정렬할 블럭이 사라졌다. 다시 공급한다")
            self.enter("feed")
            return
        rem = self.remaining(self.cluster_center(lead))
        if abs(rem) <= self.tolerance_px:
            self.enter("hold")
            return
        if self.align_tries >= self.max_align:
            self.get_logger().warn("%d번 맞춰도 %.0f px 남았다. 이 자리에서 세운다" % (self.align_tries, rem))
            self.enter("hold")
            return
        # 남은 px만큼 카메라 쪽으로 옮긴다. 음수면 지나친 것이라 반대로 되돌린다.
        mm = self.camera_dir * rem / abs(self.px_per_mm)
        self.align_tries += 1
        self.align_wait_since = None
        self.move_pub.publish(Float32(data=float(mm)))
        self.get_logger().info("정렬 %d: %.0f px → 벨트 %.1f mm" % (self.align_tries, rem, mm))

    def step_hold(self, now):
        here = [t for t in self.tracks.values()
                if abs(self.remaining(self.along(t.x, t.y))) <= self.hold_half_px]
        ids = {t.id for t in here}
        for gone in self.hold_ids - ids:
            self.get_logger().info("블럭 #%d 치워짐" % gone)
        self.hold_ids = ids
        targets = [{"id": t.id, "color": t.color, "x": int(t.x), "y": int(t.y)}
                   for t in sorted(here, key=lambda t: t.id)]
        self.last_targets = json.dumps(targets)
        # 좌표는 매 프레임 1 px씩 흔들린다. 구성(ID·색)이 바뀔 때만 바로 내고, 좌표는 1초마다 갱신한다.
        key = [(t["id"], t["color"]) for t in targets]
        if key != self.last_target_key:
            self.last_target_key = key
            self.report(force=True)
        if here:
            self.empty_since = None
            return
        # 하나를 치워도 남은 게 있으면 위에서 돌아간다. 다 비고 clear_sec 지나면 다음으로.
        if self.empty_since is None:
            self.empty_since = now
        elif now - self.empty_since >= self.clear_sec:
            self.get_logger().info("중앙이 비었다. 다음 블럭을 받는다")
            self.enter("feed")

    def report(self, force=False):
        now = time.time()
        if not force and now - self.last_report < 1.0:
            return
        self.last_report = now
        text = self.state
        if self.state == "hold":
            text = "hold %d" % len(self.hold_ids)
            if self.last_targets:
                self.targets_pub.publish(String(data=self.last_targets))
        self.state_pub.publish(String(data=text))


def main():
    rclpy.init()
    node = LineController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # Ctrl+C가 발행 콜백 도중에 오면 이미 닫힌 context로 발행하다 오류가 난다.
        if rclpy.ok():
            raise
    finally:
        # 터미널 Ctrl+C는 노드에 한 번, launch를 거쳐 한 번 더 온다. 정리 중에는 받지 않는다.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
