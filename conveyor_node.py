#!/usr/bin/env python3
# 컨베이어를 ROS 2 토픽으로 움직이는 노드다. 카메라는 OpenMV IDE에 그대로 둔다.
# 벨트 쪽 통신은 conveyor.py의 Conveyor(RS485, MS4220)를 그대로 쓴다.
# colcon 빌드 없이 ROS 2 Humble 환경을 불러온 셸에서 바로 실행한다.
#
#   python3 conveyor_node.py --ros-args -p port:=/dev/ttyUSB0 -p address:=10 -p pause_sec:=2.0
#
# 구독
#   /conveyor/speed  std_msgs/Int32  -100~100, 부호가 방향, 0이 정지
#   /conveyor/stop   std_msgs/Empty  바로 정지
#   /blocks          std_msgs/String block_bridge_node.py가 내는 판별 결과, 없으면 ""
#                    블럭이 새로 보이면 벨트를 pause_sec 초 세웠다가 원래 속도로 돌린다.
#                    같은 블럭이 계속 보이는 동안은 다시 세우지 않는다. 0이면 세우지 않는다.
# 발행
#   /conveyor/state  std_msgs/Int32  보드가 받아들인 마지막 속도, 1초마다와 바뀔 때
#
#   ros2 topic pub --once /conveyor/speed std_msgs/msg/Int32 "{data: 40}"
#   ros2 topic pub --once /conveyor/stop std_msgs/msg/Empty "{}"

import serial
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Empty, Int32, String

from conveyor import SPEED_LIMIT, Conveyor


class ConveyorNode(Node):
    def __init__(self):
        super().__init__("conveyor")
        port = self.declare_parameter("port", "/dev/ttyUSB0").value
        address = self.declare_parameter("address", 10).value
        self.pause_sec = float(self.declare_parameter("pause_sec", 2.0).value)
        self.belt = Conveyor(port, address)
        self.current = None
        self.seeing = False
        self.resume_speed = None
        self.resume_at = None
        self.state_pub = self.create_publisher(Int32, "conveyor/state", 10)
        self.create_subscription(Int32, "conveyor/speed", self.on_speed, 10)
        self.create_subscription(Empty, "conveyor/stop", self.on_stop, 10)
        self.create_subscription(String, "blocks", self.on_blocks, 10)
        self.create_timer(1.0, self.publish_state)
        self.create_timer(0.05, self.check_resume)
        # 노드가 뜰 때 벨트가 이전 속도로 돌고 있을 수 있어 먼저 세운다.
        self.apply(0)
        self.get_logger().info("컨베이어 %s 주소 %d" % (port, address))

    def apply(self, value):
        value = max(-SPEED_LIMIT, min(SPEED_LIMIT, int(value)))
        reply = self.belt.speed(value)
        if reply:
            self.current = value
            self.get_logger().info("속도 %d 응답 %s" % (value, reply))
        else:
            # 응답이 없으면 보드가 받았는지 모른다. state는 바꾸지 않는다.
            self.get_logger().warn("속도 %d 응답 없음" % value)
        self.publish_state()

    def on_speed(self, msg):
        # 사람이 속도를 주면 블럭 때문에 기다리던 재가동은 버린다.
        self.resume_at = None
        self.apply(msg.data)

    def on_stop(self, _msg):
        self.resume_at = None
        self.apply(0)

    def on_blocks(self, msg):
        seeing = bool(msg.data.strip())
        # 안 보이다가 보이는 순간에만 세운다. 카메라는 같은 결과를 반복해 보낸다.
        newly = seeing and not self.seeing
        self.seeing = seeing
        if not newly or self.pause_sec <= 0 or self.resume_at is not None:
            return
        if not self.current:
            # 벨트가 이미 서 있으면 다시 돌릴 속도가 없다.
            return
        self.get_logger().info("블럭 %s, %.1f초 정지" % (msg.data, self.pause_sec))
        self.resume_speed = self.current
        self.apply(0)
        self.resume_at = self.get_clock().now().nanoseconds / 1e9 + self.pause_sec

    def check_resume(self):
        if self.resume_at is None:
            return
        if self.get_clock().now().nanoseconds / 1e9 < self.resume_at:
            return
        self.resume_at = None
        self.get_logger().info("재가동 %d" % self.resume_speed)
        self.apply(self.resume_speed)

    def publish_state(self):
        if self.current is not None:
            self.state_pub.publish(Int32(data=self.current))

    def shutdown(self):
        # Ctrl+C로 끝나도 벨트를 세우고 포트를 닫는다.
        try:
            self.belt.stop()
        finally:
            self.belt.close()


def main():
    rclpy.init()
    try:
        node = ConveyorNode()
    except serial.SerialException as exc:
        print("벨트 포트를 열지 못했다: %s" % exc)
        rclpy.shutdown()
        return 1
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Humble은 Ctrl+C를 ExternalShutdownException으로 끝내기도 한다.
        pass
    except Exception:
        # Ctrl+C가 발행 콜백 도중에 오면 이미 닫힌 context로 발행하다 오류가 난다.
        # 종료 중이면 정상 종료로 보고, 아래 finally에서 벨트를 세운다.
        if rclpy.ok():
            raise
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
