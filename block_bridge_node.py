#!/usr/bin/env python3
# OpenMV를 USB로 직접 돌리고, 판별 결과와 화면을 ROS 2 토픽으로 옮긴다.
# OpenMV는 rclpy를 돌릴 수 없어서 PC가 이 다리를 맡는다. 카메라는 USB 케이블 하나로 잇는다.
# IDE가 카메라 포트를 잡고 있으면 연결하지 못하니 IDE 연결을 끊고 실행한다.
#
#   python3 block_bridge_node.py
#   python3 block_bridge_node.py --ros-args -p port:=/dev/ttyACM0
#   ros2 run rqt_image_view rqt_image_view /camera/image_raw
#
# 통신 방식은 펌웨어에 따라 다르다. protocol:=auto(기본)면 펌웨어 버전을 묻고 고른다.
#   4.4.x 이하(제출용 H7 4.4.2)  옛 IDE 프로토콜, omv_legacy.py
#   그보다 새 펌웨어(RT1062)     openmv 패키지
#
# 하는 일
#   1. camera-ros.py를 보드에 올려 실행하고, 노출이 잠길 때까지 기다린다.
#   2. print() 줄 가운데 "BLOCKS ..."만 /blocks로 낸다. 나머지 줄은 로그로 남긴다.
#   3. IDE 화면에 가던 프레임(테두리와 이름이 그려진 것)을 /camera/image_raw로 낸다.
# 발행
#   /blocks            std_msgs/String     "RED@160,120 BLUE@80,90", 블럭이 없으면 ""
#   /camera/image_raw  sensor_msgs/Image   rgb8, 보드에서 받는 대로

import os
import sys
import time
from pathlib import Path

# see_cam이 불러오는 pygame의 인사 문구를 로그에 남기지 않는다.
os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

# see_cam이 openmv 1.0.7의 이벤트 처리를 고쳐 둔다. 같은 고침을 그대로 쓴다.
import see_cam
from openmv import Camera

from board_script import strip_comments
from omv_legacy import LegacyCamera, read_fw_version

PREFIX = "BLOCKS"
# 옛 IDE 프로토콜을 쓰는 마지막 펌웨어다. 이보다 새 펌웨어는 openmv 패키지로 붙는다.
LEGACY_MAX_FW = (4, 4)
# 이 시간 동안 BLOCKS 줄이 없으면 카메라 스크립트가 멈춘 것으로 본다.
SILENT_SEC = 2.0
# 이전 스크립트를 멈추면 보드가 soft reboot 한다. 그 뒤에 올려야 새 스크립트가 남는다.
REBOOT_WAIT = 4.0


def find_camera_port():
    # 펌웨어마다 USB 이름이 다르다. H7 4.4.2는 OpenMV, RT1062는 MicroPython으로 잡힌다.
    by_id = Path("/dev/serial/by-id")
    if by_id.is_dir():
        for pattern in ("*OpenMV*", "*MicroPython*"):
            found = sorted(by_id.glob(pattern))
            if found:
                return str(found[0])
    return see_cam.default_port()


def open_camera(port, protocol, logger):
    if protocol == "auto":
        fw = read_fw_version(port)
        protocol = "legacy" if fw is not None and tuple(fw[:2]) <= LEGACY_MAX_FW else "new"
        logger.info("펌웨어 %s, %s 방식" % (".".join(map(str, fw)) if fw else "모름", protocol))
    cam = LegacyCamera(port) if protocol == "legacy" else Camera(port)
    cam.connect()
    return cam


class BlockBridge(Node):
    def __init__(self):
        super().__init__("block_bridge")
        port = self.declare_parameter("port", "auto").value
        if port in ("", "auto"):
            port = find_camera_port()
        protocol = self.declare_parameter("protocol", "auto").value
        script = self.declare_parameter(
            "script", str(Path(__file__).with_name("camera-ros.py"))).value
        self.frame_id = self.declare_parameter("frame_id", "camera").value
        self.blocks_pub = self.create_publisher(String, "blocks", 10)
        self.image_pub = self.create_publisher(Image, "camera/image_raw", 1)
        self.buf = ""
        self.silent = False
        # 첫 BLOCKS 줄 전에는 노출을 잠그는 중이라 침묵 경고를 내지 않는다.
        self.last_line = None
        self.last_error = 0.0

        self.cam = open_camera(port, protocol, self.get_logger())
        self.cam.stop()
        deadline = time.time() + REBOOT_WAIT
        while time.time() < deadline:
            self.cam.read_stdout()
            time.sleep(0.05)
        # 원문을 그대로 올리면 H7 힙이 모자라 MemoryError가 난다. 주석을 빼고 올린다.
        self.cam.exec(strip_comments(Path(script).read_text(encoding="utf-8")))
        self.cam.streaming(True)
        self.get_logger().info("카메라 %s에서 %s 실행" % (port, Path(script).name))
        self.create_timer(0.02, self.poll)

    def poll(self):
        try:
            self.read_camera()
        except IOError as exc:
            # 옛 프로토콜은 답이 덜 오면 그 명령만 버리고 다음 주기에 다시 묻는다.
            if time.time() - self.last_error > 1.0:
                self.last_error = time.time()
                self.get_logger().warn("카메라 응답 오류: %s" % exc)
            return
        if self.last_line is None:
            return
        if time.time() - self.last_line > SILENT_SEC and not self.silent:
            self.silent = True
            self.get_logger().warn("BLOCKS 줄이 %.0f초째 없다" % SILENT_SEC)

    def read_camera(self):
        text = self.cam.read_stdout()
        if text:
            self.buf += text
            # 줄 가운데서 끊긴 마지막 조각은 다음 읽기까지 남긴다.
            *lines, self.buf = self.buf.split("\n")
            for line in lines:
                self.on_line(line.strip())
        frame = self.cam.read_frame()
        if frame is not None:
            self.publish_frame(frame)

    def on_line(self, line):
        if not line:
            return
        if not line.startswith(PREFIX):
            # 노출 판단, 색 측정값, 오류는 보드가 찍은 그대로 로그에 남긴다.
            if "Traceback" in line or "Error" in line:
                self.get_logger().error("카메라: %s" % line)
            elif line.startswith("노출판단"):
                # 매 프레임 찍히는 줄이라 평소에는 숨긴다. --log-level debug로 본다.
                self.get_logger().debug("카메라: %s" % line)
            else:
                self.get_logger().info("카메라: %s" % line)
            return
        self.last_line = time.time()
        if self.silent:
            self.silent = False
            self.get_logger().info("BLOCKS 줄 다시 받음")
        self.blocks_pub.publish(String(data=line[len(PREFIX):].strip()))

    def publish_frame(self, frame):
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.width = frame["width"]
        msg.height = frame["height"]
        msg.encoding = "rgb8"
        msg.step = frame["width"] * 3
        msg.data = bytes(frame["data"])
        self.image_pub.publish(msg)

    def close(self):
        # 보드에 스크립트를 남겨 두지 않는다. 다음에 IDE로 바로 붙을 수 있게 멈추고 끊는다.
        # 스크립트를 먼저 멈추고 화면 전송을 끈다. 반대 순서면 H7 4.4.2가 정지를 늦게 받았다.
        try:
            self.cam.stop()
            self.cam.streaming(False)
        except Exception as exc:
            print("카메라 정리 중 오류: %s" % exc, file=sys.stderr)
        finally:
            self.cam.disconnect()


def main():
    rclpy.init()
    try:
        node = BlockBridge()
    except Exception as exc:
        hint = ""
        if "busy" in str(exc):
            hint = " (OpenMV IDE 같은 다른 프로그램이 포트를 쓰고 있다. 연결을 끊고 다시 띄운다)"
        print("카메라를 열지 못했다: %s%s" % (exc, hint), file=sys.stderr)
        rclpy.shutdown()
        return 1
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # Ctrl+C가 발행 콜백 도중에 오면 이미 닫힌 context로 발행하다 오류가 난다.
        # 종료 중이면 정상 종료로 본다.
        if rclpy.ok():
            raise
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
