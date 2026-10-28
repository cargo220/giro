# 컨베이어 라인 한 세트를 띄운다. colcon 빌드 없이 파일 경로로 실행한다.
#
#   ros2 launch launch/giro_line.launch.py
#   ros2 launch launch/giro_line.launch.py belt_port:=/dev/ttyUSB0 cam_port:=/dev/ttyACM0 speed:=40
#
# 띄우는 것
#   conveyor_node.py      벨트(MS4220, RS485). /conveyor/speed·/conveyor/stop·/blocks 구독
#   block_bridge_node.py  USB로 OpenMV에 camera-ros.py를 올려 /blocks·/camera/image_raw 발행
#                         camera:=false면 띄우지 않는다. 그동안 OpenMV IDE는 연결을 끊어 둔다
#   rqt_image_view        /camera/image_raw 화면. image_view:=false면 띄우지 않는다
#   conveyor_teleop.py    키보드 조작. ros2 launch는 자식에게 키 입력을 넘기지 않아 새 xterm 창으로 연다
#                         keyboard:=false면 띄우지 않는다. giro_line.sh는 이 값으로 tmux 창에 따로 둔다
# 키보드 창을 닫거나 q를 누르면 벨트가 서고, 그 창이 끝나면 라인 전체가 내려간다.

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration

ROOT = Path(__file__).resolve().parent.parent


def script(name, *params):
    # 노드 스크립트를 python3로 돌리고 ROS 파라미터를 붙인다.
    cmd = ["python3", str(ROOT / name), "--ros-args"]
    for key, value in params:
        cmd += ["-p", [key, ":=", value]]
    return cmd


def generate_launch_description():
    args = [
        DeclareLaunchArgument("belt_port", default_value="/dev/ttyUSB0",
                              description="컨베이어 RS485 변환기 포트"),
        DeclareLaunchArgument("address", default_value="10",
                              description="MS4220 RS485 주소"),
        DeclareLaunchArgument("pause_sec", default_value="2.0",
                              description="블럭을 보면 벨트를 세울 초, 0이면 세우지 않음"),
        DeclareLaunchArgument("camera", default_value="true",
                              description="카메라 중계 노드를 띄울지"),
        DeclareLaunchArgument("cam_port", default_value="auto",
                              description="OpenMV USB 포트, auto면 /dev/serial/by-id의 MicroPython 보드"),
        DeclareLaunchArgument("image_view", default_value="true",
                              description="rqt_image_view로 카메라 화면을 띄울지"),
        DeclareLaunchArgument("speed", default_value="40",
                              description="키보드로 출발할 때의 처음 속도"),
        DeclareLaunchArgument("keyboard", default_value="true",
                              description="키보드 조작 창을 띄울지"),
        DeclareLaunchArgument("terminal",
                              default_value="xterm -fa 'Noto Sans Mono CJK KR' -fs 12 -T conveyor -e",
                              description="키보드 창을 여는 명령, 예: 'gnome-terminal --wait --'"),
    ]

    conveyor = ExecuteProcess(
        name="conveyor",
        cmd=script("conveyor_node.py",
                   ("port", LaunchConfiguration("belt_port")),
                   ("address", LaunchConfiguration("address")),
                   ("pause_sec", LaunchConfiguration("pause_sec"))),
        output="screen",
    )
    bridge = ExecuteProcess(
        name="block_bridge",
        cmd=script("block_bridge_node.py",
                   ("port", LaunchConfiguration("cam_port"))),
        output="screen",
        condition=IfCondition(LaunchConfiguration("camera")),
    )
    image_view = ExecuteProcess(
        name="image_view",
        cmd=["ros2", "run", "rqt_image_view", "rqt_image_view", "/camera/image_raw"],
        output="screen",
        condition=IfCondition(LaunchConfiguration("image_view")),
    )
    teleop = ExecuteProcess(
        name="conveyor_teleop",
        cmd=script("conveyor_teleop.py",
                   ("speed", LaunchConfiguration("speed"))),
        prefix=LaunchConfiguration("terminal"),
        output="screen",
        condition=IfCondition(LaunchConfiguration("keyboard")),
    )
    # 키보드 창이 끝나면 라인을 내린다. 벨트 노드는 내려갈 때 벨트를 세운다.
    stop_with_teleop = RegisterEventHandler(OnProcessExit(
        target_action=teleop,
        on_exit=[EmitEvent(event=Shutdown(reason="키보드 창 종료"))],
    ))

    return LaunchDescription(args + [conveyor, bridge, image_view, teleop, stop_with_teleop])
