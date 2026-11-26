# 컨베이어 라인 한 세트를 띄운다. colcon 빌드 없이 파일 경로로 실행한다.
#
#   ros2 launch launch/giro_line.launch.py
#   ros2 launch launch/giro_line.launch.py speed:=40 pause_sec:=1.5
#   ros2 launch launch/giro_line.launch.py line:=line2     # 두 번째 벨트·카메라
# line은 config/devices.json의 lines 이름이다. 그 라인의 namespace, 컨트롤러, 카메라, 보정 파일을 쓴다.
# line1은 namespace가 없어 토픽이 /blocks, /conveyor/... 그대로다. line2는 /line2/blocks처럼 앞에 붙는다.
# 두 라인을 같이 돌리려면 터미널마다 하나씩 띄운다(giro_line.sh line:=line2).
# belt_port, cam_port를 주면 라인 설정 대신 그 값(경로나 장치 이름)을 쓴다.
# 시작 전: OpenMV IDE 연결을 끊고, Mirobot이 Alarm이면 팔 주변을 비운 뒤 원점 복귀를 해 둔다.
#
# 띄우는 것
#   conveyor_node.py      벨트(기본은 다기능 컨트롤러 D축). /conveyor/speed·/conveyor/stop·/blocks 구독
#   block_bridge_node.py  USB로 OpenMV에 camera-ros.py를 올려 /blocks·/camera/image_raw 발행
#                         camera:=false면 띄우지 않는다. 그동안 OpenMV IDE는 연결을 끊어 둔다
#   rqt_image_view        /camera/image_raw 화면. image_view:=false면 띄우지 않는다
#   line_controller_node.py  블럭을 카메라 쪽으로 보내 측정 범위 중앙에 세운다. config/line.yaml을 읽는다
#                         키보드 창에서 a로 켜고 끈다. auto:=true면 켜진 채로 시작한다
#   conveyor_teleop.py    키보드 조작. ros2 launch는 자식에게 키 입력을 넘기지 않아 새 xterm 창으로 연다
#                         keyboard:=false면 띄우지 않는다. giro_line.sh는 이 값으로 tmux 창에 따로 둔다
# 키보드 창을 닫거나 q를 누르면 벨트가 서고, 그 창이 끝나면 라인 전체가 내려간다.

from pathlib import Path

from launch import LaunchDescription
import sys

from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
import devices  # noqa: E402  저장소 폴더를 경로에 넣은 뒤에 불러온다


def script(name, namespace, *params):
    # 노드 스크립트를 python3로 돌리고 namespace와 ROS 파라미터를 붙인다.
    cmd = ["python3", str(ROOT / name), "--ros-args"]
    if namespace:
        cmd += ["-r", "__ns:=/" + namespace]
    for key, value in params:
        cmd += ["-p", [key, ":=", value]]
    return cmd


def generate_launch_description():
    args = [
        DeclareLaunchArgument("line", default_value="line1",
                              description="config/devices.json의 라인 이름. namespace·포트·보정 파일을 정한다"),
        DeclareLaunchArgument("belt_mode", default_value="exbox",
                              description="exbox(다기능 컨트롤러 D축) 또는 ms4220(RS485 직결)"),
        DeclareLaunchArgument("belt_port", default_value="",
                              description="벨트 포트(경로나 장치 이름). 비우면 라인의 컨트롤러"),
        DeclareLaunchArgument("address", default_value="10",
                              description="MS4220 RS485 주소"),
        DeclareLaunchArgument("pause_sec", default_value="0.0",
                              description="conveyor_node의 옛 정지 동작(초). 자동 정렬과 겹치니 0으로 둔다"),
        DeclareLaunchArgument("auto", default_value="false",
                              description="자동 라인을 켠 채로 시작할지"),
        DeclareLaunchArgument("camera", default_value="true",
                              description="카메라 중계 노드를 띄울지"),
        DeclareLaunchArgument("cam_port", default_value="",
                              description="OpenMV 포트(경로나 장치 이름). 비우면 라인의 카메라"),
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

    return LaunchDescription(args + [OpaqueFunction(function=line_actions)])


def line_actions(context):
    """라인 이름으로 namespace·포트·보정 파일을 정해 노드들을 만든다."""
    arg = lambda name: LaunchConfiguration(name).perform(context)
    line = devices.line(arg("line"))
    ns = line.get("namespace", "")
    belt_port = arg("belt_port") or line["controller"]
    cam_port = arg("cam_port") or line["camera"]
    params_file = str(ROOT / line["params"])
    image_topic = "/%s/camera/image_raw" % ns if ns else "/camera/image_raw"

    conveyor = ExecuteProcess(
        name="conveyor",
        cmd=script("conveyor_node.py", ns,
                   ("mode", LaunchConfiguration("belt_mode")),
                   ("port", belt_port),
                   ("address", LaunchConfiguration("address")),
                   ("pause_sec", LaunchConfiguration("pause_sec"))),
        output="screen",
    )
    bridge = ExecuteProcess(
        name="block_bridge",
        cmd=script("block_bridge_node.py", ns,
                   ("port", cam_port)),
        output="screen",
        condition=IfCondition(LaunchConfiguration("camera")),
    )
    controller = ExecuteProcess(
        name="line_controller",
        cmd=script("line_controller_node.py", ns,
                   ("auto", LaunchConfiguration("auto")))
        + ["--params-file", params_file],
        output="screen",
    )
    image_view = ExecuteProcess(
        name="image_view",
        cmd=["ros2", "run", "rqt_image_view", "rqt_image_view", image_topic],
        output="screen",
        # Qt 창은 SIGINT에 잘 반응하지 않는다. 오래 기다리지 않고 1초 뒤 SIGTERM으로 닫는다.
        sigterm_timeout="1",
        condition=IfCondition(LaunchConfiguration("image_view")),
    )
    teleop = ExecuteProcess(
        name="conveyor_teleop",
        cmd=script("conveyor_teleop.py", ns,
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
    return [conveyor, bridge, controller, image_view, teleop, stop_with_teleop]
