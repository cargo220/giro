#!/usr/bin/env bash
# 컨베이어 라인을 tmux 한 창에서 띄운다. 위 칸은 노드 로그, 아래 칸은 키보드 조작이다.
# 넘긴 인자는 그대로 ros2 launch로 간다.
#
#   ./giro_line.sh
#   ./giro_line.sh belt_port:=/dev/ttyUSB0 speed:=40
# 카메라는 USB로 연결된 OpenMV를 자동으로 찾는다. 화면은 rqt_image_view 창으로 뜬다.
#
# 아래 칸에서 q를 누르면 벨트를 세우고, 위 칸의 launch를 Ctrl+C로 내린 뒤 세션을 닫는다.
# 이미 같은 세션이 있으면 새로 만들지 않고 그 세션으로 붙는다.

set -eu

SESSION=giro_line
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP=/opt/ros/humble/setup.bash

if tmux has-session -t "$SESSION" 2>/dev/null; then
    exec tmux attach -t "$SESSION"
fi

# 칸마다 ROS 환경을 불러온다. ROS_DOMAIN_ID는 이 셸의 값을 그대로 넘긴다.
env_cmd="source $ROS_SETUP; export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}; cd '$ROOT'"
launch_args="keyboard:=false"
for arg in "$@"; do
    launch_args+=" $(printf '%q' "$arg")"
done
speed=40
for arg in "$@"; do
    case "$arg" in speed:=*) speed="${arg#speed:=}" ;; esac
done

tmux new-session -d -s "$SESSION" -n line \
    "bash -c \"$env_cmd; ros2 launch launch/giro_line.launch.py $launch_args; echo; read -rp '엔터를 누르면 닫는다'\""
tmux split-window -v -l 6 -t "$SESSION:line" \
    "bash -c \"$env_cmd; python3 conveyor_teleop.py --ros-args -p speed:=$speed; tmux send-keys -t $SESSION:line.0 C-c; sleep 3; tmux kill-session -t $SESSION\""
tmux select-pane -t "$SESSION:line.1"
exec tmux attach -t "$SESSION"
