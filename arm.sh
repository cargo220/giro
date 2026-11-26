#!/usr/bin/env bash
# Mirobot 키보드 조종을 tmux 한 창에서 띄운다. 위 칸은 키보드 조종, 아래 칸은 저장한 자리 목록이다.
# 넘긴 인자는 그대로 arm_teleop.py로 간다.
#
#   ./arm.sh
#   ./arm.sh --arm arm2
# 라인(giro_line.sh)과 같은 컨트롤러 포트를 쓰니 라인을 끄고 띄운다.
#
# 위 칸에서 q를 누르면 끝나고, 엔터를 한 번 더 누르면 세션을 닫는다. Ctrl+C로 끝나도 세션을 닫는다.
# 이미 같은 세션이 있으면 그 세션으로 붙는다. 조종 칸이 없는 세션이면 닫고 새로 띄운다.

set -eu

SESSION=giro_arm
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if tmux has-session -t "$SESSION" 2>/dev/null; then
    if [ "$(tmux list-panes -t "$SESSION" | wc -l)" -ge 2 ]; then
        exec tmux attach -t "$SESSION"
    fi
    tmux kill-session -t "$SESSION"
fi

args=""
for arg in "$@"; do
    args+=" $(printf '%q' "$arg")"
done

tmux new-session -d -s "$SESSION" -n arm \
    "bash -c \"trap 'tmux kill-session -t $SESSION' EXIT; cd '$ROOT'; python3 arm_teleop.py$args; echo; read -rp '엔터를 누르면 닫는다'\""
# 저장한 자리 목록은 포트를 열지 않고 config/arm_poses.json만 읽는다.
tmux split-window -v -l 10 -t "$SESSION:arm" \
    "bash -c \"cd '$ROOT'; watch -t -n 1 python3 arm_teleop.py --poses\""
tmux select-pane -t "$SESSION:arm.0"
exec tmux attach -t "$SESSION"
