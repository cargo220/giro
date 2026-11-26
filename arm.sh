#!/usr/bin/env bash
# Mirobot 키보드 조종을 tmux 한 창에서 띄운다. 명령 칸과 설명 칸으로 나뉜다.
#   명령 칸  arm_teleop.py. 여기서 키를 눌러 팔에 명령을 보낸다. 보낸 명령과 컨트롤러 답이 쌓인다
#   설명 칸  키 설명, 안전 높이, 저장한 자리. 1초마다 다시 그린다. 포트를 열지 않는다
# 넘긴 인자는 그대로 arm_teleop.py로 간다.
#
#   ./arm.sh
#   ./arm.sh --arm arm2
# 라인(giro_line.sh)과 같은 컨트롤러 포트를 쓰니 라인을 끄고 띄운다.
# 터미널 폭이 120칸 이상이면 설명 칸을 오른쪽에, 좁으면 위에 둔다.
#
# 명령 칸에서 q를 누르면 끝나고, 엔터를 한 번 더 누르면 세션을 닫는다. Ctrl+C로 끝나도 세션을 닫는다.
# 이미 같은 세션이 있으면 그 세션으로 붙는다. 칸이 하나만 남은 세션이면 닫고 새로 띄운다.

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
# 붙기 전에 만든 세션은 80×24로 잡힌다. 그 크기로 칸을 나누면 붙은 뒤 명령 칸이 좁아지니 지금 터미널 크기로 만든다.
cols=$(tput cols 2>/dev/null || echo 160)
lines=$(tput lines 2>/dev/null || echo 40)

control=$(tmux new-session -d -P -F '#{pane_id}' -s "$SESSION" -n arm -x "$cols" -y "$lines" \
    "bash -c \"trap 'tmux kill-session -t $SESSION' EXIT; cd '$ROOT'; python3 arm_teleop.py$args; echo; read -rp '엔터를 누르면 닫는다'\"")
help_cmd="bash -c \"cd '$ROOT'; watch -t -n 1 python3 arm_teleop.py --keys\""
if [ "$cols" -ge 120 ]; then
    tmux split-window -h -l 56 -t "$control" "$help_cmd"
else
    # 설명은 22줄쯤이다. 명령 칸에 10줄은 남기고, 모자라면 설명 칸을 줄인다(아래 줄이 잘린다).
    help_lines=$(( lines - 10 < 22 ? lines - 10 : 22 ))
    [ "$help_lines" -lt 8 ] && help_lines=8
    tmux split-window -v -b -l "$help_lines" -t "$control" "$help_cmd"
fi
tmux select-pane -t "$control"
exec tmux attach -t "$SESSION"
