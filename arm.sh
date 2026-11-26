#!/usr/bin/env bash
# Mirobot 키보드 조종을 tmux 한 창에서 띄운다. 팔마다 명령 칸이 하나씩, 그리고 설명 칸이 하나다.
#   명령 칸  팔 하나 전용. config/arms.json 순서대로 왼쪽이 1번 팔, 오른쪽이 2번 팔이다.
#            각 칸은 자기 팔의 컨트롤러 포트만 연다. 두 칸이 같은 포트를 열지 않는다
#   설명 칸  키 설명, 팔 목록, 안전 높이, 저장한 자리. 1초마다 다시 그린다. 포트를 열지 않는다
# 넘긴 인자는 그대로 arm_teleop.py로 간다. --arm을 주면 그 팔 칸 하나만 띄운다.
#
#   ./arm.sh
#   ./arm.sh --arm arm2
# 라인(giro_line.sh)과 같은 컨트롤러 포트를 쓰니 라인을 끄고 띄운다.
# 터미널 폭이 160칸 이상이면 설명 칸을 오른쪽에, 좁으면 위에 둔다.
#
# 칸은 마우스로 눌러 고른다(이 세션만 마우스를 켠다). Ctrl+b 다음 방향키로도 옮긴다.
# 칸 위쪽 줄에 팔 이름이 있고, 키가 들어가는 칸에는 [입력 중]이 붙는다.
# 명령 칸에서 q를 누르면 그 팔 칸만 끝나고, 엔터를 한 번 더 누르면 그 칸이 닫힌다.
# 마지막 명령 칸이 닫히면 세션을 닫는다. Ctrl+C로 끝나도 같다.
# 이미 같은 세션이 있으면 그 세션으로 붙는다. 칸이 하나만 남은 세션이면 닫고 새로 띄운다.

set -eu

SESSION=giro_arm
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PANE_TITLE=arm-ctrl

# 명령 칸 안에서 도는 쪽. 끝날 때 남은 명령 칸이 없으면 세션을 닫는다.
# 명령 칸 이름은 "arm-ctrl arm1"처럼 바깥에서 붙인다. 이 이름으로 남은 명령 칸을 센다.
if [ "${1:-}" = "--pane" ]; then
    shift
    close_if_last() {
        session=$(tmux display -p -t "$TMUX_PANE" '#{session_name}')
        left=$(tmux list-panes -t "=$session" -F '#{pane_title}' | grep -c "^$PANE_TITLE" || true)
        if [ "$left" -le 1 ]; then
            tmux kill-session -t "=$session"
        fi
    }
    trap close_if_last EXIT
    cd "$ROOT"
    python3 arm_teleop.py "$@" || true
    echo
    read -rp '엔터를 누르면 닫는다' || true
    exit 0
fi

# tmux는 이름 앞부분만 맞아도 그 세션을 찾는다. "="를 붙여 이름이 똑같은 세션만 찾는다.
if tmux has-session -t "=$SESSION" 2>/dev/null; then
    if [ "$(tmux list-panes -t "=$SESSION" | wc -l)" -ge 2 ]; then
        exec tmux attach -t "=$SESSION"
    fi
    tmux kill-session -t "=$SESSION"
fi

args=""
single=""
for arg in "$@"; do
    args+=" $(printf '%q' "$arg")"
    case "$arg" in --arm|--arm=*) single=1 ;; esac
done
if [ -n "$single" ]; then
    arms=("")
else
    read -r -a arms <<< "$(cd "$ROOT" && python3 -c 'import mirobot; print(" ".join(mirobot.load_config()["arms"]))')"
    # 팔이 하나면 칸 하나에 그 팔만 둔다. 둘 이상이면 팔마다 그 팔만 여는 칸을 둔다.
    [ "${#arms[@]}" -le 1 ] && arms=("")
fi
pane_cmd() {
    if [ -n "$1" ]; then
        echo "bash $(printf '%q' "$ROOT/arm.sh") --pane --arm $1 --only$args"
    else
        echo "bash $(printf '%q' "$ROOT/arm.sh") --pane$args"
    fi
}
# 붙기 전에 만든 세션은 80×24로 잡힌다. 그 크기로 칸을 나누면 붙은 뒤 칸이 좁아지니 지금 터미널 크기로 만든다.
cols=$(tput cols 2>/dev/null || echo 160)
lines=$(tput lines 2>/dev/null || echo 40)

first=$(tmux new-session -d -P -F '#{pane_id}' -s "$SESSION" -n arm -x "$cols" -y "$lines" "$(pane_cmd "${arms[0]}")")
tmux select-pane -t "$first" -T "$PANE_TITLE ${arms[0]:-팔}"
prev=$first
for arm in "${arms[@]:1}"; do
    prev=$(tmux split-window -h -P -F '#{pane_id}' -t "$prev" "$(pane_cmd "$arm")")
    tmux select-pane -t "$prev" -T "$PANE_TITLE $arm"
done
tmux select-layout -t "$first" even-horizontal
# 마우스로 칸을 고르게 하고, 칸마다 위쪽 줄에 이름과 [입력 중]을 보인다. 이 세션에만 적용된다.
tmux set -t "$first" mouse on
tmux set -t "$first" pane-border-status top
tmux set -t "$first" pane-border-format ' #{?pane_active,[입력 중] ,}#{s/^arm-ctrl //:pane_title} '

help_cmd="bash -c \"cd '$ROOT'; watch -t -n 1 python3 arm_teleop.py --keys\""
if [ "$cols" -ge 160 ]; then
    help=$(tmux split-window -h -f -l 56 -P -F '#{pane_id}' -t "$first" "$help_cmd")
else
    # 위쪽 설명은 18줄쯤이다. 명령 칸에 12줄은 남기고, 모자라면 설명 칸을 줄인다(아래 줄이 잘린다).
    help_lines=$(( lines - 12 < 18 ? lines - 12 : 18 ))
    [ "$help_lines" -lt 8 ] && help_lines=8
    help=$(tmux split-window -v -b -f -l "$help_lines" -P -F '#{pane_id}' -t "$first" "$help_cmd")
fi
tmux select-pane -t "$help" -T "설명 (키는 팔 칸에서)"
tmux select-pane -t "$first"
exec tmux attach -t "=$SESSION"
