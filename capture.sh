#!/usr/bin/env bash
# 판정용 원본 프레임 수집을 tmux 한 창에서 띄운다. 위 칸은 키보드 조작, 아래 칸은 최근에 저장한 파일 이름이다.
# 넘긴 인자는 그대로 capture_dataset.py로 간다.
#
#   ./capture.sh --part P003
#   ./capture.sh --part P003 --exposure 3000 7500
#   ./capture.sh --part P003 --kind dent --depth 0.6 --width 10
# 화면(DISPLAY)이 있으면 미리보기 창도 띄운다. 키는 위 칸에서 누른다. 창에서도 한 글자 키는 받는다.
# 띄어쓰기가 있는 메모는 인자 대신 위 칸에서 n으로 넣는다.
#
# 위 칸에서 q를 누르면 카메라를 멈추고, 엔터를 한 번 더 누르면 세션을 닫는다.
# 위 칸이 Ctrl+C로 끝나도 세션을 닫는다. 파일 목록 칸만 남지 않게 한다.
# 이미 같은 세션이 있으면 새로 만들지 않고 그 세션으로 붙는다. 조작 칸이 없는 세션이면 닫고 새로 띄운다.

set -eu

SESSION=giro_capture
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# tmux는 이름 앞부분만 맞아도 그 세션을 찾는다. "="를 붙여 이름이 똑같은 세션만 찾는다.
if tmux has-session -t "=$SESSION" 2>/dev/null; then
    if [ "$(tmux list-panes -t "=$SESSION" | wc -l)" -ge 2 ]; then
        exec tmux attach -t "=$SESSION"
    fi
    tmux kill-session -t "=$SESSION"
fi

args=""
for arg in "$@"; do
    args+=" $(printf '%q' "$arg")"
done
if [ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
    args="--view$args"
fi

tmux new-session -d -s "$SESSION" -n capture \
    "bash -c \"trap 'tmux kill-session -t =$SESSION' EXIT; cd '$ROOT'; python3 capture_dataset.py $args; echo; read -rp '엔터를 누르면 닫는다'\""
# 파일 이름에 그때 넣은 값이 들어가므로, 이름만 봐도 무엇을 찍었는지 보인다.
tmux split-window -v -l 8 -t "=$SESSION:capture" \
    "bash -c \"cd '$ROOT'; watch -t -n 1 'ls -t dataset/frames/*/*.png 2>/dev/null | head -8 | xargs -r -n1 basename'\""
tmux select-pane -t "=$SESSION:capture.0"
exec tmux attach -t "=$SESSION"
