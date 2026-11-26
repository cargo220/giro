# 터미널에서 키를 한 글자씩 바로 받고, 맨 아래 한 줄을 상태 줄로 덮어쓴다.
# capture_dataset.py와 arm_teleop.py가 같이 쓴다. tmux 칸에서도 그대로 돈다.
# 한/영 입력이 영문이어야 글자 키가 들어온다. 한글이면 s가 ㄴ으로 들어와 받지 못한다.

import os
import select
import shutil
import sys
import termios
import time
import tty
import unicodedata


def width(text):
    """터미널에서 보이는 폭. 한글처럼 넓은 글자는 두 칸으로 센다."""
    return sum(2 if unicodedata.east_asian_width(ch) in ("W", "F") else 1 for ch in text)


def clip(text, cols):
    """터미널 한 줄에 들어가게 자른다. 한글은 두 칸으로 센다."""
    out, width = [], 0
    for ch in text:
        w = 2 if unicodedata.east_asian_width(ch) in ("W", "F") else 1
        if width + w > cols - 1:
            break
        out.append(ch)
        width += w
    return "".join(out)


class Terminal:
    """키는 한 글자씩 바로 받고, 맨 아래 한 줄을 상태 줄로 덮어쓴다. 값을 물을 때만 줄 입력으로 바꾼다."""

    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.saved = termios.tcgetattr(self.fd)
        self.status = ""
        self.hint_at = 0.0    # 한글 입력 안내를 마지막으로 띄운 때. 3초에 한 번만 띄운다

    def __enter__(self):
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, *exc):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved)
        sys.stdout.write("\n")
        sys.stdout.flush()

    def say(self, msg):
        sys.stdout.write("\r\033[K%s\n" % msg)
        self.draw()

    def show(self, status):
        if status != self.status:
            self.status = status
            self.draw()

    def draw(self):
        sys.stdout.write("\r\033[K" + clip(self.status, shutil.get_terminal_size().columns))
        sys.stdout.flush()

    def ask(self, prompt):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved)
        sys.stdout.write("\r\033[K")
        sys.stdout.flush()
        try:
            return input(prompt)
        except (EOFError, KeyboardInterrupt):
            sys.stdout.write("\n")
            return None
        finally:
            tty.setcbreak(self.fd)
            self.draw()

    def key(self, timeout):
        # sys.stdin.read는 남은 글자를 파이썬 버퍼로 미리 가져가 select가 못 본다. fd에서 바로 읽는다.
        ready, _, _ = select.select([self.fd], [], [], timeout)
        if not ready:
            return None
        ch = os.read(self.fd, 1)
        if ch == b"\x1b":
            # 화살표(ESC [ A)의 '['를 각도 키로 받지 않게 뒤 글자까지 같이 버린다.
            ready, _, _ = select.select([self.fd], [], [], 0.05)
            if ready:
                os.read(self.fd, 2)
            return None
        if ch[0] >= 0xC0:
            # 한글처럼 여러 바이트인 글자는 나머지 바이트까지 읽는다. 한/영이 한글이면 w가 ㅈ으로 들어온다.
            more = 1 if ch[0] < 0xE0 else 2 if ch[0] < 0xF0 else 3
            ch += os.read(self.fd, more)
            now = time.time()
            if now - self.hint_at > 3.0:
                self.hint_at = now
                self.say("한글 입력이 들어왔다(%s). 키는 영문으로 받는다. 한/영 키로 영문으로 바꾼다"
                         % ch.decode("utf-8", "replace"))
            return None
        return ch.decode("utf-8", "replace")
