#!/usr/bin/env python3
# OpenMV에 올릴 스크립트에서 주석과 빈 줄을 뺀 사본을 만든다.
# IDE는 원문을 통째로 보드 RAM에 올려 그 자리에서 컴파일한다. H7은 RAM이 1 MB이고
# 프레임 버퍼를 빼면 힙이 작아, camera-ros.py가 80 KB를 넘자 MemoryError가 났다.
# 원문의 30%가 주석이라 주석만 빼도 여유가 생긴다. 코드는 한 글자도 바꾸지 않는다.
#
#   python3 board_script.py                 # camera-ros.py → build/camera-ros.py
#   python3 board_script.py camera.py       # camera.py → build/camera.py
#
# IDE에서는 build/ 아래 사본을 열어 실행한다. 고칠 때는 원본을 고치고 다시 만든다.
# 사본과 원본의 구문 트리가 같은지 확인하고, 다르면 쓰지 않는다.

import ast
import io
import sys
import tokenize
from pathlib import Path

ROOT = Path(__file__).resolve().parent
BUILD = ROOT / "build"


def strip_comments(source):
    tokens = list(tokenize.generate_tokens(io.StringIO(source).readline))
    cut = {}       # 줄 번호 → 주석이 시작하는 칸
    keep = set()   # 여러 줄 문자열 안의 줄은 비어 있어도 지우지 않는다
    for tok in tokens:
        if tok.type == tokenize.COMMENT:
            cut[tok.start[0]] = tok.start[1]
        elif tok.type == tokenize.STRING and tok.start[0] != tok.end[0]:
            keep.update(range(tok.start[0] + 1, tok.end[0] + 1))
    out = []
    for row, line in enumerate(source.splitlines(), 1):
        if row in cut:
            line = line[:cut[row]]
        line = line.rstrip()
        if not line and row not in keep:
            continue
        out.append(line)
    return "\n".join(out) + "\n"


def same_code(a, b):
    return ast.dump(ast.parse(a)) == ast.dump(ast.parse(b))


def build(name="camera-ros.py"):
    src_path = ROOT / name
    source = src_path.read_text(encoding="utf-8")
    stripped = strip_comments(source)
    if not same_code(source, stripped):
        raise SystemExit("주석을 빼다 코드가 달라졌다. %s를 쓰지 않는다." % name)
    BUILD.mkdir(exist_ok=True)
    out_path = BUILD / name
    out_path.write_text(stripped, encoding="utf-8")
    return src_path, out_path


def main():
    name = sys.argv[1] if len(sys.argv) > 1 else "camera-ros.py"
    src_path, out_path = build(name)
    a = src_path.stat().st_size
    b = out_path.stat().st_size
    print("%s %.1f KB → %s %.1f KB (%.0f%%)" % (
        src_path.name, a / 1024, out_path.relative_to(ROOT), b / 1024, 100 * b / a))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
