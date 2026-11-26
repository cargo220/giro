#!/usr/bin/env python3
# 장치 이름(ctrl1, cam1 …)과 실제 포트를 잇는다. 벨트·카메라가 두 대가 되어도 어느 장치를 쓸지 정해 둔다.
# 설정은 config/devices.json이다. 모든 도구가 포트 자리에 경로, 장치 이름, auto 가운데 하나를 받는다.
#
#   python3 devices.py                  # 연결된 장치와 설정을 견줘 보인다. 포트를 열지 않는다
#   python3 devices.py namespace line2  # 라인의 ROS namespace (giro_line.sh가 쓴다)
#
# 포트를 정하는 규칙
#   /dev/…        그대로 쓴다
#   장치 이름     devices.json의 그 장치 port. 그 값이 auto면 아래 auto 규칙
#   auto          그 종류 장치가 하나만 연결돼 있으면 그것. 둘 이상이면 고르지 않고 멈춘다
# 다기능 컨트롤러(CH340)는 장치 이름(by-id)에 일련번호가 없어 두 대가 같은 이름이 된다.
# 그래서 USB 꽂은 자리 기준 이름(/dev/serial/by-path/…)으로 적는다. 컨트롤러마다 꽂는 USB 자리를 정해 둔다.
# OpenMV는 장치 이름에 일련번호가 있어 by-id로 적는다.

import json
import sys
from pathlib import Path

import serial

ROOT = Path(__file__).resolve().parent
DEVICES = ROOT / "config" / "devices.json"
BY_ID = Path("/dev/serial/by-id")
BY_PATH = Path("/dev/serial/by-path")
KINDS = {"controller": "controllers", "camera": "cameras"}
LABEL = {"controller": "다기능 컨트롤러(CH340)", "camera": "OpenMV 카메라"}
DEFAULT = {
    "controllers": {"ctrl1": {"port": "auto", "note": "벨트1(D축)과 팔이 붙은 다기능 컨트롤러"}},
    "cameras": {"cam1": {"port": "auto", "note": "벨트1 측정용 OpenMV"}},
    "lines": {"line1": {"namespace": "", "controller": "ctrl1", "camera": "cam1", "params": "config/line.yaml"}},
}


class DeviceError(serial.SerialException):
    """포트를 정하지 못했다. 시리얼 오류를 받는 곳에서 그대로 잡히게 SerialException을 잇는다."""


def load(path=None):
    path = path or DEVICES
    if not path.exists():
        return json.loads(json.dumps(DEFAULT))
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def _links(folder):
    """{실제 장치: 링크 경로} 목록. 폴더가 없으면 빈 사전."""
    if not folder.is_dir():
        return {}
    return {str(p.resolve()): str(p) for p in sorted(folder.iterdir())}


def connected(kind):
    """연결된 그 종류 장치의 포트 목록. 컨트롤러는 by-path, 카메라는 by-id로 돌려준다."""
    by_id = _links(BY_ID)
    by_path = _links(BY_PATH)
    out = []
    if kind == "controller":
        # by-id 이름이 겹치면 링크가 하나만 남으니, by-path의 장치마다 by-id 이름을 따로 찾지 않고
        # sysfs의 제조사 번호로 CH340(1a86)인지 본다.
        for dev, link in by_path.items():
            if _vendor(dev) == "1a86":
                out.append(link)
        if not out:
            out = [link for dev, link in by_id.items() if "1a86" in link]
    else:
        out = [link for dev, link in by_id.items() if "OpenMV" in link or "MicroPython" in link]
    return out


def _vendor(dev):
    """/dev/ttyUSB0 같은 장치의 USB 제조사 번호. 모르면 None."""
    node = Path("/sys/class/tty") / Path(dev).name / "device"
    if not node.exists():
        return None
    # tty 장치의 몇 단계 위에 USB 장치 폴더가 있고, 그 안에 idVendor가 있다.
    real = node.resolve()
    for parent in [real] + list(real.parents)[:4]:
        vid = parent / "idVendor"
        if vid.exists():
            return vid.read_text().strip()
    return None


def resolve(kind, value, cfg=None):
    """포트 자리에 들어온 값(경로·장치 이름·auto)을 실제 포트로 바꾼다. 정하지 못하면 DeviceError."""
    value = (value or "auto").strip()
    if value.startswith("/"):
        return value
    cfg = cfg or load()
    named = cfg.get(KINDS[kind], {})
    name = None
    if value != "auto":
        if value not in named:
            raise DeviceError("모르는 %s 이름: %s (config/devices.json의 %s: %s)" % (
                LABEL[kind], value, KINDS[kind], ", ".join(named) or "없음"))
        name, value = value, (named[value].get("port") or "").strip()
        if not value:
            raise DeviceError("config/devices.json의 %s 포트가 비어 있다. python3 devices.py로 보고 적는다" % name)
        if value.startswith("/"):
            return value
    found = connected(kind)
    if len(found) == 1:
        return found[0]
    who = name or "auto"
    if not found:
        raise DeviceError("연결된 %s 없음 (%s)" % (LABEL[kind], who))
    raise DeviceError("%s %d대가 연결돼 있어 %s로는 고르지 않는다. config/devices.json에 포트를 적는다: %s" % (
        LABEL[kind], len(found), who, ", ".join(found)))


def line(name, cfg=None):
    cfg = cfg or load()
    lines = cfg.get("lines", {})
    if name not in lines:
        raise DeviceError("모르는 라인: %s (config/devices.json의 lines: %s)" % (name, ", ".join(lines)))
    return lines[name]


def report():
    cfg = load()
    for kind, key in KINDS.items():
        found = connected(kind)
        print("%s 연결 %d대" % (LABEL[kind], len(found)))
        for port in found:
            print("  %s" % port)
        for name, dev in cfg.get(key, {}).items():
            try:
                port = resolve(kind, name, cfg)
                state = "→ %s%s" % (port, "" if Path(port).exists() else "  (지금 없음)")
            except DeviceError as exc:
                state = "→ 못 정함: %s" % exc
            print("  설정 %-6s port %-8s %s" % (name, dev.get("port"), state))
    print("라인")
    for name, ln in cfg.get("lines", {}).items():
        print("  %-6s namespace %-8s 컨트롤러 %-6s 카메라 %-6s 보정 %s" % (
            name, ln.get("namespace") or "(없음)", ln.get("controller"), ln.get("camera"), ln.get("params")))
    if not DEVICES.exists():
        print("config/devices.json이 없어 기본값을 보였다")
    return 0


def main():
    if len(sys.argv) == 3 and sys.argv[1] == "namespace":
        try:
            print(line(sys.argv[2]).get("namespace", ""))
        except DeviceError as exc:
            print(exc, file=sys.stderr)
            return 1
        return 0
    if len(sys.argv) > 1:
        print("쓰는 법: python3 devices.py  또는  python3 devices.py namespace <라인>", file=sys.stderr)
        return 2
    return report()


if __name__ == "__main__":
    raise SystemExit(main())
