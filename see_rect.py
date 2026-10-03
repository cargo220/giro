#!/usr/bin/env python3
# 컴퓨터가 프레임에서 육면체를 찾는다. 색 이름은 카메라가 낸다.
# 색 중심이 없는 사각형만 카메라의 hint 채널로 보낸다.
# 1은 어두움(노출을 올림), 2는 밝음(노출을 내림), 3은 둘 다, 0은 없음.
# OpenMV IDE가 포트를 잡고 있으면 연결하지 않는다. IDE를 완전히 종료한 뒤 실행한다.

import argparse
import os
import struct
import subprocess
import sys
import time
from pathlib import Path

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

import logging

import cv2
import numpy as np
import openmv.transport as omv_transport
from openmv import Camera
from openmv.constants import EventType

# openmv 1.0.7은 패킷 로그에서 `opcode in Opcode`를 호출한다.
# Python 3.10의 IntEnum은 그 검사를 TypeError로 막는다. 로그는 쓰지 않는다.
def _log_off(self, *args, **kwargs):
    return None


omv_transport.Transport.log = _log_off


def _event_name(event):
    # `event in EventType`도 같은 TypeError다. 값으로 찾아서 없으면 번호만 남긴다.
    try:
        return EventType(event).name
    except ValueError:
        return "0x%04X" % event


def _handle_event(self, channel_id, event):
    if channel_id == 0:
        logging.info("system event %s", _event_name(event))
        if event == EventType.SOFT_REBOOT:
            logging.info("soft reboot")
        elif event == EventType.CHANNEL_REGISTERED:
            self.pending_channel_events += 1
        return
    if channel_id in self.channels_by_id:
        channel = self.channels_by_id[channel_id]["name"]
        if channel == "stream":
            self.frame_event = True
        return
    logging.warning("unknown event channel %s event %s", channel_id, event)


Camera._handle_event = _handle_event

HINT_TEXT = {0: "없음", 1: "어두움", 2: "밝음", 3: "둘다"}


def default_port():
    by_id = Path("/dev/serial/by-id")
    if by_id.is_dir():
        found = sorted(by_id.glob("*MicroPython*"))
        if found:
            return str(found[0])
    return "/dev/ttyACM0"


def port_busy(port):
    real = os.path.realpath(port)
    found = subprocess.run(["fuser", real], capture_output=True, text=True)
    return found.returncode == 0


def find_rects(data, width, height, centers):
    # 한 면은 사각형, 면이 둘이나 셋이면 외곽이 육각형이다.
    # 그 꼭짓수일 때만 육면체로 본다. 길거나 들쭉날쭉한 덩어리는 버린다.
    # 색 중심이 그 안에 있으면 이미 측정된 육면체다.
    arr = np.frombuffer(data, dtype=np.uint8).reshape(height, width, 3)
    r = arr[:, :, 0].astype(np.int16)
    g = arr[:, :, 1].astype(np.int16)
    b = arr[:, :, 2].astype(np.int16)
    bright = (r * 3 + g * 6 + b) // 10
    chroma = np.maximum(np.maximum(r, g), b) - np.minimum(np.minimum(r, g), b)
    mask = ((bright > 28) & (chroma > 16)).astype(np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    count, labels, stats, _cent = cv2.connectedComponentsWithStats(mask, 8)
    missed = []
    seen = 0
    i = 1
    while i < count:
        x, y, w, h, area = (int(v) for v in stats[i])
        if area >= 400 and w >= 16 and h >= 16 and area <= width * height * 0.45:
            if _cube_outline(labels, i, x, y, w, h):
                seen += 1
                if not _holds(x, y, w, h, centers):
                    kind = _kind(bright, labels, i, x, y, w, h)
                    if kind == "dark" or kind == "bright":
                        missed.append((x, y, w, h, kind))
        i += 1
    return seen, missed


def _cube_outline(labels, index, x, y, w, h):
    # 카메라에 보이는 육면체는 면 하나면 4꼭짓, 면이 더 보이면 5나 6꼭짓이다.
    view = np.zeros((h, w), np.uint8)
    part = labels[y:y + h, x:x + w]
    view[part == index] = 255
    contours, _hier = cv2.findContours(view, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return False
    cnt = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(cnt)
    peri = cv2.arcLength(cnt, True)
    if area < 300 or peri < 1.0:
        return False
    approx = cv2.approxPolyDP(cnt, 0.04 * peri, True)
    if len(approx) > 6:
        approx = cv2.approxPolyDP(cnt, 0.08 * peri, True)
    n = len(approx)
    # 원은 둘레가 둥글어 원형도가 높다. 육면체 면은 직선이라 그보다 낮다.
    circ = 12.566 * area / (peri * peri)
    if n < 4 or n > 6 or circ > 0.86 or not cv2.isContourConvex(approx):
        return False
    hull_area = cv2.contourArea(cv2.convexHull(cnt))
    if hull_area < 1.0 or area / hull_area < 0.82:
        return False
    (_c, (rw, rh), _a) = cv2.minAreaRect(cnt)
    short = min(rw, rh)
    long = max(rw, rh)
    if short < 12.0 or long / short > (2.2 if n == 4 else 2.6):
        return False
    return True


def _holds(x, y, w, h, centers):
    pad = 6
    for cx, cy, _nid in centers:
        if (x - pad) <= cx < (x + w + pad) and (y - pad) <= cy < (y + h + pad):
            return True
    return False


def _kind(bright, labels, index, x, y, w, h):
    view = bright[y:y + h, x:x + w]
    sel = labels[y:y + h, x:x + w] == index
    if not np.any(sel):
        return None
    vals = view[sel]
    mean = float(vals.mean())
    hot = float((vals > 230).mean())
    if hot > 0.12 or mean > 200:
        return "bright"
    if mean < 80:
        return "dark"
    return "mid"


def centers_from(buf):
    if not buf or len(buf) < 2:
        return None
    n = buf[1]
    if len(buf) < 2 + n * 5:
        return None
    out = []
    i = 0
    while i < n:
        cx, cy, nid = struct.unpack_from("<HHB", buf, 2 + i * 5)
        out.append((cx, cy, nid))
        i += 1
    return out


def hint_byte(missed):
    dark = False
    bright = False
    for _x, _y, _w, _h, kind in missed:
        if kind == "dark":
            dark = True
        elif kind == "bright":
            bright = True
    if dark and bright:
        return 3
    if dark:
        return 1
    if bright:
        return 2
    return 0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default=default_port())
    parser.add_argument("--script", default=str(Path(__file__).with_name("camera-2.py")))
    args = parser.parse_args()
    if port_busy(args.port):
        print("포트가 사용 중이다. OpenMV IDE를 완전히 종료한 뒤 다시 실행한다.", file=sys.stderr)
        return 1
    script = Path(args.script).read_text(encoding="utf-8")
    last_send = None
    hold_hint = None
    hold_n = 0
    with Camera(args.port) as cam:
        cam.stop()
        deadline = time.time() + 4.0
        while time.time() < deadline:
            text = cam.read_stdout()
            if text:
                sys.stdout.write(text)
            time.sleep(0.05)
        cam.exec(script)
        cam.streaming(True)
        text_buf = ""
        ready = False
        channels = False
        try:
            while True:
                text = cam.read_stdout()
                if text:
                    sys.stdout.write(text)
                    text_buf += text
                    if "잠금" in text_buf or "Traceback" in text_buf:
                        ready = True
                if ready and not channels:
                    # 채널은 스크립트가 시작된 뒤에 생긴다. 연결 때의 목록에는 없다.
                    cam.update_channels()
                    channels = cam.has_channel("faces") and cam.has_channel("hint")
                    if not channels:
                        print("faces 채널이 없다.", file=sys.stderr)
                        return 1
                frame = cam.read_frame()
                if not ready or not channels or frame is None:
                    time.sleep(0.02)
                    continue
                raw = cam.channel_read("faces")
                centers = centers_from(raw)
                if centers is None:
                    time.sleep(0.02)
                    continue
                seen, missed = find_rects(frame["data"], frame["width"], frame["height"], centers)
                hint = hint_byte(missed)
                # 한 프레임짜리 어두운 덩어리는 바닥 잡음이다. 세 프레임 같을 때만 보낸다.
                if hint != hold_hint:
                    hold_hint = hint
                    hold_n = 1
                else:
                    hold_n += 1
                send = hint if hint == 0 or hold_n >= 3 else 0
                if cam.has_channel("hint"):
                    try:
                        cam.channel_write("hint", bytes([send]))
                    except Exception as exc:
                        print("힌트 전달 실패 %s" % exc)
                        time.sleep(0.05)
                        continue
                if send != last_send:
                    print("육면체 %d 색 %d 힌트 %s" % (seen, len(centers), HINT_TEXT[send]))
                    last_send = send
        except KeyboardInterrupt:
            pass
        finally:
            cam.streaming(False)
            cam.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
