#!/usr/bin/env python3
# OpenMV IDE가 포트를 잡고 있으면 이 스크립트는 연결하지 못한다.
# IDE 연결을 끊은 뒤 실행한다. 카메라에 camera.py를 올리고
# 프레임을 captures/에 PNG로 남긴다. 화면 창은 열지 않는다.

import argparse
import os
import sys
import time
from pathlib import Path

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

import logging

import pygame
import openmv.transport as omv_transport
from openmv import Camera
from openmv.constants import EventType

# openmv 1.0.7은 `opcode in Opcode`, `event in EventType`에서 TypeError가 난다.
# Python 3.10 IntEnum은 정수 멤버 검사를 예외로 막고, 그 예외가 연결을 끊는다.
def _log_off(self, *args, **kwargs):
    return None


omv_transport.Transport.log = _log_off


def _event_name(event):
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


def save_png(path, frame):
    width = frame["width"]
    height = frame["height"]
    surface = pygame.image.frombuffer(frame["data"], (width, height), "RGB")
    pygame.image.save(surface, str(path))


def default_port():
    by_id = Path("/dev/serial/by-id")
    if by_id.is_dir():
        found = sorted(by_id.glob("*MicroPython*"))
        if found:
            return str(found[0])
    return "/dev/ttyACM0"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default=default_port())
    parser.add_argument("--script", default=str(Path(__file__).with_name("camera.py")))
    parser.add_argument("--out", default=str(Path(__file__).with_name("captures")))
    parser.add_argument("--count", type=int, default=3)
    parser.add_argument("--wait", type=float, default=8.0)
    args = parser.parse_args()

    script_path = Path(args.script)
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    script = script_path.read_text(encoding="utf-8")

    def pull_text(cam, log):
        text = cam.read_stdout()
        if not text:
            return ""
        sys.stdout.write(text)
        if log is not None:
            log.write(text)
            log.flush()
        return text

    pygame.init()
    log_path = out_dir / "stdout.txt"
    saved = 0

    with Camera(args.port) as cam:
        # 이전 스크립트를 멈추면 보드가 soft reboot 한다.
        # 그 재부팅이 끝난 뒤에 올려야 새 스크립트가 지워지지 않는다.
        cam.stop()
        deadline = time.time() + 4.0
        while time.time() < deadline:
            pull_text(cam, None)
            time.sleep(0.05)
        cam.exec(script)
        cam.streaming(True)
        started = time.time()
        text_buf = ""
        ready = False
        with log_path.open("w", encoding="utf-8") as log:
            while saved < args.count and (time.time() - started) < args.wait:
                text_buf += pull_text(cam, log)
                if "잠금" in text_buf or "Traceback" in text_buf:
                    ready = True
                frame = cam.read_frame()
                if frame is None or not ready:
                    time.sleep(0.05)
                    continue
                saved += 1
                path = out_dir / ("frame-%02d.png" % saved)
                save_png(path, frame)
                print("saved %s %dx%d" % (path, frame["width"], frame["height"]))
                time.sleep(0.4)
            cam.streaming(False)
            cam.stop()

    if saved == 0:
        print("프레임을 받지 못했다.", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
