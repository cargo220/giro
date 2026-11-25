#!/usr/bin/env python3
# 학습용 원본 프레임을 모은다. 카메라에 camera-capture.py를 올리고, 받은 프레임을
# dataset/frames/<부품 ID>/ 아래에 PNG와 기록 JSON으로 남긴다.
# IDE가 카메라 포트를 잡고 있으면 연결하지 못하니 IDE 연결을 끊고 실행한다.
# ROS는 쓰지 않는다. 포트가 하나라 라인(block_bridge_node.py)과 동시에 돌리지 않는다.
#
#   python3 capture_dataset.py --part P001
#   python3 capture_dataset.py --part P001 --exposure 3000 7500   # 두 노출을 번갈아 찍는다
#   python3 capture_dataset.py --part P001 --auto 0.5             # 0.5초마다 저장. 벨트로 훑을 때
#   python3 capture_dataset.py --part M001 --angle 0 --angle-step 15
#
# 처음 쓰는 부품이면 dataset/parts/<부품 ID>.json 틀을 만들고 멈춘다. 채운 뒤 다시 실행한다.
# 장비 기록 dataset/rig.json도 같다. 카메라나 링 높이를 바꾸면 이 파일을 고친다.
#
# 창 키
#   s, 스페이스  지금 프레임 저장        a  자동 저장 켜고 끄기
#   [ ]          각도를 한 칸 내리고 올린다 (--angle을 줬을 때)
#   q, Esc       끝
#
# 프레임은 RGB565 그대로 받아 RGB888 PNG로 남긴다. 5·6비트를 8비트로 늘린 것이라 잃는 값이 없다.
# 미리보기 창의 글자는 창에만 그리고 저장하는 프레임에는 넣지 않는다.

import argparse
import base64
import logging
import os
import re
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

from board_script import strip_comments
from dataset_meta import (FRAMES, PART_TEMPLATE, RIG, RIG_TEMPLATE, check_part, check_rig,
                          load_json, part_path, write_json)

BOARD_SCRIPT = Path(__file__).with_name("camera-capture.py")
# 표본 픽셀과 이만큼 넘게 다르면 바이트 순서를 믿지 않는다. 5비트를 8비트로 늘리는 반올림 차이보다 크다.
SAMPLE_TOL = 8.0
WB_FAIL = "채널 게인을 쓰지 못함"

log = logging.getLogger("capture")


class FrameParser:
    """camera-capture.py의 print() 줄을 모아 한 프레임씩 내놓는다."""

    def __init__(self):
        self.buf = ""
        self.cur = None
        self.broken = 0

    def feed(self, text):
        """(완성된 프레임 목록, 프레임이 아닌 줄 목록)을 돌려준다."""
        frames, messages = [], []
        self.buf += text
        # 줄 가운데서 끊긴 마지막 조각은 다음 읽기까지 남긴다.
        *lines, self.buf = self.buf.split("\n")
        for line in lines:
            line = line.strip()
            if not line:
                continue
            head, _, rest = line.partition(" ")
            if head == "FRAME":
                if self.cur is not None:
                    self.broken += 1
                seq, w, h, expo, gain = rest.split()
                self.cur = {"seq": int(seq), "width": int(w), "height": int(h),
                            "exposure_us": int(expo), "gain_db": float(gain),
                            "samples": [], "rows": [None] * int(h)}
            elif head == "PX" and self.cur is not None:
                self.cur["samples"].append(tuple(int(v) for v in rest.split()))
            elif head == "ROW" and self.cur is not None:
                y, data = rest.split(" ", 1)
                row = base64.b64decode(data)
                y = int(y)
                if len(row) == self.cur["width"] * 2 and 0 <= y < self.cur["height"]:
                    self.cur["rows"][y] = row
            elif head == "END":
                cur, self.cur = self.cur, None
                if cur is None or cur["seq"] != int(rest) or any(r is None for r in cur["rows"]):
                    self.broken += 1
                    continue
                cur["raw"] = b"".join(cur.pop("rows"))
                frames.append(cur)
            else:
                messages.append(line)
        return frames, messages


def rgb565_to_rgb(raw, width, height, order):
    px = np.frombuffer(raw, "<u2" if order == "little" else ">u2").astype(np.uint32)
    r = ((px >> 11) & 0x1F) * 255 // 31
    g = ((px >> 5) & 0x3F) * 255 // 63
    b = (px & 0x1F) * 255 // 31
    return np.stack([r, g, b], axis=-1).astype(np.uint8).reshape(height, width, 3)


def decode(frame):
    """바이트 순서 둘을 다 풀어 보고, 보드가 get_pixel()로 읽은 표본에 맞는 쪽을 고른다."""
    best = None
    for order in ("little", "big"):
        rgb = rgb565_to_rgb(frame["raw"], frame["width"], frame["height"], order)
        if frame["samples"]:
            err = float(np.mean([np.abs(rgb[y, x].astype(int) - (r, g, b)).mean()
                                 for x, y, r, g, b in frame["samples"]]))
        else:
            err = float("nan")
        if best is None or err < best[2]:
            best = (rgb, order, err)
    return best


def next_index(folder):
    nums = [int(p.stem) for p in folder.glob("*.png") if p.stem.isdigit()]
    return max(nums, default=0) + 1


def board_source(exposures):
    """보드 스크립트의 노출 줄을 바꾸고 주석을 뺀다. 잠그는 채널 값도 함께 돌려준다.
    게인과 노출은 보드가 프레임마다 실제 값을 알려 주므로 여기서 읽지 않는다."""
    text = BOARD_SCRIPT.read_text(encoding="utf-8")
    value = "(%s,)" % ", ".join(str(int(e)) for e in exposures)
    text, n = re.subn(r"^EXPOSURES_US = .*$", "EXPOSURES_US = %s" % value, text, flags=re.M)
    if n != 1:
        raise SystemExit("%s에서 EXPOSURES_US 줄을 찾지 못했다" % BOARD_SCRIPT.name)
    rgb = [float(v) for v in re.search(r"^RGB_GAIN_DB = \((.+)\)$", text, re.M).group(1).split(",")]
    return strip_comments(text), rgb


def ready_records(part_id):
    """부품·장비 기록을 읽는다. 없으면 틀을 만들고, 빈 칸이 있으면 알려 주고 None을 돌려준다."""
    ok = True
    path = part_path(part_id)
    if not path.exists():
        template = dict(PART_TEMPLATE, part_id=part_id)
        write_json(path, template)
        log.error("부품 기록 틀을 만들었다: %s  채운 뒤 다시 실행한다", path)
        ok = False
    else:
        problems = check_part(load_json(path), part_id)
        for p in problems:
            log.error("%s: %s", path.name, p)
        ok = ok and not problems
    if not RIG.exists():
        write_json(RIG, RIG_TEMPLATE)
        log.error("장비 기록 틀을 만들었다: %s  채운 뒤 다시 실행한다", RIG)
        ok = False
    else:
        problems = check_rig(load_json(RIG))
        for p in problems:
            log.error(p)
        ok = ok and not problems
    return load_json(RIG) if ok else None


def preview(rgb, status):
    view = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    view = cv2.resize(view, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST)
    # Hershey 글꼴은 한글을 못 그려서 창 글자는 영문으로 둔다.
    cv2.putText(view, status, (8, view.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(view, status, (8, view.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (255, 255, 255), 1, cv2.LINE_AA)
    return view


def main():
    ap = argparse.ArgumentParser(description="학습용 원본 프레임 수집")
    ap.add_argument("--part", required=True, help="부품 ID. dataset/parts/<ID>.json")
    ap.add_argument("--port", default="auto")
    ap.add_argument("--protocol", default="auto", choices=("auto", "legacy", "new"))
    ap.add_argument("--exposure", type=int, nargs="+", default=[7500], help="노출 µs, 여럿이면 번갈아")
    ap.add_argument("--auto", type=float, default=0.0, help="이 초마다 저장. 0이면 키로만")
    ap.add_argument("--count", type=int, default=0, help="이만큼 저장하면 끝. 0이면 끝없이")
    ap.add_argument("--angle", type=float, help="머플러 각도(°). 주면 [ ] 키로 바꾼다")
    ap.add_argument("--angle-step", type=float, default=15.0)
    ap.add_argument("--belt-mm", type=float, help="벨트 위치를 손으로 적을 때")
    ap.add_argument("--note", default="", help="이번 촬영 전체에 붙일 메모")
    ap.add_argument("--no-view", action="store_true", help="창 없이. --auto가 있어야 저장한다")
    args = ap.parse_args()
    logging.basicConfig(format="%(message)s", level=logging.INFO)

    if args.no_view and args.auto <= 0:
        log.error("--no-view에는 --auto가 필요하다. 창이 없으면 키로 저장할 수 없다")
        return 2
    if not args.no_view and not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
        log.error("화면이 없다. --no-view --auto 초 로 실행한다")
        return 2
    rig = ready_records(args.part)
    if rig is None:
        return 1
    source, rgb_gain = board_source(args.exposure)
    out_dir = FRAMES / args.part
    out_dir.mkdir(parents=True, exist_ok=True)
    index = next_index(out_dir)

    # 카메라 모듈은 여기서 불러온다. 기록이 덜 찼을 때는 openmv·pygame을 불러오지 않는다.
    from camera_link import REBOOT_WAIT, find_camera_port, open_camera
    port = find_camera_port() if args.port in ("", "auto") else args.port
    try:
        cam, fw = open_camera(port, args.protocol, log)
    except Exception as exc:
        hint = " (OpenMV IDE가 포트를 쓰고 있으면 연결을 끊는다)" if "busy" in str(exc) else ""
        log.error("카메라를 열지 못했다: %s%s", exc, hint)
        return 1

    parser = FrameParser()
    last = None          # (rgb, frame, order, err)
    saved_seq = None
    saved = 0
    auto_on = args.auto > 0
    next_auto = 0.0
    angle = args.angle
    wb_ok = True
    arrivals = []
    fresh, shown = False, None
    last_error = 0.0
    try:
        cam.stop()
        deadline = time.time() + REBOOT_WAIT
        while time.time() < deadline:
            cam.read_stdout()
            time.sleep(0.05)
        cam.exec(source)
        # IDE 화면 전송은 끈다. 켜 두면 보드가 프레임마다 JPEG를 만드느라 느려진다.
        cam.streaming(False)
        log.info("카메라 %s에서 %s 실행, 노출 %s µs", port, BOARD_SCRIPT.name,
                 " / ".join(map(str, args.exposure)))
        if not args.no_view:
            log.info("창에서 s 저장, a 자동 저장, q 끝")
        while True:
            try:
                text = cam.read_stdout()
            except IOError as exc:
                if time.time() - last_error > 1.0:
                    last_error = time.time()
                    log.warning("카메라 응답 오류: %s", exc)
                text = ""
            if text:
                frames, messages = parser.feed(text)
                for m in messages:
                    if m == WB_FAIL:
                        wb_ok = False
                    log.info("카메라: %s", m)
                for frame in frames:
                    rgb, order, err = decode(frame)
                    if not err <= SAMPLE_TOL:
                        log.warning("프레임 %d: 표본 픽셀과 %.1f 다르다. 바이트 순서를 믿지 않는다",
                                    frame["seq"], err)
                    last = (rgb, frame, order, err)
                    fresh = True
                    arrivals = [t for t in arrivals if t > time.time() - 5.0] + [time.time()]
            else:
                time.sleep(0.005)

            want = False
            now = time.time()
            if auto_on and last is not None and now >= next_auto and last[1]["seq"] != saved_seq:
                want = True
                next_auto = now + args.auto
            if not args.no_view and last is not None:
                # 새 프레임이 왔거나 글자가 바뀔 때만 다시 그린다. 키는 매 바퀴 받는다.
                fps = (len(arrivals) - 1) / (arrivals[-1] - arrivals[0]) if len(arrivals) > 1 else 0.0
                status = "%s  E%dus  %s  saved %d  %.1f fps" % (
                    args.part, last[1]["exposure_us"], "AUTO" if auto_on else "manual", saved, fps)
                if angle is not None:
                    status += "  ang %g" % angle
                if fresh or status != shown:
                    cv2.imshow("capture %s" % args.part, preview(last[0], status))
                    fresh, shown = False, status
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
                if key in (ord("s"), ord(" ")):
                    want = True
                elif key == ord("a"):
                    auto_on = not auto_on
                    next_auto = 0.0
                elif key == ord("]") and angle is not None:
                    angle += args.angle_step
                elif key == ord("[") and angle is not None:
                    angle -= args.angle_step
            if want and last is not None and last[1]["seq"] != saved_seq:
                rgb, frame, order, err = last
                name = "%06d" % index
                cv2.imwrite(str(out_dir / (name + ".png")), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
                write_json(out_dir / (name + ".json"), {
                    "part_id": args.part,
                    "file": name + ".png",
                    "time": datetime.now().astimezone().isoformat(timespec="seconds"),
                    "board_seq": frame["seq"],
                    "width": frame["width"],
                    "height": frame["height"],
                    "pixformat": "RGB565",
                    "byte_order": order,
                    "sample_error": None if err != err else round(err, 2),
                    "exposure_us": frame["exposure_us"],
                    "gain_db": frame["gain_db"],
                    "rgb_gain_db": rgb_gain if wb_ok else None,
                    "firmware": list(fw) if fw else None,
                    "angle_deg": angle,
                    "belt_mm": args.belt_mm,
                    "rig": rig,
                    "note": args.note,
                })
                saved_seq = frame["seq"]
                saved += 1
                index += 1
                log.info("저장 %s/%s.png  노출 %d µs%s", args.part, name, frame["exposure_us"],
                         "" if angle is None else "  각도 %g°" % angle)
                if args.count and saved >= args.count:
                    break
    except KeyboardInterrupt:
        pass
    finally:
        try:
            cam.stop()
        except Exception as exc:
            print("카메라 정리 중 오류: %s" % exc, file=sys.stderr)
        finally:
            cam.disconnect()
        if not args.no_view:
            cv2.destroyAllWindows()
    if parser.broken:
        log.warning("깨져서 버린 프레임 %d개", parser.broken)
    log.info("모두 %d장 저장, %s", saved, out_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
