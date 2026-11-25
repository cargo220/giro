# OpenMV USB 포트를 찾고, 펌웨어에 맞는 방식으로 연다.
# block_bridge_node.py와 capture_dataset.py가 같이 쓴다. ROS 없이도 불러올 수 있게 따로 둔다.
#
# 통신 방식은 펌웨어에 따라 다르다. protocol "auto"면 펌웨어 버전을 묻고 고른다.
#   4.4.x 이하(제출용 H7 4.4.2)  옛 IDE 프로토콜, omv_legacy.py
#   그보다 새 펌웨어(RT1062)     openmv 패키지

import os
import time
from pathlib import Path

# see_cam이 불러오는 pygame의 인사 문구를 로그에 남기지 않는다.
os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")

# see_cam이 openmv 1.0.7의 이벤트 처리를 고쳐 둔다. 같은 고침을 그대로 쓴다.
import see_cam
from openmv import Camera

from omv_legacy import LegacyCamera, read_fw_version

# 옛 IDE 프로토콜을 쓰는 마지막 펌웨어다. 이보다 새 펌웨어는 openmv 패키지로 붙는다.
LEGACY_MAX_FW = (4, 4)
# 이전 스크립트를 멈추면 보드가 soft reboot 한다. 그 뒤에 올려야 새 스크립트가 남는다.
REBOOT_WAIT = 4.0


def find_camera_port():
    # 펌웨어마다 USB 이름이 다르다. H7 4.4.2는 OpenMV, RT1062는 MicroPython으로 잡힌다.
    by_id = Path("/dev/serial/by-id")
    if by_id.is_dir():
        for pattern in ("*OpenMV*", "*MicroPython*"):
            found = sorted(by_id.glob(pattern))
            if found:
                return str(found[0])
    return see_cam.default_port()


def open_camera(port, protocol, logger):
    """(카메라, 펌웨어 버전)을 돌려준다. 버전을 묻지 않았거나 답이 없으면 None이다."""
    fw = None
    if protocol == "auto":
        # 켜진 직후나 앞 스크립트를 정리하는 중에는 한동안 답이 없을 때가 있다. 5초까지 다시 묻는다.
        deadline = time.time() + 5.0
        fw = read_fw_version(port)
        while fw is None and time.time() < deadline:
            time.sleep(0.5)
            fw = read_fw_version(port)
        protocol = "legacy" if fw is not None and tuple(fw[:2]) <= LEGACY_MAX_FW else "new"
        logger.info("펌웨어 %s, %s 방식" % (".".join(map(str, fw)) if fw else "모름", protocol))
    cam = LegacyCamera(port) if protocol == "legacy" else Camera(port)
    cam.connect()
    return cam, fw
