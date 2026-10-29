# 펌웨어 4.4.x 이하 OpenMV와 USB로 말한다. 옛 IDE 디버그 프로토콜이다.
# openmv 패키지(1.0.7)는 새 프로토콜만 알아서 H7 4.4.2에 붙지 못한다.
# 명령은 openmv/openmv v4.4.2의 tools/pyopenmv.py(MIT)를 따른다.
# 그 파일은 스크립트 길이를 글자 수로 보내 한글 주석이 있으면 틀어진다. 여기서는 바이트 수로 보낸다.
#
# 쓰는 쪽은 openmv.Camera와 같은 이름을 부른다.
#   connect() stop() exec(script) streaming(on) read_stdout() read_frame() disconnect()

import codecs
import struct
import time

import cv2
import numpy as np
import serial

CMD = 0x30
FW_VERSION = 0x80
FRAME_SIZE = 0x81
FRAME_DUMP = 0x82
SCRIPT_EXEC = 0x05
SCRIPT_STOP = 0x06
SCRIPT_RUNNING = 0x87
FB_ENABLE = 0x0D
TX_BUF_LEN = 0x8E
TX_BUF = 0x8F


def read_fw_version(port, timeout=1.0, tries=3):
    """옛 프로토콜로 펌웨어 버전을 묻는다. 답이 없거나 말이 안 되면 None이다.
    포트를 열 수 없으면(IDE가 잡고 있는 등) SerialException을 그대로 낸다."""
    for _ in range(tries):
        with serial.Serial(port, 921600, timeout=timeout) as s:
            s.reset_input_buffer()
            s.write(struct.pack("<BBI", CMD, FW_VERSION, 12))
            data = s.read(12)
        if len(data) != 12:
            return None
        version = struct.unpack("<III", data)
        # 앞서 돌던 스크립트의 출력이 섞이면 터무니없는 수가 온다. 그때는 다시 묻는다.
        if all(v < 100 for v in version):
            return version
        time.sleep(0.2)
    return None


class LegacyCamera:
    def __init__(self, port, timeout=1.0):
        self.port = port
        self.timeout = timeout
        self.serial = None
        self._decoder = codecs.getincrementaldecoder("utf-8")("ignore")

    def connect(self):
        self.serial = serial.Serial(self.port, 921600, timeout=self.timeout)
        self.serial.reset_input_buffer()

    def disconnect(self):
        if self.serial is not None:
            self.serial.close()
            self.serial = None

    def _command(self, op, length, payload=b""):
        # 머리 6바이트와 데이터는 따로 보낸다. 한 묶음으로 가면 펌웨어가 머리를 못 가른다.
        self.serial.write(struct.pack("<BBI", CMD, op, length))
        self.serial.flush()
        if payload:
            self.serial.write(payload)

    def _read_exact(self, n):
        data = self.serial.read(n)
        if len(data) != n:
            # 답이 덜 오면 다음 명령과 섞이지 않게 남은 것을 버린다.
            self.serial.reset_input_buffer()
            raise IOError("카메라 응답이 %d/%d 바이트에서 끊겼다" % (len(data), n))
        return data

    def script_running(self):
        self._command(SCRIPT_RUNNING, 4)
        return bool(struct.unpack("<I", self._read_exact(4))[0])

    def stop(self, timeout=4.0):
        # 4.4.2는 스크립트가 도는 중에 받은 새 스크립트를 버리면서도 받는 중 상태에 머문다.
        # 그 뒤 명령을 모두 스크립트로 삼켜 응답이 끊기니, 정말 멈출 때까지 기다린다.
        # 정지 명령이 한 번에 먹지 않을 때가 있어 0.5초마다 다시 보낸다.
        deadline = time.time() + timeout
        next_stop = 0.0
        while True:
            if time.time() >= next_stop:
                self._command(SCRIPT_STOP, 0)
                next_stop = time.time() + 0.5
            time.sleep(0.1)
            # 스크립트는 매 프레임 print한다. 출력 버퍼가 차면 보드가 print에서 막혀 정지를 못 받으니 비운다.
            self.read_stdout()
            if not self.script_running():
                break
            if time.time() > deadline:
                raise IOError("카메라 스크립트가 %.0f초 안에 멈추지 않았다" % timeout)
        self.serial.reset_input_buffer()

    def exec(self, script):
        if self.script_running():
            raise IOError("스크립트가 도는 중에는 새 스크립트를 올리지 않는다")
        data = script.encode("utf-8")
        self._command(SCRIPT_EXEC, len(data), data)

    def streaming(self, enable):
        self._command(FB_ENABLE, 4, struct.pack("<I", 1 if enable else 0))

    def read_stdout(self):
        self._command(TX_BUF_LEN, 4)
        size = struct.unpack("<I", self._read_exact(4))[0]
        if not size:
            return ""
        self._command(TX_BUF, size)
        # 한글 한 글자(3바이트)가 두 번에 나뉘어 올 수 있어 이어지는 디코더로 푼다.
        return self._decoder.decode(self._read_exact(size))

    def read_frame(self):
        """IDE 화면에 가는 프레임을 RGB888로 돌려준다. 아직 없으면 None이다."""
        self._command(FRAME_SIZE, 12)
        width, height, bpp = struct.unpack("<III", self._read_exact(12))
        if not width:
            return None
        # bpp가 2보다 크면 JPEG 바이트 수다. IDE용 프레임은 보드가 JPEG로 줄여 둔다.
        size = bpp if bpp > 2 else width * height * bpp
        self._command(FRAME_DUMP, size)
        raw = self._read_exact(size)
        if bpp > 2:
            bgr = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
            if bgr is None:
                return None
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        elif bpp == 2:
            px = np.frombuffer(raw, ">u2")
            r = ((px >> 11) & 0x1F) * 255 // 31
            g = ((px >> 5) & 0x3F) * 255 // 63
            b = (px & 0x1F) * 255 // 31
            rgb = np.stack([r, g, b], axis=-1).astype(np.uint8)
        else:
            rgb = np.repeat(np.frombuffer(raw, np.uint8)[:, None], 3, axis=1)
        rgb = rgb.reshape(height, width, 3)
        return {"width": width, "height": height, "data": rgb.tobytes()}
