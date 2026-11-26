# 판정용 원본 프레임을 PC로 보낸다. OpenMV 보드에서 돈다.
# 화면에 아무것도 그리지 않고, JPEG로 줄이지 않는다. RGB565 픽셀을 그대로 보낸다.
# PC의 capture_dataset.py가 이 파일을 올리고, 아래 대문자 설정 줄을 바꿔 실행한다.
# IDE에서 직접 열어도 돌지만, 그때는 터미널에 base64 줄이 쏟아진다.
#
# IDE 화면으로 가는 프레임은 보드가 JPEG로 줄이고, camera-ros.py는 그 위에 테두리와
# 이름을 그린다. 그 프레임으로 판정하면 압축에 뭉개진 링과 그려진 선까지 꺾임으로 잡힌다.
# 그래서 판정용 프레임은 print() 줄로 따로 보낸다.
#
# 한 프레임은 이렇게 나간다. 줄마다 print() 한 번이다.
#   FRAME <번호> <가로> <세로> <노출 µs> <게인 dB>
#   PX <x> <y> <r> <g> <b>    get_pixel()로 읽은 표본 픽셀. PC가 바이트 순서를 맞춰 본다
#   ROW <y> <base64>          RGB565 한 줄, 가로×2 바이트
#   END <번호>
# 출력 버퍼가 차면 print()가 PC가 읽을 때까지 기다린다. 줄을 잃지 않는 대신 느리다.

import sensor
import binascii

# 차례로 돌며 찍는다. 둘 이상이면 노출을 바꿀 때마다 SETTLE_FRAMES를 버린다.
# 링 띠가 잘리지 않는 노출과 도장 색이 직선 구간에 드는 노출을 번갈아 남길 때 쓴다.
EXPOSURES_US = (7500,)
# 게인과 채널은 camera-ros.py와 같은 값으로 잠근다. 색 판별과 같은 조건으로 찍기 위해서다.
GAIN_DB = 24.0
RGB_GAIN_DB = (64.4, 60.0, 63.7)
SETTLE_FRAMES = 5
# 바이트 순서를 맞춰 볼 자리. 화면 네 귀퉁이 근처다.
SAMPLES = ((40, 30), (280, 30), (40, 210), (280, 210))

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False, gain_db=GAIN_DB)
sensor.skip_frames(time=200)
try:
    sensor.set_auto_whitebal(False, rgb_gain_db=RGB_GAIN_DB)
except Exception:
    sensor.set_auto_whitebal(False)
    print("채널 게인을 쓰지 못함")
sensor.skip_frames(time=200)

index = 0
sensor.set_auto_exposure(False, exposure_us=EXPOSURES_US[index])
skip = SETTLE_FRAMES
seq = 0

while True:
    img = sensor.snapshot()
    if skip:
        skip -= 1
        continue
    seq += 1
    w = img.width()
    h = img.height()
    print("FRAME %d %d %d %d %.1f" % (seq, w, h, sensor.get_exposure_us(), sensor.get_gain_db()))
    for x, y in SAMPLES:
        p = img.get_pixel(x, y)
        print("PX %d %d %d %d %d" % (x, y, p[0], p[1], p[2]))
    buf = memoryview(img.bytearray())
    step = w * 2
    for y in range(h):
        line = binascii.b2a_base64(buf[y * step:(y + 1) * step]).decode().strip()
        print("ROW %d %s" % (y, line))
    print("END %d" % seq)
    if len(EXPOSURES_US) > 1:
        index = (index + 1) % len(EXPOSURES_US)
        sensor.set_auto_exposure(False, exposure_us=EXPOSURES_US[index])
        skip = SETTLE_FRAMES
