import board
from adafruit_ov7670 import (  # pylint: disable=unused-import
    OV7670,
    OV7670_SIZE_DIV8, # 80x60 image
    OV7670_COLOR_YUV,
)

import sys
import time

import digitalio
import busio

with digitalio.DigitalInOut(board.GP10) as reset:
    reset.switch_to_output(False)
    time.sleep(0.001)
    bus = busio.I2C(scl=board.GP5, sda=board.GP4)

cam = OV7670(
    bus,
    data_pins=[
        board.GP12,
        board.GP13,
        board.GP14,
        board.GP15,
        board.GP16,
        board.GP17,
        board.GP18,
        board.GP19,
    ],
    clock=board.GP11,
    vsync=board.GP7,
    href=board.GP21,
    mclk=board.GP20,
    shutdown=board.GP22,
    reset=board.GP10,
)

pid = cam.product_id
ver = cam.product_version
#print(f"Detected pid={pid:x} ver={ver:x}")

cam.size = OV7670_SIZE_DIV8
#print(cam.width)
#print(cam.height)

cam.colorspace = OV7670_COLOR_YUV
cam.flip_y = True

buf = bytearray(2 * cam.width * cam.height)
chars = b" .:-=+*#%@"

width = cam.width
row = bytearray(2 * width)

sys.stdout.write("\033[2J")
while True:
    cam.capture(buf)
    for j in range(cam.height):
        sys.stdout.write(f"\033[{j}H")
        for i in range(cam.width):
            row[i * 2] = row[i * 2 + 1] = chars[
                buf[2 * (width * j + i)] * (len(chars) - 1) // 255
            ]
        sys.stdout.write(row)
        sys.stdout.write("\033[K")
    sys.stdout.write("\033[J")
    time.sleep(0.05)