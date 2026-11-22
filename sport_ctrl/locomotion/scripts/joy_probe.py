#!/usr/bin/env python3
"""实时显示手柄所有轴和按键，用来确认摇杆对应哪个 axis。Ctrl+C 退出。"""

import os
import struct
import sys

PATH = sys.argv[1] if len(sys.argv) > 1 else "/dev/input/js0"


def main() -> int:
    fd = os.open(PATH, os.O_RDONLY)
    axes = [0.0] * 8
    print(f"读取 {PATH}，依次推左摇杆、右摇杆、十字键，看哪个 axis 在变。")
    try:
        while True:
            _t, value, kind, number = struct.unpack("IhBB", os.read(fd, 8))
            kind &= ~0x80
            if kind == 0x02 and number < 8:
                axes[number] = value / 32767.0
                line = "  ".join(f"a{i} {v:+.2f}" for i, v in enumerate(axes))
                print(f"\r{line}", end="", flush=True)
            elif kind == 0x01 and value == 1:
                print(f"\n按键 {number}")
    except KeyboardInterrupt:
        print()
    finally:
        os.close(fd)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
