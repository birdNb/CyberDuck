#!/usr/bin/env python3
"""已迁到 sport_ctrl/wheel_run/tune_zero.py。

用法：
  sudo systemctl stop wheel-drive
  cd /home/aidlux/cyberDuck/sport_ctrl
  python3 wheel_run/tune_zero.py
"""
from __future__ import annotations

import runpy
import sys
from pathlib import Path

TARGET = Path(__file__).resolve().parents[2] / "sport_ctrl" / "wheel_run" / "tune_zero.py"


def main() -> int:
    print(f"请改用: python3 {TARGET}", flush=True)
    if not TARGET.is_file():
        print(f"找不到 {TARGET}", file=sys.stderr)
        return 2
    sys.argv[0] = str(TARGET)
    runpy.run_path(str(TARGET), run_name="__main__")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
