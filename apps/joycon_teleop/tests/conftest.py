# -*- coding: utf-8 -*-
"""pytest 路径准备：让 tests/ 里能 import joycon_teleop 与 d1t_sdk。

python3 -m pytest apps/joycon_teleop/tests -q（仓库根执行）；不需要控制器与手柄。
"""
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))               # apps/joycon_teleop（import joycon_teleop）
sys.path.insert(0, str(HERE.parents[2] / "sdk"))   # 仓库 sdk/（import d1t_sdk，joycon_teleop 也会自补）
