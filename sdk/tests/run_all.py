# -*- coding: utf-8 -*-
"""离线测试一键跑：python3 tests/run_all.py（不上臂，不需要控制器）。"""
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / "d1t_sdk" / ".."))
sys.path.insert(0, str(HERE.parents[1]))  # sdk/，使 import d1t_sdk 可用


def main() -> int:
    loader = unittest.TestLoader()
    suite = loader.discover(str(HERE), top_level_dir=str(HERE))
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    n = result.testsRun
    print(f"\n===== 汇总: 共 {n} 项, 失败 {len(result.failures)}, "
          f"错误 {len(result.errors)} =====")
    return 0 if result.wasSuccessful() else 1


if __name__ == "__main__":
    sys.exit(main())
