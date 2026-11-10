# -*- coding: utf-8 -*-
"""python3 -m d1t_sdk <state|ping|home|free|lock|estop|reset|joint|line|gripper>"""
import sys

from .d1arm import _cli

if __name__ == "__main__":
    sys.exit(_cli())
