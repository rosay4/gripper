# ===== 标准库 =====
import os
import sys
import time
import threading
import select
import termios
import tty
import readline
import pprint
import json
from pathlib import Path
import curses
import subprocess

# ===== 第三方 =====
import numpy as np
import rb_python
import hblog
import pinocchio as pin

# ===== 工程路径 =====
cur_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(cur_dir)
folder_utils = os.path.join(project_root, 'utils')
if folder_utils not in sys.path:
    sys.path.insert(0, folder_utils)

# ===== 工程模块 =====
from visualize import draw
from topp import TOPP
from robotConfig_util import *
from logger_util import setup_logger
from base_guide import BaseGuide
# from dommy_base_guide import BaseGuide
from logModule import LogModule
from motionModule import MotionModule

# ===== 常量 =====
GREEN = "\x1b[32m"
YELLOW = "\x1b[33m"
CYAN = "\x1b[36m"
RESET = "\x1b[0m"
CONTROL_HZ = 20