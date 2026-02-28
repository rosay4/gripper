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
from base_guide_old import BaseGuide
# from dommy_base_guide import BaseGuide
from logModule import LogModule
from motionModule_without_ui import MotionModule

# ===== 常量 =====
GREEN = "\x1b[32m"
YELLOW = "\x1b[33m"
CYAN = "\x1b[36m"
RESET = "\x1b[0m"
CONTROL_HZ = 20

class LoadcellGuide(BaseGuide):
    def __init__(self):
        super().__init__()
        self.logger = LogModule(self)
        self.motion = MotionModule(self)
        class FeedbackItem:
            def __init__(self):
                self.rail_pos = -1
                self.VActual = -1
                self.last_update_time = -1
                self.rb_time = -1
        self.feedbackData = FeedbackItem()
    ## 抽象函数实现
    def _get_feedback(self):
        while not self._stop_event.is_set():
            try:
                feed = self.robot.get_states()

                # 无数据 → 继续下一次
                if not feed:
                    time.sleep(0.02)
                    continue

                states = feed.get("rail", None)
                if not states:
                    time.sleep(0.02)
                    continue

                rail_pos = states.get("position")
                v_actual = states.get("velocity")
                rb_t = states.get("time")
                if isinstance(rail_pos, (list, tuple)):
                    self.feedbackData.rail_pos = np.array(rail_pos)
                    self.feedbackData.VActual = np.array(v_actual)
                    self.feedbackData.last_update_time = time.time()
                    secs = rb_t.get("secs")
                    nanos = rb_t.get("nanos") * 1e-9
                    self.feedbackData.rb_time = secs + nanos

                if self.record_flag.is_set():
                    self.logger.push_highfreq(rb_time=self.feedbackData.rb_time,
                                              feedback_pos=self.feedbackData.rail_pos,
                                              feedback_vel=self.feedbackData.VActual)

            except Exception as e:
                print(f"[反馈线程异常] {e}")
                # 给系统一点缓冲时间，避免异常狂刷
                time.sleep(0.1)
            time.sleep(0.004)
    def _load_config(self):
        os.system('clear')
        config_path = str(f"/opt/robot/rb_hardware/rail.yaml")
        MyRail = RailConfig.create(config_path)
        MyHardware = HardwareConfig.create_from_container({"rail": MyRail})
        MyRobot = RobotConfig.recreate({
            "hardware": MyHardware.to_dict_container(),
            "planner":None,
            "robot_model":""
        })
        self.config = MyRobot
        print(f"{GREEN}已选择导轨: rail{RESET}\n已加载配置文件: {config_path}")
    ## 菜单
    def task_tracking_error(self):
        key_path = "rail.max_velocity"
        menu = {
            "1": {
                "description": "设置速度限制参数", 
                "callback": self.motion.set_max_vel_acc,
            },
            "2": {
                "description": "梯形速度规划移动",
                "callback": lambda: self.motion.cubic_move_record(start_data=self.feedbackData.rail_pos,
                                                                  max_vel=[self.motion.max_vel],
                                                                  max_acc=[self.motion.max_acc],
                                                                  part="rail",
                                                                  pos_name="rail_pos"),
            },
            "3": {
                "description": "阶跃下发响应测试",
                "callback": lambda: self.motion.step_move_record(part="rail",
                                                                 pos_name="rail_pos"),
            },
            "4":{
                "description": "可选修改最大速度配置",
                "callback": lambda: self.edit_config(key_path=key_path),
            }
        }
        # 使用菜单栈
        self.push_menu(menu,"20Hz下的轨迹跟踪测试")
    
    def task_verify(self):
        key_path = "rail.invert_directions"
        menu = {
            "1": {
                "description": "移动导轨到零位", 
                "callback": lambda: self.motion._run_point(
                    q_name="rail_pos",
                    q_pos=0.0,
                    part="rail"
                ),
            },
            "2": {
                "description": "少量正方向移动导轨以验证方向",
                "callback": lambda: self.motion._run_point(
                    q_name="rail_pos",
                    q_pos=0.1,
                    part="rail"
                ),
            },
            "3": {
                "description": "以需要位置移动导轨(可跳过)",
                "callback": lambda: self.motion._run_point_with_input(
                    q_name="rail_pos",
                    dof=len(self.feedbackData.rail_pos),
                    part="rail"
                ),
            },
            "4": {
                "description": "修改配置",
                "callback": lambda: self.edit_config(key_path=key_path),
            },
            "5":{
                "description": "获取当前范围",
                "callback": lambda: self.motion.get_limits(part="rail"),
            },
            "6":{
                "description": "设置关节范围",
                "callback": lambda: self.motion.set_limits(part="rail",limit_low=0.0,limit_high=0.5)
            },
        }
        # 使用菜单栈
        self.push_menu(menu,"验证关节移动方向正确")
    
if __name__ == "__main__":
    guide = LoadcellGuide()
    guide.push_menu({
        "1":{"description":"验证关节移动方向正确","callback":guide.task_verify},
        "2":{"description":"轨迹跟踪测试","callback":guide.task_tracking_error},
    },"主菜单")
    guide.run()