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

class ArmGuide(BaseGuide):
    def __init__(self):
        super().__init__()
        ## 将记录功能实现分到Logger类
        self.logger = LogModule(self)
        ## 将运动功能实现分到Motion类
        self.motion = MotionModule(self)

        class FeedbackItem:
            def __init__(self):
                self.QActual = -1
                self.VActual = -1
                self.last_update_time = time.time()
                self.rb_time = -1
        self.feedbackData = FeedbackItem()
    ## 抽象方法实现
    def _get_feedback(self):
        while not self._stop_event.is_set():
            try:
                feed = self.robot.get_states()

                # 无数据 → 继续下一次
                if not feed:
                    time.sleep(0.02)
                    continue

                arm_states = feed.get(self.selected_arm, None)
                if not arm_states:
                    time.sleep(0.02)
                    continue

                q_actual = arm_states.get("position")
                v_actual = arm_states.get("velocity")
                rb_t = arm_states.get("time")
                if isinstance(q_actual, (list, tuple)):
                    self.feedbackData.QActual = np.array(q_actual)
                    self.feedbackData.VActual = np.array(v_actual)
                    self.feedbackData.last_update_time = time.time()
                    secs = rb_t.get("secs")
                    nanos = rb_t.get("nanos") * 1e-9
                    self.feedbackData.rb_time = secs + nanos

                if self.record_flag.is_set():
                    self.logger.push_highfreq(rb_time=self.feedbackData.rb_time,
                                              feedback_pos=self.feedbackData.QActual,
                                              feedback_vel=self.feedbackData.VActual)

            except Exception as e:
                print(f"[反馈线程异常] {e}")
                # 给系统一点缓冲时间，避免异常狂刷
                time.sleep(0.1)
            time.sleep(0.004)
    def _load_config(self):
        os.system("clear")
        print(f"{CYAN}=== 选择左/右侧机械臂 ==={RESET}")
        opts = ["left_arm", "right_arm"]
        for i, opt in enumerate(opts,1):
            print(f"[{i}] {opt}")
        while True:
            key = self.get_single_key()
            if key in ['1','2']:
                self.selected_arm = opts[int(key)-1]
                break
        config_path = str(f"/opt/robot/rb_hardware/{self.selected_arm}.yaml")
        MyArm = ArmConfig.create(config_path)
        MyHardware = HardwareConfig.create_from_container({self.selected_arm: MyArm})
        MyRobot = RobotConfig.recreate({
            "hardware": MyHardware.to_dict_container(),
            "planner":None,
            "robot_model":""
        })
        self.config = MyRobot
        print(f"{GREEN}已选择机械臂: {self.selected_arm}{RESET}\n已加载配置文件: {config_path}")
    ## 重载函数
    def _init_robot(self):
        config = self.config.get_data()
        self.robot = rb_python.robot.Robot(config)
        self.robot.wait_for_operational(3)
        print("使用默认的位置模式.")
        self.robot.send_command(f"{self.selected_arm}", {"command": "set_control_mode", "mode": "position"})
        print(f"{GREEN}机器人初始化完成{RESET}")
        input("按回车键继续...")

    ## 任务菜单
    def task_verify(self):
        key_path = self.selected_arm + ".invert_directions"
        menu = {
            "0": {
                "description": "切换到力矩模式",
                "callback": lambda: self.motion.set_torque_mode(part=self.selected_arm),
            },
            "1": {
                "description": "移动机械臂到关节零位", 
                "callback": lambda: self.motion._run_point(q_name="QActual",
                                                            q_pos=[0.0]*7,
                                                            part=self.selected_arm),
            },
            "2": {
                "description": "少量正方向移动机械臂以验证方向",
                "callback": lambda: self.motion._run_point(q_name="QActual",
                                                            q_pos=[0.1]*7,
                                                            part=self.selected_arm),
            },
            "3": {
                "description": "修改配置",
                "callback": lambda: self.edit_config(key_path=key_path),
            },
            "4":{
                "description": "以csv文件走点",
                "callback": self.motion.move_multiple_points,
            }
        }
        self.push_menu(menu,"验证关节旋转方向正确")
    def task_tracking_error(self):
        key_path = self.selected_arm + ".max_velocity"
        menu = {
            "1": {
                "description": "设置速度限制参数", 
                "callback": self.motion.set_max_vel_acc,
            },
            "2": {
                "description": "梯形速度规划移动并测试跟随性",
                "callback": lambda: self.motion.cubic_move_record(start_data=self.feedbackData.QActual,
                                                          max_vel=[self.motion.max_vel]*7,
                                                          max_acc=[self.motion.max_acc]*7,
                                                          part=self.selected_arm,
                                                          pos_name="QActual"),
            },
            "3": {
                "description": "阶跃下发响应测试",
                "callback": lambda: self.motion.step_move_record(part=self.selected_arm,
                                                                 pos_name="QActual")
            },
            "4": {
                "description": "可选修改最大速度配置",
                "callback": lambda: self.edit_config(key_path=key_path),
            }
        }
        self.push_menu(menu,"跟踪误差测试")

if __name__ == "__main__":
    config = {
    "refresh_rate": "30 seconds",
    "appenders": {
        "stderr": {
            "kind": "console",
            "target": "stderr",
            "encoder": {"pattern": "{h({d(%Y-%m-%d %H:%M:%S.%3f)} [{t}] {l} {m})}{n}"},
        },
        "file": {
            "kind": "file",
            "path": "log/file.log",
            "encoder": {"pattern": "{h({d(%Y-%m-%d %H:%M:%S.%3f)} [{t}] {l} {m})}{n}"},
        },
    },
    "root": {"level": "debug", "appenders": ["stderr"]},
    "loggers": {},
    }
    # hblog.start(config)
    guide = ArmGuide()
    guide.push_menu({
        "1":{"description":"验证关节旋转方向正确","callback":guide.task_verify},
        "2":{"description":"跟踪误差测试","callback":guide.task_tracking_error},
    },"主菜单")
    guide.run()