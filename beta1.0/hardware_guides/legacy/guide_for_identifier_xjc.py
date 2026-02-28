from utils.config_manager import ConfigManager
import rb_python
import numpy as np
import pprint
import threading
from utils.logger_util import setup_logger
from utils.keyboard_util2 import KeyboardCommander, input_with_flush
from pathlib import Path
import time
import os, sys, select, termios, tty
import readline
import pinocchio as pin
from utils.topp import TOPP
from utils.visualize import draw,draw_wrench,draw_wrench_comp
import json
from datetime import datetime
import hblog
import signal
# ANSI 颜色码
GREEN = "\x1b[32m"
YELLOW = "\x1b[33m"
CYAN = "\x1b[36m"
RESET = "\x1b[0m"
readline.parse_and_bind("tab: complete")
BASE_DIR = Path(__file__).parent
CONTROL_HZ = 20
fd = sys.stdin.fileno()
ORIGINAL_TERMIOS = termios.tcgetattr(fd)

def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    global control_running
    control_running = False
    backend._robot.shutdown()
    print('Robot shutdown finish')
    exit()

def get_single_key(timeout=0.5):
    """返回单个按键，如果超时没有按键返回 None"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)  # 切换到 cbreak 模式（单字符可读）
        rlist, _, _ = select.select([fd], [], [], timeout)
        if rlist:
            return sys.stdin.read(1)  # 立即读取单个字符
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return None

class Guide:
    def __init__(self):
        
        class FeedbackItem:
            def __init__(self):
                self.QActual = -1
                self.Wrench = -1
                self.rb_time = -1
                # ... 自定义反馈数据
                '''
                self.Wrench = -1
                '''
        self.feedbackData = FeedbackItem() # 自定义反馈数据结构体
        # 标定过程计算
        self.M = None
        self.F = None
        self.R = None
        self.f = None
        self.len_data = 0

        # 状态管理
        self.running = True
        self.selected_arm = "" ## "left_arm" "right_arm"
        self.expected_sensor = "" ## "left_force_sensor" "right_force_sensor"
        self.feedback_thread = None
        self._stop_event = threading.Event()
        # pin calc
        self.urdf_file = "urdf/beta1_0_left_arm.urdf"
        self.model = pin.buildModelFromUrdf(self.urdf_file)
        self.data = self.model.createData()

        # DEBUG
        # self.mass_center_position = [-0.014012  ,0.006732 , 0.064750]
        # self.force_zero = [-5.912231,10.736421,-0.507257]
        # self.torque_zero = [-0.666468,-0.415116,-0.165188]
        # self.gravity_base = [-0.140460 ,  0.235275 ,-12.894626]

        # 日志
        self.record_flag = threading.Event()
        self.highfreq_log = []
        self.lowfreq_log = []

    ## 辨识
    
    def _add_data(self, wrench: np.ndarray, ee_pose: np.ndarray):
        force = wrench[:3].copy()
        torque = wrench[3:].copy()
        if self.M is None:
            self.M = torque.reshape((3, 1))
            f_skew = self.__skew_symmetric(force)
            self.F = np.hstack((f_skew, np.eye(3)))
        else:
            self.M = np.vstack((self.M, torque.reshape((3, 1))))
            f_skew = self.__skew_symmetric(force)
            self.F = np.vstack((self.F, np.hstack((f_skew, np.eye(3)))))
        
        if self.R is None:
            self.R = np.hstack((ee_pose.T, np.eye(3)))
            self.f = force.reshape((3, 1))
        else:
            self.R = np.vstack((self.R, np.hstack((ee_pose.T, np.eye(3)))))
            self.f = np.vstack((self.f, force.reshape((3, 1))))
        self.len_data += 1
    
    def _solve(self):
        A, residuals, rank, s = np.linalg.lstsq(self.F, self.M, rcond=None)
        B, residuals, rank, s = np.linalg.lstsq(self.R, self.f, rcond=None)
        
        x, y, z, k1, k2, k3 = A.flatten()
        Gx, Gy, Gz, Fx0, Fy0, Fz0 = B.flatten()
        
        mass_center = np.array([x, y, z])
        F_0 = np.array([Fx0, Fy0, Fz0])
        M_x0 = k1 - Fy0 * z + Fz0 * y
        M_y0 = k2 - Fz0 * x + Fx0 * z
        M_z0 = k3 - Fx0 * y + Fy0 * x
        M_0 = np.array([M_x0, M_y0, M_z0])
        gravity_base = np.array([Gx, Gy, Gz])
        return mass_center, F_0, M_0, gravity_base

    def _read_data_once(self,num):
        sum = np.zeros(6)
        i = 0
        # 读取num次数据，取平均值
        while i < num:
            data = np.array(self.feedbackData.Wrench,dtype=np.float32)
            if data is not None:
                sum += data
                i += 1 
            time.sleep(0.2)
        return  sum/i
    
    def __skew_symmetric(self, v: np.ndarray):
        """ 从3维向量生成对应的3x3反对称矩阵 """
        if len(v) != 3:
            raise ValueError("输入数组的长度必须为3!")
        return np.array([[0, v[2], -v[1]],
                        [-v[2], 0, v[0]],
                        [v[1], -v[0], 0]])
    
    def _set_joint(self,target_joints):
        """
        target_joints: list or np.array 7DOF
        """
        if isinstance(target_joints,np.ndarray):
            target_joints = target_joints.tolist()
        print(f"目标关节角度:{target_joints}")
        while np.max(np.abs(self.feedbackData.QActual-np.array(target_joints))) > 1e-3:
            self.robot.set_actions({self.selected_arm: {"type": "position", "position": target_joints}})
            time.sleep(1 / CONTROL_HZ)
        print("到达!")
        print(f"move done")
    
    def _getForwardKin(self,joints_list,end=0,start=8):
        """getForwardKin

        Args:
            joints_list (list): [q1,...,q7]
            end (int, optional): Defaults to 0.
            start (int, optional): Defaults to 9.

        Returns:
            numpy darray: 4x4 homogeneous matrix
        """
        joints_array = np.array(joints_list)
        pin.forwardKinematics(self.model,self.data,joints_array)
        pin.updateFramePlacements(self.model,self.data)
        if self.selected_arm == "left_arm":
            frame_id_9 = self.model.getFrameId("left-link_tcp")
            frame_id_8 = self.model.getFrameId("left-link_ee_ft_sensor")
        if self.selected_arm == "right_arm":
            frame_id_9 = self.model.getFrameId("right-link_tcp")
            frame_id_8 = self.model.getFrameId("right-link_ee_ft_sensor")
        if end==0 and start==9:
            T9o = self.data.oMf[frame_id_9].homogeneous
            T_res = T9o
        if end==0 and start==8:
            T8o = self.data.oMf[frame_id_8].homogeneous
            T_res = T8o
        if end==8 and start==9:
            M9o = self.data.oMf[frame_id_9]
            M8o = self.data.oMf[frame_id_8]
            M98 = M8o.inverse() * M9o
            T98 = M98.homogeneous
            T_res = T98
        return T_res

    def _identify_param_auto(self,sample_num = 6):
        q_list = [[-1.100000, 0.417000, 0.457000, 0.622000, 0.664000, 0.895000, -0.204000],
                    [-1.100000, 0.417000, 0.457000, 0.622000, 0.664000, 0.895000, 0.233000],
                    [-1.100000, 0.417000, 0.457000, 0.622000, 0.664000, 0.895000, 0.670000],
                    [-1.100000, 0.417000, 0.457000, 0.622000, 0.664000, 0.895000, -0.125000],
                    [-1.100000, 0.417000, 0.457000, 0.622000, 0.664000, 0.895000, -0.920000],
                    [-1.100000, 0.417000, 0.457000, 0.622000, 0.664000, 1.182500, -0.562000],
                    [-1.100000, 0.417000, 0.457000, 0.622000, 0.664000, 1.470000, -0.204000],
                    [-1.100000, 0.417000, 0.457000, 0.622000, 0.664000, 0.735000, -0.204000],
                    [-1.100000, 0.417000, 0.457000, 0.622000, 0.664000, 0.000000, -0.204000],
                    [-1.100000, 0.417000, 0.457000, 0.622000, 0.664000, -0.368000, -0.204000],
                    [-1.100000, 0.417000, 0.457000, 0.622000, 0.664000, -0.736000, -0.204000],
                    [-1.100000, 0.417000, 0.457000, 0.622000, 0.172500, 0.226500, 0.162000],
                    [-1.100000, 0.417000, 0.457000, 0.622000, -0.319000, 1.189000, 0.528000],
                    [-1.100000, 0.417000, 0.457000, 0.622000, 0.172500, 1.189000, 0.528000],
                    [-1.100000, 0.417000, 0.457000, 0.622000, 0.664000, 1.189000, 0.528000],
                    [-1.100000, 0.417000, 0.457000, 0.622000, 1.120000, 0.226500, 0.162000],
                    [-1.100000, 0.417000, 0.457000, 0.622000, 1.576000, -0.736000, -0.204000],
                    [-1.100000, 0.417000, 0.457000, 0.622000, 0.523000, -0.736000, -0.204000],
                    [-1.100000, 0.417000, 0.457000, 0.622000, -0.530000, -0.736000, -0.204000],
                    [-1.100000, 0.417000, 0.457000, 0.622000, 0.839500, 0.079500, -0.750000],
                    [-1.100000, 0.417000, 0.457000, 0.622000, 2.209000, 0.895000, -1.296000],
                    [-1.100000, 0.417000, 0.457000, 0.622000, -0.421500, 0.895000, -1.296000],
                    [-1.100000, 0.417000, 0.457000, 0.622000, -3.052000, 0.895000, -1.296000],
                    [-1.100000, 0.417000, 0.457000, 0.622000, -3.052000, 0.895000, -0.956000],
                    [-1.100000, 0.417000, 0.457000, 0.622000, -3.052000, 0.895000, -0.616000],
                    [-1.100000, 0.417000, 0.457000, 0.622000, -3.052000, 0.895000, 0.062000],
                    [-1.100000, 0.417000, 0.457000, 0.622000, -3.052000, 0.895000, 0.740000],
                    [-1.100000, 0.417000, 0.457000, 0.622000, -3.052000, -0.116000, 0.740000],
                    [-1.100000, 0.417000, 0.457000, 0.622000, -3.052000, -1.127000, 0.740000],
                    ]
        for i,q in enumerate(q_list):
            print("move to q:",q)
            q_calc = q + [0.0,0.0]
            self._set_joint(q)
            time.sleep(2)
            wrench = self._read_data_once(sample_num)
            print(f"wrench: {wrench}")
            print("read done")
            cur_pose = self._getForwardKin(q_calc,end=0,start=8) # tform
            b_R_e = cur_pose[:3,:3]
            self._add_data(wrench, b_R_e)
            print(f"=======添加了第{i+1}组数据=======")
            i += 1
            time.sleep(0.5)
        mass_center, F_0, M_0, gravity_base = self._solve()
        # output all results
        print(f"mass_center_position: {mass_center}")
        print(f"force_zero: {F_0}")
        print(f"torque_zero: {M_0}")
        print(f"gravity_base: {gravity_base}")
        ## 记录结果
        self.mass_center_position = mass_center.tolist()
        self.force_zero = F_0.tolist()
        self.torque_zero = M_0.tolist()
        self.gravity_base = gravity_base.tolist()
    ## 

    def _get_launch_config(self):
        cfg = ConfigManager(config_path=str(BASE_DIR) + "/configs/temp.yaml")
        cfg.set_dict(new_data=self.config.data,
                    base_key="hardware")
        cfg.set_dict(new_data=self.xjc_config.data,
                    base_key="hardware")
        cfg.set_dict(new_data=self.waist_config.data,
                    base_key="hardware")
        cfg.set("robot_model","")
        dict_planner = {
            "urdf": "/home/hillbot/ziwei/admittance_ws/scripts/beta1_0_xjc.urdf",
            "enable_collision_checking":False,
            "stop_margin":0.01,
            "collision_margin": 0.05,
        }
        cfg.set_dict(new_data=dict_planner,
                    base_key="planner")
        return cfg.data

    def _GetFeedback(self):
        while not self._stop_event.is_set():
            feedInfo = self.robot.get_states()
            if feedInfo is not None:
                self.feedbackData.QActual = feedInfo[self.selected_arm]["position"]
                self.feedbackData.rb_time = feedInfo[self.selected_arm]["time"]["secs"]+feedInfo[self.selected_arm]["time"]["nanos"] * 1e-9
                sensor_data = feedInfo.get(self.expected_sensor)
                if sensor_data and 'force' in sensor_data:
                    self.feedbackData.Wrench = feedInfo[self.expected_sensor]['force']+feedInfo[self.expected_sensor]['torque']
                else:
                    # do nothing
                    time.sleep(0.01)

                if self.record_flag.is_set():
                    self.highfreq_log.append({
                        "rb_time":self.feedbackData.rb_time,
                        "Wrench":self.feedbackData.Wrench,
                    })
            time.sleep(0.005)

    def _start(self):
        self._stop_event.clear()
        self.robot = rb_python.robot.Robot(self.launch_config_dict)
        time.sleep(3)
        # self.robot.wait_for_operational(10)
        print("使用默认的位置模式.")
        self.robot.send_command(self.selected_arm, {"command": "set_control_mode", "mode": "position"})
        # 状态反馈线程
        feedback_thread = threading.Thread(
            target=self._GetFeedback
        )
        feedback_thread.daemon = True
        feedback_thread.start()
        print("机械臂已使能")
        # 存储初始关节位置
        while self.feedbackData.Wrench == -1:
            # block
            time.sleep(0.01)
        self.init_q_pos = self.feedbackData.QActual
        self.init_wrench = self.feedbackData.Wrench

    def _stop_background_measure(self):
        self._stop_event.set()
        if self.feedback_thread is not None:
            self.feedback_thread.join()
        print("测量线程停止...")

    def _run_point_in_joint(self,q_pos,type=None):
        # print("Moving...\n")
        if type == None:
            while np.max(np.abs(self.feedbackData.QActual-np.array(q_pos))) > 1e-3:
                self.robot.set_actions({self.selected_arm: {"type": "position", "position": q_pos}})
                time.sleep(1 / CONTROL_HZ)
            # print("到达!")
            # input_with_flush("回车以结束")
        if type == 'waist':
            while np.max(np.abs(self.feedbackData.QActual-np.array(q_pos))) > 1e-3:
                self.robot.set_actions({type: {"type": "position", "position": q_pos}})
                time.sleep(1 / CONTROL_HZ)
            # print("到达!")
            # input_with_flush("回车以结束")

    def select_arm(self):
        os.system('clear')
        print(f"{CYAN}=== 选择左/右侧机械臂 ==={RESET}")
        options = ["left_arm","right_arm"]
        for i, arm in enumerate(options, 1):
            print(f"{CYAN}[{i}] {arm}{RESET}")
        print(f"{CYAN}\n按下1或2来选择左侧或右侧...{CYAN}")
        while True:
            key = get_single_key()
            if key in ["1", "2"]:
                self.selected_arm = ["left_arm", "right_arm"][int(key)-1]
                self.expected_sensor = ["left_force_sensor","right_force_sensor"][int(key)-1]
                break
        ## DEBUG
        config_path = str(BASE_DIR) + f"/configs/{self.selected_arm}.yaml"
        config_xjc_path = str(BASE_DIR) + f"/configs/{self.expected_sensor}.yaml"
        config_waist_path = str(BASE_DIR) + f"/configs/waist.yaml"
        self.config = ConfigManager(config_path=config_path)
        self.xjc_config = ConfigManager(config_path=config_xjc_path)
        self.waist_config = ConfigManager(config_path=config_waist_path)
        self.launch_config_dict = self._get_launch_config()
        print(f"✅ 已选择: {self.selected_arm},{self.expected_sensor}")
        pprint.pprint(self.launch_config_dict)
        self._start()
        input(f"Press Enter to continue...")
    
    def _run_point_with_input(self):
        try:
            raw_input = input_with_flush(f"以逗号分隔的方式输入7个关节弧度 (e.g., 0.1,0,...): ")
            q_pos = [float(x.strip()) for x in raw_input.split(',')]

            if len(q_pos) != 7:
                print(f"\nError: Expected 7 values, but got {len(q_pos)}.")
                return
            
            print(f"\n移动到新位置: {[f'{p:.3f}' for p in q_pos]}")
            self._run_point_in_joint([q_pos])
        
        except Exception as e:
            print(f"\nError:{e}")
            input("回车以返回上一级目录")

    def _void(self):
        return
    
    def edit_config(self, key_path: str, input_data=None):
        os.system('clear')
        print("=== 编辑配置文件 ===\n")
        self.config.show()

        data = self.config.data
        keys = key_path.split(".")

        # --- Step 1: 定位目标字段 ---
        target = data
        for k in keys[:-1]:
            if k not in target:
                print(f"未找到键: '{k}' not found.")
                input("按回车返回...")
                return
            target = target[k]

        last_key = keys[-1]
        if last_key not in target:
            print(f"无效键: '{last_key}' not found.")
            input("按回车返回...")
            return

        old_value = target[last_key]
        print(f"\n当前值 '{key_path}':\n  {old_value}")
        print(f"类型: {type(old_value).__name__}")

        # --- Step 2: 决定新值来源 ---
        if input_data is None:
            # 手动输入模式
            readline.parse_and_bind("set editing-mode emacs")
            readline.parse_and_bind("set history-preserve-point on")

            while True:
                new_val_str = input("\n输入新值 (输入 q 放弃修改): ").strip()
                if not new_val_str:
                    print("输入为空，请重试")
                    continue
                if new_val_str.lower() == 'q':
                    print("已取消修改。")
                    input("按回车返回...")
                    return

                try:
                    if isinstance(old_value, bool):
                        new_value = new_val_str.lower() in ["true", "1", "yes"]
                    elif isinstance(old_value, int):
                        new_value = int(new_val_str)
                    elif isinstance(old_value, float):
                        new_value = float(new_val_str)
                    elif isinstance(old_value, list):
                        if new_val_str.startswith("["):
                            new_value = eval(new_val_str)
                        else:
                            new_value = [eval(x.strip()) for x in new_val_str.split(",")]
                    else:
                        new_value = new_val_str
                    break
                except Exception as e:
                    print(f"格式错误: {e}")
                    continue
        else:
            # 自动输入模式
            print(f"\n检测到自动更新输入: {input_data}")
            print(f"{key_path}:\n  {old_value} → {input_data}")

            # ★ 如果 input_data 是 dict，则调用 set_dict 来递归更新
            if isinstance(input_data, dict):
                confirm = input("检测到输入为 dict,是否递归更新该字段? (回车=是 / n=否): ").strip().lower()
                if confirm == "n":
                    print("值更新已取消。")
                    input("按回车返回...")
                    return
                print("\n➡ 调用 set_dict 接口递归更新...")
                # ★ 这里由你实现 set_dict 的逻辑
                self.config.set_dict(new_data=input_data,base_key=key_path)
                if hasattr(self.config, "save"):
                    self.config.save()
                print("\n✅ 配置更新成功（递归更新 dict)")
                input("按回车以继续...")
                return
            else:
                # 普通替换
                new_value = input_data

        # --- Step 3: 确认修改 ---
        confirm = input("是否应用修改? (回车=是 / n=否): ").strip().lower()
        if confirm == "n":
            print("值更新已取消。")
            input("按回车返回...")
            return

        # --- Step 4: 更新并保存 ---
        target[last_key] = new_value
        if hasattr(self.config, "save"):
            self.config.save()

        print("\n✅ 配置更新成功。")
        input("按回车以继续...")

    def _auto_update_params(self):
        key_path = f"{self.selected_arm}.admittance_config"
        input_data = {
                "admittance_config": {
                    "param_wrench_zero": self.force_zero + self.torque_zero,
                    "param_gravity": self.gravity_base,
                    "param_mass_pos": self.mass_center_position,
                }
        }
        self.edit_config(key_path=key_path,input_data=input_data["admittance_config"])

    def _edit_menu(self):
        os.system('clear')
        task_steps = {
            "0":{
                "description":"一键自动更新所有标定参数",
                "callback": self._auto_update_params
            },
            "1":{
                "description":"手动修改param_mass",
                "callback": lambda: self.edit_config(key_path=f"{self.selected_arm}.admittance_config.param_mass")
            },
            "2":{
                "description":"手动修改param_stiff",
                "callback": lambda: self.edit_config(key_path=f"{self.selected_arm}.admittance_config.param_stiff")
            },
            "3":{
                "description":"手动修改param_damp",
                "callback": lambda: self.edit_config(key_path=f"{self.selected_arm}.admittance_config.param_damp")
            },
            "4":{
                "description":"手动修改param_wrench_zero",
                "callback": lambda: self.edit_config(key_path=f"{self.selected_arm}.admittance_config.param_wrench_zero")
            },
            "5":{
                "description":"手动修改param_gravity",
                "callback": lambda: self.edit_config(key_path=f"{self.selected_arm}.admittance_config.param_gravity")
            },
            "6":{
                "description":"手动修改param_mass_pos",
                "callback": lambda: self.edit_config(key_path=f"{self.selected_arm}.admittance_config.param_mass_pos")
            },
        }
        self.menu_stack.append((task_steps,"参数修改"))

    def _check_calibration_status(self, auto_check: bool = True):
        """
        检查标定参数状态。
        - 自动检查各参数是否存在与合理；
        - 若存在，则人工确认；
        - 若不存在，提示需要重新标定。
        """
        import numpy as np
        import os

        os.system("clear")
        print("=== 力传感器标定检查 ===\n")

        required_fields = {
            "mass_center_position": getattr(self, "mass_center_position", None),
            "force_zero": getattr(self, "force_zero", None),
            "torque_zero": getattr(self, "torque_zero", None),
            "gravity_base": getattr(self, "gravity_base", None),
        }

        # --- Step 1: 检查缺失 ---
        missing = [k for k, v in required_fields.items() if v is None]
        if missing:
            print(f"⚠️ 检测到以下字段尚未标定：{missing}")
            print("请先运行传感器标定程序，再进行检查。")
            input("\n按回车返回...")
            return False

        # --- Step 2: 打印当前值 ---
        print("✅ 检测到全部标定参数存在：\n")
        for name, val in required_fields.items():
            arr = np.array(val)
            print(f"🔹 {name}: {arr.tolist()} (shape={arr.shape})")

            # 自动检查逻辑
            if auto_check:
                if np.allclose(arr, 0, atol=1e-6):
                    print(f"  ⚠️ 值似乎全为零，请检查是否标定失败。")
                if np.any(np.isnan(arr)):
                    print(f"  ⚠️ 含有 NaN 值，请重新标定。")
                if arr.ndim == 1 and len(arr) not in (3, 6):
                    print(f"  ⚠️ 维度异常(期望3或6维)，当前维度: {len(arr)}")
                # --- 特定字段的物理合理性检查 ---
                if name.lower() == "gravity":
                    if len(arr) >= 3 and arr[2] >= 0:
                        print(f"  ⚠️ gravity 第三项应为负值（通常为 -12 左右），当前值: {arr[2]:.3f}")
                if name.lower() == "mass_pos":
                    if len(arr) >= 3 and arr[2] <= 0:
                        print(f"  ⚠️ mass_pos 第三项应为正值（质心通常在工具下方），当前值: {arr[2]:.3f}")

            print()

        # --- Step 3: 人工确认 ---
        confirm = input("是否确认这些标定值正确？(回车=确认 / n=否): ").strip().lower()
        if confirm == "n":
            input("\n已取消确认, 请重新标定。\n回车以继续")
            return False

        input("\n✅ 标定检查完成，一切正常。\n回车以继续")
        return True
    
    def _save_logs(self):
        """保存日志文件"""
        log_dir = BASE_DIR / "logs"
        log_dir.mkdir(exist_ok=True)
        tstamp = time.strftime("%Y%m%d_%H%M%S")

        def safe_convert(data):
            """递归地把 numpy 类型转成可序列化类型"""
            if isinstance(data, np.ndarray):
                return data.tolist()
            elif isinstance(data, (np.float32, np.float64)):
                return float(data)
            elif isinstance(data, (np.int32, np.int64)):
                return int(data)
            elif isinstance(data, dict):
                return {k: safe_convert(v) for k, v in data.items()}
            elif isinstance(data, list):
                return [safe_convert(v) for v in data]
            else:
                return data

        highfreq = safe_convert(self.highfreq_log)
        lowfreq = safe_convert(self.lowfreq_log)

        with open(log_dir / f"highfreq_{tstamp}_{self.expected_sensor}.json", "w") as f:
            json.dump(highfreq, f, indent=2)

        with open(log_dir / f"lowfreq_{tstamp}_{self.expected_sensor}.json", "w") as f:
            json.dump(lowfreq, f, indent=2)

        print(f"✅ 日志已保存到 {log_dir}")

        highfreq_file_name = f"highfreq_{tstamp}_{self.expected_sensor}.json"
        lowfreq_file_name = f"lowfreq_{tstamp}_{self.expected_sensor}.json"

        return highfreq_file_name,tstamp
    
    def zero_shifting_test(self):
        os.system('clear')
        # === 用户友好输入测试时长 ===
        while True:
            try:
                user_input = input("请输入零点漂移测试时长（秒, 默认 10s, 输入 q 退出）: ").strip()
                if user_input.lower() == "q":
                    print("已取消测试。")
                    return
                duration = float(user_input) if user_input else 10.0
                if duration <= 0:
                    print("时长必须为正数，请重新输入。")
                    continue
                break
            except ValueError:
                print("输入无效，请输入数字。")
        """零点漂移测试：保持静止并记录一段时间的力数据"""
        print(f"\n=== 开始零点偏移测试 ({duration}s) ===")
        self.highfreq_log.clear()
        self.record_flag.set()

        start_time = time.time()
        while time.time() - start_time < duration:
            time.sleep(0.01)

        self.record_flag.clear()
        print("数据采集完成。")

        # 提取力数据
        if not self.highfreq_log:
            print("⚠️ 未采集到任何数据")
            return

        wrenches = np.array([entry["Wrench"] for entry in self.highfreq_log])
        mean_wrench = np.mean(wrenches, axis=0)
        std_wrench = np.std(wrenches, axis=0)

        print("\n=== 零点偏移结果 ===")
        print(f"平均值: {mean_wrench}")
        print(f"标准差: {std_wrench}")
        print(f"共采集 {len(wrenches)} 帧数据")

        # 可选择保存
        confirm = input("是否保存结果? (y/n): ").strip().lower()
        if confirm == "y":
            filename, tstamp = self._save_logs()
            print("结果已保存")
            confirm = input("是否可视化? (y/n): ").strip().lower()
            if confirm == "y":
                draw_wrench(log_dir='logs',highfile=filename,savefig=f'viz_multi_{tstamp}_{self.expected_sensor}')
            else:
                print("已跳过可视化")
        else:
            print("结果未保存")

    def identify(self):
        os.system('clear')
        task_steps = {
            "1": {
                "description": "运行机械臂到参数辨识任务起始位姿", 
                "callback": lambda: self._run_point_in_joint([-1.100000, 0.417000, 0.457000, 0.622000, 0.664000, 0.895000, -0.204000]),
            },
            "2": {
                "description": "运行自动标定程序",
                "callback": self._identify_param_auto,
            },
            "3": {
                "description": "检查判断标定结果是否正常",
                "callback": self._check_calibration_status,
            },
            "4": {
                "description": "修改配置",
                "callback": self._edit_menu,
            }
        }
        # 使用菜单栈
        self.menu_stack.append((task_steps,"六维力传感器参数辨识"))
    
    def gravity_compensation_test(self):
        """
        动态导纳控制测试流程：
        1. 提示用户确认安全姿态；
        2. 切换控制模式至导纳模式；
        3. 按顺序移动到若干测试姿态；
        4. 每次移动之间暂停等待；
        5. 支持中断退出。
        """
        os.system("clear")
        print("=== 动态导纳控制测试 ===")
        print("说明：")
        print(" - 请确保机器人处于安全姿态，防止意外碰撞。")
        print(" - 测试过程中可随时按 Ctrl+C 终止。")
        print("=================================\n")

        confirm = input("按 Enter 键进入导纳模式，或输入 'q' 退出：").strip().lower()
        if confirm == "q":
            print("已取消测试。")
            return

        try:
            # === 切换导纳控制模式 ===
            print("[INFO] 正在切换至导纳控制模式...")
            self.robot.send_command(self.selected_arm, {
                "command": "set_control_mode",
                "mode": "admittance"
            })
            time.sleep(1.0)
            print("[INFO] 已进入导纳控制模式。\n")

            # === 定义测试姿态 ===
            q_list = [
                [-1.100, 0.417, 0.457, 0.622, 0.664, 0.895, -0.204],
                [-1.100, 0.417, 0.457, 0.622, 0.664, 0.895,  0.233],
                [-1.100, 0.417, 0.457, 0.622, 0.664, 0.895,  0.670],
                [-1.100, 0.417, 0.457, 0.622, 0.664, 0.895, -0.125],
            ]

            # === 执行测试 ===
            print("[INFO] 开始执行关节姿态测试...\n")
            for i, q in enumerate(q_list):
                print(f"({i+1}/{len(q_list)}) 移动至目标关节位置：{q}")
                self._set_joint(q)
                time.sleep(1.5)

            print("\n=== 动态导纳控制测试完成 ===")
            self.robot.send_command(self.selected_arm, {
                    "command": "set_control_mode",
                    "mode": "position"
                })
            input("[INFO] 已切换回位置模式。回车返回。")

        except KeyboardInterrupt:
            print("\n[WARN] 检测到用户中断，正在安全退出导纳模式...")
            # 可选：切换回位置模式
            try:
                self.robot.send_command(self.selected_arm, {
                    "command": "set_control_mode",
                    "mode": "position"
                })
                print("[INFO] 已切换回位置模式。")
            except Exception as e:
                print(f"[ERROR] 退出导纳模式时出错：{e}")

        except Exception as e:
            print(f"[ERROR] 测试过程中出现异常：{e}")

        finally:
            print("\n[INFO] 导纳控制测试结束。")
        
    def static_admittance_test(self):
        """
        静态导纳控制测试：
        - 切换到导纳控制模式
        - 保持当前关节姿态
        - 用户可手动推机器人验证顺从性
        - 按下 'q' 可立即退出
        """
        os.system("clear")
        print("=== 静态导纳控制测试 ===")
        print("说明：请确保机器人处于安全姿态，防止意外碰撞。")
        confirm = input("按 Enter 进入导纳模式，或输入 'q' 退出：").strip().lower()
        if confirm == "q":
            print("已取消测试。")
            return

        # === 切换导纳控制模式 ===
        print("切换至导纳控制模式中...")
        self.robot.send_command(self.selected_arm, {
            "command": "set_control_mode",
            "mode": "admittance"
        })
        time.sleep(1.0)

        cur_pos = list(self.feedbackData.QActual)
        print(f"当前关节位置：{np.round(cur_pos, 3)}")
        print("导纳控制已开启，可轻推机器人感受顺从性。")
        print("按下 'q' 键即可立即退出测试。")

        # === 设置终端为即时输入模式 ===
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        tty.setcbreak(fd)  # 单字符输入（无需回车）

        try:
            while True:
                # 保持姿态
                self._run_point_in_joint(cur_pos)

                # 检查键盘输入（非阻塞）
                dr, _, _ = select.select([sys.stdin], [], [], 0)
                if dr:
                    key = sys.stdin.read(1)
                    if key.lower() == "q":
                        print("\n检测到退出信号，正在关闭导纳模式...")
                        break

                time.sleep(0.02)

        except KeyboardInterrupt:
            print("\n测试被中断。")

        finally:
            # 恢复终端输入模式
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

            # === 退出导纳模式 ===
            self.robot.send_command(self.selected_arm, {
                "command": "set_control_mode",
                "mode": "position"
            })
            input("\n已切回位置控制模式。测试结束。回车返回。")

    def motion_admittance_test(self):
        """
        动态导纳控制测试：
        - 切换到导纳控制模式
        - 暂定为两个姿态的来回运动
        - 用户可手动阻止机器人验证顺从性
        - 按下 'q' 可立即退出
        """
        os.system("clear")
        print("=== 动态导纳控制测试 ===")
        print("说明：请确保机器人处于安全姿态，防止意外碰撞。")
        confirm = input("按 Enter 进入导纳模式，或输入 'q' 退出：").strip().lower()
        if confirm == "q":
            print("已取消测试。")
            return

        # === 切换导纳控制模式 ===
        print("切换至导纳控制模式中...")
        self.robot.send_command(self.selected_arm, {
            "command": "set_control_mode",
            "mode": "admittance"
        })
        time.sleep(1.0)


        print("导纳控制已开启，可阻止机器人感受顺从性。")
        print("按下 'q' 键即可立即退出测试。")

        # === 设置终端为即时输入模式 ===
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        tty.setcbreak(fd)  # 单字符输入（无需回车）
        pos1 = [-1.1,0.417,0.457,0.622,0.664,0.895,-0.204]
        pos2 = [-1.177,0.534,0.306,1.047,0.616,0.736,-0.634]
        try:
            while True:
                # 保持姿态
                self._run_point_in_joint(pos1)
                self._run_point_in_joint(pos2)

                # 检查键盘输入（非阻塞）
                dr, _, _ = select.select([sys.stdin], [], [], 0)
                if dr:
                    key = sys.stdin.read(1)
                    if key.lower() == "q":
                        print("\n检测到退出信号，正在关闭导纳模式...")
                        break

                time.sleep(0.02)

        except KeyboardInterrupt:
            print("\n测试被中断。")

        finally:
            # 恢复终端输入模式
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

            # === 退出导纳模式 ===
            self.robot.send_command(self.selected_arm, {
                "command": "set_control_mode",
                "mode": "position"
            })
            input("\n已切回位置控制模式。测试结束。回车返回。")

    def task_test(self):
        os.system('clear')
        task_steps = {
            "1": {
                "description": "参数辨识后六维力传感器零飘曲线", 
                "callback": self.zero_shifting_test,
            },
            "2": {
                "description": "重力补偿效果验证",
                "callback": self.gravity_compensation_test,
            },
            "3": {
                "description": "定点导纳交互测试",
                "callback": self.static_admittance_test,
            },
            "4": {
                "description": "移动导纳交互测试",
                "callback": self.motion_admittance_test,
            }
        }
        # 使用菜单栈
        self.menu_stack.append((task_steps,"导纳相关性能测试"))

    def run(self):
        self.menu_stack = []  # 初始化菜单栈
        self.select_arm()

        # 主菜单
        main_menu = {
            "1": ("六维力传感器参数辨识", self.identify),
            "2": ("导纳相关性能测试", self.task_test)
        }
        self.menu_stack.append((main_menu,"Main Menu"))

        while self.running:
            os.system('clear')
            # --- 状态刷新 ---
            print(f"{GREEN}=== 机器人实时状态 ({self.selected_arm}) ==={RESET}")
            print(f"{GREEN}关节位置: {self.feedbackData.QActual}{RESET}")
            print(f"{GREEN}FX: {self.feedbackData.Wrench[0]}{RESET}")
            print(f"{GREEN}FY: {self.feedbackData.Wrench[1]}{RESET}")
            print(f"{GREEN}FZ: {self.feedbackData.Wrench[2]}{RESET}")
            print(f"{GREEN}TX: {self.feedbackData.Wrench[3]}{RESET}")
            print(f"{GREEN}TY: {self.feedbackData.Wrench[4]}{RESET}")
            print(f"{GREEN}TZ: {self.feedbackData.Wrench[5]}{RESET}")
            print("\n" + "-"*40)

            # --- 菜单显示 ---
            current_menu, menu_title = self.menu_stack[-1]
            print(f"\n{CYAN}=== {menu_title} ==={CYAN}\n")
            for key, val in current_menu.items():
                if isinstance(val, tuple): ## (description, action)
                    desc, _ = val
                else: ## {"description": str, "callback": action_or_submenu}
                    desc = val["description"]
                print(f"{CYAN}[{key}] {desc}{RESET}")
            if menu_title == "Main Menu":
                print("[q] 退出")
            else:
                print("[b] 返回   [q] 退出")

            key = get_single_key()
            if key:
                if key == 'q' or key == 'Q':
                    self.running = False
                    break
                elif (key == 'b' or key == 'B') and len(self.menu_stack) > 1:
                    self.menu_stack.pop()
                    continue
                elif key in current_menu:
                    val = current_menu[key]
                    if isinstance(val, tuple):
                        _, action = val
                    else:
                        action = val["callback"]

                    if callable(action):
                        action()
                    elif isinstance(action, dict):
                        self.menu_stack.append(action)

            time.sleep(0.1)
        
        print("\nShutting down...")
        self.robot.shutdown()

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
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
        "root": {"level": "warn", "appenders": ["stderr","file"]},
        "loggers": {},
    }
    hblog.start(config)
    try:
        arm_guide = Guide()
        arm_guide.run()
    finally:
        print("shutdown...")
        arm_guide.robot.shutdown()