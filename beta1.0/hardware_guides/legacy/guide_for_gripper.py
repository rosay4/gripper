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
from utils.topp import TOPP
from utils.visualize import draw
import json
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

class guide:
    def __init__(self):
        
        class feedbackItem:
            def __init__(self):
                self.gripper_pos = -1
                self.rb_time = -1
                # ... 自定义反馈数据
                '''
                self.Wrench = -1
                '''
        self.feedbackData = feedbackItem() # 自定义反馈数据结构体
        # 状态管理
        self.running = True
        self.selected_gripper = "" ## "left_gripper" "right_gripper"
        # 默认TOPP速度限制
        self.max_vel = 0.1
        self.max_acc = 0.1
        # 日志
        self.record_flag = threading.Event()
        self.highfreq_log = []
        self.lowfreq_log = []

    def _get_launch_config(self):
        cfg = ConfigManager(config_path=str(BASE_DIR) + "/configs/temp.yaml")
        cfg.set_dict(new_data=self.config.data,
                          base_key="hardware")
        cfg.set("robot_model","")
        cfg.set("planner",None)
        return cfg.data

    def _GetFeedback(self):
        while self.running:
            feedInfo = self.robot.get_states()
            if feedInfo is not None:
                self.feedbackData.gripper_pos = feedInfo[self.selected_gripper]["position"]
                self.feedbackData.rb_time = feedInfo[self.selected_gripper]["time"]["secs"]+feedInfo[self.selected_gripper]["time"]["nanos"] * 1e-9
            
                if self.record_flag.is_set():
                    self.highfreq_log.append({
                        "rb_time":self.feedbackData.rb_time,
                        f"highfreq_pos":self.feedbackData.gripper_pos,
                    })
            time.sleep(0.005)

    def _start(self):
        self.robot = rb_python.robot.Robot(self.launch_config_dict)
        self.robot.wait_for_operational(10)
        print("使用默认的位置模式.")
        self.robot.send_command(f"{self.selected_gripper}", {"command": "set_control_mode", "mode": "position"})
        # 状态反馈线程
        feedback_thread = threading.Thread(
            target=self._GetFeedback
        )
        feedback_thread.daemon = True
        feedback_thread.start()
        print("已使能")

    def _run_point(self,pos):
        print("Moving...\n")
        while np.abs(self.feedbackData.gripper_pos-np.array(pos)) > 1e-5:
            self.robot.set_actions({self.selected_gripper: {"type": "position", "position": [pos]}})
            time.sleep(0.05)
            print(f"pos:{pos}")
        print("到达!")
        input_with_flush("回车以结束")

    def _manual_control(self):
        print("按W/S控制夹爪张合, 按Q退出")
        step = 0.0005
        while True:
            key = get_single_key(timeout=0.05)
            current = self.feedbackData.gripper_pos[0]
            if key == 'w':
                new_pos = min(current + step, 1000)
            elif key == 's':
                new_pos = max(current - step, -1000)
            elif key == 'q':
                break
            else:
                continue
            self.robot.set_actions({self.selected_gripper:{"type":"position","position":[new_pos]}})
            time.sleep(0.05)
            # 清空终端输入缓冲
            sys.stdin.flush()
        return current
 
    def set_limits(self):
        print("\n请点动闭合夹爪至最小位置, 按 Q 确认")
        min_pos = self._manual_control()
        print("\n请点动闭合夹爪至最大位置, 按 Q 确认")
        max_pos = self._manual_control()
        self.robot.send_command(self.selected_gripper, {"command": "set_limit",
                                                        "enabled": [False],
                                                        "lower":[min_pos],
                                                        "upper":[max_pos]})
        input("标定结束，回车以返回")

    def avoid_zero_issue(self):
        # 避免零点丢失
        # print("\n清点动夹爪至闭合,设置零点, 按Q确认")
        # self._manual_control()
        # self.robot.send_command(self.selected_gripper, {"command": "set_zero"})
        print("\n请点动夹爪至行程1/2处, 按 Q 确认")
        self.middle_pos = self._manual_control()
        self.edit_config(f"{self.selected_gripper}.offset_at_hardware_zero",[self.middle_pos])
        self.robot.send_command(self.selected_gripper, {"command": "set_zero"})
        time.sleep(1)
        input("已更新参数，需要重启生效")

    def calibrate_length_per_radian(self):
        print("\n请点动张开夹爪至最大位置, 按 Q 确认")
        max_pos = self._manual_control()
        print("\n请点动闭合夹爪至最小位置, 按 Q 确认")
        min_pos = self._manual_control()
        distance_in_rad = max_pos-min_pos
        length_per_radian = self.measured_half_range/distance_in_rad
        print(length_per_radian)
        self.edit_config(key_path=f"{self.selected_gripper}.length_per_radian",input_data=[length_per_radian])
        input("已更新参数，需要重启生效")

    def _update_max_torque(self):
        self.edit_config(key_path=f"{self.selected_gripper}.max_torque",input_data=[2000])
        input("需要重启生效")
        os.system('clear')

    def _update_length_per_radian(self):
        self.edit_config(key_path=f"{self.selected_gripper}.length_per_radian",input_data=[1.0])
        print("需要重启生效")
        os.system('clear')
    
    def _init_update_limits(self):
        min_pos = -100
        max_pos = 100
        self.robot.send_command(self.selected_gripper, {"command": "set_limit",
                                                        "enabled": [False],
                                                        "lower":[min_pos],
                                                        "upper":[max_pos]})
        input("success")

    def select_gripper(self):
        os.system('clear')
        print(f"{CYAN}=== 选择左/右侧夹爪 ==={RESET}")
        options = ["left_gripper","right_gripper"]
        for i, name in enumerate(options, 1):
            print(f"{CYAN}[{i}] {name}{RESET}")
        print(f"{CYAN}\n按下1或2来选择左侧或右侧...{CYAN}")
        while True:
            key = get_single_key()
            if key in ["1", "2"]:
                self.selected_gripper = ["left_gripper", "right_gripper"][int(key)-1]
                break
        config_path = str(BASE_DIR) + f"/configs/{self.selected_gripper}.yaml"
        self.config = ConfigManager(config_path=config_path)
        self.launch_config_dict = self._get_launch_config()
        print(f"✅ 已选择: {self.selected_gripper}")
        self._start()
        input(f"Press Enter to continue...")
    
    def _run_point_with_input(self):
        try:
            raw_input = input_with_flush(f"输入位置 (e.g. 0.1): ")
            pos = float(raw_input)

            if len([pos]) != 1:
                print(f"\nError: Expected 1 value, but got {len([pos])}.")
                return
            
            print(f"\n正在移动到新位置: {[f'{pos:.3f}']}")
            self._run_point(pos)
        
        except Exception as e:
            print(f"\nError:{e}")
            input("回车以返回上一级目录")

    def _void(self):
        return
    
    def set_max_vel_acc(self):

        os.system('clear')

        # Helper 函数：安全读取正浮点数
        def read_positive_float(prompt):
            while True:
                raw_input = input_with_flush(prompt)
                try:
                    value = float(raw_input)
                    if value <= 0:
                        print("⚠️ 必须输入正数，请重试。")
                        continue
                    return value
                except ValueError:
                    print("⚠️ 输入无效，请输入数字。")

        # 读取最大速度
        self.max_vel = read_positive_float("请输入最大速度设置 (rad/s): ")

        # 读取最大加速度
        self.max_acc = read_positive_float("请输入最大加速度设置 (rad/s²): ")

        input(f"✅ 设置完成：最大速度 = {self.max_vel} rad/s, 最大加速度 = {self.max_acc} rad/s²\n回车以继续...")

    def _record_lowfreq(self,t,q):
        """控制侧日志(20Hz)"""
        self.lowfreq_log.append({
            "pc_time":time.time(),
            "t_in_traj":t,
            "cmd_position":q,
            "feedback_pos":self.feedbackData.gripper_pos,
            "rb_time":self.feedbackData.rb_time,
        })

    def cubic_move(self):
        start_point = float(self.feedbackData.gripper_pos[0])
        
        # 输入并转换
        end_point_str = input_with_flush("输入目标位置:")
        try:
            end_point = float(end_point_str)
        except ValueError:
            print("输入无效，请输入数字")
            return

        # 构造路径
        path = np.array([[start_point], [end_point]], dtype=float)

        # 轨迹规划
        try:
            ts, qs, qds, qdds, duration = TOPP(path, [self.max_vel], [self.max_acc], 0.05)
        except RuntimeError as e:
            print("轨迹规划失败:", e)
            return

        # 执行轨迹
        print("移动中...")
        
        self.record_flag.set()
        print("开始高频+低频记录...")
        start_time = time.time()
        for t, q in zip(ts, qs):
            self.robot.set_actions({self.selected_gripper: {"type": "position", "position": q.tolist()}})
            self._record_lowfreq(t,q)
            while time.time() - start_time < t:
                time.sleep(0.001)
        self.record_flag.clear()
        print("高频记录结束，共收集高频数据", len(self.highfreq_log))
        print("低频记录结束，共收集低频数据", len(self.lowfreq_log))

        print("\n=== 数据记录 ===")
        confirm = input("是否保存日志? (y/n): ").strip().lower()
        if confirm == "y":
            # 保存日志
            highfreq_filename, lowfreq_filename,tstamp = self._save_logs()

            confirm = input("是否可视化记录数据? (y/n): ").strip().lower()
            if confirm != "y":
                print("数据可视化取消")
                input("回车以返回")
                return
            draw(log_dir='logs/',lowfile=lowfreq_filename,highfile=highfreq_filename,savefig=f"viz_{self.selected_gripper}_{tstamp}.png")
        else:
            print("日志保存更新取消")
            input("回车以返回")
            return
    
    def _save_logs(self):
        """保存日志文件"""
        log_dir = BASE_DIR / "logs"
        print(log_dir)
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

        with open(log_dir / f"highfreq_{tstamp}_{self.selected_gripper}.json", "w") as f:
            json.dump(highfreq, f, indent=2)

        with open(log_dir / f"lowfreq_{tstamp}_{self.selected_gripper}.json", "w") as f:
            json.dump(lowfreq, f, indent=2)

        print(f"✅ 日志已保存到 {log_dir}")
        highfreq_file_name = f"highfreq_{tstamp}_{self.selected_gripper}.json"
        lowfreq_file_name = f"lowfreq_{tstamp}_{self.selected_gripper}.json" 
        return highfreq_file_name,lowfreq_file_name,tstamp  
    
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

    def task_tracking_error(self):
        os.system('clear')
        key_path = self.selected_gripper + ".max_velocity"
        task_steps = {
            "1": {
                "description": "设置速度限制参数", 
                "callback": self.set_max_vel_acc,
            },
            "2": {
                "description": "梯形速度规划移动",
                "callback": self.cubic_move,
            },
            "3":{
                "description": "可选修改最大速度配置",
                "callback": lambda: self.edit_config(key_path=key_path),
            }
        }
        # 使用菜单栈
        self.menu_stack.append((task_steps,"20Hz下的轨迹跟踪测试"))

    def task_verify(self):
        os.system('clear')
        key_path = self.selected_gripper + ".invert_directions"
        task_steps = {
            "1": {
                "description": "移动夹爪到零位", 
                "callback": lambda: self._run_point(0.0),
            },
            "2": {
                "description": "少量正方向移动夹爪以验证方向",
                "callback": lambda: self._run_point(0.001),
            },
            "3": {
                "description": "以需要位置移动夹爪(可跳过)",
                "callback": self._run_point_with_input,
            },
            "4": {
                "description": "修改配置",
                "callback": lambda: self.edit_config(key_path=key_path),
            }
        }
        # 使用菜单栈
        self.menu_stack.append((task_steps,"验证关节移动方向正确"))
    
    def _input_gripper_range(self):
        """交互输入夹爪行程测量结果（两指之间距离，单位：米），自动除以2存入类属性"""
        try:
            raw = input("\n请输入夹爪两指之间的距离 (单位: 米)：").strip()
            distance = float(raw)
            half_range = distance / 2
            self.measured_half_range = half_range
            print(f"\n✅ 已记录夹爪单指行程: {half_range:.4f} m (总距离 {distance:.4f} m)")
            input("按回车返回继续...")
        except Exception as e:
            print(f"\n❌ 输入无效，请输入数字。错误信息: {e}")
            input("按回车返回继续...")

    def _check_gripper_calibration(self, auto_check: bool = True):
        """
        检查夹爪标定状态。
        - 自动检查夹爪行程、零位、比例等是否存在与合理；
        - 若存在异常则提示；
        - 最后可人工确认。
        """
        import numpy as np
        import os

        os.system("clear")
        print("=== 夹爪标定结果检查 ===\n")

        # --- Step 1: 获取需要检查的字段 ---
        required_fields = {
            "offset_at_hardware_zero": self.config.get(f"hardware.{self.selected_gripper}.offset_at_hardware_zero", None),
            "length_per_radian": self.config.get(f"hardware.{self.selected_gripper}.length_per_radian", None),
        }

        # --- Step 2: 检查缺失 ---
        missing = [k for k, v in required_fields.items() if v is None]
        if missing:
            print(f"⚠️ 检测到以下标定字段缺失：{missing}")
            print("请先执行完整的夹爪标定步骤。")
            input("\n按回车返回...")
            return False

        # --- Step 3: 打印当前标定值 ---
        print("✅ 检测到夹爪标定参数如下：\n")
        for name, val in required_fields.items():
            arr = np.array(val)
            print(f"🔹 {name}: {arr.tolist()}")

            if auto_check:
                # 自动合理性检查
                if np.any(np.isnan(arr)):
                    print(f"  ⚠️ {name} 含有 NaN 值，请重新标定。")
                if np.allclose(arr, 0, atol=1e-6):
                    print(f"  ⚠️ {name} 值几乎为零，请检查是否标定错误。")

                # 物理合理性
                if name == "length_per_radian":
                    if np.any((arr <= 0) | (arr > 0.1)):
                        print(f"  ⚠️ 异常比例系数 (应为 0 < ratio < 1.0)，当前值: {arr.tolist()}")
                if name == "measured_half_range":
                    if not (0.005 <= arr <= 0.05):
                        print(f"  ⚠️ 夹爪单指行程超出合理范围 (5mm~50mm)，当前值: {arr:.4f}")

            print()

        # --- Step 4: 对照标准值 ---
        # 例如标准行程 25mm（两指总行程 50mm）
        standard_half_range = 0.025
        measured = required_fields["measured_half_range"]
        diff = abs(measured - standard_half_range)
        print(f"🔸 标准单指行程: {standard_half_range:.4f} m")
        print(f"🔸 当前标定结果: {measured:.4f} m")
        print(f"🔸 偏差: {diff:.4f} m")

        if diff > 0.005:
            print("⚠️ 偏差较大，请检查机械结构或重新标定。")

        # --- Step 5: 人工确认 ---
        confirm = input("\n是否确认这些标定结果正常？(回车=确认 / n=否): ").strip().lower()
        if confirm == "n":
            input("\n已取消确认, 请重新标定。\n回车以继续")
            return False

        input("\n✅ 标定检查完成，一切正常。\n回车以继续")
        return True

    def _auto_update_params(self):
        key_path = f"{self.selected_gripper}"
        # TEST
        input_data = {
                f"{self.selected_gripper}": {
                    "length_per_radian": self.length_per_radian,
                    "offset_at_hardware_zero": self.middle_pos,
                }
        }
        self.edit_config(key_path=key_path,input_data=input_data[f"{self.selected_gripper}"])

    def _edit_menu(self):
        os.system('clear')
        task_steps = {
            "0":{
                "description":"一键自动更新所有标定参数",
                "callback": self._auto_update_params
            },
            "1":{
                "description":"手动修改length_per_radian",
                "callback": lambda: self.edit_config(key_path=f"{self.selected_gripper}.length_per_radian")
            },
            "2":{
                "description":"手动修改offset_at_hardware_zero",
                "callback": lambda: self.edit_config(key_path=f"{self.selected_gripper}.offset_at_hardware_zero")
            },
            "3":{
                "description":"手动修改max_torque",
                "callback": lambda: self.edit_config(key_path=f"{self.selected_gripper}.max_torque")
            },
        }
        self.menu_stack.append((task_steps,"参数修改"))

    def task_calibrate(self):
        os.system('clear')
        task_steps = {
            "0": {
                "description": "更改运动上下限",
                "callback": self._init_update_limits,
            },
            "1": {
                "description": "设置更宽容的力矩限制并重启生效", 
                "callback": self._update_max_torque,
            },
            "2": {
                "description": "设置传动比为1并重启生效",
                "callback": self._update_length_per_radian,
            },
            "3": {
                "description": "进入点按模式将夹爪运动到最大张开位置",
                "callback": self._manual_control,
            },
            "4": {
                "description": "填写夹爪行程测量结果(直接填写两指之间距离，程序自动转换行程)",
                "callback": self._input_gripper_range,
            },
            "5": {
                "description": "标定传动比并重启生效",
                "callback": self.calibrate_length_per_radian,
            },
            "6": {
                "description": "设置硬件零点偏移并重启生效",
                "callback": self.avoid_zero_issue,
            },
            "7": {
                "description": "设置夹爪最大值最小值，标定结束",
                "callback": self.set_limits,
            },
            "8": {
                "description": "修改配置",
                "callback": self._edit_menu,
            },
            "9": {
                "description": "验证标定结果是否正常",
                "callback": self._check_gripper_calibration,
            }
        }
        # 使用菜单栈
        self.menu_stack.append((task_steps,"验证关节移动方向正确"))

    def run(self):
        self.menu_stack = []  # 初始化菜单栈
        self.select_gripper()

        # 主菜单
        main_menu = {
            "1": ("验证夹爪移动方向正确", self.task_verify),
            "2": ("标定硬件零位偏移和转换系数", self.task_calibrate),
            "3": ("20Hz夹爪轨迹跟踪测试", self.task_tracking_error),
        }
        self.menu_stack.append((main_menu,"Main Menu"))

        while self.running:
            os.system('clear')
            # --- 状态刷新 ---
            print(f"{GREEN}=== 夹爪实时状态 ({self.selected_gripper}) ==={RESET}")
            print(f"{GREEN}关节位置: {self.feedbackData.gripper_pos}{RESET}")
            print(f"{GREEN}RB内部时间: {self.feedbackData.rb_time} sec{RESET}")
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
    try:
        gripper_guide = guide()
        gripper_guide.run()
    finally:
        print("shutdown...")
        gripper_guide.robot.shutdown()