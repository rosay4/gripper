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
import hblog
import csv
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

class Guide:
    def __init__(self):
        
        class FeedbackItem:
            def __init__(self):
                self.QActual = -1
                self.rb_time = -1
                # ... 自定义反馈数据
                '''
                self.Wrench = -1
                '''
        self.feedbackData = FeedbackItem() # 自定义反馈数据结构体
        # 状态管理
        self.running = True
        self.selected_arm = "" ## "left_arm" "right_arm"
        # 默认TOPP速度限制
        self.max_vel = 0.1
        self.max_acc = 0.1
        # 日志
        self.record_flag = threading.Event()
        self.highfreq_log = []
        self.lowfreq_log = []
        # 走点
        self.queue = {}
        self.queue_idx = 1

    def _get_launch_config(self):
        cfg = ConfigManager(config_path=str(BASE_DIR) + "/configs/temp.yaml")
        cfg.set_dict(new_data=self.config.data,
                          base_key="hardware")
        cfg.set("robot_model","")
        cfg.set("planner",None)
        return cfg.data

    def _GetFeedback(self):
        while True:
            feedInfo = self.robot.get_states()
            if feedInfo is not None:
                self.feedbackData.QActual = feedInfo[self.selected_arm]["position"]
                self.feedbackData.rb_time = feedInfo[self.selected_arm]["time"]["secs"]+feedInfo[self.selected_arm]["time"]["nanos"] * 1e-9
            
                if self.record_flag.is_set():
                    self.highfreq_log.append({
                        "rb_time":self.feedbackData.rb_time,
                        "highfreq_pos":self.feedbackData.QActual,
                    })
            time.sleep(0.005)

    def _start(self):
        self.robot = rb_python.robot.Robot(self.launch_config_dict)
        print(self.launch_config_dict)
        # self.robot.wait_for_operational(10)
        time.sleep(2)
        print("使用默认的位置模式.")
        self.robot.send_command(f"{self.selected_arm}", {"command": "set_control_mode", "mode": "position"})
        # 状态反馈线程
        feedback_thread = threading.Thread(
            target=self._GetFeedback
        )
        feedback_thread.daemon = True
        feedback_thread.start()
        print("机械臂已使能")

    def _run_point_in_joint(self,q_pos):
        print("Moving...\n")
        while np.max(np.abs(self.feedbackData.QActual-np.array(q_pos))) > 1e-3:
            self.robot.set_actions({self.selected_arm: {"type": "position", "position": q_pos}})
            time.sleep(1 / CONTROL_HZ)
        print("到达!")
        input_with_flush("回车以结束")

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
                break
        config_path = str(BASE_DIR) + f"/configs/{self.selected_arm}.yaml"
        self.config = ConfigManager(config_path=config_path)
        self.launch_config_dict = self._get_launch_config()
        print(f"✅ 已选择: {self.selected_arm}")
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
            self._run_point_in_joint(q_pos)
        
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
            "feedback_pos":self.feedbackData.QActual,
            "rb_time":self.feedbackData.rb_time,
        })
    
    def cubic_move_record(self):
        start_point = np.array(self.feedbackData.QActual,dtype=float)
        
        print("当前点位:")
        print(" ".join([f"{v:.6f}" for v in start_point]))
        print("\n请输入要修改的维度（格式：index=value）")
        print("支持多个修改，例如： 2=0.3  5=-1.2")
        print("直接回车 = 使用当前点位作为目标\n")

        user_input = input_with_flush("修改指令: ").strip()

        # --- 1. 用户直接回车：不改动任何值 ---
        if user_input == "":
            end_point = start_point.copy()
            print("未输入，保持当前位置作为目标点。")

        # --- 2. 解析修改指令 ---
        end_point = start_point.copy()
        parts = user_input.split()

        try:
            for p in parts:
                # 必须是 index=value
                if "=" not in p:
                    print(f"格式错误：{p}（必须是 i=value）")
                    return

                idx_str, value_str = p.split("=")
                idx = int(idx_str)       # 维度索引（1~7）
                value = float(value_str) # 该维的新值

                if not (1 <= idx <= 7):
                    print(f"维度 {idx} 超出范围（必须 1~7）")
                    return

                end_point[idx - 1] = value

        except ValueError:
            print("解析失败，请确保格式是 i=value，且 value 可转换为数字")
            return

        print("\n最终目标点位:")
        print(end_point.tolist())
        input("确认目标点位，回车以开始走点")
        # 构造路径
        path = np.vstack([start_point,end_point])

        # 轨迹规划
        sample_rate = 20
        step = 1/sample_rate
        try:
            ts, qs, qds, qdds, duration = TOPP(
                path, 
                [self.max_vel]*7, 
                [self.max_acc]*7, 
                step
            )
        except RuntimeError as e:
            print("轨迹规划失败:", e)
            return

        # 执行轨迹
        print("移动中...")
        
        self.record_flag.set()
        print("开始高频+低频记录...")
        start_time = time.time()
        for t, q in zip(ts, qs):
            # print(q)
            self.robot.set_actions({self.selected_arm: {"type": "position", "position": q.tolist()}})
            self._record_lowfreq(t,q)
            while time.time() - start_time < t:
                time.sleep(0.001)
        self.record_flag.clear()
        print("高频记录结束，共收集高频数据", len(self.highfreq_log))
        print("低频记录结束，共收集低频数据", len(self.lowfreq_log))

        print("\n=== 数据记录 ===")
        confirm = input("是否保存日志? (y/n): ").strip().lower()
        if confirm != "y":
            print("日志保存更新取消")
            input("回车以返回")
            return

        # 保存日志
        highfreq_filename, lowfreq_filename,tstamp = self._save_logs()

        confirm = input("是否可视化记录数据? (y/n): ").strip().lower()
        if confirm != "y":
            print("数据可视化取消")
            input("回车以返回")
            return

        draw(log_dir='logs',lowfile=lowfreq_filename,highfile=highfreq_filename,savefig=f"viz_{self.selected_arm}_{tstamp}.png")
    
    def cubic_move(self):
        start_point = np.array(self.feedbackData.QActual,dtype=float)
        
        print("当前点位:")
        print(" ".join([f"{v:.6f}" for v in start_point]))
        print("\n请输入要修改的维度（格式：index=value）")
        print("支持多个修改，例如： 2=0.3  5=-1.2")
        print("直接回车 = 使用当前点位作为目标\n")

        user_input = input_with_flush("修改指令: ").strip()

        # --- 1. 用户直接回车：不改动任何值 ---
        if user_input == "":
            end_point = start_point.copy()
            print("未输入，保持当前位置作为目标点。")

        # --- 2. 解析修改指令 ---
        end_point = start_point.copy()
        parts = user_input.split()

        try:
            for p in parts:
                # 必须是 index=value
                if "=" not in p:
                    print(f"格式错误：{p}（必须是 i=value）")
                    return

                idx_str, value_str = p.split("=")
                idx = int(idx_str)       # 维度索引（1~7）
                value = float(value_str) # 该维的新值

                if not (1 <= idx <= 7):
                    print(f"维度 {idx} 超出范围（必须 1~7）")
                    return

                end_point[idx - 1] = value

        except ValueError:
            print("解析失败，请确保格式是 i=value，且 value 可转换为数字")
            return

        print("\n最终目标点位:")
        print(end_point.tolist())
        input("确认目标点位，回车以开始走点")
        # 构造路径
        path = np.vstack([start_point,end_point])

        # 轨迹规划
        sample_rate = 20
        step = 1/sample_rate
        try:
            ts, qs, qds, qdds, duration = TOPP(
                path, 
                [self.max_vel]*7, 
                [self.max_acc]*7, 
                step
            )
        except RuntimeError as e:
            print("轨迹规划失败:", e)
            input("回车以返回")
            return

        # 执行轨迹
        print("移动中...")
        start_time = time.time()
        for t, q in zip(ts, qs):
            self.robot.set_actions({self.selected_arm: {"type": "position", "position": q.tolist()}})
            self._record_lowfreq(t,q)
            while time.time() - start_time < t:
                time.sleep(0.001)

    def cubic_move_home(self):
        start_point = np.array(self.feedbackData.QActual,dtype=float)
        
        print("当前点位:")
        print(" ".join([f"{v:.6f}" for v in start_point]))

        print("\n目标HOME点位:")
        end_point = np.array([0, 0.6, 0, -1.6, 0, -0.6, 0])
        print(f"{end_point}")
        input("确认目标点位，回车以开始走点")
        # 构造路径
        path = np.vstack([start_point,end_point])

        # 轨迹规划
        sample_rate = 20
        step = 1/sample_rate
        try:
            ts, qs, qds, qdds, duration = TOPP(
                path, 
                [self.max_vel]*7, 
                [self.max_acc]*7, 
                step
            )
        except RuntimeError as e:
            print("轨迹规划失败:", e)
            input("回车以返回")
            return

        # 执行轨迹
        print("移动中...")
        start_time = time.time()
        for t, q in zip(ts, qs):
            self.robot.set_actions({self.selected_arm: {"type": "position", "position": q.tolist()}})
            self._record_lowfreq(t,q)
            while time.time() - start_time < t:
                time.sleep(0.001)

    def move_multiple_points(self):
        #从csv文件加载为dict
        filename = input("请输入点位文件名(例如points.csv):").strip()
        result = {}
        try:
            with open(filename,'r') as f:
                reader = csv.reader(f)
                header = next(reader)
                for row in reader:
                    idx = int(row[0])
                    q = [float(v) for v in row[1:8]]
                    result[idx] = q
            print(f"成功加载点位，共{len(result)}个")
        except FileNotFoundError:
            print(f"文件未找到:{filename}")
            input("回车以返回")
            return
        # --- 1. 按 key 排序保证顺序 ---
        sorted_keys = sorted(result.keys())

        # --- 2. 转换为 numpy 的目标点列表 ---
        points = []
        for k in sorted_keys:
            q = np.array(result[k], dtype=float)
            if q.shape[0] != 7:
                input(f"第 {k} 个点不是7个关节角，实际是 {len(q)} 个")
                return
            points.append(q)

        # --- 3. 初始位置为当前关节角 ---
        current_q = np.array(self.feedbackData.QActual, dtype=float)

        # --- 4. 分段执行 TOPP + 每点停留 5 秒 ---
        for key, target_q in zip(sorted_keys, points):

            print(f"\n===== 段 {key}: {current_q.tolist()} -> {target_q.tolist()} =====")

            # 构造两点路径
            path = np.vstack([current_q, target_q])

            # TOPP 插值
            sample_rate = 20
            step = 1 / sample_rate
            try:
                ts, qs, qds, qdds, duration = TOPP(
                    path,
                    [self.max_vel] * 7,
                    [self.max_acc] * 7,
                    step
                )
            except RuntimeError as e:
                input("轨迹规划失败:", e)
                return

            # 执行轨迹
            print("移动中...")
            start_time = time.time()
            for t, q in zip(ts, qs):
                self.robot.set_actions({
                    self.selected_arm: {"type": "position", "position": q.tolist()}
                })
                self._record_lowfreq(t, q)
                while time.time() - start_time < t:
                    time.sleep(0.001)

            # 停留 5 秒
            # print(f"到达第 {key} 个点，停留 5 秒")
            # stay_start = time.time()
            # while time.time() - stay_start < 5.0:
            #     t = time.time() - stay_start
            #     self._record_lowfreq(t, target_q)
            #     time.sleep(0.01)

            # 手动控制走下一个点
            input(f"到达第 {key} 个点，确认测量成功后回车前往下一个点位")
            time.sleep(1)

            # 下一段起点 = 当前段目标点
            current_q = target_q.copy()

        print("\n所有点已执行完成。")
        input("回车以返回")
        return

    def set_torque_mode(self):
        confirm = input_with_flush("请在切换力矩模式前托住机械臂，否则机械臂将自由下落 (y/n): ").strip().lower()
        if confirm == "y":
            print("已切换到力矩模式")
            self.robot.send_command(self.selected_arm, {"command": "set_control_mode", "mode": "torque"})
        else:
            print("Cancelled.")
            input("回车以返回")
            return
        confirm = input_with_flush("输入y以离开力矩模式返回到位置模式 (y)")
        if confirm == "y":
            print("已切换到位置模式")
            self.robot.send_command(self.selected_arm, {"command":"set_control_mode", "mode": "position"})
            input("回车以返回")
            return


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

        with open(log_dir / f"highfreq_{tstamp}_{self.selected_arm}.json", "w") as f:
            json.dump(highfreq, f, indent=2)

        with open(log_dir / f"lowfreq_{tstamp}_{self.selected_arm}.json", "w") as f:
            json.dump(lowfreq, f, indent=2)

        print(f"✅ 日志已保存到 {log_dir}")
        highfreq_file_name = f"highfreq_{tstamp}_{self.selected_arm}.json"
        lowfreq_file_name = f"lowfreq_{tstamp}_{self.selected_arm}.json" 
        return highfreq_file_name,lowfreq_file_name,tstamp  

    def edit_config(self,key_path:str):
        os.system('clear')
        print("=== 编辑配置文件 ===\n")
        self.config.show()

        data = self.config.data

        # Step 1: 输入链式路径
        # key_path = input("\n输入键路径 (e.g. left_arm.admittance_config.param_mass): ").strip()
        keys = key_path.split(".")

        # Step 2: 找到目标字段
        target = data
        for k in keys[:-1]:
            if k not in target:
                print(f"未找到键: '{k}' not found.")
                input("Press Enter to return...")
                return
            target = target[k]

        last_key = keys[-1]
        if last_key not in target:
            print(f"Invalid key: '{last_key}' not found.")
            input("Press Enter to return...")
            return

        old_value = target[last_key]
        print(f"\nCurrent value of '{key_path}':\n  {old_value}")
        print(f"Type: {type(old_value).__name__}")

        readline.parse_and_bind("set editing-mode emacs")
        readline.parse_and_bind("set history-preserve-point on")
        # Step 3: 输入新值并尝试类型匹配
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
                # 自动类型推断（按原类型解析）
                if isinstance(old_value, bool):
                    new_value = new_val_str.lower() in ["true", "1", "yes"]
                elif isinstance(old_value, int):
                    new_value = int(new_val_str)
                elif isinstance(old_value, float):
                    new_value = float(new_val_str)
                elif isinstance(old_value, list):
                    # 允许输入 Python 风格或逗号分隔字符串
                    if new_val_str.startswith("["):
                        new_value = eval(new_val_str)
                    else:
                        new_value = [eval(x.strip()) for x in new_val_str.split(",")]
                else:
                    new_value = new_val_str
                break
            except Exception as e:
                print(f"Invalid format: {e}")
                continue

        # Step 4: 确认修改
        print("\n=== 确认更改 ===")
        print(f"{key_path}:\n  {old_value}  →  {new_value}")
        confirm = input("是否应用修改? (y/n): ").strip().lower()
        if confirm != "y":
            print("值更新取消")
            input("回车以返回")
            return

        # Step 5: 写入并保存
        target[last_key] = new_value
        if hasattr(self.config, "save"):
            self.config.save()

        print("\n配置更新成功.")
        input("回车以继续")

    def task_tracking_error(self):
        os.system('clear')
        key_path = self.selected_arm + ".max_velocity"
        task_steps = {
            "1": {
                "description": "设置速度限制参数", 
                "callback": self.set_max_vel_acc,
            },
            "2": {
                "description": "梯形速度规划移动并测试跟随性",
                "callback": self.cubic_move_record,
            },
            "3":{
                "description": "可选修改最大速度配置",
                "callback": lambda: self.edit_config(key_path=key_path),
            },
            "4":{
                "description": "梯形速度规划移动",
                "callback": self.cubic_move,
            },
            "5":{
                "description": "记录点位",
                "callback": self.record_current_point,
            },
            "6":{
                "description": "保存记录点位到本地",
                "callback": self.save_recorded_points,
            },
            "7":{
                "description": "按输入点轨迹字典走点",
                "callback": self.move_multiple_points,
            },
            "8":{
                "description": "GO HOME",
                "callback": self.cubic_move_home,
            }
        }
        # 使用菜单栈
        self.menu_stack.append((task_steps,"20Hz下的轨迹跟踪测试"))

    def task_verify(self):
        os.system('clear')
        key_path = self.selected_arm + ".invert_directions"
        task_steps = {
            "0": {
                "description": "切换到力矩模式",
                "callback": self.set_torque_mode,
            },
            "1": {
                "description": "移动机械臂到关节零位", 
                "callback": lambda: self._run_point_in_joint([0.0]*7),
            },
            "2": {
                "description": "少量正方向移动机械臂以验证方向",
                "callback": lambda: self._run_point_in_joint([0.1]*7),
            },
            "3": {
                "description": "关节空间移动机械臂(可跳过)",
                "callback": self._run_point_with_input,
            },
            "4": {
                "description": "修改配置",
                "callback": lambda: self.edit_config(key_path=key_path),
            }
        }
        # 使用菜单栈
        self.menu_stack.append((task_steps,"验证关节旋转方向正确"))
    
    def run(self):
        self.menu_stack = []  # 初始化菜单栈
        self.select_arm()

        # 主菜单
        main_menu = {
            "1": ("验证关节旋转方向正确", self.task_verify),
            "2": ("标定硬件零位偏移(未做完)", self._void),
            "3": ("20Hz机械臂关节空间轨迹跟踪测试", self.task_tracking_error)
        }
        self.menu_stack.append((main_menu,"Main Menu"))

        while self.running:
            os.system('clear')
            # --- 状态刷新 ---
            print(f"{GREEN}=== 机器人实时状态 ({self.selected_arm}) ==={RESET}")
            print(f"{GREEN}关节位置: {self.feedbackData.QActual}{RESET}")
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
    hblog.start(config)
    try:
        arm_guide = Guide()
        arm_guide.run()
    finally:
        arm_guide.robot.shutdown()