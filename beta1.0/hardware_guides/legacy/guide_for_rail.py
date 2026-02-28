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
                self.rail_pos = -1
                self.rb_time = -1
                # ... 自定义反馈数据
                '''
                self.Wrench = -1
                '''
        self.feedbackData = feedbackItem() # 自定义反馈数据结构体
        # 状态管理
        self.running = True
        self.name = "rail"
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
        while True:
            feedInfo = self.robot.get_states()
            # feedInfo = None
            if feedInfo is not None:
                self.feedbackData.rail_pos = feedInfo[self.name]["position"]
                self.feedbackData.rb_time = feedInfo[self.name]["time"]["secs"]+feedInfo[self.name]["time"]["nanos"] * 1e-9
            
                if self.record_flag.is_set():
                    self.highfreq_log.append({
                        "rb_time":self.feedbackData.rb_time,
                        f"highfreq_pos":self.feedbackData.rail_pos,
                    })
            time.sleep(0.005)

    def _start(self):
        self.robot = rb_python.robot.Robot(self.launch_config_dict)
        self.robot.wait_for_operational(10)
        # 状态反馈线程
        feedback_thread = threading.Thread(
            target=self._GetFeedback
        )
        feedback_thread.daemon = True
        feedback_thread.start()
        print("已使能.")

    def _run_point(self,pos):
        print("Moving...\n")
        while np.abs(self.feedbackData.rail_pos-np.array(pos)) > 1e-3:
            self.robot.set_actions({self.name: {"type": "position", "position": pos}})
            time.sleep(1 / CONTROL_HZ)
        print("到达!")
        input_with_flush("回车以结束")

    def select_none(self):
        os.system('clear')
        config_path = str(BASE_DIR) + f"/configs/{self.name}.yaml"
        self.config = ConfigManager(config_path=config_path)
        self.launch_config_dict = self._get_launch_config()
        print(f"✅ 已选择: {self.name}")
        self._start()
        input(f"回车以进入主菜单")
    
    def _run_point_with_input(self):
        try:
            raw_input = input_with_flush(f"输入位置 (e.g. 0.1): ")
            pos = float(raw_input)

            if len([pos]) != 1:
                print(f"\nError: Expected 1 value, but got {len([pos])}.")
                return
            
            print(f"\n正在移动到新位置: {[f'{pos:.3f}']}")
            self._run_point([pos])
        
        except Exception as e:
            print(f"\nError:{e}")
            input("回车以返回上一级目录")

    def _void(self):
        return
    
    def set_max_vel_acc(self):
        import os

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
            "feedback_pos":self.feedbackData.rail_pos,
            "rb_time":self.feedbackData.rb_time,
        })
    
    def cubic_move(self):
        start_point = float(self.feedbackData.rail_pos[0])
        
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
            self.robot.set_actions({self.name: {"type": "position", "position": q.tolist()}})
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

        draw(log_dir='logs',lowfile=lowfreq_filename,highfile=highfreq_filename,savefig=f"viz_{self.name}_{tstamp}.png")
        
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

        with open(log_dir / f"highfreq_{tstamp}_{self.name}.json", "w") as f:
            json.dump(highfreq, f, indent=2)

        with open(log_dir / f"lowfreq_{tstamp}_{self.name}.json", "w") as f:
            json.dump(lowfreq, f, indent=2)

        print(f"✅ 日志已保存到 {log_dir}")

        highfreq_file_name = f"highfreq_{tstamp}_{self.name}.json"
        lowfreq_file_name = f"lowfreq_{tstamp}_{self.name}.json"

        return highfreq_file_name,lowfreq_file_name,tstamp

    def edit_config(self,key_path:str):
        os.system('clear')
        print("=== 编辑配置文件 ===\n")
        print("目前的配置文件:")
        pprint.pprint(self.config.data, indent=2, width=80, depth=3)

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
        key_path = self.name + ".max_velocity"
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
        key_path = self.name + ".invert_directions"
        task_steps = {
            "1": {
                "description": "移动导轨到零位", 
                "callback": lambda: self._run_point([0.0]),
            },
            "2": {
                "description": "少量正方向移动导轨以验证方向",
                "callback": lambda: self._run_point([0.1]),
            },
            "3": {
                "description": "以需要位置移动导轨(可跳过)",
                "callback": self._run_point_with_input,
            },
            "4": {
                "description": "修改配置",
                "callback": lambda: self.edit_config(key_path=key_path),
            },
            "5":{
                "description": "获取当前范围",
                "callback": self._get_limits,
            },
            "6":{
                "description": "设置关节范围",
                "callback": lambda: self._set_limits(limit_low=0.0,limit_high=0.5)
            },
        }
        # 使用菜单栈
        self.menu_stack.append((task_steps,"验证关节移动方向正确"))
    
    def _get_limits(self):
        print("当前关节范围:",self.robot.send_command(self.name, {"command": "get_limit"}))
    
    def _set_limits(self,limit_low,limit_high):
        self.robot.send_command(self.name,{"command": "set_limit", "enabled": [False], "lower": [limit_low],"upper": [limit_high],}),

    def run(self):
        self.menu_stack = []  # 初始化菜单栈
        self.select_none()

        # 主菜单
        main_menu = {
            "1": ("验证导轨移动方向正确", self.task_verify),
            "2": ("20Hz导轨轨迹跟踪测试", self.task_tracking_error),
        }
        self.menu_stack.append((main_menu,"Main Menu"))

        while self.running:
            os.system('clear')
            # --- 状态刷新 ---
            print(f"{GREEN}=== 导轨实时状态 ({self.name}) ==={RESET}")
            print(f"{GREEN}关节位置: {self.feedbackData.rail_pos}{RESET}")
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
        # self.robot.shutdown()

if __name__ == "__main__":
    try:
        rail_guide = guide()
        rail_guide.run()
    finally:
        print("shutdown...")
        rail_guide.robot.shutdown()