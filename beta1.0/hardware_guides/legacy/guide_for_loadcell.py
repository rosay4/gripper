from utils.config_manager import ConfigManager
# import rb_python
import numpy as np
import pprint
import threading
from utils.logger_util import setup_logger
from utils.keyboard_util2 import KeyboardCommander, input_with_flush
from pathlib import Path
import time
import os, sys, select, termios, tty
import readline
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
                self.loadcell_ch1 = -1
                self.loadcell_ch2 = -1
                # ... 自定义反馈数据
                '''
                self.Wrench = -1
                '''
        self.feedbackData = feedbackItem() # 自定义反馈数据结构体
        # 状态管理
        self.running = True
        self.selected_loadcell = "" ## "left_loadcell" "right_loadcell"
        self.feedback_thread = None
        self._stop_event = threading.Event()

    def _get_launch_config(self):
        cfg = ConfigManager(config_path=str(BASE_DIR) + "/configs/temp.yaml")
        cfg.set_dict(new_data=self.config.data,
                          base_key="hardware")
        cfg.set("robot_model","")
        cfg.set("planner",None)
        return cfg.data

    def _GetFeedback(self):
        while not self._stop_event.is_set():
            # feedInfo = self.robot.get_states()
            feedInfo = None
            if feedInfo is not None:
                self.feedbackData.loadcell_ch1 = feedInfo[self.selected_loadcell]['values'][0]['value'][0] * 0.01
                self.feedbackData.loadcell_ch2 = feedInfo[self.selected_loadcell]['values'][0]['value'][0] * 0.01
            time.sleep(0.005)
        input("反馈线程已安全退出")

    def _start(self):
        self._stop_event.clear()
        # self.robot = rb_python.robot.Robot(self.launch_config_dict)
        # self.robot.wait_for_operational(10)
        # 状态反馈线程
        feedback_thread = threading.Thread(
            target=self._GetFeedback
        )
        feedback_thread.daemon = True
        feedback_thread.start()
        input(f"\n已使能\nPress Enter to continue...")
    
    def _stop_background_measure(self):
        self._stop_event.set()
        if self.feedback_thread is not None:
            self.feedback_thread.join()
        print("测量线程停止...")

    def _try_restart(self):
        self._stop_background_measure()
        # self.robot.shutdown()
        print("重启...")
        # time.sleep(3)
        self._start()

    def select_loadcell(self):
        os.system('clear')
        print(f"{CYAN}=== 选择左/右侧指尖传感器 ==={RESET}")
        options = ["left_loadcell","right_loadcell"]
        for i, name in enumerate(options, 1):
            print(f"{CYAN}[{i}] {name}{RESET}")
        print(f"{CYAN}\n按下1或2来选择左侧或右侧...{CYAN}")
        while True:
            key = get_single_key()
            if key in ["1", "2"]:
                self.selected_loadcell = ["left_loadcell", "right_loadcell"][int(key)-1]
                break
        config_path = str(BASE_DIR) + f"/configs/{self.selected_loadcell}.yaml"
        self.config = ConfigManager(config_path=config_path)
        self.launch_config_dict = self._get_launch_config()
        print(f"✅ 已选择: {self.selected_loadcell}")
        self._start()
        
    
    def setzero(self):
        # self.robot.send_command(self.selected_loadcell,{"command":"manual_zero"})
        input("置零")
        time.sleep(1)

    def _void(self):
        return
    
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

    
    def task_verify(self):
        os.system('clear')
        key_path_ip = self.selected_loadcell + ".tcp_ip"
        key_path_port = self.selected_loadcell + ".tcp_port"
        task_steps = {
            "1": {
                "description": "尝试重启", 
                "callback": self._try_restart,
            },
            "2": {
                "description": "测试传感器置零",
                "callback": self.setzero,
            },
            "3": {
                "description": "修改ip配置",
                "callback": lambda: self.edit_config(key_path=key_path_ip),
            },
            "4": {
                "description": "修改端口配置",
                "callback": lambda: self.edit_config(key_path=key_path_port),
            }
        }
        # 使用菜单栈
        self.menu_stack.append((task_steps,"验证传感器通讯和数值正常"))
    
    def run(self):
        self.menu_stack = []  # 初始化菜单栈
        self.select_loadcell()

        # 主菜单
        main_menu = {
            "1": ("验证传感器通讯和数值正常", self.task_verify),
        }
        self.menu_stack.append((main_menu,"Main Menu"))

        while self.running:
            os.system('clear')
            # --- 状态刷新 ---
            print(f"{GREEN}=== 传感器实时状态 ({self.selected_loadcell}) ==={RESET}")
            print(f"{GREEN}通道1: {[0.0]*7}{RESET}")
            print(f"{GREEN}通道2: {[0.0]*7}{RESET}")
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
    loadcell_guide = guide()
    loadcell_guide.run()