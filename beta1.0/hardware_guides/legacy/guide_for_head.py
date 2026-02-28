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
                self.head_pos = -1
                self.rb_time = -1
                # ... 自定义反馈数据
                '''
                self.Wrench = -1
                '''
        self.feedbackData = feedbackItem() # 自定义反馈数据结构体
        # 状态管理
        self.running = True
        self.name = "head"

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
                self.feedbackData.head_pos = feedInfo[self.name]["position"]
                self.feedbackData.rb_time = feedInfo[self.name]["time"]["secs"]+feedInfo[self.name]["time"]["nanos"] * 1e-9
            time.sleep(0.005)

    def _start(self):
        self.robot = rb_python.robot.Robot(self.launch_config_dict)
        self.robot.wait_for_operational(10)
        print("使用默认的位置模式.")
        self.robot.send_command(f"{self.name}", {"command": "set_control_mode", "mode": "position"})
        # 状态反馈线程
        feedback_thread = threading.Thread(
            target=self._GetFeedback
        )
        feedback_thread.daemon = True
        feedback_thread.start()
        print("已使能.")

    def _run_point(self,pos):
        print("Moving...\n")
        while np.abs(self.feedbackData.head_pos-np.array(pos)) > 1e-3:
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
    
    def _set_zero(self):
        self.robot.send_command(self.name, {"command": "set_zero"})
        input("零点设置成功！")
    
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
        key_path = self.name + ".invert_directions"
        task_steps = {
            "1": {
                "description": "移动头部到零位", 
                "callback": lambda: self._run_point([0.0]),
            },
            "2": {
                "description": "少量负方向移动头部以验证方向",
                "callback": lambda: self._run_point([-0.1]),
            },
            "3": {
                "description": "以需要位置移动头部(可跳过)",
                "callback": self._run_point_with_input,
            },
            "4": {
                "description": "修改配置",
                "callback": lambda: self.edit_config(key_path=key_path),
            },
            "5": {
                "description": "设置当前位置为零点",
                "callback": self._set_zero,
            }
        }
        # 使用菜单栈
        self.menu_stack.append((task_steps,"验证关节旋转方向正确"))
    
    def run(self):
        self.menu_stack = []  # 初始化菜单栈
        self.select_none()

        # 主菜单
        main_menu = {
            "1": ("验证头部移动方向正确", self.task_verify),
        }
        self.menu_stack.append((main_menu,"Main Menu"))

        while self.running:
            os.system('clear')
            # --- 状态刷新 ---
            print(f"{GREEN}=== 头部实时状态 ({self.name}) ==={RESET}")
            print(f"{GREEN}关节位置: {self.feedbackData.head_pos}{RESET}")
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
        head_guide = guide()
        head_guide.run()
    finally:
        print("\nShutdown...")
        head_guide.robot.shutdown()