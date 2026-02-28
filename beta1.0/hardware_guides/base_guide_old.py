import threading
import time
import os
from abc import ABC, abstractmethod
import os, sys, select, termios, tty, pprint
cur_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(cur_dir)
folder_utils = os.path.join(project_root,'utils')
sys.path.insert(0,folder_utils)
from robotConfig_util import RobotConfig
import rb_python
# ANSI 颜色码
GREEN = "\x1b[32m"
YELLOW = "\x1b[33m"
CYAN = "\x1b[36m"
RESET = "\x1b[0m"

class BaseGuide(ABC):
    """
    机器人控制/传感器反馈基类
    - 提供线程管理,反馈采集,配置管理,菜单管理基础功能
    - 子类需要实现机器人初始化,反馈解析和任务逻辑
    """
    def __init__(self):
        self.robot = None
        self.running = True
        self.menu_stack = []
        self._stop_event = threading.Event()
        self.feedback_thread = None
        self.feedbackData = None
        self.config: RobotConfig = None
        self.model = None
        self.planner = None
        self.FeedbackItem = None
        self.record_flag = threading.Event()
        self.highfreq_log = []
        self.lowfreq_log = []
    

    # ---反馈数据结构体和采集线程---
    @abstractmethod
    def _get_feedback(self):
        """
        反馈采集线程函数
        子类需要实现具体的反馈采集逻辑
        """
        pass

    def start_feedback_thread(self):
        """
        启动反馈采集线程
        """
        self._stop_event.clear()

        def wrapper():
            try:
                self._get_feedback()
                self.feedbackData.last_update_time = time.time()
            except Exception as e:
                print(f"[反馈线程异常] {e}")
            finally:
                print("反馈线程退出")

        self.feedback_thread = threading.Thread(
            target=wrapper,
            daemon=False
        )
        self.feedback_thread.start()

    
    def stop_feedback_thread(self):
        self._stop_event.set()
        if self.feedback_thread is not None and self.feedback_thread.is_alive():
            self.feedback_thread.join(timeout=2.0)
            if self.feedback_thread.is_alive():
                print("警告: 反馈线程未按时退出（可能卡在 I/O）")
        print("反馈线程已停止")


    # ---菜单管理---
    def push_menu(self,menu:dict,title:str):
        """
        压入新菜单
        """
        self.menu_stack.append((title,menu))
    def pop_menu(self):
        """
        弹出当前菜单
        """
        if len(self.menu_stack) > 1:
            self.menu_stack.pop()
    # ---机器人初始化和配置管理---
    def _init_robot(self):
        """
        机器人初始化
        子类需要实现具体的机器人初始化逻辑
        """
        config = self.config.get_data()
        # import pdb;pdb.set_trace()
        self.robot = rb_python.robot.Robot(config)
        time.sleep(1)
        print(f"{GREEN}机器人初始化完成{RESET}")
        input("按回车键继续...")

    @abstractmethod
    def _load_config(self,config_path:str):
        """
        加载配置文件
        子类需要实现具体的配置加载逻辑
        """
        pass
    # ---主循环---
    def run(self):
        """
        主循环:显示反馈和菜单
        """
        try:
            self._load_config()
            self._init_robot()
            self.start_feedback_thread()

            while self.running:
                self._display_feedback()
                self._display_menu()
                self._process_input()
                time.sleep(0.1)

        except KeyboardInterrupt:
            print(f"{YELLOW}\n检测到 Ctrl+C，正在安全关闭...{RESET}")
            self.running = False

        finally:
            print("Shutting down...")
            self.stop_feedback_thread()
            self._shutdown_robot()
            print(f"{GREEN}程序已安全退出{RESET}")

    
    # --- 可选覆盖的方法 ---
    def get_single_key(self,timeout=0.5):
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
    def _display_feedback(self):
        """
        通用显示反馈数据
        子类可覆盖实现具体的显示逻辑
        """
        os.system("clear")
        print("=== 反馈数据(基础版) ===")
        def dump(obj,indent=0):
            prefix = " " * indent
            # case 1:dict
            if isinstance(obj,dict):
                for k,v in obj.items():
                    print(f"{prefix}{k}:")
                    dump(v,indent+2)
            # case 2:list/tuple
            elif isinstance(obj,(list,tuple)):
                for i,v in enumerate(obj):
                    print(f"{prefix}[{i}]:")
                    dump(v,indent+2)
            # case 3:dataclass
            elif hasattr(obj,"__dataclass_fields__"):
                for k in obj.__dataclass_fields__.keys():
                    v = getattr(obj,k)
                    print(f"{prefix}{k}:")
                    dump(v,indent+2)
            # case 4: 普通对象（自动打印所有非__开头的属性）
            elif hasattr(obj, "__dict__"):
                for k, v in vars(obj).items():
                    if not k.startswith("_"):
                        print(f"{prefix}{k}:")
                        dump(v, indent + 2)

            # case 5: 原子类型（数字、字符串等）
            else:
                print(f"{prefix}{obj}")
        delta_time = time.time() - getattr(self.feedbackData, 'last_update_time', 0)
        if delta_time > 1.0:
            print(f"{YELLOW}警告: 反馈数据超过 {delta_time:.2f} 秒未更新！{RESET}")
        else:
            print("(最近更新时间: {:.3f} 秒前)".format(time.time() - getattr(self.feedbackData, 'last_update_time', 0)))
        dump(self.feedbackData)
        print("-" * 40)

    def _display_menu(self):
        """
        显示菜单
        子类可覆盖实现具体的菜单显示逻辑
        """
        title,menu = self.menu_stack[-1]
        print(f"{CYAN}=== {title} ==={RESET}")
        for key,item in menu.items():
            print(f"[{key}] {item['description']}")
        if len(self.menu_stack) > 1:
            print("[b] 返回上级菜单")
        print("[q] 退出")


    def _process_input(self):
        """
        处理用户输入
        子类可覆盖实现具体的输入处理逻辑
        """
        key = self.get_single_key()
        if not key:
            return
        if key.lower() == 'q':
            self.running = False
            return
        if key.lower() == 'b' and len(self.menu_stack) > 1:
            self.pop_menu()
            return
        title,menu = self.menu_stack[-1]
        if key in menu:
            cb = menu[key]["callback"]
            cb()
    # ---编辑配置文件---
    def edit_config(self,key_path:str):
        """
        编辑配置文件
        子类可覆盖实现具体的配置编辑逻辑
        """
        os.system("clear")
        print(f"{CYAN}=== 编辑配置文件 ==={RESET}")
        print("目前的配置文件:")
        pprint.pprint(self.config.get_data(),indent=2,width=80,depth=3)
        self.config.set_component(key_path=key_path)

    # ---机器人关闭---
    def _shutdown_robot(self):
        """
        机器人关闭逻辑
        子类需要实现具体的关闭逻辑
        """
        print("关闭机器人连接...")
        self.robot.shutdown()
        time.sleep(1)
        print(f"{GREEN}机器人已关闭{RESET}")

