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
import copy
from base_guide_curses import UILogger,CursesUI,HBlogBuffer,HBlogTailer

class BaseGuide(ABC):
    """
    机器人控制/传感器反馈基类
    - 提供线程管理,反馈采集,配置管理,菜单管理基础功能
    - 子类需要实现机器人初始化,反馈解析和任务逻辑
    """
    def __init__(self):
        ## --- runtime ---
        self.running = True
        self._stop_event = threading.Event()

        ## --- feedback ---
        self.feedback_lock = threading.Lock()
        self.feedbackData = None
        self.feedback_thread = None

        ## --- menu ---
        self.menu_stack = []

        ## --- logger (UI safe) ---
        self.loggerUI = UILogger()
        self.hblog_buffer = HBlogBuffer()
        self.hblog_tailer = HBlogTailer(path=f"{project_root}/logs/file.log",buffer=self.hblog_buffer)
        self.hblog_tailer.start()

        ## --- infra ---
        self.config: RobotConfig = None
        self.robot = None
        self.model = None
        self.planner = None
        self.FeedbackItem = None
        self.record_flag = threading.Event()
        self.highfreq_log = []
        self.lowfreq_log = []
    # --- lifecycle ---
    def run(self):
        self._load_config()
        self._init_robot()
        self.start_feedback_thread()
        self.ui = CursesUI(self) ## cursesUI接受guide的所有属性和函数
        self.ui.run()
        self.stop_feedback_thread()
        self._shutdown_robot()
    
    # ---反馈数据结构体和采集线程---
    @abstractmethod
    def _get_feedback(self):
        pass

    def start_feedback_thread(self):
        """
        启动反馈采集线程
        """
        self._stop_event.clear()
        self.feedback_thread = threading.Thread(
            target=self._feedback_loop,
            daemon=True
        )
        self.feedback_thread.start()

    def stop_feedback_thread(self):
        self._stop_event.set()
        if self.feedback_thread:
            self.feedback_thread.join(timeout=2.0)

    def _feedback_loop(self):
        try:
            self._get_feedback()
        except Exception as e:
            self.loggerUI.error(f"feedback thread error:{e}")
    def get_feedback_value(self, attr_name):
        """线程安全地获取反馈数据"""
        with self.feedback_lock:
            return getattr(self.feedbackData, attr_name).copy() if hasattr(self.feedbackData, attr_name) else None
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
        self.loggerUI.info("robot init done")

    @abstractmethod
    def _load_config(self,config_path:str):
        """
        加载配置文件
        子类需要实现具体的配置加载逻辑
        """
        pass
    # ---编辑配置文件---
    def edit_config(self,key_path:str):
        """
        编辑配置文件
        子类可覆盖实现具体的配置编辑逻辑
        """
        os.system("clear")
        print("=== 编辑配置文件 ===")
        print("目前的配置文件:")
        pprint.pprint(self.config.get_data(),indent=2,width=80,depth=3)
        self.config.set_component(key_path=key_path)

    # ---机器人关闭---
    def _shutdown_robot(self):
        """
        机器人关闭逻辑
        子类需要实现具体的关闭逻辑
        """
        self.robot.shutdown()
        self.hblog_tailer.stop()
        time.sleep(1)
        print("robot shutdown")