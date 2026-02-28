from prelude import *

class LoadcellGuide(BaseGuide):
    def __init__(self):
        super().__init__()
        self.logger = LogModule(self)
        self.motion = MotionModule(self)
        class FeedbackItem:
            def __init__(self):
                self.loadcell_ch1 = -1
                self.loadcell_ch2 = -1
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

                states = feed.get(self.selected_loadcell, None)
                if not states:
                    time.sleep(0.02)
                    continue

                channels = states.get("value")
                rb_t = states.get("time")
                if isinstance(channels, (list, tuple)):
                    self.feedbackData.loadcell_ch1 = np.array(channels[0])
                    self.feedbackData.loadcell_ch2 = np.array(channels[1])
                    self.feedbackData.last_update_time = time.time()
                    secs = rb_t.get("secs")
                    nanos = rb_t.get("nanos") * 1e-9
                    self.feedbackData.rb_time = secs + nanos

                if self.record_flag.is_set():
                    self.logger.push_highfreq(rb_time=self.feedbackData.rb_time,
                                              feedback_pos=self.feedbackData.loadcell_ch1)

            except Exception as e:
                print(f"[反馈线程异常] {e}")
                # 给系统一点缓冲时间，避免异常狂刷
                time.sleep(0.1)
            time.sleep(0.004)
    def _load_config(self):
        os.system('clear')
        print(f"{CYAN}=== 选择左/右侧指尖传感器 ==={RESET}")
        options = ["left_loadcell","right_loadcell"]
        for i, name in enumerate(options, 1):
            print(f"{CYAN}[{i}] {name}{RESET}")
        print(f"{CYAN}\n按下1或2来选择左侧或右侧...{CYAN}")
        while True:
            key = self.get_single_key()
            if key in ["1", "2"]:
                self.selected_loadcell = ["left_loadcell", "right_loadcell"][int(key)-1]
                break
        config_path = str(f"/opt/robot/rb_hardware/{self.selected_loadcell}.yaml")
        MyLoadcell = LoadcellConfig.create(config_path)
        MyHardware = HardwareConfig.create_from_container({self.selected_loadcell: MyLoadcell})
        MyRobot = RobotConfig.recreate({
            "hardware": MyHardware.to_dict_container(),
            "planner":None,
            "robot_model":""
        })
        self.config = MyRobot
        print(f"{GREEN}已选择一维力传感器: {self.selected_loadcell}{RESET}\n已加载配置文件: {config_path}")
    ## 菜单
    def task_verify(self):
        key_path_ip = self.selected_loadcell + ".usb_bus"
        key_path_port = self.selected_loadcell + ".usb_ports"
        menu = {
            "1": {
                "description": "测试传感器ch1置零",
                "callback": lambda: self.motion.set_zero_loadcell(part=self.selected_loadcell,ch=0),
            },
            "2": {
                "description": "测试传感器ch2置零",
                "callback": lambda: self.motion.set_zero_loadcell(part=self.selected_loadcell,ch=1),
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
        self.push_menu(menu,"验证传感器通讯和数值正常")
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
    guide = LoadcellGuide()
    guide.push_menu({
        "1":{"description":"验证传感器通讯和数值正常","callback":guide.task_verify},
    },"主菜单")
    guide.run()