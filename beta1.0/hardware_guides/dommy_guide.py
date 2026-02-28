from prelude import *

class GripperGuide(BaseGuide):
    def __init__(self):
        super().__init__()
        ## 将记录功能实现分到Logger类
        self.logger = LogModule(self)
        ## 将运动功能实现分到Motion类
        self.motion = MotionModule(self)
        class FeedbackItem:
            def __init__(self):
                self.gripper_pos = -1
                self.last_update_time = time.time()
                self.rb_time = -1
        self.feedbackData = FeedbackItem()
    ## 抽象方法实现
    def _get_feedback(self):
        period = 1.0 / 250.0
        next_time = time.perf_counter()
        seq = 0

        while not self._stop_event.is_set():
            try:
                # feed = self.robot.get_states()
                now = time.perf_counter()
                if now < next_time:
                    time.sleep(next_time - now) 
                
                t = time.time()
                fake_pos = 0.1 + 0.01 * (seq % 20)
                feed = {
                    self.selected_gripper: {
                        "position": [fake_pos],  # 保持 list，兼容你下面判断
                        "time": {
                            "secs": int(t),
                            "nanos": int((t % 1) * 1e9),
                        }
                    }
                }
                # 无数据 → 继续下一次
                if not feed:
                    time.sleep(0.001)
                    continue

                gripper_states = feed.get(self.selected_gripper, None)
                
                if not gripper_states:
                    time.sleep(0.001)
                    continue

                q_actual = gripper_states.get("position")
                rb_t = gripper_states.get("time")
                if isinstance(q_actual, (list, tuple)):
                    with self.feedback_lock:
                        self.feedbackData.gripper_pos = np.array(q_actual)
                        self.feedbackData.last_update_time = time.time()
                        secs = rb_t.get("secs",0)
                        nanos = rb_t.get("nanos",0) * 1e-9
                        self.feedbackData.rb_time = secs + nanos
                    # dirty flag
                    self.feedback_dirty = True
                    if self.record_flag.is_set():
                        self.logger.push_highfreq(
                            rb_time=self.feedbackData.rb_time,
                            feedback_pos=self.feedbackData.gripper_pos
                        )
                seq += 1
                next_time += period
            except Exception as e:
                self.loggerUI.error(f"feedback error:{e}")
                # 给系统一点缓冲时间，避免异常狂刷
                time.sleep(0.1)
    def _load_config(self):
        opts = ["left_gripper","right_gripper"]
        selected_idx = 0

        def menu_loop(stdscr):
            nonlocal selected_idx
            curses.curs_set(0)
            h,w = stdscr.getmaxyx()
            win_h = len(opts)+4
            win_w = max(len(o) for o in opts) + 10
            start_y = (h - win_h) // 2
            start_x = (w - win_w) // 2
            win = curses.newwin(win_h,win_w,start_y,start_x)
            win.keypad(True)
            win.box()
            while True:
                win.erase()
                win.box()
                win.addstr(0,2,"选择夹爪")
                for i,opt in enumerate(opts):
                    if i == selected_idx:
                        win.attron(curses.A_REVERSE)
                    win.addstr(2+i,4,opt)
                    if i == selected_idx:
                        win.attroff(curses.A_REVERSE)
                win.refresh()
                key = win.getch()
                if key in [curses.KEY_UP,ord('k')]:
                    selected_idx = (selected_idx - 1) % len(opts)
                elif key in [curses.KEY_DOWN, ord('j')]:
                    selected_idx = (selected_idx + 1) % len(opts)
                elif key in [curses.KEY_ENTER, ord('\n')]:
                    break
        curses.wrapper(menu_loop)
        self.selected_gripper = opts[selected_idx]
        config_path = Path("/opt/robot/rb_config.yaml")
        MyRobot = RobotConfig.recreate({
            "hardware": RobotConfig.create(config_path).hardware,
            "planner": RobotConfig.create(config_path).planner,
            "robot_model": RobotConfig.create(config_path).robot_model,
        })
        self.config = MyRobot
        self.loggerUI.info(f"已选择夹爪: {self.selected_gripper}")
        self.loggerUI.info(f"已加载配置文件: {config_path}")


    def task_tracking_error(self):
        key_path = self.selected_gripper + ".max_velocity"
        menu = {
            "1": {
                "description": "设置速度限制参数", 
                "callback": self.motion.set_max_vel_acc,
            },
            "2": {
                "description": "梯形速度规划移动",
                "callback": lambda: self.motion.cubic_move_record(start_data=self.feedbackData.gripper_pos,
                                                          max_vel=[self.motion.max_vel],
                                                          max_acc=[self.motion.max_acc],
                                                          part=self.selected_gripper,
                                                          pos_name="gripper_pos"),
            },
            "3":{
                "description": "可选修改最大速度配置",
                "callback": lambda: self.edit_config(key_path=key_path),
            }
        }
        # 使用菜单栈
        self.push_menu(menu,"20Hz下的轨迹跟踪测试")
    def task_verify(self):
        key_path = self.selected_gripper + ".invert_directions"
        menu = {
            "1": {
                "description": "移动夹爪到零位", 
                "callback": lambda: self.motion._run_point(q_name="gripper_pos",
                                                           q_pos=0.0,
                                                           part=self.selected_gripper),
            },
            "2": {
                "description": "少量正方向移动夹爪以验证方向",
                "callback": lambda: self.motion._run_point(q_name="gripper_pos",
                                                    q_pos=0.01,
                                                    part=self.selected_gripper),
            },
            "3": {
                "description": "以需要位置移动夹爪(可跳过)",
                "callback": lambda: self.motion._run_point_with_input(q_name="gripper_pos",
                                                              dof=len(self.feedbackData.gripper_pos),
                                                              part=self.selected_gripper),
            },
            "4": {
                "description": "修改配置",
                "callback": lambda: self.edit_config(key_path=key_path),
            }
        }
        # 使用菜单栈
        self.push_menu(menu,"验证关节移动方向正确")
    def _edit_menu(self):
        menu = {
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
        self.push_menu(menu,"参数修改")
    def task_calibrate(self):
        menu = {
            "1": {
                "description": "更改运动上下限",
                "callback": lambda: self.motion._init_update_limits(min_pos=[-1],
                                                                    max_pos=[1],
                                                                    part=self.selected_gripper),
            },
            "2": {
                "description": "设置更宽容的力矩限制 重启生效", 
                "callback": lambda: self.edit_config(key_path=f"{self.selected_gripper}.max_torque"),
            },
            "3": {
                "description": "设置传动比为1.0 重启生效",
                "callback": lambda: self.edit_config(key_path=f"{self.selected_gripper}.length_per_radian"),
            },
            "4": {
                "description": "点按操作模式",
                "callback": lambda: self.motion._manual_control_1dof(data_name="gripper_pos",
                                                                     part=self.selected_gripper),
            },
            "5": {
                "description": "设置传动比并重启生效",
                "callback": lambda: self.edit_config(key_path=f"{self.selected_gripper}.length_per_radian"),
            },
            "6": {
                "description": "设置硬件零点并重启生效",
                "callback": lambda: self.motion.set_zero(part=self.selected_gripper)
            },
            "7": {
                "description": "设置硬件零点偏移并重启生效",
                "callback": lambda: self.edit_config(key_path=f"{self.selected_gripper}.offset_at_hardware_zero"),
            }
        }
        # 使用菜单栈
        self.push_menu(menu,"验证关节移动方向正确")
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
    hblog.info("test","Gripper Guide Start")
    guide = GripperGuide()
    guide.push_menu({
        "1":{"description":"验证夹爪移动方向正确","callback":guide.task_verify},
        "2":{"description":"标定硬件零位偏移和转换系数","callback":guide.task_calibrate},
        "3":{"description":"20Hz夹爪轨迹跟踪测试","callback":guide.task_tracking_error},
    },"主菜单")
    guide.run()