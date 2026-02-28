from prelude import *

class HeadGuide(BaseGuide):
    def __init__(self):
        super().__init__()
        self.logger = LogModule(self)
        self.motion = MotionModule(self)
    
        class feedbackItem:
            def __init__(self):
                self.head_pos = -1
                self.rb_time = -1
        self.feedbackData = feedbackItem()
    ## 抽象方法实现
    def _get_feedback(self):
        while not self._stop_event.is_set():
            try:
                feed = self.robot.get_states()

                # 无数据 → 继续下一次
                if not feed:
                    time.sleep(0.02)
                    continue

                states = feed.get("head", None)
                if not states:
                    time.sleep(0.02)
                    continue

                q_actual = states.get("position")
                rb_t = states.get("time")
                if isinstance(q_actual, (list, tuple)):
                    self.feedbackData.head_pos = np.array(q_actual)
                    self.feedbackData.last_update_time = time.time()
                    secs = rb_t.get("secs")
                    nanos = rb_t.get("nanos") * 1e-9
                    self.feedbackData.rb_time = secs + nanos

                if self.record_flag.is_set():
                    self.logger.push_highfreq(rb_time=self.feedbackData.rb_time,
                                              feedback_pos=self.feedbackData.head_pos)

            except Exception as e:
                print(f"[反馈线程异常] {e}")
                # 给系统一点缓冲时间，避免异常狂刷
                time.sleep(0.1)
            time.sleep(0.004)
    def _load_config(self, config_path):
        os.system("clear")
        config_path = str(f"/opt/robot/rb_hardware/head.yaml")
        MyHead = HeadConfig.create(config_path)
        MyHardware = HardwareConfig.create_from_container({"head": MyHead})
        MyRobot = RobotConfig.recreate({
            "hardware": MyHardware.to_dict_container(),
            "planner":None,
            "robot_model":""
        })
        self.config = MyRobot
        print(f"{GREEN}已选择机器人头部: head{RESET}\n已加载配置文件: {config_path}")
    
    def task_verify(self):
        key_path = self.name + ".invert_directions"
        menu = {
            "1": {
                "description": "移动头部到零位", 
                "callback": lambda: self.motion._run_point(
                    cur_q=self.feedbackData.head_pos,
                    q_pos=0.0,
                    part="head"
                ),
            },
            "2": {
                "description": "少量负方向移动头部以验证方向",
                "callback": lambda: self.motion._run_point(
                    cur_q=self.feedbackData.head_pos,
                    q_pos=-0.1,
                    part="head"
                ),
            },
            "3": {
                "description": "以需要位置移动头部(可跳过)",
                "callback": self.motion._run_point_with_input(
                    cur_q=self.feedbackData.head_pos,
                    dof=len(self.feedbackData.head_pos),
                    part="head",
                ),
            },
            "4": {
                "description": "修改方向配置",
                "callback": lambda: self.edit_config(key_path=key_path),
            },
            "5": {
                "description": "设置当前位置为零点",
                "callback": self.motion.set_zero(part="head"),
            }
        }
        # 使用菜单栈
        self.push_menu((menu,"验证关节旋转方向正确"))
if __name__ == "__main__":
    guide = HeadGuide()
    guide.push_menu({
        "1":{"description":"验证头部移动方向正确","callback":guide.task_verify},
    },"主菜单")
    guide.run()