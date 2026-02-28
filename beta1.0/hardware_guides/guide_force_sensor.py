from prelude import *

class ForceSensorGuide(BaseGuide):
    def __init__(self):
        super().__init__()
        class FeedbackItem:
            def __init__(self):
                self.Wrench = -1
                self.last_update_time = time.time()
        self.feedbackData = FeedbackItem()
    ## 抽象方法实现
    def _get_feedback(self):
        while not self._stop_event.is_set():
            try:
                feed = self.robot.get_states()
                # 无数据 → 继续下一次
                if not feed:
                    time.sleep(0.02)
                    continue

                sensor = feed.get(self.selected_force_sensor, None)
                if not sensor:
                    time.sleep(0.02)
                    continue

                # 保护性检测 force/torque
                f = sensor.get("force")
                t = sensor.get("torque")

                if isinstance(f, (list, tuple)) and isinstance(t, (list, tuple)):
                    # 防崩保护
                    if len(f) == 3 and len(t) == 3:
                        self.feedbackData.Wrench = np.array(f + t)
                        self.feedbackData.last_update_time = time.time()
                
            except Exception as e:
                print(f"[反馈线程异常] {e}")
                # 给系统一点缓冲时间，避免异常狂刷
                time.sleep(0.1)
            time.sleep(0.004)

    def _load_config(self):
        os.system("clear")
        print(f"{CYAN}=== 选择左/右侧六维力传感器 ==={RESET}")
        opts = ["left_force_sensor", "right_force_sensor"]
        for i, s in enumerate(opts, 1):
            print(f"[{i}] {s}")
        while True:
            key = self.get_single_key()
            if key in ["1", "2"]:
                self.selected_force_sensor = opts[int(key)-1]
                break
        config_path = str(f"/opt/robot/rb_hardware/{self.selected_force_sensor}.yaml")
        
        MySensor = ForceSensorConfig.create(config_path)
        MyHardware = HardwareConfig.create_from_container({self.selected_force_sensor: MySensor})

        MyRobot = RobotConfig.recreate({
            "hardware": MyHardware.to_dict_container(),
            "planner": None,
            "robot_model": ""
        })
        self.config = MyRobot
        print(f"{GREEN}已选择传感器: {self.selected_force_sensor}{RESET}\n已加载配置文件: {config_path}")

    ## 一般方法重写
    # 使用基类的方法
        
    ## 任务菜单
    def task_verify(self):
        key_id = f"{self.selected_force_sensor}.id"
        key_freq = f"{self.selected_force_sensor}.freq"

        menu = {
            "1":{
                "description":"修改 can id 配置",
                 "callback":lambda: self.edit_config(key_id)
            },
            "2":{"description":"修改 ports 配置",
                 "callback":lambda: self.edit_config(key_freq)
            },
        }
        self.push_menu(menu,"六维力传感器配置修改菜单")
    
if __name__ == "__main__":
    guide = ForceSensorGuide()
    guide.push_menu({"1":{"description":"验证传感器","callback":guide.task_verify}},"主菜单")
    guide.run()
    