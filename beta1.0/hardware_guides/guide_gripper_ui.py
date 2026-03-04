from prelude import *
from enum import Enum, auto
import serial
import serial.tools.list_ports
import yaml
from laser_indicator import LaserIndicator
# Laser USB identifiers (from your device list)
LASER_USB_VID = 6790
LASER_USB_PID = 21978
LASER_USB_SERIAL = "0123456789"
LASER_SCAN_COOLDOWN_SEC = 2.0

class FeedbackMode(Enum):
    BASIC = auto()     # 杞ㄨ抗娴嬭瘯 / 楂橀
    FULL = auto()      # 姝ｅ父杩愯 / 鍏ㄩ噺
class GripperGuide(BaseGuide):
    def __init__(self):
        super().__init__()
        ## 灏嗚褰曞姛鑳藉疄鐜板垎鍒癓ogger绫?
        self.logger = LogModule(self)
        ## 灏嗚繍鍔ㄥ姛鑳藉疄鐜板垎鍒癕otion绫?
        self.motion = MotionModule(self)
        ## laser
        self.laser_left = None
        self.laser_right = None
        self.laser_read_interval = 0.05  # seconds
        self._laser_next_time = 0.0
        self.laser_scan_interval = LASER_SCAN_COOLDOWN_SEC
        self._laser_next_scan = 0.0
        self._laser_last_status = None
        class FeedbackItem:
            def __init__(self):
                self.gripper_pos = -1
                self.gripper_vel = -1
                self.gripper_torque = -1
                self.force_tip = [-1,-1]
                self.force_six_axis = [-1,-1,-1,-1,-1,-1]
                self.rb_time = -1
                self.device_fps = {}
                self.gripper_limits = [-1,-1]
                self.following_error_window = -1
                self.block_torque = -1
                self.temperature = -1
                self.runtime = "00:00:00.000"
                self.laser_left = None
                self.laser_right = None
                self.real_distance = -1
                self.laser_status = "disabled"
                self.laser_ports = {"left": None, "right": None}
                self.config_length_per_radian = None
                self.config_offset_at_hardware_zero = None
        self.feedbackData = FeedbackItem()
        self.feedback_mode = FeedbackMode.FULL
    ## 鎶借薄鏂规硶瀹炵幇
    def _get_feedback(self):
        period = 1.0 / 100.0
        next_time = time.perf_counter()
        start_time = time.perf_counter()
        while not self._stop_event.is_set():
            try:
                now = time.perf_counter()
                if now < next_time:
                    time.sleep(next_time - now)
                # 0. Laser
                if now >= self._laser_next_scan:
                    self._ensure_lasers()
                    self._laser_next_scan = now + self.laser_scan_interval
                if self.laser_left or self.laser_right:
                    if now >= self._laser_next_time:
                        left_val, right_val, status = self._read_lasers()
                        with self.feedback_lock:
                            if left_val is not None:
                                self.feedbackData.laser_left = [left_val]
                            if right_val is not None:
                                self.feedbackData.laser_right = [right_val]
                            if left_val is not None and right_val is not None:
                                self.feedbackData.real_distance = left_val + right_val
                            self.feedbackData.laser_status = status
                        if status.startswith("error"):
                            # drop connections to allow re-scan
                            self._disconnect_lasers()
                        self._laser_next_time = now + self.laser_read_interval
                # 1. 鍩虹鐘舵€?
                feed = self.robot.get_states()
                if not feed:
                    time.sleep(0.001)
                    continue

                gripper_states = feed.get(self.selected_gripper, None)
                loadcell_states = feed.get(self.selected_loadcell, None)
                if not gripper_states:
                    time.sleep(0.001)
                    continue

                q_actual = gripper_states.get("position")
                v_actual = gripper_states.get("velocity")
                torque_actual = gripper_states.get("torque")

                rb_t = gripper_states.get("time")
                # 2. 鍏朵粬淇℃伅
                if self.feedback_mode == FeedbackMode.FULL:
                    force_sensor_states = feed.get(self.selected_force_sensor, None)
                    limit_feed = self.robot.send_command(self.selected_gripper, {"command": "get_limit"})
                    window_feed = self.robot.send_command(self.selected_gripper, {"command": "get_following_error_window"})
                    block_torque_feed = self.robot.send_command(self.selected_gripper, {"command": "get_block_torque"})
                    temperature_feed = self.robot.send_command(self.selected_gripper,{"command": "get_temperature"})
                    # 璁＄畻杩愯鏃堕棿
                    elapsed = now - start_time
                    hours = int(elapsed // 3600)
                    minutes = int((elapsed % 3600) // 60)
                    seconds = int(elapsed % 60)
                    elapsed_str = f"{hours:02d}:{minutes:02d}:{seconds:02d}"
                    self.feedbackData.runtime = elapsed_str

                
                with self.feedback_lock:
                    if q_actual is not None:
                        self.feedbackData.gripper_pos = np.array(q_actual)
                        self.feedbackData.gripper_vel = np.array(v_actual)
                        self.feedbackData.gripper_torque = np.array(torque_actual)
                    # if loadcell_states:
                    #     self.feedbackData.force_tip[0] = loadcell_states['values'][0]['value'][0]/100
                    #     self.feedbackData.force_tip[1] = loadcell_states['values'][1]['value'][0]/100
                    if loadcell_states and 'value' in loadcell_states:
                        self.feedbackData.force_tip[:] = loadcell_states['value'][:2]
                    if rb_t:
                        secs = rb_t.get("secs", 0)
                        nanos = rb_t.get("nanos", 0) * 1e-9
                        self.feedbackData.rb_time = secs + nanos
                    if self.feedback_mode == FeedbackMode.FULL:
                        if force_sensor_states and 'force' in force_sensor_states:
                            self.feedbackData.force_six_axis = force_sensor_states['force'] + force_sensor_states['torque']
                        if limit_feed['lower'] and limit_feed['upper']:
                            self.feedbackData.gripper_limits = [limit_feed['lower'][0],limit_feed['upper'][0]]
                        if window_feed:
                            self.feedbackData.following_error_window = window_feed['value'][0]
                        if block_torque_feed:
                            self.feedbackData.block_torque = block_torque_feed['value'][0]
                        if temperature_feed:
                            self.feedbackData.temperature = temperature_feed['value'][0]

                if self.record_flag.is_set():
                    self.logger.push_highfreq(
                        rb_time=self.feedbackData.rb_time,
                        feedback_pos=self.feedbackData.gripper_pos,
                        feedback_vel=self.feedbackData.gripper_vel,
                        feedback_torque=self.feedbackData.gripper_torque,
                    )
                next_time += period
            except Exception as e:
                self.loggerUI.error(f"feedback error:{e}")
                # 缁欑郴缁熶竴鐐圭紦鍐叉椂闂达紝閬垮厤寮傚父鐙傚埛
                time.sleep(0.1)

    def _coerce_cfg_scalar(self, value):
        if isinstance(value, list):
            if not value:
                return None
            value = value[0]
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    def _sync_gripper_yaml_config_to_feedback(self):
        yaml_path = f"/opt/robot/rb_hardware/{self.selected_gripper}.yaml"
        length_val = None
        offset_val = None
        try:
            with open(yaml_path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
            length_val = self._coerce_cfg_scalar(data.get("length_per_radian"))
            offset_val = self._coerce_cfg_scalar(data.get("offset_at_hardware_zero"))
        except Exception as e:
            self.loggerUI.warn(f"read yaml params failed: {e}")

        with self.feedback_lock:
            self.feedbackData.config_length_per_radian = length_val
            self.feedbackData.config_offset_at_hardware_zero = offset_val

    def _load_config(self):
        opts = ["left_gripper","right_gripper"]
        opts_loadcell = ["left_loadcell","right_loadcell"]
        opts_six_axis = ["left_force_sensor","right_force_sensor"]
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
                win.addstr(0,2,"閫夋嫨澶圭埅")
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
        self.selected_loadcell = opts_loadcell[selected_idx]
        self.selected_force_sensor = opts_six_axis[selected_idx]
        # config_path = Path("/opt/robot/rb_config.yaml")
        # MyRobot = RobotConfig.recreate({
        #     "hardware": RobotConfig.create(config_path).hardware,
        #     "planner": RobotConfig.create(config_path).planner,
        #     "robot_model": RobotConfig.create(config_path).robot_model,
        # })
        gripper_path = str(f"/opt/robot/rb_hardware/{self.selected_gripper}.yaml")
        loadcell_path = str(f"/opt/robot/rb_hardware/{self.selected_loadcell}.yaml")
        force_sensor_path = str(f"/opt/robot/rb_hardware/{self.selected_force_sensor}.yaml")
        MyGripper = GripperConfig.create(gripper_path)
        MyLoadcell = LoadcellConfig.create(loadcell_path)
        MyForceSensor = ForceSensorConfig.create(force_sensor_path)
        MyHardware = HardwareConfig.create_from_container({self.selected_gripper: MyGripper,
                                                           self.selected_loadcell: MyLoadcell,
                                                           self.selected_force_sensor: MyForceSensor})
        MyRobot = RobotConfig.recreate({
            "hardware": MyHardware.to_dict_container(),
            "planner":None,
            "robot_model":""
        })
        self.config = MyRobot
        self.loggerUI.info(f"宸查€夋嫨澶圭埅: {self.selected_gripper}")
        self.loggerUI.info(f"宸插姞杞介厤缃枃浠? {gripper_path}, {loadcell_path}, {force_sensor_path}")
        self._sync_gripper_yaml_config_to_feedback()
        self._init_lasers()

    def _rebuild_config_from_selected(self):
        gripper_path = str(f"/opt/robot/rb_hardware/{self.selected_gripper}.yaml")
        loadcell_path = str(f"/opt/robot/rb_hardware/{self.selected_loadcell}.yaml")
        force_sensor_path = str(f"/opt/robot/rb_hardware/{self.selected_force_sensor}.yaml")
        my_gripper = GripperConfig.create(gripper_path)
        my_loadcell = LoadcellConfig.create(loadcell_path)
        my_force_sensor = ForceSensorConfig.create(force_sensor_path)
        my_hardware = HardwareConfig.create_from_container(
            {
                self.selected_gripper: my_gripper,
                self.selected_loadcell: my_loadcell,
                self.selected_force_sensor: my_force_sensor,
            }
        )
        self.config = RobotConfig.recreate(
            {
                "hardware": my_hardware.to_dict_container(),
                "planner": None,
                "robot_model": "",
            }
        )
        self.loggerUI.info(
            f"reloaded config files: {gripper_path}, {loadcell_path}, {force_sensor_path}"
        )
        self._sync_gripper_yaml_config_to_feedback()

    def reload_robot_from_yaml(self):
        self.loggerUI.info("reloading robot from current yaml")
        self.stop_feedback_thread()
        try:
            if self.robot is not None:
                self.robot.shutdown()
                time.sleep(0.2)
        except Exception as e:
            self.loggerUI.warn(f"old robot shutdown failed: {e}")

        self._rebuild_config_from_selected()
        self._init_robot()
        self.start_feedback_thread()
        self.loggerUI.info("robot reload done")

    def _scan_laser_ports(self):
        ports = list(serial.tools.list_ports.comports())
        candidates = []
        for p in ports:
            device = p.device
            if not device.startswith("/dev/ttyACM"):
                continue
            if p.vid != LASER_USB_VID or p.pid != LASER_USB_PID:
                continue
            if LASER_USB_SERIAL and p.serial_number and p.serial_number != LASER_USB_SERIAL:
                continue
            candidates.append(device)
        candidates.sort()
        return candidates[:2]

    def _disconnect_lasers(self):
        if self.laser_left:
            try:
                self.laser_left.disconnect()
            except Exception:
                pass
            self.laser_left = None
        if self.laser_right:
            try:
                self.laser_right.disconnect()
            except Exception:
                pass
            self.laser_right = None

    def _ensure_lasers(self):
        if self.laser_left and self.laser_right:
            return
        ports = self._scan_laser_ports()
        if len(ports) < 2:
            with self.feedback_lock:
                self.feedbackData.laser_status = "disconnected"
                self.feedbackData.laser_ports = {"left": None, "right": None}
            if self._laser_last_status != "disconnected":
                self.loggerUI.warn(f"laser disconnected, ports found: {ports}")
                self._laser_last_status = "disconnected"
            return
        # drop stale connections if device node vanished
        if self.laser_left and self.laser_left.port not in ports:
            self._disconnect_lasers()
        if self.laser_right and self.laser_right.port not in ports:
            self._disconnect_lasers()
        left_port, right_port = ports[0], ports[1]
        if not self.laser_left:
            try:
                self.laser_left = LaserIndicator("LaserLeft", left_port)
                self.laser_left.connect()
                if self.laser_left.serial is not None:
                    self.laser_left.serial.timeout = 0.05
            except Exception as e:
                self.laser_left = None
                self.loggerUI.error(f"laser left connect failed: {e}")
        if not self.laser_right:
            try:
                self.laser_right = LaserIndicator("LaserRight", right_port)
                self.laser_right.connect()
                if self.laser_right.serial is not None:
                    self.laser_right.serial.timeout = 0.05
            except Exception as e:
                self.laser_right = None
                self.loggerUI.error(f"laser right connect failed: {e}")
        with self.feedback_lock:
            self.feedbackData.laser_ports = {"left": left_port, "right": right_port}
            if self.laser_left or self.laser_right:
                self.feedbackData.laser_status = "connected"
                if self._laser_last_status != "connected":
                    self.loggerUI.info(f"laser connected: {left_port}, {right_port}")
                    self._laser_last_status = "connected"
            else:
                self.feedbackData.laser_status = "error"
                if self._laser_last_status != "error":
                    self.loggerUI.error("laser connect error")
                    self._laser_last_status = "error"

    def _init_lasers(self):
        """
        Initialize laser distance sensors using auto-scan ports.
        """
        self._ensure_lasers()

    def _read_lasers(self):
        left_val = None
        right_val = None
        status = "connected"
        try:
            if self.laser_left:
                left_val = self.laser_left.get_value()
            if self.laser_right:
                right_val = self.laser_right.get_value()
        except OSError:
            status = "error"
            self._disconnect_lasers()
        except Exception:
            status = "error"
        return left_val, right_val, status

    def _shutdown_robot(self):
        if self.laser_left:
            try:
                self.laser_left.disconnect()
            except Exception as e:
                self.loggerUI.error(f"laser left disconnect error: {e}")
        if self.laser_right:
            try:
                self.laser_right.disconnect()
            except Exception as e:
                self.loggerUI.error(f"laser right disconnect error: {e}")
        super()._shutdown_robot()

    def gripper_basic(self):
        key_path = self.selected_gripper + ".invert_directions"
        menu = {
            "1": {
                "description": "闃惰穬涓嬪彂绉诲姩澶圭埅",
                "callback": lambda: self.motion._run_point_with_input(q_name="gripper_pos",
                                                              dof=len(self.feedbackData.gripper_pos),
                                                              part=self.selected_gripper),
            },
            "2": {
                "description": "TOPP杞ㄨ抗鍙傛暟淇敼",
                "callback": lambda: self.motion.set_max_vel_acc(),
            },
            "3": {
                "description": "TOPP杞ㄨ抗涓嬪彂娴嬭瘯",
                "callback": lambda: self.motion.cubic_move(start_data=self.feedbackData.gripper_pos,
                                                           max_vel=[self.motion.max_vel],
                                                           max_acc=[self.motion.max_acc],
                                                           part=self.selected_gripper,
                                                           pos_name="gripper_pos",
                                                           dof=len(self.feedbackData.gripper_pos)),
            },
            "4": {
                "description": "鐐规寜鎿嶄綔妯″紡",
                "callback": lambda: self.motion.start_manual_control_1dof(data_name="gripper_pos",
                                                                     part=self.selected_gripper),
            },
            "5": {
                "description": "澶圭埅璁鹃浂",
                "callback": lambda: self.motion.set_zero(part=self.selected_gripper)
            },
            "6": {
                "description": "鏇存敼杩愬姩涓婁笅闄?,
                "callback": lambda: self.motion.set_limits(part=self.selected_gripper),
            },
            "7": {
                "description": "璁剧疆鐢垫満璺熻釜绐楀彛",
                "callback": lambda: self.motion.set_following_error_window(part=self.selected_gripper),
            },
            "8": {
                "description": "鎭㈠鍙姩鎬?,
                "callback": lambda: self.motion.recover_mobility(
                    part=self.selected_gripper,
                    pos_name="gripper_pos",
                ),
            },
        }
        # 浣跨敤鑿滃崟鏍?
        self.push_menu(menu,"澶圭埅鍩虹鍔熻兘")
    
    def gripper_motion_accuracy(self):
        menu = {
            "1": {
                "description": "TOPP杞ㄨ抗鍙傛暟淇敼",
                "callback": lambda: self.motion.set_max_vel_acc(),
            },
            "2": {
                "description": "TOPP杞ㄨ抗璺熻釜娴嬭瘯",
                "callback": lambda: self.motion.cubic_move_record(start_data=self.feedbackData.gripper_pos,
                                                           max_vel=[self.motion.max_vel],
                                                           max_acc=[self.motion.max_acc],
                                                           part=self.selected_gripper,
                                                           pos_name="gripper_pos",
                                                           dof=len(self.feedbackData.gripper_pos)),
            },
            "3":{
                "description": "闃惰穬涓嬪彂鎺у埗娴嬭瘯",
                "callback": lambda: self.motion.step_move_record(part=self.selected_gripper,
                                                          pos_name="gripper_pos"),
            },
            "4":{
                "description": "绌鸿浇杩炵画杩愬姩娴嬭瘯",
                "callback": lambda: self.motion.step_move_repetitively(part=self.selected_gripper,
                                                            pos_name="gripper_pos"),
            },
            "5":{
                "description": "鍫佃浆鎶撳彇绋冲畾娴嬭瘯",
                "callback": lambda: self.motion.grasp_test(part=self.selected_gripper,
                                                            pos_name="gripper_pos")
            },
            "6": {
                "description": "闆嗘垚鎶撳彇缁煎悎娴嬭瘯",
                "callback": lambda: self.motion.integrity_test(part=self.selected_gripper,
                                                            pos_name="gripper_pos")
            }
        }
        # 浣跨敤鑿滃崟鏍?
        self.push_menu(menu,"澶圭埅杩愭帶绮惧害娴嬭瘯")

    def gripper_function_test(self):
        menu = {
            "1": {
                "description": "閲嶅瀹氫綅绮惧害",
                "callback": lambda: self.motion.repeatability_position_accuracy_test(
                    part=self.selected_gripper,
                    pos_name="gripper_pos",
                ),
            },
            "2": {
                "description": "缁濆瀹氫綅绮惧害",
                "callback": lambda: self.motion.absolute_position_accuracy_test(
                    part=self.selected_gripper,
                    pos_name="gripper_pos",
                ),
            },
            "3": {
                "description": "涓€缁村姏绮惧害",
                "callback": lambda: self.motion.one_dim_force_accuracy_test(
                    part=self.selected_loadcell,
                ),
            },
        }
        self.push_menu(menu, "鍩虹鍔熻兘娴嬭瘯")

    def task_gripper_param_calibration(self):
        menu = {
            "1": {
                "description": "test: step to 0.05",
                "callback": lambda: self.motion.gripper_calibration_motion_test(
                    part=self.selected_gripper,
                    pos_name="gripper_pos",
                ),
            },
            "2": {
                "description": "run: auto calibration",
                "callback": lambda: self.motion.calibrate_gripper_kinematic_params_auto(
                    part=self.selected_gripper,
                    pos_name="gripper_pos",
                ),
            },
            "3": {
                "description": "set: following_error_window=10000000",
                "callback": lambda: self.motion.set_following_error_window_10000000(
                    part=self.selected_gripper,
                ),
            },
            "4": {
                "description": "mode: switch torque mode",
                "callback": lambda: self.motion.set_torque_mode(
                    part=self.selected_gripper,
                ),
            },
            "5": {
                "description": "set: point mode step",
                "callback": self.motion.set_manual_control_step,
            },
        }
        self.push_menu(menu, "gripper calibration")

    def task_loadcell(self):
        menu = {
            "1": {
                "description": "涓€缁村姏浼犳劅鍣ㄩ€氶亾0缃浂", 
                "callback": lambda: self.motion.set_zero_loadcell(part=self.selected_loadcell,ch=0),
            },
            "2": {
                "description": "涓€缁村姏浼犳劅鍣ㄩ€氶亾1缃浂", 
                "callback": lambda: self.motion.set_zero_loadcell(part=self.selected_loadcell,ch=1),
            },
            "3":{
                "description": "涓€缁村姏浼犳劅鍣ㄩ€氶亾0鏍囧畾",
                "callback": lambda: self.motion.calibrate_loadcell(part=self.selected_loadcell,ch=0),
            },
            "4":{
                "description": "涓€缁村姏浼犳劅鍣ㄩ€氶亾1鏍囧畾",
                "callback": lambda: self.motion.calibrate_loadcell(part=self.selected_loadcell,ch=1),
            },
        }
        self.push_menu(menu,"涓€缁村姏浼犳劅鍣ㄨ闆朵笌鏍囧畾")

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
                "path": f"{project_root}/logs/file.log",
                "encoder": {"pattern": "{h({d(%Y-%m-%d %H:%M:%S.%3f)} [{t}] {l} {m})}{n}"},
            },
        },
        "root": {"level": "debug", "appenders": ["file"]},
        "loggers": {},
    }
    hblog.start(config)
    hblog.info("test","Gripper Guide Start")
    guide = GripperGuide()
    guide.push_menu({
        "1":{"description":"澶圭埅鍙傛暟鑷姩鐭","callback":guide.task_gripper_param_calibration},
        "2":{"description":"涓€缁村姏浼犳劅鍣ㄨ闆朵笌鏍囧畾","callback":guide.task_loadcell},
        "3":{"description":"澶圭埅鍩虹鍔熻兘","callback":guide.gripper_basic},
        "4":{"description":"澶圭埅杩愭帶娴嬭瘯","callback":guide.gripper_motion_accuracy},
        "5":{"description":"鍩虹鍔熻兘娴嬭瘯","callback":guide.gripper_function_test},
    },"涓昏彍鍗?)
    guide.run()

