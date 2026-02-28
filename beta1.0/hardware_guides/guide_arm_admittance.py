from prelude import *

class AdmittanceGuide(BaseGuide):
    def __init__(self):
        super().__init__()
        self.logger = LogModule(self)
        self.motion = MotionModule(self)
        class FeedbackItem:
            def __init__(self):
                self.QActual = -1
                self.Wrench = -1
                self.rb_time = -1
                self.last_update_time = -1
        self.feedbackData = FeedbackItem()
        # 标定过程计算
        self.M = None
        self.F = None
        self.R = None
        self.f = None
        self.len_data = 0
        # pin calc
        self.urdf_file = "/home/hillbot/ziwei/RoboTest/beta1.0/urdf/beta1.0_v3_20251211_arm.urdf"
        self.model = pin.buildModelFromUrdf(self.urdf_file)
        self.data = self.model.createData()
        self.selected_arm = "left_arm"
        self.selected_force_sensor = "left_force_sensor"
    ## identifier func
    def _add_data(self, wrench: np.ndarray, ee_pose: np.ndarray):
        force = wrench[:3].copy()
        torque = wrench[3:].copy()
        if self.M is None:
            self.M = torque.reshape((3, 1))
            f_skew = self.__skew_symmetric(force)
            self.F = np.hstack((f_skew, np.eye(3)))
        else:
            self.M = np.vstack((self.M, torque.reshape((3, 1))))
            f_skew = self.__skew_symmetric(force)
            self.F = np.vstack((self.F, np.hstack((f_skew, np.eye(3)))))
        
        if self.R is None:
            self.R = np.hstack((ee_pose.T, np.eye(3)))
            self.f = force.reshape((3, 1))
        else:
            self.R = np.vstack((self.R, np.hstack((ee_pose.T, np.eye(3)))))
            self.f = np.vstack((self.f, force.reshape((3, 1))))
        self.len_data += 1
    
    def _solve(self):
        A, residuals, rank, s = np.linalg.lstsq(self.F, self.M, rcond=None)
        B, residuals, rank, s = np.linalg.lstsq(self.R, self.f, rcond=None)
        
        x, y, z, k1, k2, k3 = A.flatten()
        Gx, Gy, Gz, Fx0, Fy0, Fz0 = B.flatten()
        
        mass_center = np.array([x, y, z])
        F_0 = np.array([Fx0, Fy0, Fz0])
        M_x0 = k1 - Fy0 * z + Fz0 * y
        M_y0 = k2 - Fz0 * x + Fx0 * z
        M_z0 = k3 - Fx0 * y + Fy0 * x
        M_0 = np.array([M_x0, M_y0, M_z0])
        gravity_base = np.array([Gx, Gy, Gz])
        return mass_center, F_0, M_0, gravity_base

    def _read_data_once(self,num):
        sum = np.zeros(6)
        i = 0
        # 读取num次数据，取平均值
        while i < num:
            data = np.array(self.feedbackData.Wrench,dtype=np.float32)
            if data is not None:
                sum += data
                i += 1 
            time.sleep(0.2)
        return  sum/i
    
    def __skew_symmetric(self, v: np.ndarray):
        """ 从3维向量生成对应的3x3反对称矩阵 """
        if len(v) != 3:
            raise ValueError("输入数组的长度必须为3!")
        return np.array([[0, v[2], -v[1]],
                        [-v[2], 0, v[0]],
                        [v[1], -v[0], 0]])
    
    def _set_joint(self,target_joints):
        """
        target_joints: list or np.array 7DOF
        """
        if isinstance(target_joints,np.ndarray):
            target_joints = target_joints.tolist()
        print(f"目标关节角度:{target_joints}")
        while np.max(np.abs(self.feedbackData.QActual-np.array(target_joints))) > 1e-4:
            self.robot.set_actions({"left_arm": {"type": "position", "position": target_joints}})
            time.sleep(1 / CONTROL_HZ)
        print("到达!")
        print(f"move done")
    
    def _getForwardKin(self,joints_list,end=0,start=8):
        """getForwardKin

        Args:
            joints_list (list): [q1,...,q7]
            end (int, optional): Defaults to 0.
            start (int, optional): Defaults to 9.

        Returns:
            numpy darray: 4x4 homogeneous matrix
        """
        joints_array = np.array(joints_list)
        pin.forwardKinematics(self.model,self.data,joints_array)
        pin.updateFramePlacements(self.model,self.data)
        if self.selected_arm == "left_arm":
            frame_id_9 = self.model.getFrameId("left-link_tcp")
            frame_id_8 = self.model.getFrameId("left-link_ee_ft_sensor")
        if self.selected_arm == "right_arm":
            frame_id_9 = self.model.getFrameId("right-link_tcp")
            frame_id_8 = self.model.getFrameId("right-link_ee_ft_sensor")
        if end==0 and start==9:
            T9o = self.data.oMf[frame_id_9].homogeneous
            T_res = T9o
        if end==0 and start==8:
            T8o = self.data.oMf[frame_id_8].homogeneous
            T_res = T8o
        if end==8 and start==9:
            M9o = self.data.oMf[frame_id_9]
            M8o = self.data.oMf[frame_id_8]
            M98 = M8o.inverse() * M9o
            T98 = M98.homogeneous
            T_res = T98
        return T_res

    def _identify_param_auto(self,sample_num = 6):
        q_list = [[-1.100000, 0.417000, 0.457000, 0.622000, 0.664000, 0.895000, -0.204000],
                    [-1.100000, 0.417000, 0.457000, 0.622000, 0.664000, 0.895000, 0.233000],
                    [-1.100000, 0.417000, 0.457000, 0.622000, 0.664000, 0.895000, 0.670000],
                    [-1.100000, 0.417000, 0.457000, 0.622000, 0.664000, 0.895000, -0.125000],
                    [-1.100000, 0.417000, 0.457000, 0.622000, 0.664000, 0.895000, -0.920000],
                    [-1.100000, 0.417000, 0.457000, 0.622000, 0.664000, 1.182500, -0.562000],
                    [-1.100000, 0.417000, 0.457000, 0.622000, 0.664000, 1.470000, -0.204000],
                    [-1.100000, 0.417000, 0.457000, 0.622000, 0.664000, 0.735000, -0.204000],
                    [-1.100000, 0.417000, 0.457000, 0.622000, 0.664000, 0.000000, -0.204000],
                    [-1.100000, 0.417000, 0.457000, 0.622000, 0.664000, -0.368000, -0.204000],
                    [-1.100000, 0.417000, 0.457000, 0.622000, 0.664000, -0.736000, -0.204000],
                    [-1.100000, 0.417000, 0.457000, 0.622000, 0.172500, 0.226500, 0.162000],
                    [-1.100000, 0.417000, 0.457000, 0.622000, -0.319000, 1.189000, 0.528000],
                    [-1.100000, 0.417000, 0.457000, 0.622000, 0.172500, 1.189000, 0.528000],
                    [-1.100000, 0.417000, 0.457000, 0.622000, 0.664000, 1.189000, 0.528000],
                    [-1.100000, 0.417000, 0.457000, 0.622000, 1.120000, 0.226500, 0.162000],
                    [-1.100000, 0.417000, 0.457000, 0.622000, 1.576000, -0.736000, -0.204000],
                    [-1.100000, 0.417000, 0.457000, 0.622000, 0.523000, -0.736000, -0.204000],
                    [-1.100000, 0.417000, 0.457000, 0.622000, -0.530000, -0.736000, -0.204000],
                    [-1.100000, 0.417000, 0.457000, 0.622000, 0.839500, 0.079500, -0.750000],
                    [-1.100000, 0.417000, 0.457000, 0.622000, 2.209000, 0.895000, -1.296000],
                    [-1.100000, 0.417000, 0.457000, 0.622000, -0.421500, 0.895000, -1.296000],
                    [-1.100000, 0.417000, 0.457000, 0.622000, -3.052000, 0.895000, -1.296000],
                    [-1.100000, 0.417000, 0.457000, 0.622000, -3.052000, 0.895000, -0.956000],
                    [-1.100000, 0.417000, 0.457000, 0.622000, -3.052000, 0.895000, -0.616000],
                    [-1.100000, 0.417000, 0.457000, 0.622000, -3.052000, 0.895000, 0.062000],
                    [-1.100000, 0.417000, 0.457000, 0.622000, -3.052000, 0.895000, 0.740000],
                    [-1.100000, 0.417000, 0.457000, 0.622000, -3.052000, -0.116000, 0.740000],
                    [-1.100000, 0.417000, 0.457000, 0.622000, -3.052000, -1.127000, 0.740000],
                    ]
        for i,q in enumerate(q_list):
            print("move to q:",q)
            q_calc = q + [0.0,0.0]
            self._set_joint(q)
            time.sleep(2)
            wrench = self._read_data_once(sample_num)
            print(f"wrench: {wrench}")
            print("read done")
            cur_pose = self._getForwardKin(q_calc,end=0,start=8) # tform
            b_R_e = cur_pose[:3,:3]
            self._add_data(wrench, b_R_e)
            print(f"=======添加了第{i+1}组数据=======")
            i += 1
            time.sleep(0.5)
        mass_center, F_0, M_0, gravity_base = self._solve()
        # output all results
        print(f"mass_center_position: {mass_center}")
        print(f"force_zero: {F_0}")
        print(f"torque_zero: {M_0}")
        print(f"gravity_base: {gravity_base}")
        ## 记录结果
        self.mass_center_position = mass_center.tolist()
        self.force_zero = F_0.tolist()
        self.torque_zero = M_0.tolist()
        self.gravity_base = gravity_base.tolist()
        input("ENTER TO GO BACK")
    ##

    def _get_feedback(self):
        while not self._stop_event.is_set():
            try:
                feed = self.robot.get_states()

                # 无数据 → 继续下一次
                if not feed:
                    time.sleep(0.02)
                    continue

                arm_states = feed.get(self.selected_arm, None)
                if not arm_states:
                    time.sleep(0.02)
                    continue

                q_actual = arm_states.get("position")
                rb_t = arm_states.get("time")

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

                if isinstance(q_actual, (list, tuple)):
                    self.feedbackData.QActual = np.array(q_actual)
                    self.feedbackData.last_update_time = time.time()
                    secs = rb_t.get("secs")
                    nanos = rb_t.get("nanos") * 1e-9
                    self.feedbackData.rb_time = secs + nanos

                if self.record_flag.is_set():
                    self.logger.push_highfreq(rb_time=self.feedbackData.rb_time,
                                              feedback_pos=self.feedbackData.QActual)

            except Exception as e:
                print(f"[反馈线程异常] {e}")
                # 给系统一点缓冲时间，避免异常狂刷
                time.sleep(0.1)
            time.sleep(0.004)
    def _load_config(self):
        config_path = Path("/opt/robot/rb_config_xjc.yaml")
        MyRobot = RobotConfig.recreate({
            "hardware": RobotConfig.create(config_path).hardware,
            "planner": RobotConfig.create(config_path).planner,
            "robot_model": RobotConfig.create(config_path).robot_model
        })
        self.config = MyRobot
        print(f"已加载配置文件: {config_path}")
    ## menu
    def identify(self):
        # os.system('clear')
        menu = {
            "1": {
                "description": "运行机械臂到参数辨识任务起始位姿", 
                "callback": lambda: self.motion._run_point(q_name="QActual",
                                                           q_pos=[-1.100000, 0.417000, 0.457000, 0.622000, 0.664000, 0.895000, -0.204000],
                                                           part="left_arm")
            },
            "2": {
                "description": "运行自动标定程序",
                "callback": self._identify_param_auto,
            },
        }
        # 使用菜单栈
        self.push_menu(menu,"六维力传感器参数辨识")
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
    guide = AdmittanceGuide()
    guide.push_menu({
        "1":{"description":"identifier","callback":guide.identify},
    },"主菜单")
    guide.run()