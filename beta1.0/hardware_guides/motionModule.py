import os 
import time
import numpy as np
import sys
cur_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(cur_dir)
folder_utils = os.path.join(project_root,'utils')
sys.path.insert(0,folder_utils)
from visualize import draw, draw_step_response_analysis
from topp import TOPP
import csv
from collections import deque
import threading
import matplotlib.pyplot as plt
import yaml
MAX_POINTS = 1000
CONTROL_HZ = 20
TOL = 1e-3
EXTRA_TIME = 0.2 # s
# decorator
def hide_ui_while(func):
    """
    装饰器：执行函数时隐藏UI，执行完再恢复
    假设函数所在对象 self 有 self.g.ui 属性
    """
    def wrapper(self, *args, **kwargs):
        # 隐藏 UI
        if hasattr(self.g, 'ui'):
            self.g.ui.simulate_key('h')
        try:
            result = func(self, *args, **kwargs)
        finally:
            # 恢复 UI
            if hasattr(self.g, 'ui'):
                self.g.ui.simulate_key('\n')
        return result
    return wrapper

class MotionModule:
    def __init__(self,guide):
        self.g = guide
        self.manual_control_active = False  # 标记是否进入手动模式
        self.max_acc = 0.1
        self.max_vel = 0.1

    def _coerce_scalar(self, value):
        if value is None:
            return None
        if isinstance(value, (list, tuple)):
            if not value:
                return None
            value = value[0]
        if isinstance(value, np.ndarray):
            if value.size == 0:
                return None
            value = value.reshape(-1)[0]
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    def _move_to_target(self, part: str, pos_name: str, target: float, timeout_s: float = 3.0):
        target_arr = np.array([target], dtype=float)
        start_time = time.monotonic()
        while True:
            with self.g.feedback_lock:
                cur_q = getattr(self.g.feedbackData, pos_name)
            if np.max(np.abs(np.array(cur_q, dtype=float) - target_arr)) <= TOL:
                return True
            if time.monotonic() - start_time > timeout_s:
                return False
            self.g.robot.set_actions({part: {"type": "position", "position": target_arr.tolist()}})
            time.sleep(1 / CONTROL_HZ)

    def _get_feedback_scalar(self, name: str):
        with self.g.feedback_lock:
            raw = getattr(self.g.feedbackData, name, None)
        return self._coerce_scalar(raw)

    def _hold_position_command(self, part: str, target: float, hold_s: float):
        end_t = time.monotonic() + hold_s
        cmd = [float(target)]
        while time.monotonic() < end_t:
            self.g.robot.set_actions({part: {"type": "position", "position": cmd}})
            time.sleep(1 / CONTROL_HZ)

    def _write_gripper_yaml_params(self, part: str, length_per_radian: float = None, offset_at_hardware_zero: float = None):
        yaml_path = f"/opt/robot/rb_hardware/{part}.yaml"
        with open(yaml_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}

        if length_per_radian is not None:
            old = data.get("length_per_radian")
            data["length_per_radian"] = [float(length_per_radian)] if isinstance(old, list) else float(length_per_radian)
        if offset_at_hardware_zero is not None:
            old = data.get("offset_at_hardware_zero")
            data["offset_at_hardware_zero"] = [float(offset_at_hardware_zero)] if isinstance(old, list) else float(offset_at_hardware_zero)

        with open(yaml_path, "w", encoding="utf-8") as f:
            yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)

        return yaml_path

    def set_torque_mode(self,part):
        confirm = input("请在切换力矩模式前托住机械臂，否则机械臂将自由下落 (y/n): ").strip().lower()
        if confirm == "y":
            print("已切换到力矩模式")
            self.g.robot.send_command(part, {"command": "set_control_mode", "mode": "torque"})
        else:
            print("Cancelled.")
            input("回车以返回")
            return
        confirm = input("输入y以离开力矩模式返回到位置模式 (y)")
        if confirm == "y":
            print("已切换到位置模式")
            self.g.robot.send_command(part, {"command":"set_control_mode", "mode": "position"})
            input("回车以返回")
            return
    def _run_point(self,q_name,q_pos,part,timeout_s=3.0):
        '''
        _run_point 的 Docstring
        
        :param q_name: self.g.feedbackData.QActual for arm
        :param q_pos: target pos
        :param part: self.g.selected_arm for arm
        :param timeout_s: timeout seconds 
        '''
        print("Moving...\n")
        start_time = time.monotonic()
        while True:
            cur_q = getattr(self.g.feedbackData, q_name)
            if np.max(np.abs(cur_q - np.array(q_pos))) <= TOL:
                print("到达！")
                self.g.loggerUI.info("到达！")
                break
            if time.monotonic() - start_time > timeout_s:
                print(f"走点在{timeout_s}后超时退出")
                self.g.loggerUI.warn(f"走点在{timeout_s}后超时退出")
                break
            self.g.robot.set_actions({part: {"type": "position", "position": q_pos}})
            time.sleep(1 / CONTROL_HZ)
        input("回车以结束")

    @hide_ui_while
    def _run_point_with_input(self,q_name,dof,part):
        '''
        _run_point_with_input 的 Docstring
        
        :param cur_q: self.g.feedbackData.QActual for arm
        :param dof: target pos 
        :param part: self.g.selected_arm for arm 
        '''
        try:
            raw_input = input(f"以逗号分隔的方式输入N自由度的期望移动位置 (e.g. 0.1 or 0.1,0.2,...,N 或 'q' 退出): ").strip()
            if raw_input.lower() == 'q':
                print("结束")
                return
            q_pos = [float(x.strip()) for x in raw_input.split(',')]

            if len(q_pos) != dof:
                print(f"\nError: Expected {dof} values, but got {len(q_pos)}.")
                return
            raw_input = input(f"设置超时时间(秒),默认3秒，回车继续: ")
            timeout_s = float(raw_input) if raw_input else 3.0
            print(f"\n移动到新位置: {[f'{p:.3f}' for p in q_pos]}")
            self._run_point(q_name=q_name,q_pos=q_pos,part=part,timeout_s=timeout_s)
        
        except Exception as e:
            print(f"\nError:{e}")
            input("回车以返回上一级目录")

    def _run_point_without_hide(self,q_name,dof,part):
        '''
        _run_point_without_hide 的 Docstring
        
        :param cur_q: self.g.feedbackData.QActual for arm
        :param dof: target pos 
        :param part: self.g.selected_arm for arm 
        '''
        try:
            while True:
                raw_input = input(f"以逗号分隔的方式输入{dof}自由度的期望移动位置 (e.g. 0.1 or 0.1,0.2,...,N): ")
                try:
                    q_pos = [float(x.strip()) for x in raw_input.split(',')]
                except ValueError:
                    print("\nError: 输入中包含无法转换为数字的值，请重新输入。")
                    continue
                if len(q_pos) != dof:
                    print(f"\nError: 期望 {dof} 个值，但输入了 {len(q_pos)} 个，请重新输入。")
                    continue
                # 如果长度正确，退出循环
                break
            raw_input = input(f"设置超时时间(秒),默认3秒，回车继续: ")
            timeout_s = float(raw_input) if raw_input else 3.0
            print(f"\n移动到新位置: {[f'{p:.3f}' for p in q_pos]}")
            self._run_point(q_name=q_name,q_pos=q_pos,part=part,timeout_s=timeout_s)
        
        except Exception as e:
            print(f"\nError:{e}")
            input("回车以返回上一级目录")

    @hide_ui_while
    def set_max_vel_acc(self):
        os.system('clear')
        # Helper 函数：安全读取正浮点数
        def read_positive_float(prompt):
            while True:
                raw_input = input(prompt)
                try:
                    value = float(raw_input)
                    if value <= 0:
                        print("⚠️ 必须输入正数，请重试。")
                        continue
                    return value
                except ValueError:
                    print("⚠️ 输入无效，请输入数字。")
        # 读取最大速度
        self.max_vel = read_positive_float("请输入最大速度设置 (rad/s): ")
        # 读取最大加速度
        self.max_acc = read_positive_float("请输入最大加速度设置 (rad/s²): ")
        input(f"✅ 设置完成：最大速度 = {self.max_vel} rad/s, 最大加速度 = {self.max_acc} rad/s²\n回车以继续...")

    @hide_ui_while
    def cubic_move(self,start_data,max_vel:list,max_acc:list,part:str,pos_name:str,dof:int):
        '''
        cubic_move_record 的 Docstring

        :param start_data: self.g.feedbackData.QActual
        :param max_vel: [self.max_vel]*7 for arm
        :type max_vel: list
        :param max_acc: [self.max_acc]*7 for arm
        :type max_acc: list
        :part: self.g.selected_arm
        :type part: str
        :pos_name: getattr(self.g.feedbackData, "rb_time")
        :type pos_name: str
        '''
        start_point = np.array(start_data,dtype=float)
        
        print("当前点位:")
        print(" ".join([f"{v:.6f}" for v in start_point]))
        
        raw_input = input(f"以逗号分隔的方式输入N自由度的期望移动位置 (e.g. 0.1 or 0.1,0.2,...,N 或 'q' 退出): ")
        if raw_input.lower() == 'q':
            print("结束")
            return
        end_point = [float(x.strip()) for x in raw_input.split(',')]

        if len(end_point) != dof:
            self.g.loggerUI.error(f"Expected {dof} values, but got {len(end_point)}.")
            return

        print("\n最终目标点位:")
        print(end_point)
        input("确认目标点位，回车以开始走点")
        # 构造路径
        path = np.vstack([start_point,end_point])

        # 轨迹规划
        sample_rate = 20
        step = 1/sample_rate
        try:
            ts, qs, qds, qdds, duration = TOPP(
                path, 
                max_vel, 
                max_acc, 
                step
            )
        except RuntimeError as e:
            print("轨迹规划失败:", e)
            return

        # 执行轨迹
        print("移动中...")
        
        start_time = time.time()
        for t, q in zip(ts, qs):
            self.g.robot.set_actions({part: {"type": "position", "position": q.tolist()}})
            while time.time() - start_time < t:
                time.sleep(0.05)
        print("移动结束")

    @hide_ui_while
    def cubic_move_record(self,start_data,max_vel:list,max_acc:list,part:str,pos_name:str,dof:int):
        '''
        cubic_move_record 的 Docstring

        :param start_data: self.g.feedbackData.QActual
        :param max_vel: [self.max_vel]*7 for arm
        :type max_vel: list
        :param max_acc: [self.max_acc]*7 for arm
        :type max_acc: list
        :part: self.g.selected_arm
        :type part: str
        :pos_name: getattr(self.g.feedbackData, "rb_time")
        :type pos_name: str
        '''
        start_point = np.array(start_data,dtype=float)
        # 执行轨迹前清空日志
        self.g.highfreq_log.clear()
        self.g.lowfreq_log.clear()
        # 减少反馈负担
        self.g.feedback_mode = self.g.feedback_mode.BASIC
        print("当前点位:")
        print(" ".join([f"{v:.6f}" for v in start_point]))
        
        raw_input = input(f"以逗号分隔的方式输入N自由度的期望移动位置 (e.g. 0.1 or 0.1,0.2,...,N): ")
        end_point = [float(x.strip()) for x in raw_input.split(',')]
        if len(end_point) != dof:
            self.g.loggerUI.error(f"Expected {dof} values, but got {len(end_point)}.")
            return

        print("\n最终目标点位:")
        print(end_point)
        input("确认目标点位，回车以开始走点")
        # 构造路径
        path = np.vstack([start_point,end_point])

        # 轨迹规划
        sample_rate = 20
        step = 1/sample_rate
        try:
            ts, qs, qds, qdds, duration = TOPP(
                path, 
                max_vel, 
                max_acc, 
                step
            )
        except RuntimeError as e:
            print("轨迹规划失败:", e)
            return

        # 执行轨迹
        print("移动中...")
        self.g.record_flag.set()
        print("开始高频+低频记录...")
        start_time = time.time()
        for t, q in zip(ts, qs):
            # 使用锁保护读取操作
            with self.g.feedback_lock:
                cur_q = getattr(self.g.feedbackData, pos_name)
                rb_time = getattr(self.g.feedbackData, "rb_time")
            self.g.robot.set_actions({part: {"type": "position", "position": q.tolist()}})
            self.g.logger._record_lowfreq(t=time.perf_counter(),q=q,
                                          rb_time=rb_time,
                                          feedback_pos=cur_q)
            while time.time() - start_time < t:
                time.sleep(0.05)
        # 延迟结束
        extra_record = time.perf_counter()
        while time.perf_counter() - extra_record <= EXTRA_TIME:
            with self.g.feedback_lock:
                cur_q = getattr(self.g.feedbackData, pos_name)
                rb_time = getattr(self.g.feedbackData, "rb_time")
            self.g.logger._record_lowfreq(t=time.time(),q=q,
                                    rb_time=rb_time,
                                    feedback_pos=cur_q)
            time.sleep(1 / CONTROL_HZ)
        self.g.record_flag.clear()
        print("高频记录结束，共收集高频数据", len(self.g.highfreq_log))
        print("低频记录结束，共收集低频数据", len(self.g.lowfreq_log))
        # 减少反馈负担
        self.g.feedback_mode = self.g.feedback_mode.FULL
        print("\n=== 数据记录 ===")
        confirm = input("是否保存日志? (y/n): ").strip().lower()
        if confirm != "y":
            print("日志保存更新取消")
            input("回车以返回")
            return

        # 保存日志
        highfreq_filename, lowfreq_filename,tstamp = self.g.logger._save_logs(part)

        confirm = input("是否可视化记录数据? (y/n): ").strip().lower()
        if confirm != "y":
            print("数据可视化取消")
            input("回车以返回")
            return

        draw(log_dir=f'{project_root}/logs',lowfile=lowfreq_filename,highfile=highfreq_filename,savefig=f"viz_{tstamp}.png")

    @hide_ui_while
    def step_move_record(self,part:str,pos_name:str):
        '''
        step_move_record 的 Docstring

        :part: self.g.selected_arm
        :type part: str
        :pos_name: getattr(self.g.feedbackData, "rb_time")
        :type pos_name: str
        '''
        # 执行轨迹前清空日志
        self.g.highfreq_log.clear()
        self.g.lowfreq_log.clear()
        # 减少反馈负担
        self.g.feedback_mode = self.g.feedback_mode.BASIC
        user_input = input("请输入目标位置 (逗号分隔) 或 'q' 退出: ").strip()
        if user_input.lower() == 'q':
            print("结束单步移动记录")
            return
        raw_input = input(f"设置超时时间(秒),默认3秒，回车继续: ")
        timeout_s = float(raw_input) if raw_input else 3.0
        try:
            q_pos = [float(x.strip()) for x in user_input.split(',')]
            print(f"移动到新位置: {[f'{p:.3f}' for p in q_pos]}")
            
            # 执行移动并记录
            self.g.record_flag.set()
            # 占位
            print("开始高频+低频记录...")
            print("Moving...\n")
            start_time = time.monotonic()
            while True:
                # 使用锁保护读取操作
                with self.g.feedback_lock:
                    cur_q = getattr(self.g.feedbackData, pos_name)
                    rb_time = getattr(self.g.feedbackData, "rb_time")
                if np.max(np.abs(cur_q - np.array(q_pos))) <= TOL:
                    print("到达！")
                    self.g.loggerUI.info("到达！")
                    break
                elif time.monotonic() - start_time > timeout_s:
                    print(f"走点在{timeout_s}后超时退出")
                    self.g.loggerUI.warn(f"走点在{timeout_s}后超时退出")
                    break
                self.g.robot.set_actions({part: {"type": "position", "position": q_pos}})
                self.g.logger._record_lowfreq(t=time.perf_counter(),q=q_pos,
                                        rb_time=rb_time,
                                        feedback_pos=cur_q)
                time.sleep(1 / CONTROL_HZ)
            # 延迟结束
            extra_record = time.perf_counter()
            while time.perf_counter() - extra_record <= EXTRA_TIME:
                with self.g.feedback_lock:
                    cur_q = getattr(self.g.feedbackData, pos_name)
                    rb_time = getattr(self.g.feedbackData, "rb_time")
                self.g.logger._record_lowfreq(t=time.time(),q=q_pos,
                                        rb_time=rb_time,
                                        feedback_pos=cur_q)
                time.sleep(1 / CONTROL_HZ)
            self.g.record_flag.clear()
            print("高频记录结束，共收集高频数据", len(self.g.highfreq_log))
            print("低频记录结束，共收集低频数据", len(self.g.lowfreq_log))
            # 减少反馈负担
            self.g.feedback_mode = self.g.feedback_mode.FULL
            print("\n=== 数据记录 ===")
            confirm = input("是否保存日志? (y/n): ").strip().lower()
            if confirm != "y":
                print("日志保存更新取消")
                input("回车以返回")
                return

            # 保存日志
            highfreq_filename, lowfreq_filename,tstamp = self.g.logger._save_logs(part)

            confirm = input("是否可视化记录数据? (y/n): ").strip().lower()
            if confirm != "y":
                print("数据可视化取消")
                input("回车以返回")
                return

            # draw(log_dir=f'{project_root}/logs',lowfile=lowfreq_filename,highfile=highfreq_filename,savefig=f"viz_{tstamp}.png")
            draw_step_response_analysis(log_dir=f'{project_root}/logs',
                 lowfile=lowfreq_filename,
                 highfile=highfreq_filename,
                 savefig=f"viz_{tstamp}.png",
                 target_pos=q_pos,
                 threshold=0.02,
                 )

        except Exception as e:
            print(e)

    @hide_ui_while
    def set_limits(self,part):
        min_pos = input("输入最小软限位:")
        max_pos = input("输入最大软限位:")
        min_list = [float(min_pos)]
        max_list = [float(max_pos)]
        self.g.robot.send_command(part, {"command": "set_limit",
                                        "enabled": [True],
                                        "lower":min_list,
                                        "upper":max_list})
        self.g.loggerUI.info("已设置软限位为: "+str(min_pos)+" ~ "+str(max_pos))

    @hide_ui_while
    def set_following_error_window(self,part):
        window_size = input("输入电机跟踪误差窗口大小/脉冲整数:")
        window_list = [int(window_size)]
        self.g.robot.send_command(part, {"command": "set_following_error_window",
                                        "value":window_list})
        self.g.loggerUI.info("已设置电机跟踪误差窗口为: "+str(window_size)+" pulses")

    @hide_ui_while
    def repeatability_position_accuracy_test(self, part: str, pos_name: str):
        repeats = 10
        targets = [0.0, 0.05]
        settle_s = 1.0
        timeout_s = 3.0

        print("开始重复定位精度测试")
        print(f"流程: {targets} 循环 {repeats} 次, 每次到位后停留 {settle_s}s 采样")
        rows = []

        for idx in range(repeats):
            print(f"\n第 {idx + 1}/{repeats} 轮")
            aborted = False
            for target in targets:
                ok = self._move_to_target(part=part, pos_name=pos_name, target=target, timeout_s=timeout_s)
                if not ok:
                    self.g.loggerUI.warn(f"移动到 {target} 超时, 本次测试提前结束")
                    print(f"移动到 {target} 超时, 本次测试提前结束")
                    aborted = True
                    break

                time.sleep(settle_s)
                with self.g.feedback_lock:
                    left_raw = getattr(self.g.feedbackData, "laser_left", None)
                    right_raw = getattr(self.g.feedbackData, "laser_right", None)
                    real_raw = getattr(self.g.feedbackData, "real_distance", None)

                left_val = self._coerce_scalar(left_raw)
                right_val = self._coerce_scalar(right_raw)
                real_val = self._coerce_scalar(real_raw)
                rows.append([target, left_val, right_val, real_val])
                print(f"目标 {target:.3f} -> A:{left_val} B:{right_val} real:{real_val}")
            if aborted:
                break

        if not rows:
            print("没有采集到有效数据")
            input("回车返回")
            return

        out_dir = os.path.join(project_root, "logs")
        os.makedirs(out_dir, exist_ok=True)
        tstamp = time.strftime("%Y%m%d_%H%M%S")
        out_file = os.path.join(out_dir, f"repeatability_accuracy_{part}_{tstamp}.csv")

        with open(out_file, "w", newline="", encoding="utf-8-sig") as f:
            writer = csv.writer(f)
            writer.writerow([
                "期望夹爪距离",
                "激光位移传感器A读数(mm)",
                "激光位移传感器B读数(mm)",
                "激光位移传感器行程(mm)",
            ])
            writer.writerows(rows)

        self.g.loggerUI.info(f"重复定位精度测试完成, CSV: {out_file}")
        print(f"\n测试完成, 已保存 CSV: {out_file}")
        input("回车返回")

    @hide_ui_while
    def absolute_position_accuracy_test(self, part: str, pos_name: str):
        targets_m = [0.01, 0.02, 0.03, 0.04, 0.05]
        settle_s = 1.0
        timeout_s = 3.0

        print("开始绝对定位精度测试")
        print(f"目标位置: {targets_m}, 每个点到位后停留 {settle_s}s 采样")
        rows = []

        for target_m in targets_m:
            ok = self._move_to_target(part=part, pos_name=pos_name, target=target_m, timeout_s=timeout_s)
            if not ok:
                self.g.loggerUI.warn(f"移动到 {target_m} 超时, 本次测试提前结束")
                print(f"移动到 {target_m} 超时, 本次测试提前结束")
                break

            time.sleep(settle_s)
            with self.g.feedback_lock:
                left_raw = getattr(self.g.feedbackData, "laser_left", None)
                right_raw = getattr(self.g.feedbackData, "laser_right", None)
                real_raw = getattr(self.g.feedbackData, "real_distance", None)

            left_val = self._coerce_scalar(left_raw)
            right_val = self._coerce_scalar(right_raw)
            real_val = self._coerce_scalar(real_raw)
            expected_stroke_mm = target_m * 2000.0  # 期望行程=2*夹爪移动位置(m), 再换算为mm

            rows.append([target_m, expected_stroke_mm, left_val, right_val, real_val])
            print(
                f"目标 {target_m:.3f}m (期望行程 {expected_stroke_mm:.3f}mm) "
                f"-> A:{left_val} B:{right_val} real:{real_val}"
            )

        if not rows:
            print("没有采集到有效数据")
            input("回车返回")
            return

        out_dir = os.path.join(project_root, "logs")
        os.makedirs(out_dir, exist_ok=True)
        tstamp = time.strftime("%Y%m%d_%H%M%S")
        out_file = os.path.join(out_dir, f"absolute_accuracy_{part}_{tstamp}.csv")

        with open(out_file, "w", newline="", encoding="utf-8-sig") as f:
            writer = csv.writer(f)
            writer.writerow([
                "期望夹移动位置(m)",
                "期望行程(mm)",
                "激光位移传感器A读数(mm)",
                "激光位移传感器B读数(mm)",
                "激光位移传感器行程(mm)",
            ])
            writer.writerows(rows)

        self.g.loggerUI.info(f"绝对定位精度测试完成, CSV: {out_file}")
        print(f"\n测试完成, 已保存 CSV: {out_file}")
        input("回车返回")

    @hide_ui_while
    def one_dim_force_accuracy_test(self, part: str):
        print("一维力精度功能待实现")
        input("回车返回")

    @hide_ui_while
    def calibrate_gripper_kinematic_params_auto(self, part: str, pos_name: str):
        hold_extreme_s = 3.0
        settle_s = 1.0
        timeout_s = 3.0
        open_cmd = 1.0
        close_cmd = -0.05

        print("开始夹爪参数自动矫正（自动下发版）")
        print("步骤1: 先将yaml中的 length_per_radian=1.0, offset_at_hardware_zero=0.0")
        try:
            yaml_path = self._write_gripper_yaml_params(
                part=part,
                length_per_radian=1.0,
                offset_at_hardware_zero=0.0,
            )
            self.g.loggerUI.info(f"已重置参数: {yaml_path}")
            print(f"已重置参数: {yaml_path}")
        except Exception as e:
            self.g.loggerUI.error(f"重置yaml失败: {e}")
            print(f"重置yaml失败: {e}")
            input("回车返回")
            return

        try:
            self.g.robot.send_command(part, {"command": "set_control_mode", "mode": "position"})
        except Exception:
            pass

        user_hold = input("输入极限持续下发时间(秒, 默认3): ").strip()
        if user_hold:
            hold_extreme_s = float(user_hold)
        user_open = input("输入张开大目标(默认1.0): ").strip()
        if user_open:
            open_cmd = float(user_open)
        user_close = input("输入闭合小目标(默认-0.05): ").strip()
        if user_close:
            close_cmd = float(user_close)

        print(f"自动张开: 持续下发目标 {open_cmd:.4f}, 持续 {hold_extreme_s}s")
        self._hold_position_command(part=part, target=open_cmd, hold_s=hold_extreme_s)
        time.sleep(settle_s)
        real_distance = self._get_feedback_scalar("real_distance")
        rad1 = self._get_feedback_scalar(pos_name)

        print(f"自动闭合: 持续下发目标 {close_cmd:.4f}, 持续 {hold_extreme_s}s")
        self._hold_position_command(part=part, target=close_cmd, hold_s=hold_extreme_s)
        time.sleep(settle_s)
        rad2 = self._get_feedback_scalar(pos_name)

        if real_distance is None or rad1 is None or rad2 is None:
            print("采样失败：real_distance/rad1/rad2 存在空值，终止")
            input("回车返回")
            return

        delta_rad = abs(rad1 - rad2)
        if delta_rad < 1e-9:
            print("计算失败：|rad1-rad2| 过小")
            input("回车返回")
            return

        length_per_radian = real_distance / (2000.0 * delta_rad)
        length_per_radian_10 = round(length_per_radian, 10)

        print("保持闭合，执行夹爪设零")
        self.set_zero(part=part)

        print("阶跃下发移动夹爪到 0.025")
        move_ok = self._move_to_target(part=part, pos_name=pos_name, target=0.025, timeout_s=timeout_s)
        if not move_ok:
            print("移动到 0.025 超时")
        print("执行夹爪设零")
        self.set_zero(part=part)

        offset_at_hardware_zero = 0.025
        try:
            yaml_path = self._write_gripper_yaml_params(
                part=part,
                length_per_radian=length_per_radian_10,
                offset_at_hardware_zero=offset_at_hardware_zero,
            )
            self.g.loggerUI.info(
                f"[参数矫正完成] yaml={yaml_path}, length_per_radian={length_per_radian_10:.10f}, "
                f"offset_at_hardware_zero={offset_at_hardware_zero:.3f}, real_distance={real_distance}, "
                f"rad1={rad1}, rad2={rad2}"
            )
            print(f"\n已写入yaml: {yaml_path}")
        except Exception as e:
            self.g.loggerUI.error(f"写入yaml失败: {e}")
            print(f"\n写入yaml失败: {e}")

        print("\n参数计算结果:")
        print(f"length_per_radian = {length_per_radian_10:.10f}")
        print(f"offset_at_hardware_zero = {offset_at_hardware_zero:.3f}")
        print(f"rad1={rad1}, rad2={rad2}, real_distance={real_distance}")
        print("提示：若下发超时后电机不再响应，建议先退出该流程再重新进入。")
        input("回车返回")

    def start_manual_control_1dof(self, data_name: str, part: str):
        self.manual_control_active = True
        self.manual_data_name = data_name
        self.manual_part = part
        self.g.loggerUI.info("进入1D手动控制模式,按W/S以控制夹爪移动,按Q退出")

    def stop_manual_control_1dof(self):
        self.manual_control_active = False
        self.g.loggerUI.info("退出1D手动控制模式")

    def manual_control_1dof_step(self):
        """UI每帧调用一次"""
        if not self.manual_control_active:
            return
        data_name = self.manual_data_name
        part = self.manual_part
        step = 0.0005
        key = None
        try:
            ch = self.g.ui.win_menu.getch()
            if ch != -1:
                key = chr(ch).lower()
        except:
            return

        if key is None:
            return

        value = getattr(self.g.feedbackData, data_name)
        current = value[0]

        if key == 'w':
            new_pos = current + step
            self.g.loggerUI.info(f"手动控制('q'退出): {data_name} 从 {current:.4f} -> {new_pos:.4f}")
        elif key == 's':
            new_pos = current - step
            self.g.loggerUI.info(f"手动控制('q'退出): {data_name} 从 {current:.4f} -> {new_pos:.4f}")
        elif key == 'q':
            self.stop_manual_control_1dof()
            return
        else:
            return

        self.g.robot.set_actions({part: {"type": "position", "position": [new_pos]}})


    def set_zero(self,part):
        self.g.robot.send_command(part,{"command":"set_zero"})
        time.sleep(1)
        self.g.loggerUI.info(f"{part}已硬件设零")
    def set_zero_loadcell(self,part:str,ch=0):
        self.g.robot.send_command(part, {"command": "calibrate_zero","index":ch}) ## index 0 for left setzero
        time.sleep(1)
        self.g.loggerUI.info(f"{part}的通道{ch}已硬件设零")

    @hide_ui_while
    def calibrate_loadcell(self,part:str,ch:int):
        known_force = input("请输入已知的标定力值(单位N),回车继续:")
        self.g.robot.send_command(part,{"command":"calibrate_force","index":ch,"force":float(known_force)})
        self.g.loggerUI.info(f"{part}的通道{ch}已标定为{known_force}N")
    
    def get_limits(self,part):
        print("当前关节范围:",self.g.robot.send_command(part, {"command": "get_limit"}))
        input("回车以返回")
    
    def move_multiple_points(self):
        #从csv文件加载为dict
        filename = input("请输入点位文件名(例如points.csv):").strip()
        points = []
        try:
            with open(filename,'r') as f:
                reader = csv.reader(f)
                header = next(reader)
                for row_idx, row in enumerate(reader):
                    q = [float(v) for v in row]
                    if len(q) != 7:
                        input(f"第{row_idx}行不是7个关节角，实际是{len(q)}个")
                        return
                    points.append(np.array(q,dtype=float))
            print(f"成功加载点位，共{len(points)}个")
        except FileNotFoundError:
            print(f"文件未找到:{filename}")
            input("回车以返回")
            return

        current_q = np.array(self.g.feedbackData.QActual, dtype=float)

        for idx, target_q in enumerate(points):

            print(f"\n===== 点 {idx}: {current_q.tolist()} -> {target_q.tolist()} =====")

            # 构造两点路径
            path = np.vstack([current_q, target_q])

            # TOPP 插值
            sample_rate = 20
            step = 1 / sample_rate
            try:
                ts, qs, qds, qdds, duration = TOPP(
                    path,
                    [self.max_vel] * 7,
                    [self.max_acc] * 7,
                    step
                )
            except RuntimeError as e:
                input("轨迹规划失败:", e)
                return

            # 执行轨迹
            print("移动中...")
            start_time = time.time()
            for t, q in zip(ts, qs):
                self.g.robot.set_actions({
                    self.g.selected_arm: {"type": "position", "position": q.tolist()}
                })
                while time.time() - start_time < t:
                    time.sleep(0.001)

            # 停留 5 秒
            print(f"到达第 {idx} 个点，停留 3.5 秒")
            stay_start = time.time()
            while time.time() - stay_start < 3.0:
                t = time.time() - stay_start
                time.sleep(0.01)

            # 手动控制走下一个点
            # input(f"到达第 {idx} 个点，确认测量成功后回车前往下一个点位")
            # time.sleep(1)

            # 下一段起点 = 当前段目标点
            current_q = target_q.copy()

        print("\n所有点已执行完成。")
        input("回车以返回")
        return

    @hide_ui_while
    def step_move_repetitively(self,part:str,pos_name:str):
        '''
        step_move_repetitively 的 Docstring

        :part: self.g.selected_arm
        :type part: str
        :pos_name: getattr(self.g.feedbackData, "rb_time")
        :type pos_name: str
        '''
        # 执行轨迹前清空日志
        self.g.highfreq_log.clear()
        self.g.lowfreq_log.clear()
        # 减少反馈负担
        self.g.feedback_mode = self.g.feedback_mode.BASIC
        user_input_start = input("请输入循环起始目标位置 (逗号分隔) 或 'q' 退出: ").strip()
        if user_input_start.lower() == 'q':
            print("结束单步移动记录")
            return
        user_input_end = input("请输入循环终止目标位置 (逗号分隔) 或 'q' 退出: ").strip()
        if user_input_end.lower() == 'q':
            print("结束单步移动记录")
            return
        while True:
            user_input = input("请输入重复次数（≥1 的整数）：")
            try:
                user_input_num = int(user_input)
                if user_input_num >= 1:
                    break
                else:
                    print("❌ 请输入大于等于 1 的整数")
            except ValueError:
                print("❌ 输入无效，请输入整数")
        print(f"✅ 输入成功：{user_input_num}")
        raw_input = input(f"设置超时时间(秒),默认3秒，回车继续: ")
        timeout_s = float(raw_input) if raw_input else 3.0
        try:
            q_pos_start = [float(x.strip()) for x in user_input_start.split(',')]
            q_pos_end = [float(x.strip()) for x in user_input_end.split(',')]
            print(f"开始空载重复测试: {q_pos_start} <-> {q_pos_end} 循环")
            
            # 执行移动并记录
            self.g.record_flag.set()
            # 占位
            print("开始高频+低频记录...")
            print("Moving...\n")
            for repeat_idx in range(user_input_num):
                print(f"\n====== 第 {repeat_idx + 1} / {user_input_num} 次循环 ======")
                for q_pos in (q_pos_start,q_pos_end):
                    print(f"目标位置: {q_pos}")
                    start_time = time.monotonic()
                    while True:
                        # 使用锁保护读取操作
                        with self.g.feedback_lock:
                            cur_q = getattr(self.g.feedbackData, pos_name)
                            rb_time = getattr(self.g.feedbackData, "rb_time")

                        # 到达判定
                        if np.max(np.abs(cur_q - np.array(q_pos))) <= TOL:
                            print("到达！")
                            self.g.loggerUI.info("到达！")
                            break

                        # 超时判定
                        elif time.monotonic() - start_time > timeout_s:
                            print(f"走点在 {timeout_s}s 后超时退出")
                            self.g.loggerUI.warn(f"走点在 {timeout_s}s 后超时退出")
                            break

                        # 发送控制命令
                        self.g.robot.set_actions({
                            part: {
                                "type": "position",
                                "position": q_pos
                            }
                        })

                        # 记录低频日志
                        self.g.logger._record_lowfreq(
                            t=time.perf_counter(),
                            q=q_pos,
                            rb_time=rb_time,
                            feedback_pos=cur_q
                        )

                        time.sleep(1 / CONTROL_HZ)
            # 延迟结束
            extra_record = time.perf_counter()
            while time.perf_counter() - extra_record <= EXTRA_TIME:
                with self.g.feedback_lock:
                    cur_q = getattr(self.g.feedbackData, pos_name)
                    rb_time = getattr(self.g.feedbackData, "rb_time")
                self.g.logger._record_lowfreq(t=time.time(),q=q_pos,
                                        rb_time=rb_time,
                                        feedback_pos=cur_q)
                time.sleep(1 / CONTROL_HZ)
            self.g.record_flag.clear()
            print("高频记录结束，共收集高频数据", len(self.g.highfreq_log))
            print("低频记录结束，共收集低频数据", len(self.g.lowfreq_log))
            # 减少反馈负担
            self.g.feedback_mode = self.g.feedback_mode.FULL
            print("\n=== 数据记录 ===")
            confirm = input("是否保存日志? (y/n): ").strip().lower()
            if confirm != "y":
                print("日志保存更新取消")
                input("回车以返回")
                return

            # 保存日志
            highfreq_filename, lowfreq_filename,tstamp = self.g.logger._save_logs(part)

            confirm = input("是否可视化记录数据? (y/n): ").strip().lower()
            if confirm != "y":
                print("数据可视化取消")
                input("回车以返回")
                return

            draw(log_dir=f'{project_root}/logs',lowfile=lowfreq_filename,highfile=highfreq_filename,savefig=f"viz_{tstamp}.png")

        except ValueError:
            print("输入无效，请输入逗号分隔的数字列表或 'exit' 退出。")

    @hide_ui_while
    def grasp_test(self, part: str, pos_name: str):
        from collections import deque
        import matplotlib.pyplot as plt
        import time
        import numpy as np
        print("[堵转测试步骤1] 移动夹爪到未抓取/堵转状态位置")

        raw_input = input(f"以逗号分隔的方式输入N自由度的期望移动位置 (e.g. 0.1 or 0.1,0.2,...,N) 或 'q' 退出: ")
        if raw_input.lower() == 'q':
            return False
        q_pos = [float(x.strip()) for x in raw_input.split(',')]
        dof = 1
        if len(q_pos) != dof:
            print(f"\nError: Expected {dof} values, but got {len(q_pos)}.")
            return False
        raw_input = input(f"设置超时时间(秒),默认3秒，回车继续: ")
        timeout_s = float(raw_input) if raw_input else 3.0
        print(f"\n移动到新位置: {[f'{p:.3f}' for p in q_pos]}")
        self._run_point(q_name=pos_name,q_pos=q_pos,part=part,timeout_s=timeout_s)

        time.sleep(0.1)
        with self.g.feedback_lock:
            record_tip_1 = np.array(getattr(self.g.feedbackData, "force_tip"))
            record_temperature_1 = np.array(getattr(self.g.feedbackData, "temperature"))

        MAX_POINTS = 500

        buf_time = deque(maxlen=MAX_POINTS)
        buf_q = deque(maxlen=MAX_POINTS)
        buf_temp = deque(maxlen=MAX_POINTS)
        buf_torque = deque(maxlen=MAX_POINTS)
        buf_force_0 = deque(maxlen=MAX_POINTS)
        buf_force_1 = deque(maxlen=MAX_POINTS)

        # ===== 主线程创建图（关键）=====
        plt.ion()
        fig, axes = plt.subplots(4, 1, figsize=(8, 8), sharex=True)

        ax_q, ax_temp, ax_torque, ax_force = axes

        line_q, = ax_q.plot([], [], 'b-', label="Position")
        line_temp, = ax_temp.plot([], [], 'r-', label="Temperature")
        line_torque, = ax_torque.plot([], [], 'g-', label="Torque")
        line_force_0, = ax_force.plot([],[],'c-',label="Tip[0]")
        line_force_1, = ax_force.plot([],[],'m-',label="Tip[1]")
        ax_q.set_ylabel("Position")
        ax_temp.set_ylabel("Temperature")
        ax_torque.set_ylabel("Torque")
        ax_torque.set_xlabel("Time (s)")
        ax_force.set_ylabel("Tip Force")
        # ===== 数值显示（右上角）=====
        txt_q = ax_q.text(
            0.98, 0.95, "",
            transform=ax_q.transAxes,
            ha="right", va="top"
        )

        txt_temp = ax_temp.text(
            0.98, 0.95, "",
            transform=ax_temp.transAxes,
            ha="right", va="top"
        )

        txt_torque = ax_torque.text(
            0.98, 0.95, "",
            transform=ax_torque.transAxes,
            ha="right", va="top"
        )

        txt_force = ax_force.text(
            0.98, 0.95, "",
            transform=ax_force.transAxes,
            ha="right", va="top"
        )
        for ax in axes:
            ax.grid(True)
            ax.legend()

        last_plot_time = 0.0
        PLOT_DT = 0.05  # 20 Hz 刷新

        # ===== 清空日志 =====
        self.g.highfreq_log.clear()
        self.g.lowfreq_log.clear()

        user_input = input("[堵转测试步骤2] 请输入目标位置 (逗号分隔) 或 'q' 退出: ").strip()
        if user_input.lower() == 'q':
            plt.close(fig)
            return

        raw_input = input("设置超时时间(秒),默认3秒，回车继续: ")
        timeout_s = float(raw_input) if raw_input else 3.0

        try:
            q_pos = np.array([float(x.strip()) for x in user_input.split(',')])
            print(f"移动到新位置: {[f'{p:.3f}' for p in q_pos]}")

            self.g.record_flag.set()
            print("开始高频+低频记录...\n")

            start_time = time.monotonic()

            while True:
                with self.g.feedback_lock:
                    cur_q = np.array(getattr(self.g.feedbackData, pos_name))
                    cur_temperature = float(getattr(self.g.feedbackData, "temperature"))
                    cur_torque = float(getattr(self.g.feedbackData, "gripper_torque"))
                    cur_tip_list = np.array(getattr(self.g.feedbackData, "force_tip"))

                # ===== 控制逻辑 =====
                if np.max(np.abs(cur_q - q_pos)) <= TOL:
                    print("到达！")
                    break
                elif time.monotonic() - start_time > timeout_s:
                    print(f"走点在 {timeout_s}s 后超时退出")
                    break

                self.g.robot.set_actions({part: {"type": "position", "position": q_pos.tolist()}})

                # ===== 记录数据 =====
                now = time.monotonic()
                buf_time.append(now)
                buf_q.append(cur_q[0])           # 单自由度
                buf_temp.append(cur_temperature)
                buf_torque.append(cur_torque)
                buf_force_0.append(cur_tip_list[0])
                buf_force_1.append(cur_tip_list[1])

                # ===== 刷新图像（主线程，安全）=====
                if now - last_plot_time > PLOT_DT and len(buf_time) > 2:
                    last_plot_time = now
                    t0 = buf_time[0]
                    t = [x - t0 for x in buf_time]

                    line_q.set_data(t, list(buf_q))
                    line_temp.set_data(t, list(buf_temp))
                    line_torque.set_data(t, list(buf_torque))
                    line_force_0.set_data(t, list(buf_force_0))
                    line_force_1.set_data(t, list(buf_force_1))
                    txt_q.set_text(f"{buf_q[-1]:.4f}")
                    txt_temp.set_text(f"{buf_temp[-1]:.2f} °C")
                    txt_torque.set_text(f"{buf_torque[-1]:.3f}")

                    txt_force.set_text(
                        f"Tip0: {buf_force_0[-1]:.3f}\n"
                        f"Tip1: {buf_force_1[-1]:.3f}"
                    )

                    for ax in axes:
                        ax.relim()
                        ax.autoscale_view()

                    plt.pause(0.001)  # 驱动 GUI

                time.sleep(1 / CONTROL_HZ)

            self.g.record_flag.clear()
            print("高频记录结束，共收集高频数据", len(self.g.highfreq_log))

            confirm = input("是否保存日志? (y/n): ").strip().lower()
            if confirm == "y":
                self.g.logger._save_logs(part)

            print("[堵转测试步骤3] 移动夹爪到未抓取/堵转状态位置")
            self._run_point_without_hide(q_name="gripper_pos",dof=1,part=part)
            time.sleep(0.1)
            with self.g.feedback_lock:
                record_tip_2 = np.array(getattr(self.g.feedbackData, "force_tip"))
                record_temperature_2 = np.array(getattr(self.g.feedbackData, "temperature"))

            self.g.loggerUI.info(f"[测试结果] 堵转前一维力:{record_tip_1}，堵转后一维力:{record_tip_2}，堵转时间:{timeout_s}")
            self.g.loggerUI.info(f"[测试结果] 堵转前电机温度:{record_temperature_1}，堵转后电机温度:{record_temperature_2},堵转时间:{timeout_s}")

        except Exception as e:
            print(e)

        finally:
            plt.ioff()
            plt.close(fig)

    @hide_ui_while
    def integrity_test(self, part: str, pos_name: str):
        from collections import deque
        import matplotlib.pyplot as plt
        import time
        import numpy as np
        print("[测试步骤1] 移动夹爪到未抓取/堵转状态位置")
        
        raw_input = input(f"以逗号分隔的方式输入N自由度的期望移动位置 (e.g. 0.1 or 0.1,0.2,...,N) 或 'q' 退出: ")
        if raw_input.lower() == 'q':
            return False
        q_pos = [float(x.strip()) for x in raw_input.split(',')]
        dof = 1
        if len(q_pos) != dof:
            print(f"\nError: Expected {dof} values, but got {len(q_pos)}.")
            return False
        raw_input = input(f"设置超时时间(秒),默认3秒，回车继续: ")
        timeout_s = float(raw_input) if raw_input else 3.0
        print(f"\n移动到新位置: {[f'{p:.3f}' for p in q_pos]}")
        self._run_point(q_name=pos_name,q_pos=q_pos,part=part,timeout_s=timeout_s)

        time.sleep(0.1)
        with self.g.feedback_lock:
            record_tip_1 = np.array(getattr(self.g.feedbackData, "force_tip"))
            record_temperature_1 = np.array(getattr(self.g.feedbackData, "temperature"))

        MAX_POINTS = 1500

        buf_time = deque(maxlen=MAX_POINTS)
        buf_q = deque(maxlen=MAX_POINTS)
        buf_temp = deque(maxlen=MAX_POINTS)
        buf_torque = deque(maxlen=MAX_POINTS)
        buf_force_0 = deque(maxlen=MAX_POINTS)
        buf_force_1 = deque(maxlen=MAX_POINTS)

        # ===== 主线程创建图（关键）=====
        plt.ion()
        fig, axes = plt.subplots(4, 1, figsize=(8, 8), sharex=True)

        ax_q, ax_temp, ax_torque, ax_force = axes

        line_q, = ax_q.plot([], [], 'b-', label="Position")
        line_temp, = ax_temp.plot([], [], 'r-', label="Temperature")
        line_torque, = ax_torque.plot([], [], 'g-', label="Torque")
        line_force_0, = ax_force.plot([],[],'c-',label="Tip[0]")
        line_force_1, = ax_force.plot([],[],'m-',label="Tip[1]")
        ax_q.set_ylabel("Position")
        ax_temp.set_ylabel("Temperature")
        ax_torque.set_ylabel("Torque")
        ax_torque.set_xlabel("Time (s)")
        ax_force.set_ylabel("Tip Force")
        # ===== 数值显示（右上角）=====
        txt_q = ax_q.text(
            0.98, 0.95, "",
            transform=ax_q.transAxes,
            ha="right", va="top"
        )

        txt_temp = ax_temp.text(
            0.98, 0.95, "",
            transform=ax_temp.transAxes,
            ha="right", va="top"
        )

        txt_torque = ax_torque.text(
            0.98, 0.95, "",
            transform=ax_torque.transAxes,
            ha="right", va="top"
        )

        txt_force = ax_force.text(
            0.98, 0.95, "",
            transform=ax_force.transAxes,
            ha="right", va="top"
        )
        for ax in axes:
            ax.grid(True)
            ax.legend()

        last_plot_time = 0.0
        PLOT_DT = 0.05  # 20 Hz 刷新

        # ===== 清空日志 =====
        self.g.highfreq_log.clear()
        self.g.lowfreq_log.clear()

        # 循环输入起始位置
        while True:
            user_input_start = input("[测试步骤2] 请输入循环起始位置 (逗号分隔) 或 'q' 退出: ").strip()
            if user_input_start.lower() == 'q':
                plt.close(fig)
                return
            try:
                start_pos = np.array([float(x.strip()) for x in user_input_start.split(',')])
                break  # 输入合法，跳出循环
            except ValueError:
                print("⚠️ 输入格式错误，请用逗号分隔数字，如 0.1,0.2,...")

        # 循环输入终止位置
        while True:
            user_input_end = input("请输入循环终止位置 (逗号分隔) 或 'q' 退出: ").strip()
            if user_input_end.lower() == 'q':
                plt.close(fig)
                return
            try:
                end_pos = np.array([float(x.strip()) for x in user_input_end.split(',')])
                break  # 输入合法，跳出循环
            except ValueError:
                print("⚠️ 输入格式错误，请用逗号分隔数字，如 0.1,0.2,...")


        raw_input = input("设置超时时间（秒）默认3秒，回车继续：")
        timeout_s = float(raw_input) if raw_input else 3.0
        while True:
            user_input = input("请输入重复次数（≥1 的整数）：")
            try:
                user_input_num = int(user_input)
                if user_input_num >= 1:
                    break
                else:
                    print("❌ 请输入大于等于 1 的整数")
            except ValueError:
                print("❌ 输入无效，请输入整数")
        print(f"✅ 输入成功：{user_input_num}")
        try:
            q_pos_start = np.array([float(x) for x in user_input_start.split(',')])
            q_pos_end   = np.array([float(x) for x in user_input_end.split(',')])

            self.g.record_flag.set()
            print("开始高频+低频记录...\n")

            for repeat_idx in range(user_input_num):
                print(f"\n====== 第 {repeat_idx+1}/{user_input_num} 次堵转循环 ======")
                
                for q_pos in (q_pos_start, q_pos_end):
                    print(f"目标位置: {q_pos.tolist()}")
                    start_time = time.monotonic()

                    while True:
                        with self.g.feedback_lock:
                            cur_q = np.array(getattr(self.g.feedbackData, pos_name))
                            cur_temperature = float(getattr(self.g.feedbackData, "temperature"))
                            cur_torque = float(getattr(self.g.feedbackData, "gripper_torque"))
                            cur_tip_list = np.array(getattr(self.g.feedbackData, "force_tip"))

                        # ===== 控制逻辑 =====
                        if np.max(np.abs(cur_q - q_pos)) <= TOL:
                            print("到达！")
                            break
                        elif time.monotonic() - start_time > timeout_s:
                            print(f"走点在 {timeout_s}s 后超时退出")
                            break

                        self.g.robot.set_actions({part: {"type": "position", "position": q_pos.tolist()}})

                        # ===== 记录数据 =====
                        now = time.monotonic()
                        buf_time.append(now)
                        buf_q.append(cur_q[0])           # 单自由度
                        buf_temp.append(cur_temperature)
                        buf_torque.append(cur_torque)
                        buf_force_0.append(cur_tip_list[0])
                        buf_force_1.append(cur_tip_list[1])

                        # ===== 刷新图像（主线程，安全）=====
                        if now - last_plot_time > PLOT_DT and len(buf_time) > 2:
                            last_plot_time = now
                            t0 = buf_time[0]
                            t = [x - t0 for x in buf_time]

                            line_q.set_data(t, list(buf_q))
                            line_temp.set_data(t, list(buf_temp))
                            line_torque.set_data(t, list(buf_torque))
                            line_force_0.set_data(t, list(buf_force_0))
                            line_force_1.set_data(t, list(buf_force_1))
                            txt_q.set_text(f"{buf_q[-1]:.4f}")
                            txt_temp.set_text(f"{buf_temp[-1]:.2f} °C")
                            txt_torque.set_text(f"{buf_torque[-1]:.3f}")

                            txt_force.set_text(
                                f"Tip0: {buf_force_0[-1]:.3f}\n"
                                f"Tip1: {buf_force_1[-1]:.3f}"
                            )

                            for ax in axes:
                                ax.relim()
                                ax.autoscale_view()

                            plt.pause(0.001)  # 驱动 GUI

                        time.sleep(1 / CONTROL_HZ)

            self.g.record_flag.clear()
            print("高频记录结束，共收集高频数据", len(self.g.highfreq_log))

            confirm = input("是否保存日志? (y/n): ").strip().lower()
            if confirm == "y":
                self.g.logger._save_logs(part)

            print("[测试步骤3] 移动夹爪到未抓取/堵转状态位置")
            self._run_point_without_hide(q_name="gripper_pos",dof=1,part=part)
            time.sleep(0.1)
            with self.g.feedback_lock:
                record_tip_2 = np.array(getattr(self.g.feedbackData, "force_tip"))
                record_temperature_2 = np.array(getattr(self.g.feedbackData, "temperature"))

            self.g.loggerUI.info(f"[测试结果] 堵转前一维力:{record_tip_1}，堵转后一维力:{record_tip_2}，堵转时间:{timeout_s}")
            self.g.loggerUI.info(f"[测试结果] 堵转前电机温度:{record_temperature_1}，堵转后电机温度:{record_temperature_2},堵转时间:{timeout_s}")

        except Exception as e:
            print(e)

        finally:
            plt.ioff()
            plt.close(fig)
