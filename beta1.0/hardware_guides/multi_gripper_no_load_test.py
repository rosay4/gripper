#!/usr/bin/env python3
import argparse
import json
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np
import rb_python

cur_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(cur_dir)
folder_utils = os.path.join(project_root, "utils")
if folder_utils not in sys.path:
    sys.path.insert(0, folder_utils)

from robotConfig_util import GripperConfig, HardwareConfig, RobotConfig


PARTS = ("left_gripper", "right_gripper")
CONTROL_HZ = 20
FEEDBACK_HZ = 100
TOL = 1e-3
EXTRA_RECORD_SEC = 0.2


class MultiGripperNoLoadTest:
    def __init__(self, config_dir: str):
        self.config_dir = Path(config_dir)
        self.robot = None
        self.stop_event = threading.Event()
        self.feedback_lock = threading.Lock()
        self.feedback = {
            part: {
                "position": None,
                "velocity": None,
                "torque": None,
                "rb_time": None,
            }
            for part in PARTS
        }
        self.highfreq_log = {part: [] for part in PARTS}
        self.lowfreq_log = {part: [] for part in PARTS}
        self.feedback_thread = None

    def build_config(self):
        container = {}
        for part in PARTS:
            yaml_path = self.config_dir / f"{part}.yaml"
            if not yaml_path.exists():
                raise FileNotFoundError(f"missing config: {yaml_path}")
            container[part] = GripperConfig.create(str(yaml_path))
            print(f"loaded {part}: {yaml_path}, ids={container[part].ids}")

        hardware = HardwareConfig.create_from_container(container)
        return RobotConfig.recreate(
            {
                "hardware": hardware.to_dict_container(),
                "planner": None,
                "robot_model": "",
            }
        )

    def start(self):
        config = self.build_config().get_data()
        print("initializing robot...")
        self.robot = rb_python.robot.Robot(config)
        print("robot initialized, waiting for grippers to become operational...")
        self.wait_for_operational(timeout_s=10.0)
        time.sleep(1.0)
        self.feedback_thread = threading.Thread(target=self._feedback_loop, daemon=True)
        self.feedback_thread.start()

    def stop(self):
        self.stop_event.set()
        if self.feedback_thread:
            self.feedback_thread.join(timeout=2.0)
        if self.robot:
            self.robot.shutdown()

    def _feedback_loop(self):
        period = 1.0 / FEEDBACK_HZ
        next_time = time.perf_counter()
        while not self.stop_event.is_set():
            now = time.perf_counter()
            if now < next_time:
                time.sleep(next_time - now)
            next_time += period

            try:
                states = self.robot.get_states()
            except Exception as exc:
                print(f"feedback error: {exc}")
                time.sleep(0.1)
                continue

            if not states:
                time.sleep(0.001)
                continue

            with self.feedback_lock:
                for part in PARTS:
                    state = states.get(part)
                    if not state:
                        continue
                    rb_time = state.get("time") or {}
                    secs = rb_time.get("secs", 0)
                    nanos = rb_time.get("nanos", 0) * 1e-9
                    self.feedback[part] = {
                        "position": _to_array(state.get("position")),
                        "velocity": _to_array(state.get("velocity")),
                        "torque": _to_array(state.get("torque")),
                        "rb_time": secs + nanos,
                    }

                    self.highfreq_log[part].append(
                        {
                            "pc_time": time.time(),
                            "rb_time": secs + nanos,
                            "highfreq_pos": _jsonable(self.feedback[part]["position"]),
                            "highfreq_vel": _jsonable(self.feedback[part]["velocity"]),
                            "highfreq_toq": _jsonable(self.feedback[part]["torque"]),
                        }
                    )

    def wait_for_initial_feedback(self, timeout_s: float = 5.0):
        print("waiting for initial feedback...")
        start = time.monotonic()
        while time.monotonic() - start <= timeout_s:
            with self.feedback_lock:
                ready = all(self.feedback[part]["position"] is not None for part in PARTS)
            if ready:
                print("initial feedback received")
                return True
            time.sleep(0.05)
        return False

    def wait_for_operational(self, timeout_s: float):
        start = time.monotonic()
        last_print = 0.0
        while time.monotonic() - start <= timeout_s:
            status = self._get_robot_status()
            ready = all(status.get(part) == "operational" for part in PARTS)
            now = time.monotonic()
            if now - last_print >= 1.0:
                print(f"status: {status}")
                last_print = now
            if ready:
                print("all grippers operational")
                return
            time.sleep(0.1)
        self.print_robot_diagnostics()
        raise RuntimeError(f"grippers did not become operational within {timeout_s}s")

    def _get_robot_status(self):
        try:
            status = self.robot.get_status()
        except Exception as exc:
            return {"error": str(exc)}
        if not isinstance(status, dict):
            return {"raw": status}
        return {part: status.get(part) for part in PARTS}

    def run_repetitive_test(self, start_pos, end_pos, repeat_count: int, timeout_s: float, dwell_s: float):
        targets = (start_pos, end_pos)
        print(f"start test: {PARTS}")
        print(
            f"targets: {start_pos} <-> {end_pos}, repeats: {repeat_count}, "
            f"timeout: {timeout_s}s, dwell: {dwell_s}s"
        )

        for repeat_idx in range(repeat_count):
            print(f"cycle {repeat_idx + 1}/{repeat_count}")
            for target in targets:
                reached = self._move_all_to(target=target, timeout_s=timeout_s)
                if reached and dwell_s > 0:
                    print(f"  dwell {dwell_s}s before reversing")
                    time.sleep(dwell_s)

        extra_start = time.perf_counter()
        while time.perf_counter() - extra_start <= EXTRA_RECORD_SEC:
            with self.feedback_lock:
                snapshot = {part: dict(self.feedback[part]) for part in PARTS}
            for part in PARTS:
                self._record_lowfreq(
                    part=part,
                    target=end_pos,
                    feedback=snapshot[part],
                    t_in_traj=time.perf_counter(),
                )
            time.sleep(1.0 / CONTROL_HZ)

    def _move_all_to(self, target, timeout_s: float):
        target_arr = np.array(target)
        start_time = time.monotonic()
        reached = {part: False for part in PARTS}

        while True:
            with self.feedback_lock:
                snapshot = {part: dict(self.feedback[part]) for part in PARTS}

            for part in PARTS:
                cur_q = snapshot[part]["position"]
                if cur_q is not None and np.max(np.abs(cur_q - target_arr)) <= TOL:
                    reached[part] = True

            if all(reached.values()):
                print(f"  reached: {target}")
                return True

            if time.monotonic() - start_time > timeout_s:
                pending = [part for part, ok in reached.items() if not ok]
                print(f"  timeout after {timeout_s}s, pending: {pending}, target: {target}")
                return False

            action = {
                part: {
                    "type": "position",
                    "position": list(target),
                }
                for part in PARTS
            }
            try:
                self.robot.set_actions(action)
            except RuntimeError as exc:
                self.print_robot_diagnostics()
                raise RuntimeError(f"set_actions failed for target {target}: {exc}") from exc

            for part in PARTS:
                self._record_lowfreq(
                    part=part,
                    target=target,
                    feedback=snapshot[part],
                    t_in_traj=time.perf_counter(),
                )

            time.sleep(1.0 / CONTROL_HZ)

    def _record_lowfreq(self, part, target, feedback, t_in_traj):
        self.lowfreq_log[part].append(
            {
                "pc_time": time.time(),
                "t_in_traj": t_in_traj,
                "cmd_position": list(target),
                "feedback_pos": _jsonable(feedback["position"]),
                "rb_time": feedback["rb_time"],
            }
        )

    def save_logs(self):
        log_dir = Path(project_root) / "logs"
        log_dir.mkdir(exist_ok=True)
        tstamp = time.strftime("%Y%m%d_%H%M%S")
        saved = []

        for part in PARTS:
            high_file = log_dir / f"highfreq_{tstamp}_{part}_multi_no_load.json"
            low_file = log_dir / f"lowfreq_{tstamp}_{part}_multi_no_load.json"
            with high_file.open("w", encoding="utf-8") as f:
                json.dump(self.highfreq_log[part], f, indent=2)
            with low_file.open("w", encoding="utf-8") as f:
                json.dump(self.lowfreq_log[part], f, indent=2)
            print(f"saved: {high_file}")
            print(f"saved: {low_file}")
            saved.append((part, low_file.name, high_file.name, f"viz_{tstamp}_{part}_multi_no_load.png"))
        return log_dir, saved

    def print_robot_diagnostics(self):
        try:
            print(f"robot status: {self.robot.get_status()}")
        except Exception as exc:
            print(f"robot status unavailable: {exc}")
        try:
            print(f"robot errors: {self.robot.get_errors()}")
        except Exception as exc:
            print(f"robot errors unavailable: {exc}")


def run_plots(log_dir, saved_logs, plot_env: str, plot_python: str | None):
    python_path = Path(plot_python) if plot_python else _find_conda_env_python(plot_env)
    if python_path is None:
        print(f"plot skipped: conda env not found: {plot_env}")
        return
    if not python_path.exists():
        print(f"plot skipped: python not found: {python_path}")
        return

    code = (
        "import sys; "
        "import matplotlib; "
        "matplotlib.use('Agg'); "
        f"sys.path.insert(0, {repr(folder_utils)}); "
        "from visualize import draw; "
        "draw(log_dir=sys.argv[1], lowfile=sys.argv[2], highfile=sys.argv[3], "
        "savefig=sys.argv[4], show_plot=False)"
    )
    for part, lowfile, highfile, savefig in saved_logs:
        print(f"plotting {part}: {savefig}")
        cmd = [
            str(python_path),
            "-c",
            code,
            str(log_dir),
            lowfile,
            highfile,
            savefig,
        ]
        try:
            env = os.environ.copy()
            env["MPLBACKEND"] = "Agg"
            subprocess.run(cmd, check=True, env=env)
        except subprocess.CalledProcessError as exc:
            print(f"plot failed for {part}: {exc}")
            print("check that the plot Python environment has matplotlib and a non-GUI backend available")


def _find_conda_env_python(env_name: str):
    candidates = []
    conda_exe = os.environ.get("CONDA_EXE")
    if conda_exe:
        candidates.append(Path(conda_exe).resolve().parents[1] / "envs" / env_name / "bin" / "python")
    home = Path.home()
    candidates.extend(
        [
            home / "miniforge3" / "envs" / env_name / "bin" / "python",
            home / "miniconda3" / "envs" / env_name / "bin" / "python",
            home / "anaconda3" / "envs" / env_name / "bin" / "python",
        ]
    )
    for candidate in candidates:
        if candidate.exists() and os.access(candidate, os.X_OK):
            return candidate
    return None


def _to_array(value):
    if value is None:
        return None
    return np.array(value)


def _jsonable(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.float32, np.float64)):
        return float(value)
    if isinstance(value, (np.int32, np.int64)):
        return int(value)
    return value


def _prompt_float(prompt: str, default: float):
    raw = input(f"{prompt} [{default}]: ").strip()
    return default if not raw else float(raw)


def _prompt_int(prompt: str, default: int):
    raw = input(f"{prompt} [{default}]: ").strip()
    return default if not raw else int(raw)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run no-load repetitive motion test on left/right grippers together."
    )
    parser.add_argument(
        "--config-dir",
        default="/opt/robot/rb_hardware",
        help="Directory containing left_gripper.yaml and right_gripper.yaml.",
    )
    parser.add_argument("--start", type=float, help="Start target position.")
    parser.add_argument("--end", type=float, help="End target position.")
    parser.add_argument("--count", type=int, help="Repeat count.")
    parser.add_argument("--timeout", type=float, help="Per-target timeout seconds.")
    parser.add_argument("--dwell", type=float, help="Dwell seconds at each endpoint before reversing.")
    parser.add_argument(
        "--yes",
        action="store_true",
        help="Use defaults for omitted test parameters without prompting.",
    )
    parser.add_argument(
        "--no-plot",
        action="store_true",
        help="Skip plotting after logs are saved.",
    )
    parser.add_argument(
        "--plot-env",
        default="pyqt6_env",
        help="Conda environment used to generate plots.",
    )
    parser.add_argument(
        "--plot-python",
        help="Python executable used to generate plots.",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    if args.yes:
        start_pos = 0.0 if args.start is None else args.start
        end_pos = 0.05 if args.end is None else args.end
        repeat_count = 2000 if args.count is None else args.count
        timeout_s = 3.0 if args.timeout is None else args.timeout
        dwell_s = 0.5 if args.dwell is None else args.dwell
    else:
        start_pos = args.start if args.start is not None else _prompt_float("start position", 0.0)
        end_pos = args.end if args.end is not None else _prompt_float("end position", 0.05)
        repeat_count = args.count if args.count is not None else _prompt_int("repeat count", 2000)
        timeout_s = args.timeout if args.timeout is not None else _prompt_float("timeout seconds", 3.0)
        dwell_s = args.dwell if args.dwell is not None else _prompt_float("dwell seconds", 0.5)

    if repeat_count < 1:
        raise ValueError("repeat count must be >= 1")

    test = MultiGripperNoLoadTest(config_dir=args.config_dir)
    log_dir = None
    saved_logs = []
    try:
        test.start()
        if not test.wait_for_initial_feedback():
            raise RuntimeError("timeout waiting for initial feedback from both grippers")
        test.run_repetitive_test(
            start_pos=[start_pos],
            end_pos=[end_pos],
            repeat_count=repeat_count,
            timeout_s=timeout_s,
            dwell_s=dwell_s,
        )
        log_dir, saved_logs = test.save_logs()
        if not args.no_plot:
            run_plots(
                log_dir=log_dir,
                saved_logs=saved_logs,
                plot_env=args.plot_env,
                plot_python=args.plot_python,
            )
    except Exception:
        if any(test.highfreq_log[part] or test.lowfreq_log[part] for part in PARTS):
            print("test interrupted or failed, saving collected logs")
            log_dir, saved_logs = test.save_logs()
            if not args.no_plot:
                run_plots(
                    log_dir=log_dir,
                    saved_logs=saved_logs,
                    plot_env=args.plot_env,
                    plot_python=args.plot_python,
                )
        raise
    finally:
        test.stop()


if __name__ == "__main__":
    main()
