import sapien.core as sapien
from sapien.utils.viewer import Viewer
import time
import numpy as np
import sys
import select


class SapienTrajectoryPlayer:
    def __init__(self, urdf_path, movable_indices, sleep_dt=0.01):
        self.engine = sapien.Engine()
        self.renderer = sapien.SapienRenderer()
        self.engine.set_renderer(self.renderer)

        self.scene = self.engine.create_scene()
        self.scene.set_ambient_light([0.5, 0.5, 0.5])
        self.scene.add_directional_light([1, -1, -1], [1, 1, 1], shadow=True)

        loader = self.scene.create_urdf_loader()
        loader.fix_root_link = True
        self.robot = loader.load(urdf_path)

        self.viewer = Viewer(self.renderer)
        self.viewer.set_scene(self.scene)
        self.viewer.set_camera_xyz(1.6, 0, 1.2)
        self.viewer.set_camera_rpy(0, -0.3, 3.14)

        self.movable_indices = movable_indices
        self.sleep_dt = sleep_dt

    def _play_once(self, qs, verbose=True):
        """播放一遍轨迹"""
        for i, q in enumerate(qs):
            if self.viewer.closed:
                return False

            qpos = self.robot.get_qpos()
            qpos[self.movable_indices] = q
            self.robot.set_qpos(qpos)

            self.scene.update_render()
            self.viewer.render()
            time.sleep(self.sleep_dt)

            if verbose and i % 20 == 0:
                print(f"[{i}/{len(qs)}] q = {np.round(q, 4)}")

        return True

    def play_with_replay(self, qs, verbose=True):
        """
        按 Enter 重放轨迹，保持自由旋转视角
        """
        print(f"[SAPIEN] Trajectory points: {len(qs)}")
        replay_flag = False  # 初始不播放
        print("\nPrint ENTER to play")
        while not self.viewer.closed:
            # --- 如果用户按了 Enter，就播放一次 ---
            if replay_flag:
                print("\n[SAPIEN] Playing trajectory...")
                ok = self._play_once(qs, verbose)
                if not ok:
                    break
                print("\n[SAPIEN] Trajectory finished.")
                replay_flag = False  # 播放完成后重置

            # --- 检测 Enter ---
            # timeout=0.1 保证 UI 依然渲染
            if select.select([sys.stdin], [], [], 0.1)[0]:
                line = sys.stdin.readline()
                if line.strip() == "":
                    replay_flag = True

            # --- 保持 UI 可旋转 ---
            self.scene.update_render()
            self.viewer.render()

        print("[SAPIEN] Viewer closed, exit.")
