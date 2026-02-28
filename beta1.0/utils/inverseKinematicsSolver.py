import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
import numpy as np
import os
import meshcat
from numpy.linalg import norm, solve
import time

class InverseKinematicsSolver:
    def __init__(self, urdf_path, package_dirs=None, joint_limits_clip=True,vis=False):
        """
        初始化 IK 求解器
        """
        self.robot = RobotWrapper.BuildFromURDF(urdf_path, package_dirs=package_dirs or [])
        self.model = self.robot.model
        self.data = self.robot.data
        self.joint_limits_clip = joint_limits_clip
        self.vis = vis

        # 可视化
        if vis:
            self.vis = meshcat.Visualizer()
            self.vis.open()
            self.robot.initViewer(viewer=self.vis, loadModel=True)

    def solve_ik(
        self,
        JOINT_ID,
        oMdes,
        q_init=None,
        eps=1e-4,
        IT_MAX=1000,
        DT=1e-1,
        damp=1e-6,
        verbose=True
    ):
        """
        使用阻尼最小二乘法求解 IK
        """
        q = np.array(q_init) if q_init is not None else pin.neutral(self.model)

        i = 0
        start_time = time.perf_counter()
        while True:
            pin.forwardKinematics(self.model, self.data, q)
            iMd = self.data.oMi[JOINT_ID].actInv(oMdes)
            err = pin.log(iMd).vector

            if norm(err) < eps:
                success = True
                break
            if i >= IT_MAX:
                success = False
                break

            J = pin.computeJointJacobian(self.model, self.data, q, JOINT_ID)
            J = -np.dot(pin.Jlog6(iMd.inverse()), J)
            v = -J.T.dot(solve(J.dot(J.T) + damp*np.eye(6), err))
            q = pin.integrate(self.model, q, v*DT)

            if self.joint_limits_clip:
                q = np.clip(q, self.model.lowerPositionLimit, self.model.upperPositionLimit)

            if verbose and i % 10 == 0:
                print(f"{i}: error = {err.T}")

            i += 1

        elapsed_time = time.perf_counter() - start_time

        if verbose:
            if success:
                print("Convergence achieved!")
            else:
                print("\nWarning: the iterative algorithm has not reached convergence to the desired precision")
            print(f"\nresult: {q.flatten().tolist()}")
            print(f"\nfinal error: {err.T}")
            print(f"elapsed_time: {elapsed_time:.4f} s")

        # 显示结果
        if self.vis:
            self.robot.display(q)

        return q, err, success, elapsed_time

    def get_joint_names(self):
        return self.model.names

    def get_joint_limits(self):
        return self.model.lowerPositionLimit, self.model.upperPositionLimit

# -------------------------
# 使用示例
# -------------------------
if __name__ == "__main__":
    cur_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(cur_dir)
    urdf_file = f"{project_root}/model/beta1_0_left_arm.urdf"
    solver = InverseKinematicsSolver(urdf_file, package_dirs=[f"{project_root}/model/"])
    # 目标位姿
    quat = np.array([0.029,0.007,-0.969,0.246]) # xyzw
    R = pin.Quaternion(quat).toRotationMatrix()
    t = np.array([0.502,0.554,1.269])
    oMdes = pin.SE3(R, t)
    # 初始关节
    q_init = np.array([0.06191577392718428, 0.9083360985512569, 1.2158680546253378, -1.400075588937896, -0.6340218847758505, -0.03992223526142489, 1.4577041783367155, 0.0, 0.0])
    sigma = 0.001  # 标准差
    q_perturbed = q_init + np.random.normal(0, sigma, size=q_init.shape)
    # 求解 IK
    q_sol, err, success, elapsed = solver.solve_ik(
        JOINT_ID=7,
        oMdes=oMdes,
        q_init=q_perturbed,
        eps=1e-4,
        IT_MAX=1000,
        DT=0.5,
        damp=1e-6,
    )
    print(f"error:{norm(q_init - q_sol)}")