from inverseKinematicsSolver import InverseKinematicsSolver
from cartesianPlanner import CartesianPlanner
from sapien_player import SapienTrajectoryPlayer
import pinocchio as pin
import numpy as np
import os

# ---------- 路径 ----------
cur_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(cur_dir)
urdf_file = f"{project_root}/model/beta1_0_left_arm.urdf"

# ---------- IK & Planner ----------
solver = InverseKinematicsSolver(
    urdf_path=urdf_file,
    package_dirs=[f"{project_root}/model/"],
    joint_limits_clip=True
)

planner = CartesianPlanner(
    ik_solver=solver,
    joint_vel_limits=solver.model.velocityLimit,
    joint_acc_limits=np.ones(solver.model.nv) * 2.0,
    cartesian_step=0.02,
    ik_max_iter=2000,
    verbose=True,
    ik_eps=1e-4,
)

# ---------- 初始关节 ----------
# q_start = np.array([-1.16664,0.37008,0.38941,1.16607,3.38852,-0.45259,0.05492,0,0])
q_start = np.array([0.0]*9)
pin.forwardKinematics(solver.model, solver.data, q_start)
pin.updateFramePlacements(solver.model, solver.data)

JOINT_ID = 7
oM_start = solver.data.oMi[JOINT_ID]

quat = np.array([-0.395,-0.577,0.714,-0.043])
R = pin.Quaternion(quat).toRotationMatrix()
t = np.array([0.538,0.506,1.224])
# R = oM_start.rotation
# t = oM_start.translation
oM_goal = pin.SE3(R, t)

# ---------- 规划 ----------
ts, qs, qds, qdds, T = planner.plan(
    JOINT_ID=JOINT_ID,
    oM_start=oM_start,
    oM_goal=oM_goal,
    q_start=q_start,
    dt=0.01,
)

print("Trajectory duration:", T)
print("Trajectory points:", len(qs))

# ---------- SAPIEN 可视化 ----------
player = SapienTrajectoryPlayer(
    urdf_path=urdf_file,
    movable_indices=[0,1,2,3,4,5,6,7,8],
    sleep_dt=0.01
)

input("Press ENTER to play trajectory...")
player.play_with_replay(qs)
