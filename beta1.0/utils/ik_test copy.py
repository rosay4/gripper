import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
import numpy as np
import os
import meshcat
from numpy.linalg import norm, solve

# --- Meshcat 可视化 ---
vis = meshcat.Visualizer()          # 创建可视化对象
vis.open()                           # 打开浏览器

# --- 项目路径 ---
cur_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(cur_dir)

# --- 机器人 URDF ---
robot = RobotWrapper.BuildFromURDF(
    f"{project_root}/model/beta1_0_left_arm.urdf",
    package_dirs=[f"{project_root}/model/"]
)
model = robot.model
data = robot.data
for i,name in enumerate(robot.model.names):
    print(i,name)
print("Lower joint limits:", robot.model.lowerPositionLimit)
print("Upper joint limits:", robot.model.upperPositionLimit)
print("Velocity limits:", robot.model.velocityLimit)
JOINT_ID = 7
quat = np.array([0.029,0.007,-0.969,0.246]) # xyzw
R = pin.Quaternion(quat).toRotationMatrix()
t = np.array([0.502,0.554,1.269])
oMdes = pin.SE3(R,t)
q = np.array(
    [0.057739500038115794, 
     0.9170245686893153, 
     1.2053679694158732, 
     -1.401221994645382, 
     -0.6229272251626734, 
     -0.03238998401929382, 
     1.466, 
     0.0, 
     0.0])
noise = 0.1
q += np.random.uniform(-noise,noise,size=q.shape)
eps = 1e-4
IT_MAX = 1000
DT = 1e-1
damp = 1e-6

i = 0
import time
tick = time.perf_counter()
while True:
    pin.forwardKinematics(model,data,q)
    iMd = data.oMi[JOINT_ID].actInv(oMdes)
    err = pin.log(iMd).vector
    if norm(err)<eps:
        success = True
        break
    if i >= IT_MAX:
        success = False
        break
    J = pin.computeJointJacobian(model,data,q,JOINT_ID)
    J = -np.dot(pin.Jlog6(iMd.inverse()),J)
    v = -J.T.dot(solve(J.dot(J.T)+damp*np.eye(6),err))
    q = pin.integrate(model,q,v*DT)
    q = np.clip(q,model.lowerPositionLimit,model.upperPositionLimit)
    if not i % 10:
        print(f"{i}:error = {err.T}")
    i += 1
tick -= time.perf_counter()
tick *= -1


if success:
    print("Convergence achieved!")
else:
    print(
        "\n"
        "Warning: the iterative algorithm has not reached convergence "
        "to the desired precision"
    )
print(f"\nresult:{q.flatten().tolist()}")
print(f"\nfinal error:{err.T}")
print(f"elapsed_time:{tick}")
# --- 初始化可视化 ---
robot.initViewer(viewer=vis, loadModel=True)
robot.display(q)  # 在浏览器显示初始姿态
