import toppra as ta
import toppra.algorithm as algo
import toppra.constraint as constraint
import numpy as np

def TOPP(path, joint_vel_limits, joint_acc_limits, step=0.1):
    """
    Time-Optimal Path Parameterization

    Args:
        path: numpy array of shape (n, dof)
        step: step size for the discretization
        verbose: if True, will print the log of TOPPRA
    """

    N_samples = path.shape[0]
    dof = path.shape[1]
    assert dof == len(joint_vel_limits)
    assert dof == len(joint_acc_limits)
    ss = np.linspace(0, 1, N_samples)
    path = ta.SplineInterpolator(ss, path)
    pc_vel = constraint.JointVelocityConstraint(joint_vel_limits)
    pc_acc = constraint.JointAccelerationConstraint(joint_acc_limits)
    instance = algo.TOPPRA([pc_vel, pc_acc], path, parametrizer="ParametrizeConstAccel")
    jnt_traj = instance.compute_trajectory()
    if jnt_traj is None:
        raise RuntimeError("Fail to parameterize path")
    # ts_sample = np.linspace(0, jnt_traj.duration, int(jnt_traj.duration / step))  # type: ignore
    ts_sample = np.arange(0, jnt_traj.duration, step)
    ts_sample = np.append(ts_sample, jnt_traj.duration)  # 确保末点
    qs_sample = jnt_traj(ts_sample)
    qds_sample = jnt_traj(ts_sample, 1)
    qdds_sample = jnt_traj(ts_sample, 2)
    return ts_sample, qs_sample, qds_sample, qdds_sample, jnt_traj.duration  # type: ignore