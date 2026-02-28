import numpy as np
import pinocchio as pin
import toppra as ta
import toppra.algorithm as algo
import toppra.constraint as constraint
from numpy.linalg import norm

class CartesianPlanner:
    def __init__(
            self,
            ik_solver,
            joint_vel_limits,
            joint_acc_limits,
            cartesian_step=0.02,
            ik_eps=1e-4,
            ik_max_iter=200,
            ik_damp=1e-4,
            ik_dt=1.0,
            verbose=False
        ):
        """
        Cartesian space planner using IK + TOPPRA
        """
        self.ik = ik_solver
        self.joint_vel_limits = joint_vel_limits
        self.joint_acc_limits = joint_acc_limits
        self.cartesian_step = cartesian_step
        self.ik_eps = ik_eps
        self.ik_max_iter = ik_max_iter
        self.ik_damp = ik_damp
        self.ik_dt = ik_dt
        self.verbose = verbose

    def interpolate_cartesian(self,oM_start,oM_goal):
        """
        interpolate SE3 linearly in position + slerp in rotation
        
        :param oM_start: 起始位置
        :param oM_goal: 目标位置
        """
        dp = oM_goal.translation - oM_start.translation
        dist = np.linalg.norm(dp)

        N = max(2,int(dist/self.cartesian_step))
        ss = np.linspace(0.0,1.0,N)

        oM_list = [
            pin.SE3.Interpolate(oM_start,oM_goal,s) for s in ss
        ]
        return oM_list

    # def solve_ik_path(self,JOINT_ID,oM_list,q_start):
    #     """
    #     solve_ik_path for a sequence of SE3 targets
    #     """
    #     q_path = []
    #     q_curr = q_start.copy()
        
    #     for i, oMdes in enumerate(oM_list):
    #         q_sol, err, success, _ = self.ik.solve_ik(
    #             JOINT_ID = JOINT_ID,
    #             oMdes = oMdes,
    #             q_init = q_curr,
    #             eps = self.ik_eps,
    #             IT_MAX = self.ik_max_iter,
    #             DT = self.ik_dt,
    #             damp = self.ik_damp,
    #         )
    #         if not success:
    #             raise RuntimeError(f"IK failed at waypoint {i},err={norm(err)}")
    #         q_path.append(q_sol.copy())
    #         q_curr = q_sol
            
    #         if self.verbose:
    #             print(f"[IK] waypoint {i}/{len(oM_list)} err={norm(err):.2e}")
    #     return np.vstack(q_path)

    def solve_ik_path(self, JOINT_ID, oM_list, q_start):
        """
        Solve IK path for a sequence of SE3 targets.
        No strict constraint: all waypoints are accepted regardless of error.
        """
        q_path = []
        q_curr = q_start.copy()

        for i, oMdes in enumerate(oM_list):
            q_sol, err, success, _ = self.ik.solve_ik(
                JOINT_ID=JOINT_ID,
                oMdes=oMdes,
                q_init=q_curr,
                eps=self.ik_eps,
                IT_MAX=self.ik_max_iter,
                DT=self.ik_dt,
                damp=self.ik_damp,
            )

            err_norm = norm(err)
            if self.verbose:
                print(f"[IK] waypoint {i}/{len(oM_list)} err={err_norm:.2e} (ignored)")

            # 接受解作为下一点初值
            q_path.append(q_sol.copy())
            q_curr = q_sol

        return np.vstack(q_path)
    
    def time_parameterize(self,q_path,dt=0.01):
        """
        time_parameterize: Apply TOPPRA to joint path
        """
        N, dof = q_path.shape
        ss = np.linspace(0,1,N)
        path = ta.SplineInterpolator(ss,q_path)
        pc_vel = constraint.JointVelocityConstraint(self.joint_vel_limits)
        pc_acc = constraint.JointAccelerationConstraint(self.joint_acc_limits)

        instance = algo.TOPPRA(
            [pc_vel,pc_acc],
            path,
            parametrizer="ParametrizeConstAccel"
        )
        traj = instance.compute_trajectory()
        if traj is None:
            raise RuntimeError("TOPPRA failed!")
        ts = np.arange(0,traj.duration,dt)
        ts = np.append(ts,traj.duration)
        qs = traj(ts)
        qds = traj(ts,1)
        qdds = traj(ts,2)
        return ts,qs,qds,qdds,traj.duration
    
    def plan(self,JOINT_ID,oM_start,oM_goal,q_start,dt=0.01,):
        """
        plan: Full pipeline,
        SE3 -> IK path -> TOPPRA
        """
        if self.verbose:
            print("[Planner] Interpolating Cartesian path")
        
        oM_list = self.interpolate_cartesian(oM_start,oM_goal)

        if self.verbose:
            print("[Planner] Solving IK path")
        
        q_path = self.solve_ik_path(JOINT_ID,oM_list,q_start)

        if self.verbose:
            print("[Planner] Time-parameterizing")
        
        return self.time_parameterize(q_path,dt)
