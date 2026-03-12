import json
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
import os
cur_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(cur_dir)
def safe_load_json(path):
    with open(path, 'r') as f:
        data = json.load(f)
    return data
def estimate_velocity(t, q):
    """
    t: (N,) 时间戳（秒）
    q: (N,) 位置
    return: (N,) 速度（首点为 NaN）
    """
    dt = np.diff(t)
    dq = np.diff(q)
    v = dq / dt
    v = np.concatenate([[np.nan], v])
    return v

def prepare_lowfreq(lowfreq):
    rb_times, pc_times, t_in_trajs = [], [], []
    cmd_pos, feedback_pos = [], []
    for entry in lowfreq:
        rb_times.append(float(entry.get("rb_time", np.nan)))
        pc_times.append(float(entry.get("pc_time", np.nan)))
        t_in_trajs.append(float(entry.get("t_in_traj", np.nan)))

        cmd = np.array(entry.get("cmd_position", []), dtype=float)
        fb = np.array(entry.get("feedback_pos", []), dtype=float)

        cmd_pos.append(cmd)
        feedback_pos.append(fb)

    return (np.array(rb_times),
            np.array(pc_times),
            np.array(t_in_trajs),
            np.stack(cmd_pos),
            np.stack(feedback_pos))

def prepare_highfreq(highfreq):
    rb_times = []
    positions = []
    velocities = []
    torques = []
    temps = []
    for entry in highfreq:
        rb_times.append(float(entry.get("rb_time", np.nan)))
        pos = entry.get("highfreq_pos", None)
        if pos is not None:
            positions.append(np.array(pos,dtype=float))
        else:
            positions.append(np.full(1,np.nan))
                # 速度
        vel = entry.get("highfreq_vel", None)
        if vel is not None:
            velocities.append(np.array(vel, dtype=float))
        else:
            velocities.append(np.full(1, np.nan))

        # 力矩/力
        tq = entry.get("highfreq_toq", None)
        if tq is not None:
            torques.append(np.array(tq, dtype=float))
        else:
            torques.append(np.full(1, np.nan))
        temp = entry.get("highfreq_temp", None)
        if temp is not None:
            temps.append(np.array([temp], dtype=float))
        else:
            temps.append(np.full(1, np.nan))
    # NumPy
    return (
        np.array(rb_times),
        np.stack(positions),
        np.stack(velocities),
        np.stack(torques),
        np.stack(temps)
    )

def draw(log_dir=".", lowfile="lowfreq.json", highfile="highfreq.json", savefig="viz_multi.png"):
    log_dir = Path(log_dir)
    
    # 加载数据
    lowfreq = safe_load_json(log_dir / lowfile)
    highfreq = safe_load_json(log_dir / highfile)
    
    if not lowfreq or not highfreq:
        print("⚠️ 高频或低频日志为空")
        return

    # 准备数据（使用原始函数）
    rb_low, pc_times, t_in_trajs, cmd_pos, fb_at_send = prepare_lowfreq(lowfreq)
    rb_high, high_freq, high_freq_vel, high_freq_torque, high_freq_temp = prepare_highfreq(highfreq)

    # 只移除NaN值，不做复杂的清理
    low_valid = ~np.isnan(rb_low) & ~np.any(np.isnan(cmd_pos), axis=1) & ~np.any(np.isnan(fb_at_send), axis=1)
    high_valid = ~np.isnan(rb_high) & ~np.any(np.isnan(high_freq), axis=1)
    
    rb_low = rb_low[low_valid]
    cmd_pos = cmd_pos[low_valid]
    fb_at_send = fb_at_send[low_valid]
    
    rb_high = rb_high[high_valid]
    high_freq = high_freq[high_valid]
    high_freq_vel = high_freq_vel[high_valid]
    high_freq_torque = high_freq_torque[high_valid]
    high_freq_temp = high_freq_temp[high_valid]

    if rb_high.size == 0 or rb_low.size == 0:
        print("⚠️ 有效数据为空")
        return

    # 使用第一个有效数据点作为时间零点
    rb0_low = rb_low[0]
    rb0_high = rb_high[0]
    
    # 保持各自的时间基准，或者统一使用低频的时间基准
    rb_low_rel = rb_low - rb0_low
    rb_high_rel = rb_high - rb0_high

    # 自动检测自由度数量
    dof = cmd_pos.shape[1]
    print(f"检测到 {dof} 个自由度")
    print(f"低频数据点: {len(rb_low)}, 高频数据点: {len(rb_high)}")

    fig, axes = plt.subplots(dof, 2, figsize=(12, 3 * dof), sharex=True)
    if dof == 1:
        axes = np.array([[axes[0], axes[1]]])

    for i in range(dof):
        # --- 左列：原始数据对比 ---
        ax1 = axes[i, 0]
        
        # 高频反馈数据（原始数据）
        ax1.plot(rb_high_rel, high_freq[:, i], 'b-', 
                markersize=1, linewidth=0.8, label="HighFreq Feedback", alpha=0.8)
        
        # 低频命令位置
        ax1.plot(rb_low_rel, cmd_pos[:, i], 'ro-', 
                markersize=4, linewidth=1.5, label="Command Position", alpha=0.8)
        
        # 发送时的反馈位置
        ax1.scatter(rb_low_rel, fb_at_send[:, i], 
                   s=30, label="Feedback at Send", color='green', alpha=0.7, zorder=5)
        
        ax1.set_ylabel(f"Joint {i+1} (rad)")
        ax1.legend(loc='best', fontsize='small')
        ax1.grid(True, alpha=0.3)
        
        # 添加数据点标记
        ax1.annotate(f'H: {len(rb_high)} pts', 
                    xy=(0.02, 0.98), xycoords='axes fraction',
                    fontsize=8, verticalalignment='top')
        ax1.annotate(f'L: {len(rb_low)} pts', 
                    xy=(0.02, 0.90), xycoords='axes fraction',
                    fontsize=8, verticalalignment='top')

        # --- 右列：简单误差计算（不插值） ---
        ax2 = axes[i, 1]
        
        # 简化：直接计算命令与发送时反馈的误差
        direct_error = cmd_pos[:, i] - fb_at_send[:, i]
    
        ax2.plot(rb_low_rel, direct_error, 'b-', 
                label="Cmd - FbAtSend", linewidth=1.5)
        
        # 零线
        ax2.axhline(y=0, color='k', lw=1, linestyle='-', alpha=0.5)
        
        ax2.set_ylabel(f"Error (rad)")
        ax2.legend(loc='best', fontsize='small')
        ax2.grid(True, alpha=0.3)
        
        # 误差统计
        min_abs_direct = np.min(np.abs(direct_error))
        max_abs_direct = np.max(np.abs(direct_error))
        
        ax2.text(0.98, 0.98, 
                f'Max Direct: {max_abs_direct:.4f}\nMin Direct: {min_abs_direct:.4f}',
                transform=ax2.transAxes,
                fontsize=8, verticalalignment='top', horizontalalignment='right',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.3))

    # X轴标签
    axes[-1, 0].set_xlabel("Relative Time from First LowFreq (s)")
    axes[-1, 1].set_xlabel("Relative Time from First LowFreq (s)")
    
    # 总标题
    fig.suptitle(f'Raw Data Visualization\nLowFreq: {lowfile}, HighFreq: {highfile}', 
                fontsize=12, y=1.02)
    
    plt.tight_layout()
    
    # 保存图像
    save_path = f"{project_root}/logs/{savefig}"
    plt.savefig(save_path, dpi=200, bbox_inches='tight')
    print(f"✅ 图像已保存为 {save_path}")

    # ===== 新图：高频速度（所有自由度） =====
    fig_v, axv = plt.subplots(figsize=(10, 4))

    colors = plt.cm.tab10.colors  # Tab10 有 10 种颜色循环
    dof = high_freq_vel.shape[1]

    for i in range(dof):
        axv.plot(rb_high - rb_high[0], high_freq_vel[:, i], 
                color=colors[i % 10], linewidth=1.2, 
                label=f'Joint {i+1} Velocity')

    axv.axhline(0, color='k', lw=1, alpha=0.5)
    axv.set_xlabel("Relative Time (s)")
    axv.set_ylabel("Velocity (rad/s)")
    axv.grid(True, alpha=0.3)
    axv.legend(fontsize='small')

    vel_save_path = f"{project_root}/logs/vel_{savefig}"
    fig_v.savefig(vel_save_path, dpi=200, bbox_inches='tight')
    print(f"✅ 高频速度图已保存为 {vel_save_path}")


    # ===== 新图：温度（单通道）=====
    if high_freq_temp.size > 0 and not np.all(np.isnan(high_freq_temp)):
        fig_t, axt = plt.subplots(figsize=(10, 4))
        axt.plot(rb_high - rb_high[0], high_freq_temp[:, 0], color='tab:red', linewidth=1.2, label="Temperature")
        axt.set_xlabel("Relative Time (s)")
        axt.set_ylabel("Temperature (°C)")
        axt.grid(True, alpha=0.3)
        axt.legend(fontsize='small')
        temp_save_path = f"{project_root}/logs/temp_{savefig}"
        fig_t.savefig(temp_save_path, dpi=200, bbox_inches='tight')
        print(f"Saved temperature plot: {temp_save_path}")

    plt.show()

def draw_wrench(log_dir=".", highfile="highfreq.json", savefig="viz_wrench.png"):
    """
    可视化高频力传感器数据（Wrench）
    支持显示 Fx, Fy, Fz, Tx, Ty, Tz 六维曲线
    """
    log_dir = Path(log_dir)
    highfreq = safe_load_json(log_dir / highfile)

    # 解析时间与力数据
    rb_times = []
    wrenches = []
    for entry in highfreq:
        if isinstance(entry, dict) and "Wrench" in entry:
            rb_times.append(float(entry.get("rb_time", np.nan)))
            wrenches.append(np.array(entry["Wrench"], dtype=float))
        elif isinstance(entry, dict) and "highfreq_log" in entry:
            # 如果是新格式的 zero_shift 结果 JSON
            for sub in entry["highfreq_log"]:
                rb_times.append(float(sub.get("rb_time", np.nan)))
                wrenches.append(np.array(sub.get("Wrench", [np.nan] * 6), dtype=float))

    if len(rb_times) == 0 or len(wrenches) == 0:
        print("⚠️ 日志中未检测到 Wrench 数据")
        return

    rb_times = np.array(rb_times)
    wrenches = np.stack(wrenches)
    rb_times -= rb_times[0]  # 从0秒开始

    labels = ["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"]
    fig, axes = plt.subplots(6, 1, figsize=(10, 10), sharex=True)

    for i in range(6):
        if i < wrenches.shape[1]:
            axes[i].plot(rb_times, wrenches[:, i], label=labels[i])
            mean_val = np.mean(wrenches[:, i])
            std_val = np.std(wrenches[:, i])
            axes[i].axhline(mean_val, color='r', linestyle='--', alpha=0.7,
                            label=f"mean={mean_val:.3f}, std={std_val:.3f}")
            axes[i].set_ylabel(labels[i])
            axes[i].legend()
            axes[i].grid(True)

    axes[-1].set_xlabel("Robot Time (s)")
    plt.tight_layout()
    plt.savefig(savefig, dpi=200)
    print(f"✅ 力传感器图像已保存为 {savefig}")
    plt.show()

def draw_wrench_comp(log_dir=".", logfile="gravity_comp.json", savefig="viz_wrench_comp.png"):
    """
    可视化力传感器数据（Wrench）
    同时显示补偿前（wrench_raw）和补偿后（wrench_comp）的六维曲线
    """
    log_dir = Path(log_dir)
    data = safe_load_json(log_dir / logfile)

    if not data or not isinstance(data, list):
        print("⚠️ 日志数据为空或格式错误")
        return

    times = np.arange(len(data))  # 用索引作为时间轴
    wrench_raw = []
    wrench_comp = []

    for entry in data:
        wrench_raw.append(np.array(entry.get("wrench_raw", [np.nan]*6), dtype=float))
        wrench_comp.append(np.array(entry.get("wrench_comp", [np.nan]*6), dtype=float))

    wrench_raw = np.stack(wrench_raw)
    wrench_comp = np.stack(wrench_comp)

    labels = ["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"]
    fig, axes = plt.subplots(6, 1, figsize=(12, 10), sharex=True)

    for i in range(6):
        axes[i].plot(times, wrench_raw[:, i], label=f"{labels[i]} raw", color="tab:blue", alpha=0.7)
        axes[i].plot(times, wrench_comp[:, i], label=f"{labels[i]} comp", color="tab:orange", alpha=0.9)
        axes[i].axhline(np.mean(wrench_raw[:, i]), color='blue', linestyle='--', alpha=0.5)
        axes[i].axhline(np.mean(wrench_comp[:, i]), color='orange', linestyle='--', alpha=0.5)
        axes[i].set_ylabel(labels[i])
        axes[i].legend()
        axes[i].grid(True)

    axes[-1].set_xlabel("Sample Index")
    plt.tight_layout()
    plt.savefig(savefig, dpi=200)
    print(f"✅ 力传感器补偿对比图已保存为 {savefig}")
    plt.show()

def draw_step_response_analysis(log_dir=".",
                                lowfile="lowfreq.json",
                                highfile="highfreq.json",
                                savefig="step_response.png",
                                target_pos: list = [1.0],
                                threshold=0.02):
    """
    Step 阶跃响应分析，用于评估控制系统性能：
    - 超调量 (Overshoot)
    - 稳定时间 (Settling Time)
    - 稳态误差 (Steady-State Error)
    """

    log_dir = Path(log_dir)
    
    # 加载数据
    lowfreq = safe_load_json(log_dir / lowfile)
    highfreq = safe_load_json(log_dir / highfile)
    
    if not lowfreq or not highfreq:
        print("⚠️ 高频或低频日志为空")
        return

    # 准备数据
    rb_low, pc_times, t_in_trajs, cmd_pos, fb_at_send = prepare_lowfreq(lowfreq)
    rb_high, high_freq, high_freq_vel, high_freq_torque, _ = prepare_highfreq(highfreq)

    # 清理 NaN
    low_valid = ~np.isnan(rb_low) & ~np.any(np.isnan(cmd_pos), axis=1) & ~np.any(np.isnan(fb_at_send), axis=1)
    high_valid = ~np.isnan(rb_high) & ~np.any(np.isnan(high_freq), axis=1)
    
    rb_low = rb_low[low_valid]
    cmd_pos = cmd_pos[low_valid]
    fb_at_send = fb_at_send[low_valid]
    
    rb_high = rb_high[high_valid]
    high_freq = high_freq[high_valid]

    if rb_low.size == 0 or rb_high.size == 0:
        print("⚠️ 有效数据为空")
        return

    # 时间基准
    t_low = rb_low - rb_low[0]
    t_high = rb_high - rb_high[0]

    dof = cmd_pos.shape[1]
    print(f"检测到 {dof} 个自由度")

    fig, axes = plt.subplots(dof, 2, figsize=(12, 3 * dof), sharex=True)
    if dof == 1:
        axes = np.array([[axes[0], axes[1]]])

    for i in range(dof):
        ax1 = axes[i, 0]
        ax2 = axes[i, 1]

        # --- 左图：阶跃响应 ---
        ax1.plot(t_low, cmd_pos[:, i], 'r--', label='Command')
        ax1.plot(t_high, high_freq[:, i], 'b-', label='HighFreq Feedback', alpha=0.7)

        # 超调量计算
        final_val = target_pos[i]
        peak_val = np.max(high_freq[:, i])
        overshoot = (peak_val - final_val) / final_val if final_val != 0 else peak_val
        overshoot_pct = overshoot * 100

        # 稳定时间计算：稳态±threshold
        steady_mask = np.abs(fb_at_send[:, i] - final_val) <= threshold * abs(final_val)
        if np.any(steady_mask):
            t_steady_idx = np.where(steady_mask)[0][0]
            settling_time = t_low[t_steady_idx]
        else:
            settling_time = np.nan

        # 标注超调
        ax1.plot(t_low[np.argmax(fb_at_send[:, i])], peak_val, 'ro', label=f'Peak {peak_val:.3f}')
        ax1.axhline(final_val, color='k', linestyle='--', alpha=0.5, label='Target')
        ax1.set_ylabel(f'Joint {i+1} Pos')
        ax1.grid(True, alpha=0.3)
        ax1.legend(fontsize='small')

        ax1.text(0.98, 0.95,
                 f'Overshoot: {overshoot_pct:.1f}%\nSettling: {settling_time:.2f}s\nSteadyErr: {final_val - fb_at_send[-1, i]:.3f}',
                 transform=ax1.transAxes,
                 ha='right', va='top',
                 fontsize=8,
                 bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.3))

        # --- 右图：误差曲线 ---
        error = target_pos[i] - fb_at_send[:, i]
        ax2.plot(t_low, error, 'm-', label='Error')
        ax2.axhline(0, color='k', linestyle='--', alpha=0.5)
        ax2.set_ylabel('Error')
        ax2.grid(True, alpha=0.3)
        ax2.legend(fontsize='small')

    axes[-1, 0].set_xlabel("Time (s)")
    axes[-1, 1].set_xlabel("Time (s)")

    fig.suptitle(f"Step Response Analysis\nLowFreq: {lowfile}, HighFreq: {highfile}", fontsize=12)
    plt.tight_layout()

    save_path = f"{project_root}/logs/{savefig}"
    plt.savefig(save_path, dpi=200, bbox_inches='tight')
    print(f"✅ Step Response 图已保存为 {save_path}")

    # ===== 新图：高频速度（所有自由度） =====
    fig_v, axv = plt.subplots(figsize=(10, 4))

    colors = plt.cm.tab10.colors  # Tab10 有 10 种颜色循环
    dof = high_freq_vel.shape[1]

    for i in range(dof):
        axv.plot(rb_high - rb_high[0], high_freq_vel[:, i], 
                color=colors[i % 10], linewidth=1.2, 
                label=f'Joint {i+1} Velocity')

    axv.axhline(0, color='k', lw=1, alpha=0.5)
    axv.set_xlabel("Relative Time (s)")
    axv.set_ylabel("Velocity (rad/s)")
    axv.grid(True, alpha=0.3)
    axv.legend(fontsize='small')

    vel_save_path = f"{project_root}/logs/vel_{savefig}"
    fig_v.savefig(vel_save_path, dpi=200, bbox_inches='tight')
    print(f"✅ 高频速度图已保存为 {vel_save_path}")

    plt.show()


if __name__ == "__main__":
    draw(
        log_dir="../logs",
        lowfile="lowfreq_20260227_183809_right_gripper.json",
        highfile="highfreq_20260227_183809_right_gripper.json",
        savefig="traj_viz_multi.png"
    )
