import time
import numpy as np
import json
from pathlib import Path
from typing import Dict,Any
BASE_DIR = Path(__file__).parent.parent
class LogModule:
    def __init__(self,guide):
        self.g = guide
    def push_highfreq(self,rb_time,feedback_pos,feedback_vel=None,feedback_torque=None):
        self.g.highfreq_log.append({
            "rb_time":rb_time,
            "highfreq_pos":feedback_pos,
            "highfreq_vel":feedback_vel,
            "highfreq_toq":feedback_torque,
        })
    def _record_lowfreq(self,t,q,rb_time,feedback_pos):
        """控制侧日志(20Hz)"""
        self.g.lowfreq_log.append({
            "pc_time":time.time(),
            "t_in_traj":t,
            "cmd_position":q,
            "feedback_pos":feedback_pos,
            "rb_time":rb_time,
        })
    def _save_logs(self,part_name:str):
        """保存日志文件"""
        '''
        part_name: self.g.selected_arm
        '''
        log_dir = BASE_DIR / "logs"
        log_dir.mkdir(exist_ok=True)
        tstamp = time.strftime("%Y%m%d_%H%M%S")

        def safe_convert(data):
            """递归地把 numpy 类型转成可序列化类型"""
            if isinstance(data, np.ndarray):
                return data.tolist()
            elif isinstance(data, (np.float32, np.float64)):
                return float(data)
            elif isinstance(data, (np.int32, np.int64)):
                return int(data)
            elif isinstance(data, dict):
                return {k: safe_convert(v) for k, v in data.items()}
            elif isinstance(data, list):
                return [safe_convert(v) for v in data]
            else:
                return data

        highfreq = safe_convert(self.g.highfreq_log)
        lowfreq = safe_convert(self.g.lowfreq_log)

        with open(log_dir / f"highfreq_{tstamp}_{part_name}.json", "w") as f:
            json.dump(highfreq, f, indent=2)

        with open(log_dir / f"lowfreq_{tstamp}_{part_name}.json", "w") as f:
            json.dump(lowfreq, f, indent=2)

        print(f"✅ 日志已保存到 {log_dir}")
        highfreq_file_name = f"highfreq_{tstamp}_{part_name}.json"
        lowfreq_file_name = f"lowfreq_{tstamp}_{part_name}.json" 
        return highfreq_file_name,lowfreq_file_name,tstamp  