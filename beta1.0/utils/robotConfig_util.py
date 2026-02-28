from blazar.cfg import Config
from dataclasses import field
from pathlib import Path
from enum import Enum
from typing import Dict,List,Optional,Any
import json
import ast

def _safe_parse(value, old_value=None):
    """类型安全解析：根据 old_value 推断类型"""
    # 如果 old_value 提供了类型，则按它解析
    if old_value is not None:
        if isinstance(old_value, bool):
            return value.lower() in ("true", "1", "yes")

        if isinstance(old_value, int):
            return int(value)

        if isinstance(old_value, float):
            return float(value)

        if isinstance(old_value, list):
            try:
                val = ast.literal_eval(value)
                if isinstance(val, list):
                    return val
            except:
                pass
            return [ast.literal_eval(v.strip()) for v in value.split(",")]

        if isinstance(old_value, tuple):
            return tuple(ast.literal_eval(value))

        if isinstance(old_value, dict):
            return ast.literal_eval(value)

    # 否则自动解析
    try:
        return ast.literal_eval(value)
    except:
        return value

# 组件注册表
COMPONENT_REGISTRY = {}

def register_component(name: str):
    def decorator(cls):
        COMPONENT_REGISTRY[name] = cls
        return cls
    return decorator

## 相机组件配置类定义

class ExtrinsicPose(Config):
    p:list
    q:list

class CalibrationResult(Config):
    old_intrinsic:list[list[float]]
    distortion:list
    intrinsic:list[list[float]]
    extrinsic_pose:ExtrinsicPose

class CameraSensorConfig(Config):
    height:int
    width:int
    is_fisheye:bool
    calibration:CalibrationResult

class CameraModuleConfig(Config):
    camera_id:str
    base_frame:str
    sensors:Dict[str,CameraSensorConfig]

class CameraSystemConfig(Config):
    cameras:Dict[str,CameraModuleConfig]

## 硬件组件配置类定义
class AdmittanceConfig(Config):
    param_mass:List[float]
    param_stiff:List[float]
    param_damp:List[float]
    param_wrench_zero:List[float]
    param_gravity:List[float]
    param_mass_pos:List[float]
    force_threshold:float
    deadband:float
    lock_rotate:bool
    time_enter:float
    time_exit:float
    # max_delta: float
    # tracking_error_exit: float
    freeze_step: int

@register_component("left_arm")
@register_component("right_arm")
class ArmConfig(Config):
    type:str
    ids:list
    length_per_radian:list
    invert_directions:list
    control_freq:int
    interpolation_points:int
    max_velocity:float
    gravity_compensation_tolerance:float
    friction_compensation_scale:float
    friction_compensation_stiffness:int
    external_protections:list
    protection_rebound:float
    offset_at_hardware_zero:list
    joint_names:list
    max_torque:list
    admittance_config:AdmittanceConfig
    force_sensor_name: str
    expected_urdf_link_name: str
    waist_angle: float

@register_component("left_gripper")
@register_component("right_gripper")
class GripperConfig(Config):
    type:str
    ids:list
    length_per_radian:list
    invert_directions:list
    control_freq:float
    max_velocity:float
    interpolation_points:int
    gravity_compensation_tolerance:float
    friction_compensation_scale:float
    friction_compensation_stiffness:float
    joint_names:list
    external_protections:Any
    offset_at_hardware_zero:list
    max_torque:list
    protection_rebound:float
    # position_limit:list
    # wrap_around_zero: list
    admittance_config:AdmittanceConfig
    force_sensor_name: str
    expected_urdf_link_name: str
    waist_angle: float
    waist_name: str

@register_component("head")
class HeadConfig(Config):
    type:str
    ids:list
    length_per_radian:list
    invert_directions:list
    control_freq:int
    interpolation_points:int
    max_velocity:float
    gravity_compensation_tolerance:float
    friction_compensation_scale:float
    friction_compensation_stiffness:float
    external_protections:list
    protection_rebound:float
    offset_at_hardware_zero:list
    joint_names:list
    max_torque:list

@register_component("rail")
class RailConfig(Config):
    type:str
    ids:list
    encoder_id:Optional[Any]
    din1:str
    din1_invert:bool
    din2:str
    din2_invert:bool
    din3:str
    din3_invert:bool
    din4:str
    din4_invert:bool
    joint_names:list
    invert_directions:list
    inc_per_rev:float
    rev_per_meter:float
    dec_per_rpm:float
    encoder_inc_per_rev:float
    encoder_max_revs:float
    control_freq:float
    interpolation_points:int
    min_position_meter:list
    max_position_meter:list
    max_velocity:float
    dec_per_rps2:float
    arms_per_dec:float
    nm_per_arms:float
    max_acc:float
    max_dec:float

@register_component("waist")
class WaistConfig(Config):
    type:str
    ids:list
    joint_names:list
    inc_per_rev:int
    min_position_rad:list
    max_position_rad:list
    control_freq:float
    interpolation_points:int
    max_velocity:float
    invert_directions:list
    max_acc:float
    max_dec:float

@register_component("base")
class BaseConfig(Config):
    type:str
    steering_ids:list
    steering_joint_names:list
    driving_ids:list
    driving_joint_names:list
    wheel_placement:list
    wheel_radius:float
    wheel_direction:list
    steering_motor_shaft_ratio:float
    steering_threshold:float
    speed_threshold:float
    max_linear_velocity:float
    max_angular_velocity:float
    start_with_calibration_mode:bool

@register_component("battery")
class BatteryConfig(Config):
    type:str
    count:int
    check_interval_sec:int

# @register_component("left_loadcell")
# @register_component("right_loadcell")
# class LoadcellConfig(Config):
#     type:str
#     tcp_ip:str
#     tcp_port:int
@register_component("left_loadcell")
@register_component("right_loadcell")
class LoadcellConfig(Config):
    type:str
    usb_bus:int
    usb_ports:list
    usb_index:int
    link_name:str

# @register_component("left_force_sensor")
# @register_component("right_force_sensor")
# class ForceSensorConfig_legacy(Config):
#     type:str
#     usb_bus:int
#     usb_ports:list
#     usb_index:int
#     id:int
#     legacy_mode:bool
#     link_name:str
#     zero_on_start:bool

@register_component("left_force_sensor")
@register_component("right_force_sensor")
class ForceSensorConfig(Config):
    type:str
    usb_bus:int
    usb_ports:list
    usb_index:int
    id:int
    use_can_version: bool
    freq: float
    legacy_mode:bool
    link_name:str
    calibration: Any | None
    zero_on_start:bool
    switch_xy:bool

class HardwareConfig(Config):
    components: Dict[str, Any]

    @classmethod
    def create_from_container(cls, container: Dict[str, Any]):
        resolved = {}
        for name, cfg in container.items():
            if name not in COMPONENT_REGISTRY:
                raise ValueError(f"Unknown component name: {name}")
            comp_cls = COMPONENT_REGISTRY[name]
            resolved[name] = comp_cls.create(cfg)
        return cls(components=resolved)

class PlannerConfig(Config):
    urdf:str | None = ''
    enable_collision_checking:bool | None = False
    stop_margin:float | None = 0.01
    collision_margin:float | None = 0.05

class RobotConfig(Config):
    hardware: Dict[str, Any]
    planner: PlannerConfig | None = None
    robot_model: str | None = ''
    
    @classmethod
    def recreate(cls, cfg: dict):
        hw = cfg.get("hardware", {})
        # 解包 hardware.components
        if isinstance(hw, dict) and "components" in hw:
            hw = hw["components"]
        
        if isinstance(hw, dict) and "components" not in hw:
            raw_hw = hw
            hw = {}
            for name, val in raw_hw.items():
                if name not in COMPONENT_REGISTRY:
                    raise ValueError(f"Unknown component name: {name}")
                comp_cls = COMPONENT_REGISTRY[name]
                hw[name] = comp_cls.create(val)

        # 结构化 planner
        planner_cfg = cfg.get("planner", {})
        if isinstance(planner_cfg, dict):
            planner = PlannerConfig(**planner_cfg)
        else:
            planner = planner_cfg

        return cls(
            hardware=hw,
            planner=planner,
            robot_model=cfg.get("robot_model", "")
        )
    def save(self,config_path:Path):
        """保存当前配置到文件"""
        self.save_pretty(config_path)

    def to_dict(self) -> dict:
        """将配置转换为字典"""
        return self.to_dict_container()

    def get_component(self, component_name: str):
        """获取硬件组件配置"""
        return self.hardware.get(component_name)

    def set_component(self, key_path: str):
        """设置硬件组件配置"""
        parts = key_path.split('.')
        node = self.hardware
        for k in parts[:-1]:
            if issubclass(type(node[k]),Config):
                node[k] = node[k].to_dict_container()
            if k not in node or not isinstance(node[k], dict):
                return input(f"Invalid path: '{k}' not found.\n回车以返回...")
            node = node[k] # 向下一级寻找
        leaf = parts[-1]
        if leaf not in node:
            return input(f"Invalid key: '{leaf}' not found.\n回车以返回...")
        old_value = node[leaf]
        print(f"\n当前值 '{key_path}':\n {old_value}")
        print(f"\n类型:\n {type(old_value).__name__}")
        while True:
            new_value = input("\n输入新值: (输入 q 放弃修改)").strip()
            if not new_value:
                print("输入为空，请重试")
                continue
            if new_value.lower() == 'q':
                print("已取消修改。")
                input("按回车返回...")
                return
            if isinstance(new_value, str):
                parsed = _safe_parse(new_value, old_value)
                break
            else:
                parsed = new_value
                break
        print("\n=== 确认更改 ===")
        print(f"{key_path}:\n  {old_value}  →  {parsed}")
        confirm = input("是否应用修改? (y/n): ").strip().lower()
        if confirm != "y":
            print("值更新取消")
            input("回车以返回")
            return
        node[leaf] = parsed
        return parsed
    
    def get_data(self):
        return self.to_dict_container()

if __name__ == "__main__":
    # config_path = Path("configs/left_arm.yaml")

    # MyArm = ArmConfig.create(config_path)
    # MyHardware = HardwareConfig.create_from_container({"left_arm": MyArm})

    # MyRobot = RobotConfig.recreate({
    #     "hardware": MyHardware.to_dict_container(),
    #     "planner": PlannerConfig().to_dict_container(),
    #     "robot_model": ""
    # })
    # print(MyRobot.get_component("left_arm")["ids"])
    # MyRobot.set_component("left_arm.type")
    # print(MyRobot.get_component("left_arm"))
    # MyRobot.save(Path("/opt/robot/rb_config.yaml"))
    # print(MyRobot.to_dict())


    config_path = Path("/opt/robot/rb_config.yaml")
    MyRobot = RobotConfig.recreate({
        "hardware": RobotConfig.create(config_path).hardware,
        "planner": RobotConfig.create(config_path).planner,
        "robot_model": RobotConfig.create(config_path).robot_model
    })
    print(MyRobot.get_data())
    print(f"\n")
    config_path = Path("/opt/robot/camera_config.yaml")
    MyRobot = CameraSystemConfig.create(config_path)
    print(MyRobot.to_dict_container())
