from blazar.cfg import Config, field
from typing import Any
import json

RBComponentConfig = dict[str, Any]


class RustBotConfig(Config):
    hardware: dict[str, RBComponentConfig] = field(default_factory=dict)
    planner: dict[str, Any] | None = None
    robot_model: str = ""


class RustBotConfigHelper:
    def __init__(self, path: str = "/opt/RoboConfig/roboassets/real_base/rb_config_base.yaml"):
        self.config = RustBotConfig.create(path)

        self.rb_config_dict = {}
        self.rb_config_dict["hardware"] = self.config.hardware
        # self.rb_config_dict['planner'] = self.config.planner
        self.rb_config_dict["planner"] = None
        self.rb_config_dict["robot_model"] = self.config.robot_model

    def get_dict(self) -> dict[str, Any]:
        return self.rb_config_dict

    def filter_for_some_hardware(
        self, component_names: list[str]
    ) -> RBComponentConfig | None:
        new_config = self.rb_config_dict.copy()
        new_config["hardware"] = {}
        for name in component_names:
            if name in self.rb_config_dict["hardware"]:
                new_config["hardware"][name] = self.rb_config_dict["hardware"][name]
            else:
                print(f"Warning: component {name} not found in config")

        return new_config

    def ignore_external_protections(
        self, d: RBComponentConfig, component_name: str
    ) -> RBComponentConfig:
        if component_name in d["hardware"]:
            d["hardware"][component_name]["external_protections"] = []
        else:
            print(f"Warning: component {component_name} not found in config")

        return d


if __name__ == "__main__":
    config = RustBotConfigHelper()
    print(f"Full config {json.dumps(config.get_dict(), indent=4)}")

    # 只取部分硬件配置
    partial_config = config.filter_for_some_hardware(["left_arm", "base"])
    print(f"Partial config {json.dumps(partial_config, indent=4)}")

    # 修改某个硬件配置参数
    partial_config["hardware"]["base"]["max_linear_velocity"] = 0.3
    print(f"Partial config after modification {json.dumps(partial_config, indent=4)}")
