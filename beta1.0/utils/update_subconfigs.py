#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from utils.config_manager import ConfigManager
from pathlib import Path
import argparse

BASE_DIR = Path(__file__).parent

def parse_args():
    parser = argparse.ArgumentParser(description="Generate subconfigs from a complete config")
    parser.add_argument(
        "--config",
        type=str,
        required=True,
        help="完整配置文件名 (不含 .yaml)，如：--config TEST_NO_2"
    )
    return parser.parse_args()


def sanitize_name(component_list):
    """将 ['left_arm', 'left_force_sensor'] → 'left_arm_and_left_force_sensor'"""
    return "_and_".join(component_list)


def generate_one_subconfig(complete_config_name: str, component_list: list):
    """
    从完整配置中提取部分组件并保存成新的 subconfig 文件
    """
    complete_path = BASE_DIR / "complete_configs" / f"{complete_config_name}.yaml"
    complete_cfg = ConfigManager(config_path=complete_path)

    # 1) 提取对应组件
    sub_dict = complete_cfg.extract_components(component_list)

    # 2) 子配置文件名
    sub_name = sanitize_name(component_list)
    output_path = BASE_DIR / "configs" / f"{sub_name}.yaml"

    # 3) 保存
    sub_cfg = ConfigManager.from_dict(
        config_path=output_path,
        config_dict=sub_dict
    )
    sub_cfg.save()

    print(f"[OK] 已生成 subconfig: {output_path}")


def main():
    args = parse_args()

    # ================================
    # 写死的组件组合列表
    # ================================
    COMPONENT_COMBOS = [
        ["head"],
        ["left_arm", "left_force_sensor"],
        ["left_arm"],
        ["left_force_sensor"],
        ["left_gripper"],
        ["left_loadcell"],
        ["rail"],
        ["right_arm"],
        ["right_force_sensor"],
        ["right_gripper"],
        ["right_loadcell"],
        ["waist"]
    ]
    # ================================

    print(f"=== 依据完整配置 [{args.config}] 生成所有子配置 ===")

    for combo in COMPONENT_COMBOS:
        generate_one_subconfig(args.config, combo)

    print("\nAll subconfigs generated ✔")


if __name__ == "__main__":
    main()
