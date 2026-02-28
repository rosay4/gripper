import yaml
from pathlib import Path
from typing import Any, Union
import pprint

class ConfigManager:
    def __init__(self, config_path: Union[str, Path]):
        self.path = Path(config_path)
        self.data = self._load_yaml()

    def _load_yaml(self) -> dict:
        if not self.path.exists():
            raise FileNotFoundError(f"Config file not found: {self.path}")
        with open(self.path, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f) or {}

    def save(self):
        """保存当前 config 到文件"""
        with open(self.path, 'w', encoding='utf-8') as f:
            yaml.safe_dump(self.data, f, allow_unicode=True, sort_keys=False)

    def get(self, key_path: str, default: Any = None) -> Any:
        """
        根据路径字符串取值，例如：
        get("robot.arm.length")
        """
        keys = key_path.split('.')
        d = self.data
        for k in keys:
            if isinstance(d, dict) and k in d:
                d = d[k]
            else:
                return default
        return d

    def set(self, key_path: str, value: Any):
        """
        根据路径字符串设置值，例如：
        set("robot.arm.length", 0.42)
        """
        keys = key_path.split('.')
        d = self.data
        for k in keys[:-1]:
            if k not in d or not isinstance(d[k], dict):
                d[k] = {}
            d = d[k]
        d[keys[-1]] = value

    def _deep_update(self, base: dict, updates: dict):
        """
        递归合并字典：保留 base 原有内容，只更新 updates 中的部分。
        """
        for k, v in updates.items():
            if isinstance(v, dict) and isinstance(base.get(k), dict):
                self._deep_update(base[k], v)
            else:
                base[k] = v


    def set_dict(self, new_data: dict, base_key: str = None, deep_merge: bool = True):
        """
        批量设置配置项。
        - 如果指定 base_key（可为链式路径 "a.b.c"），则合并到该键下；
        - 如果 deep_merge=True，则递归合并（保留原字段）。
        """
        target = self.data

        # 支持链式路径：例如 base_key = "left_arm.admittance_config"
        if base_key:
            keys = base_key.split(".")
            for k in keys:
                if k not in target or not isinstance(target[k], dict):
                    target[k] = {}
                target = target[k]

        # 执行深度或浅层合并
        if deep_merge:
            self._deep_update(target, new_data)
        else:
            target.update(new_data)

        return self


    def reload(self):
        """从磁盘重新加载"""
        self.data = self._load_yaml()

    def __getitem__(self, item):
        return self.data[item]

    def __setitem__(self, key, value):
        self.data[key] = value
    
    def _load_yaml_from_path(self, path: str) -> dict:
        """从指定路径加载 YAML 文件"""
        path = Path(path)
        if not path.exists():
            raise FileNotFoundError(f"Config file not found: {path}")
        with open(path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
        return data or {}


    @classmethod
    def from_dict(cls,config_path:Union[str,Path],config_dict:dict):
        path = Path(config_path)
        path.parent.mkdir(parents=True,exist_ok=True)
        with open(path,'w',encoding='utf-8') as f:
            yaml.safe_dump(config_dict,f,allow_unicode=True,sort_keys=False)
        instance = cls(path)
        return instance

    def show(self, key_path: str = None):
        """
        打印整个 config 或指定路径下的内容（格式化显示）
        例：
            show()                     # 打印完整配置
            show("hardware.left_arm")  # 打印左臂配置
        """
        data_to_show = self.data

        if key_path:
            keys = key_path.split('.')
            for k in keys:
                if isinstance(data_to_show, dict) and k in data_to_show:
                    data_to_show = data_to_show[k]
                else:
                    print(f"[WARN] Key path '{key_path}' 不存在。")
                    return

        print("\n🧩 当前配置内容:")
        pprint.pprint(data_to_show, sort_dicts=False, width=100)
    def extract_components(self,keys:list[str]) -> dict:
        result = {}
        for k in keys:
            if k in self.data:
                result[k] = yaml.safe_load(yaml.safe_dump(self.data[k]))
            else:
                print(f"[WARN] component '{k}' 不存在")
        return result