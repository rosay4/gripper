from components.common.device_api import LocalDevice, RemoteDevice, Device
from components.common.logger_wrapper import log_message
import numpy as np
from typing import Optional, Callable


class Lidar:

    class StateInfo:
        POSITION = "position"
        REFLECTIVITY = "reflectivity"
        TAG = "tag"

    def __init__(self, name: str, device: LocalDevice | RemoteDevice):
        self.name = name
        self.device = device
        self.last_state_time = {}
        self.position: list[(float, float, float)] = []
        self.reflectivity: list[float] = []
        self.tag: list[int] = []

    def get_state(self) -> bool:
        """获取当前状态"""
        state = self.device.get_states([self.name])
        # log_message(state)
        if state and self.name in state and state[self.name]["type"] == "points":
            self.state = state[self.name]
            self.last_state_time = self.state["time"]
            ret = True
            if self.StateInfo.POSITION in self.state:
                self.position = [
                    tuple(x)
                    for x in np.frombuffer(
                        self.state[self.StateInfo.POSITION], dtype=np.float32
                    ).reshape(-1, 3)
                ]
            else:
                ret = False
            if self.StateInfo.REFLECTIVITY in self.state:
                self.reflectivity = self.state[self.StateInfo.REFLECTIVITY]
            if self.StateInfo.TAG in self.state:
                self.tag = self.state[self.StateInfo.TAG]
            return ret
        return False

    def get_last_state_time(self) -> dict:
        """获取上次状态更新时间"""
        return self.last_state_time

    def get_status(self) -> str:
        """获取传感器状态"""
        status = self.device.get_status()
        if status and self.name in status:
            return status[self.name]
        return None

    def is_operational(self) -> bool:
        """检查触觉传感器是否处于 operational 状态"""
        status = self.get_status()
        return status == Device.Status.Operational

    def get_error(self) -> tuple[list[float], list[str]]:
        """获取报错信息"""
        errors = self.device.get_errors()
        if errors and self.name in errors:
            # wrapper_log_message(f"Motor {self.name} errors: {errors[self.name]}")
            return errors[self.name]["code"], errors[self.name]["str"]
        return None, None

    def get_position(self) -> tuple[bool, list[(float, float, float)]]:
        """获取当前点云位置"""
        if self.get_state():
            return True, self.position

        return False, []

    def get_last_position(self) -> list[(float, float, float)]:
        """获取当前缓存的点云位置"""
        return self.position

    def get_reflectivity(self) -> tuple[bool, list[float]]:
        """获取当前点云反射强度"""
        if self.get_state():
            return True, self.reflectivity

        return False, []

    def get_last_reflectivity(self) -> list[float]:
        """获取当前缓存的点云反射强度"""
        return self.reflectivity

    def get_tag(self) -> tuple[bool, list[int]]:
        """获取当前点云标签"""
        if self.get_state():
            return True, self.tag

        return False, []

    def get_last_tag(self) -> list[int]:
        """获取当前缓存的点云标签"""
        return self.tag


class LivoxLidar(Lidar):
    def __init__(self, name: str, device: LocalDevice | RemoteDevice):
        log_message(f"Initializing Livox Lidar: {name}")
        super().__init__(name, device)

    def get_state(self):
        """获取当前状态"""
        name = self.name + "/points"
        state = self.device.get_states([name])
        # log_message(state)
        if state and name in state and state[name]["type"] == "points":
            self.state = state[name]
            self.last_state_time = self.state["time"]
            ret = True
            if self.StateInfo.POSITION in self.state:
                # log_message(
                #     f"Decoding Livox Lidar position data...",
                #     len(self.state[self.StateInfo.POSITION]),
                # )

                self.position = [
                    tuple(x)
                    for x in np.frombuffer(
                        self.state[self.StateInfo.POSITION], dtype=np.float32
                    ).reshape(-1, 3)
                ]
            else:
                ret = False
            if self.StateInfo.REFLECTIVITY in self.state:
                self.reflectivity = self.state[self.StateInfo.REFLECTIVITY]
            if self.StateInfo.TAG in self.state:
                self.tag = self.state[self.StateInfo.TAG]

            name = self.name + "/imu"
            imu_state = self.device.get_states([name])
            # log_message(imu_state)
            if (
                imu_state
                and name in imu_state
                and imu_state[name]["type"] == "imu"
            ):
                self.imu_state = imu_state[name]
                log_message(self.imu_state)

            return ret

        return False


# ============================================================================
# 点云可视化模块（懒加载）
# ============================================================================


def convert_position_to_points(
    position: list[tuple[float, float, float]],
) -> np.ndarray:
    """
    将 Lidar position 数据转换为 numpy 点云数组

    Args:
        position: 点云位置列表，每个元素为 (x, y, z) 元组

    Returns:
        numpy 数组，shape 为 (N, 3)，dtype 为 float64
    """
    if not position or len(position) == 0:
        return np.array([], dtype=np.float64).reshape(0, 3)

    return np.array(position, dtype=np.float64)


class LidarVisualizer:
    """
    Lidar 点云可视化器（懒加载 Open3D）

    该类与 Lidar 类解耦，可以独立使用或替换为其他可视化方案。
    只在实例化时才导入 Open3D，避免不必要的依赖。

    使用示例：
        # 创建可视化器（此时才加载 Open3D）
        visualizer = LidarVisualizer(
            window_name="My Lidar",
            color_mode="height"
        )

        # 可视化点云
        points = lidar.get_last_position()
        visualizer.update(points)

        # 关闭窗口
        visualizer.close()
    """

    def __init__(
        self,
        window_name: str = "Lidar Point Cloud",
        width: int = 1280,
        height: int = 720,
        point_size: float = 2.0,
        background_color: tuple[float, float, float] = (0.1, 0.1, 0.1),
        color_mode: str = "height",  # "height", "intensity", "uniform"
        show_coordinate_frame: bool = True,
        coordinate_frame_size: float = 0.5,
        color_function: Optional[Callable[[np.ndarray], np.ndarray]] = None,
    ):
        """
        初始化可视化器（懒加载 Open3D）

        Args:
            window_name: 窗口名称
            width: 窗口宽度
            height: 窗口高度
            point_size: 点大小
            background_color: 背景颜色 RGB (0-1)
            color_mode: 着色模式 "height"(按高度), "intensity"(按强度), "uniform"(统一颜色)
            show_coordinate_frame: 是否显示坐标系
            coordinate_frame_size: 坐标系大小
            color_function: 自定义着色函数，接收 points (N,3) 返回 colors (N,3)
        """
        # 懒加载 Open3D
        try:
            import open3d as o3d

            self.o3d = o3d
        except ImportError:
            raise ImportError(
                "Open3D is required for visualization. "
                "Install it with: pip install open3d"
            )

        self.window_name = window_name
        self.width = width
        self.height = height
        self.point_size = point_size
        self.background_color = np.array(background_color)
        self.color_mode = color_mode
        self.show_coordinate_frame = show_coordinate_frame
        self.coordinate_frame_size = coordinate_frame_size
        self.color_function = color_function

        # 可视化对象
        self.vis = None
        self.pcd = None
        self.coordinate_frame = None
        self.initialized = False

    def _initialize_window(self, points: np.ndarray):
        """初始化可视化窗口"""
        log_message(f"Initializing Open3D visualization window...")
        log_message(f"Initial points count: {len(points)}")

        # 创建可视化器
        self.vis = self.o3d.visualization.Visualizer()
        self.vis.create_window(
            window_name=self.window_name, width=self.width, height=self.height
        )

        # 创建点云对象
        self.pcd = self.o3d.geometry.PointCloud()
        self.pcd.points = self.o3d.utility.Vector3dVector(points)

        # 设置颜色
        colors = self._compute_colors(points)
        if colors is not None:
            self.pcd.colors = self.o3d.utility.Vector3dVector(colors)

        # 添加点云到可视化器
        self.vis.add_geometry(self.pcd)

        # 添加坐标系
        if self.show_coordinate_frame:
            self.coordinate_frame = (
                self.o3d.geometry.TriangleMesh.create_coordinate_frame(
                    size=self.coordinate_frame_size, origin=[0, 0, 0]
                )
            )
            self.vis.add_geometry(self.coordinate_frame)

        # 设置渲染选项
        render_option = self.vis.get_render_option()
        render_option.point_size = self.point_size
        render_option.background_color = self.background_color

        # 设置视点控制，增加可视距离
        view_control = self.vis.get_view_control()
        # 增加远裁剪面距离，防止远处物体消失
        view_control.set_constant_z_far(50000.0)
        # 设置近裁剪面
        view_control.set_constant_z_near(0.1)

        self.initialized = True
        log_message("Visualization window created")
        log_message("Controls: Mouse left=Rotate, Wheel=Zoom, Right=Pan, Q=Quit")

    def _compute_colors(self, points: np.ndarray) -> Optional[np.ndarray]:
        """
        计算点云颜色

        Args:
            points: 点云数组 (N, 3)

        Returns:
            颜色数组 (N, 3)，RGB 值范围 0-1
        """
        if len(points) == 0:
            return None

        # 如果提供了自定义着色函数，使用它
        if self.color_function is not None:
            return self.color_function(points)

        # 按高度着色（默认）
        if self.color_mode == "height":
            z_values = points[:, 2]
            z_min, z_max = z_values.min(), z_values.max()

            if z_max - z_min > 1e-6:
                z_normalized = (z_values - z_min) / (z_max - z_min)
            else:
                z_normalized = np.zeros(len(points))

            colors = np.zeros((len(points), 3))
            colors[:, 0] = z_normalized  # R: 低处蓝，高处红
            colors[:, 1] = 1 - z_normalized  # G: 低处绿，高处少绿
            colors[:, 2] = 0.5  # B: 中等蓝色
            return colors

        # 统一颜色
        elif self.color_mode == "uniform":
            colors = np.ones((len(points), 3)) * 0.5
            return colors

        # 其他模式返回 None（使用默认白色）
        return None

    def update(self, position: list[tuple[float, float, float]]) -> bool:
        """
        更新点云显示

        Args:
            position: Lidar 返回的点云位置列表

        Returns:
            是否成功更新（如果窗口已关闭返回 False）
        """
        # 转换为 numpy 数组
        points = convert_position_to_points(position)

        if len(points) == 0:
            log_message("No point cloud data available")
            return True

        # 第一次渲染，初始化窗口
        if not self.initialized:
            self._initialize_window(points)
        else:
            # 更新点云数据
            self.pcd.points = self.o3d.utility.Vector3dVector(points)

            # 更新颜色
            colors = self._compute_colors(points)
            if colors is not None:
                self.pcd.colors = self.o3d.utility.Vector3dVector(colors)

            # 更新几何体
            self.vis.update_geometry(self.pcd)

        # 轮询事件并更新渲染
        self.vis.poll_events()
        self.vis.update_renderer()

        return True

    def is_window_open(self) -> bool:
        """检查窗口是否仍然打开"""
        if not self.initialized or self.vis is None:
            return False
        # Open3D 没有直接的 API 检查窗口状态，这里简单返回初始化状态
        return True

    def close(self):
        """关闭可视化窗口"""
        if self.vis is not None:
            self.vis.destroy_window()
            self.vis = None
            self.pcd = None
            self.coordinate_frame = None
            self.initialized = False
            log_message("Visualization window closed")


# 便捷 API：创建默认可视化器
def create_visualizer(
    window_name: str = "Lidar Point Cloud", **kwargs
) -> Optional[LidarVisualizer]:
    """
    创建默认的 Lidar 可视化器（懒加载）

    Args:
        window_name: 窗口名称
        **kwargs: 传递给 LidarVisualizer 的其他参数

    Returns:
        LidarVisualizer 实例，如果 Open3D 不可用则返回 None
    """
    try:
        return LidarVisualizer(window_name=window_name, **kwargs)
    except ImportError as e:
        log_message(f"Cannot create visualizer: {e}")
        return None


if __name__ == "__main__":
    pass
