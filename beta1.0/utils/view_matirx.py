import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def view_matrix(mat: np.ndarray, mode: str = "table"):
    """
    快速查看大矩阵
    Parameters
    ----------
    mat : np.ndarray
        需要查看的矩阵
    mode : str
        "table" - 弹出 PandasGUI 风格可滚动表格
        "heatmap" - 显示矩阵热图
    """
    if mode == "table":
        try:
            from pandasgui import show
        except ImportError:
            print("请先安装 pandasgui: pip install pandasgui")
            return
        df = pd.DataFrame(mat)
        show(df)
    elif mode == "heatmap":
        plt.figure(figsize=(min(12, mat.shape[1]//2), min(6, mat.shape[0]//2)))
        plt.imshow(mat, cmap="viridis", aspect="auto")
        plt.colorbar()
        plt.title("Matrix Heatmap")
        plt.xlabel("Column")
        plt.ylabel("Row")
        plt.show()
    else:
        print("mode 参数只能是 'table' 或 'heatmap'")

if __name__ == "__main__":
    # 假设你的雅可比矩阵
    jac = np.random.randn(6, 32)

    # 方式1：表格查看（可滚动）
    view_matrix(jac, mode="table")

    # 方式2：热图查看（直观显示大小）
    view_matrix(jac, mode="heatmap")

