"""
main.py
主程式入口 - VTK 嵌入版本
"""

import sys
import os

from PyQt5.QtWidgets import QApplication
# 添加專案根目錄到 Python 路徑
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

# 現在可以正常匯入了

from pyQtui.core.controller import AnimationController
from pyQtui.gui.main_window import RobotMainWindow

# ... 其他匯入
# 匯入控制器
from core.controller import AnimationController

# 匯入 GUI
from gui.main_window import RobotMainWindow


def main():
    """主程式"""
    print("=" * 60)
    print("RA605-710-GC 六軸機械手臂控制系統")
    print("=" * 60)
    print()

    # 初始化控制器
    print("正在初始化控制器...")
    ctrl = AnimationController()
    print("✓ 控制器初始化完成")
    print()

    # 創建 Qt 應用程式
    print("正在啟動系統...")
    app = QApplication(sys.argv)

    app.setApplicationName("RA605-710-GC 控制系統")
    app.setOrganizationName("Robot Control")

    # 創建並顯示主視窗（VTK 已嵌入其中）
    window = RobotMainWindow(ctrl)
    window.show()

    print()
    print("=" * 60)
    print("✓ 系統啟動完成！")
    print("=" * 60)
    print()
    print("功能列表：")
    print("  ✓ 3D 視覺化（VTK 嵌入式）")
    print("  ✓ 關節控制")
    print("  ✓ 位置控制 (FK/IK)")
    print("  ✓ 軌跡規劃")
    print("  ✓ 姿態管理")
    print("=" * 60)
    print()

    # 運行應用程式
    exit_code = app.exec_()

    print()
    print("程式已終止")
    sys.exit(exit_code)


if __name__ == "__main__":
    main()