"""
gui/main_window.py
主視窗 - 嵌入 Open3D 視覺化
"""

from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
import sys

# 匯入 Open3D GUI
import open3d.visualization.gui as gui

# 匯入各個模組
from core.kinematics import Kinematics
from core.trajectory import TrajectoryPlanner
from utils.preset_manager import PresetManager
from config.robot_config import JOINT_LIMITS, GUI_THEME, TRAJECTORY_CONFIG
from visualization.open3d_view import Open3DWidget


class RobotMainWindow(QMainWindow):
    """主視窗 - 整合所有功能（含嵌入式 3D 視覺化）"""

    def __init__(self, controller):
        super().__init__()
        self.ctrl = controller

        # 初始化各功能模組
        self.kinematics = Kinematics()
        self.trajectory_planner = TrajectoryPlanner()
        self.preset_manager = PresetManager()

        # Open3D Widget
        self.open3d_widget = None

        self.setWindowTitle("RA605-710-GC 六軸機械手臂控制系統")
        self.setGeometry(50, 50, 1600, 900)  # 加大視窗以容納 3D 視圖
        self.apply_theme()

        self.setup_ui()
        self.setup_signals()

        # 定時更新
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_display)
        self.update_timer.start(50)

        print("✓ 主視窗初始化完成")

    def apply_theme(self):
        """應用主題"""
        theme = GUI_THEME
        self.setStyleSheet(f"""
            QMainWindow {{
                background-color: {theme['background']};
            }}
            QWidget {{
                background-color: {theme['panel']};
                color: #ffffff;
                font-family: 'Microsoft JhengHei', 'Segoe UI', Arial;
                font-size: 12px;
            }}
            QGroupBox {{
                border: 2px solid {theme['border']};
                border-radius: 8px;
                margin-top: 10px;
                padding: 15px;
                font-weight: bold;
            }}
            QGroupBox::title {{
                color: {theme['primary']};
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }}
            QPushButton {{
                background-color: #0d7377;
                border: none;
                border-radius: 5px;
                padding: 10px;
                color: white;
                font-weight: bold;
            }}
            QPushButton:hover {{
                background-color: #14a085;
            }}
            QPushButton#emergency {{
                background-color: {theme['danger']};
            }}
            QPushButton#emergency:hover {{
                background-color: #c0392b;
            }}
            QSlider::groove:horizontal {{
                height: 8px;
                background: {theme['border']};
                border-radius: 4px;
            }}
            QSlider::handle:horizontal {{
                background: {theme['primary']};
                width: 18px;
                margin: -5px 0;
                border-radius: 9px;
            }}
            QLabel#value {{
                color: {theme['primary']};
                font-size: 16px;
                font-weight: bold;
            }}
            QTabWidget::pane {{
                border: 1px solid {theme['border']};
                border-radius: 5px;
            }}
            QTabBar::tab {{
                background: {theme['panel']};
                color: #ffffff;
                padding: 10px 20px;
                margin-right: 2px;
                border-top-left-radius: 5px;
                border-top-right-radius: 5px;
            }}
            QTabBar::tab:selected {{
                background: {theme['primary']};
            }}
        """)

    def setup_ui(self):
        """建立使用者介面"""
        central = QWidget()
        self.setCentralWidget(central)

        main_layout = QHBoxLayout(central)
        main_layout.setSpacing(10)
        main_layout.setContentsMargins(10, 10, 10, 10)

        # 左側：3D 視覺化區域（新增！）
        left_panel = self.create_visualization_panel()
        main_layout.addWidget(left_panel, stretch=3)

        # 中間：功能選單
        middle_panel = self.create_sidebar()
        main_layout.addWidget(middle_panel)

        # 右側：標籤頁（各功能面板）
        right_panel = self.create_tab_panel()
        main_layout.addWidget(right_panel, stretch=2)

        # 狀態欄
        self.statusBar().showMessage("系統就緒")
        self.statusBar().setStyleSheet(f"color: {GUI_THEME['primary']};")

    def create_visualization_panel(self):
        """創建 3D 視覺化面板"""
        widget = QGroupBox("3D 視覺化")
        layout = QVBoxLayout(widget)

        # 創建一個容器用於 Open3D Widget
        container = QWidget()
        container_layout = QVBoxLayout(container)
        container_layout.setContentsMargins(0, 0, 0, 0)

        try:
            # 初始化 Open3D Application（必須在創建 SceneWidget 之前）
            if not gui.Application.instance:
                gui.Application.instance.initialize()

            # 創建 Open3D Widget
            self.open3d_widget = Open3DWidget(self.ctrl)

            # 創建 Open3D 的 Qt Widget 包裝
            from PyQt5.QtWidgets import QWidget

            # 這裡需要使用 Open3D 的 GUI 系統
            # 注意：Open3D 的新版本可能需要不同的整合方式

            # 創建一個佔位符（如果直接嵌入有問題）
            placeholder = QLabel("3D 視覺化區域\n(Open3D 整合中...)")
            placeholder.setAlignment(Qt.AlignCenter)
            placeholder.setStyleSheet("""
                QLabel {
                    background-color: #1a1a1a;
                    color: #666666;
                    font-size: 16px;
                    border: 2px dashed #333333;
                    border-radius: 8px;
                    padding: 20px;
                }
            """)
            placeholder.setMinimumHeight(500)

            container_layout.addWidget(placeholder)

            print("⚠ Open3D 直接嵌入可能需要額外配置")
            print("  建議使用獨立視窗模式（方案 2）")

        except Exception as e:
            print(f"⚠ Open3D Widget 創建失敗: {e}")
            error_label = QLabel(f"3D 視覺化載入失敗\n{str(e)}")
            error_label.setAlignment(Qt.AlignCenter)
            error_label.setStyleSheet("color: #e74c3c;")
            container_layout.addWidget(error_label)

        layout.addWidget(container)

        # 視圖控制按鈕
        control_layout = QHBoxLayout()

        reset_view_btn = QPushButton("🔄 重置視角")
        reset_view_btn.clicked.connect(self.reset_camera_view)
        control_layout.addWidget(reset_view_btn)

        top_view_btn = QPushButton("⬆ 俯視圖")
        top_view_btn.clicked.connect(lambda: self.set_camera_view("top"))
        control_layout.addWidget(top_view_btn)

        side_view_btn = QPushButton("➡ 側視圖")
        side_view_btn.clicked.connect(lambda: self.set_camera_view("side"))
        control_layout.addWidget(side_view_btn)

        layout.addLayout(control_layout)

        return widget

    def reset_camera_view(self):
        """重置相機視角"""
        if self.open3d_widget and self.open3d_widget.widget:
            bounds = self.open3d_widget.scene.bounding_box
            self.open3d_widget.widget.setup_camera(60, bounds, bounds.get_center())

    def set_camera_view(self, view_type):
        """設定相機視角"""
        # 這裡可以實作不同的視角
        print(f"切換到 {view_type} 視角")

    # ... 其他方法保持不變 ...
    # (create_sidebar, create_tab_panel, 等等)

    def update_display(self):
        """定時更新顯示"""
        try:
            # 更新 Open3D 視覺化
            if self.open3d_widget:
                self.open3d_widget.update_joint5_marker()

            # ... 其他更新邏輯 ...

        except Exception as e:
            pass