"""
gui/main_window.py
主視窗 - 完整控制面板版本（優化關節控制介面）
"""

from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
import sys
import numpy as np

# 匯入各個模組
from core.kinematics import Kinematics
from core.trajectory import TrajectoryPlanner
from utils.preset_manager import PresetManager
from config.robot_config import JOINT_LIMITS, GUI_THEME, TRAJECTORY_CONFIG


class RobotMainWindow(QMainWindow):
    """主視窗 - 整合所有功能"""

    def __init__(self, controller):
        super().__init__()
        self.ctrl = controller

        # 初始化各功能模組
        self.kinematics = Kinematics()
        self.trajectory_planner = TrajectoryPlanner()
        self.preset_manager = PresetManager()

        # VTK Widget
        self.vtk_widget = None

        # 儲存控制元件參考
        self.joint_sliders = {}
        self.joint_labels = {}
        self.position_inputs = {}
        self.trajectory_points = []

        self.setWindowTitle("RA605-710-GC 六軸機械手臂控制系統")
        self.setGeometry(50, 50, 1800, 900)
        self.apply_theme()

        self.setup_ui()

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
                padding: 15px 10px 10px 10px;
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
                padding: 8px;
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
                height: 6px;
                background: {theme['border']};
                border-radius: 3px;
            }}
            QSlider::handle:horizontal {{
                background: {theme['primary']};
                width: 16px;
                margin: -5px 0;
                border-radius: 8px;
            }}
            QLabel#value {{
                color: {theme['primary']};
                font-size: 14px;
                font-weight: bold;
            }}
            QLabel#position {{
                color: {theme['success']};
                font-size: 18px;
                font-weight: bold;
            }}
            QTabWidget::pane {{
                border: 1px solid {theme['border']};
                border-radius: 5px;
            }}
            QTabBar::tab {{
                background: {theme['panel']};
                color: #ffffff;
                padding: 10px 15px;
                margin-right: 2px;
                border-top-left-radius: 5px;
                border-top-right-radius: 5px;
            }}
            QTabBar::tab:selected {{
                background: {theme['primary']};
            }}
            QTextEdit {{
                background-color: {theme['background']};
                border: 1px solid {theme['border']};
                border-radius: 5px;
                padding: 5px;
            }}
            QScrollArea {{
                border: none;
                background-color: transparent;
            }}
        """)

    def setup_ui(self):
        """建立使用者介面"""
        central = QWidget()
        self.setCentralWidget(central)

        main_layout = QHBoxLayout(central)
        main_layout.setSpacing(10)
        main_layout.setContentsMargins(10, 10, 10, 10)

        # 左側：側邊欄
        left_panel = self.create_sidebar()
        left_panel.setMaximumWidth(260)
        main_layout.addWidget(left_panel)

        # 中間：3D 視覺化
        visualization_panel = self.create_visualization_panel()
        main_layout.addWidget(visualization_panel, stretch=2)

        # 右側：完整控制面板
        right_panel = self.create_control_panel()
        main_layout.addWidget(right_panel, stretch=2)

        # 狀態欄
        self.statusBar().showMessage("系統就緒")
        self.statusBar().setStyleSheet(f"color: {GUI_THEME['primary']};")

    def create_visualization_panel(self):
        """創建 3D 視覺化面板"""
        widget = QGroupBox("🎥 3D 視覺化")
        widget.setMinimumWidth(500)
        widget.setMinimumHeight(600)

        layout = QVBoxLayout(widget)
        layout.setContentsMargins(5, 5, 5, 5)

        try:
            from visualization.vtk_view import VTKWidget

            self.vtk_widget = VTKWidget(self.ctrl)
            self.vtk_widget.setMinimumSize(480, 480)

            layout.addWidget(self.vtk_widget)

            print("✓ VTK 3D 視覺化已嵌入")

        except ImportError:
            placeholder = QLabel("❌ VTK 未安裝\n\n請執行: pip install vtk")
            placeholder.setAlignment(Qt.AlignCenter)
            placeholder.setStyleSheet("""
                QLabel {
                    background-color: #1a1a1a;
                    color: #e74c3c;
                    font-size: 14px;
                    border: 2px dashed #e74c3c;
                    border-radius: 8px;
                    padding: 20px;
                }
            """)
            placeholder.setMinimumHeight(400)
            layout.addWidget(placeholder)

        except Exception as e:
            error_label = QLabel(f"3D 視覺化錯誤:\n{str(e)}")
            error_label.setAlignment(Qt.AlignCenter)
            error_label.setStyleSheet("color: #e74c3c; font-size: 14px;")
            layout.addWidget(error_label)

        # 視圖控制按鈕
        control_layout = QHBoxLayout()

        reset_btn = QPushButton("🔄 重置")
        reset_btn.setMaximumWidth(70)
        reset_btn.clicked.connect(self.reset_camera_view)
        control_layout.addWidget(reset_btn)

        top_btn = QPushButton("⬆ 俯視")
        top_btn.setMaximumWidth(70)
        top_btn.clicked.connect(lambda: self.set_camera_view("top"))
        control_layout.addWidget(top_btn)

        side_btn = QPushButton("➡ 側視")
        side_btn.setMaximumWidth(70)
        side_btn.clicked.connect(lambda: self.set_camera_view("side"))
        control_layout.addWidget(side_btn)

        front_btn = QPushButton("👁 正視")
        front_btn.setMaximumWidth(70)
        front_btn.clicked.connect(lambda: self.set_camera_view("front"))
        control_layout.addWidget(front_btn)

        control_layout.addStretch()

        layout.addLayout(control_layout)

        return widget

    def reset_camera_view(self):
        """重置相機視角"""
        if self.vtk_widget:
            self.vtk_widget.reset_camera()

    def set_camera_view(self, view_type):
        """設定相機視角"""
        if self.vtk_widget:
            self.vtk_widget.set_view(view_type)

    def create_sidebar(self):
        """創建側邊欄"""
        widget = QWidget()
        layout = QVBoxLayout(widget)

        # Logo
        logo = QLabel("RA605-710-GC")
        logo.setStyleSheet(f"font-size: 18px; font-weight: bold; color: {GUI_THEME['primary']}; padding: 10px;")
        logo.setAlignment(Qt.AlignCenter)
        layout.addWidget(logo)

        # 系統狀態
        status_group = QGroupBox("系統狀態")
        status_layout = QVBoxLayout()

        self.status_indicator = QLabel("● 運行中")
        self.status_indicator.setStyleSheet(f"color: {GUI_THEME['success']}; font-size: 14px;")
        status_layout.addWidget(self.status_indicator)

        status_group.setLayout(status_layout)
        layout.addWidget(status_group)

        # 快速姿態
        preset_group = QGroupBox("快速姿態")
        preset_layout = QVBoxLayout()

        self.preset_list = QListWidget()
        self.preset_list.addItems(self.preset_manager.get_all_preset_names())
        self.preset_list.itemDoubleClicked.connect(self.load_preset)
        preset_layout.addWidget(self.preset_list)

        # 姿態按鈕
        preset_btn_layout = QHBoxLayout()

        save_btn = QPushButton("💾")
        save_btn.setToolTip("儲存當前姿態")
        save_btn.setMaximumWidth(40)
        save_btn.clicked.connect(self.save_current_preset)
        preset_btn_layout.addWidget(save_btn)

        delete_btn = QPushButton("🗑️")
        delete_btn.setToolTip("刪除選中姿態")
        delete_btn.setMaximumWidth(40)
        delete_btn.clicked.connect(self.delete_preset)
        preset_btn_layout.addWidget(delete_btn)

        preset_layout.addLayout(preset_btn_layout)
        preset_group.setLayout(preset_layout)
        layout.addWidget(preset_group)

        # 緊急停止
        emergency_btn = QPushButton("⏹ 緊急停止")
        emergency_btn.setObjectName("emergency")
        emergency_btn.setMinimumHeight(50)
        emergency_btn.clicked.connect(self.emergency_stop)
        layout.addWidget(emergency_btn)

        layout.addStretch()
        return widget

    def create_control_panel(self):
        """創建完整控制面板"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(0, 0, 0, 0)

        # 標籤頁
        tabs = QTabWidget()

        # 1. 關節控制
        tabs.addTab(self.create_joint_control_tab(), "🎮 關節控制")

        # 2. 位置控制
        tabs.addTab(self.create_position_control_tab(), "📍 位置控制")

        # 3. 軌跡規劃
        tabs.addTab(self.create_trajectory_tab(), "🔄 軌跡規劃")

        # 4. 監控面板
        tabs.addTab(self.create_monitor_tab(), "📊 監控")

        layout.addWidget(tabs)

        return widget

    def create_joint_control_tab(self):
        """創建關節控制標籤頁 - 簡潔版本"""
        widget = QWidget()
        layout = QVBoxLayout()
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(5)

        # 關節滑桿區域 - 使用滾動區域
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        scroll.setStyleSheet("QScrollArea { border: none; }")

        scroll_widget = QWidget()
        scroll_layout = QVBoxLayout(scroll_widget)
        scroll_layout.setContentsMargins(5, 5, 5, 5)
        scroll_layout.setSpacing(4)

        for i in range(1, 7):
            joint_group = QGroupBox(f"關節 {i}")
            joint_layout = QVBoxLayout()
            joint_layout.setContentsMargins(10, 8, 10, 6)
            joint_layout.setSpacing(3)

            # 當前值顯示
            value_layout = QHBoxLayout()
            value_layout.setSpacing(8)

            label = QLabel(f"J{i}:")
            label.setMinimumWidth(25)
            label.setStyleSheet("font-weight: bold; font-size: 13px;")
            value_layout.addWidget(label)

            value_label = QLabel("0.0°")
            value_label.setObjectName("value")
            value_label.setMinimumWidth(70)
            value_label.setStyleSheet("font-size: 16px; font-weight: bold; color: #3498db;")
            self.joint_labels[i] = value_label
            value_layout.addWidget(value_label)

            value_layout.addStretch()
            joint_layout.addLayout(value_layout)

            # 滑桿 + 範圍值
            slider_layout = QHBoxLayout()
            slider_layout.setSpacing(8)

            # 最小值標籤
            min_val, max_val = JOINT_LIMITS[f'J{i}']
            min_label = QLabel(f"{min_val}°")
            min_label.setStyleSheet("color: #7f8c8d; font-size: 10px;")
            min_label.setMinimumWidth(40)
            min_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            slider_layout.addWidget(min_label)

            # 滑桿
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(int(min_val))
            slider.setMaximum(int(max_val))
            slider.setValue(0)
            slider.setMinimumHeight(30)
            slider.valueChanged.connect(lambda v, idx=i: self.on_joint_slider_changed(idx, v))
            self.joint_sliders[i] = slider
            slider_layout.addWidget(slider)

            # 最大值標籤
            max_label = QLabel(f"{max_val}°")
            max_label.setStyleSheet("color: #7f8c8d; font-size: 10px;")
            max_label.setMinimumWidth(40)
            max_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            slider_layout.addWidget(max_label)

            joint_layout.addLayout(slider_layout)

            joint_group.setLayout(joint_layout)
            joint_group.setMaximumHeight(95)  # 更緊湊的高度
            scroll_layout.addWidget(joint_group)

        scroll_layout.addStretch()
        scroll.setWidget(scroll_widget)
        layout.addWidget(scroll)

        # 控制按鈕
        btn_layout = QHBoxLayout()
        btn_layout.setSpacing(8)

        home_btn = QPushButton("🏠 回原點")
        home_btn.setMinimumHeight(38)
        home_btn.clicked.connect(self.go_home)
        btn_layout.addWidget(home_btn)

        zero_btn = QPushButton("0️⃣ 全部歸零")
        zero_btn.setMinimumHeight(38)
        zero_btn.clicked.connect(self.zero_all_joints)
        btn_layout.addWidget(zero_btn)

        layout.addLayout(btn_layout)

        widget.setLayout(layout)
        return widget

    def create_position_control_tab(self):
        """創建位置控制標籤頁"""
        widget = QWidget()
        layout = QVBoxLayout()

        # 當前位置顯示
        current_group = QGroupBox("當前末端位置")
        current_layout = QGridLayout()

        self.current_pos_labels = {}
        positions = ['X', 'Y', 'Z', 'Rx', 'Ry', 'Rz']
        units = ['mm', 'mm', 'mm', '°', '°', '°']

        for i, (pos, unit) in enumerate(zip(positions, units)):
            label = QLabel(f"{pos}:")
            value = QLabel(f"0.0 {unit}")
            value.setObjectName("position")
            self.current_pos_labels[pos] = value

            row = i // 3
            col = (i % 3) * 2
            current_layout.addWidget(label, row, col)
            current_layout.addWidget(value, row, col + 1)

        current_group.setLayout(current_layout)
        layout.addWidget(current_group)

        # 目標位置輸入
        target_group = QGroupBox("目標位置")
        target_layout = QGridLayout()

        for i, (pos, unit) in enumerate(zip(positions, units)):
            label = QLabel(f"{pos}:")
            spin = QDoubleSpinBox()

            if unit == 'mm':
                spin.setRange(-1000, 1000)
                spin.setSingleStep(10)
                spin.setSuffix(" mm")
            else:
                spin.setRange(-180, 180)
                spin.setSingleStep(1)
                spin.setSuffix("°")

            self.position_inputs[pos] = spin

            row = i // 3
            col = (i % 3) * 2
            target_layout.addWidget(label, row + 2, col)
            target_layout.addWidget(spin, row + 2, col + 1)

        target_group.setLayout(target_layout)
        layout.addWidget(target_group)

        # 移動控制
        move_group = QGroupBox("移動控制")
        move_layout = QVBoxLayout()

        # 速度設定
        speed_layout = QHBoxLayout()
        speed_layout.addWidget(QLabel("速度:"))

        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(1, 100)
        self.speed_slider.setValue(50)
        speed_layout.addWidget(self.speed_slider)

        self.speed_label = QLabel("50%")
        self.speed_label.setMinimumWidth(40)
        self.speed_slider.valueChanged.connect(lambda v: self.speed_label.setText(f"{v}%"))
        speed_layout.addWidget(self.speed_label)

        move_layout.addLayout(speed_layout)

        # 移動按鈕
        btn_layout = QHBoxLayout()

        move_btn = QPushButton("▶ 移動到目標")
        move_btn.clicked.connect(self.move_to_position)
        btn_layout.addWidget(move_btn)

        linear_btn = QPushButton("📐 直線移動")
        linear_btn.clicked.connect(self.linear_move)
        btn_layout.addWidget(linear_btn)

        move_layout.addLayout(btn_layout)

        move_group.setLayout(move_layout)
        layout.addWidget(move_group)

        layout.addStretch()
        widget.setLayout(layout)
        return widget

    def create_trajectory_tab(self):
        """創建軌跡規劃標籤頁"""
        widget = QWidget()
        layout = QVBoxLayout()

        # 軌跡點列表
        points_group = QGroupBox("軌跡點")
        points_layout = QVBoxLayout()

        self.trajectory_list = QListWidget()
        points_layout.addWidget(self.trajectory_list)

        # 軌跡點操作按鈕
        point_btn_layout = QHBoxLayout()

        add_btn = QPushButton("➕ 新增當前點")
        add_btn.clicked.connect(self.add_trajectory_point)
        point_btn_layout.addWidget(add_btn)

        insert_btn = QPushButton("📝 插入")
        insert_btn.clicked.connect(self.insert_trajectory_point)
        point_btn_layout.addWidget(insert_btn)

        delete_btn = QPushButton("❌ 刪除")
        delete_btn.clicked.connect(self.delete_trajectory_point)
        point_btn_layout.addWidget(delete_btn)

        clear_btn = QPushButton("🗑️ 清空")
        clear_btn.clicked.connect(self.clear_trajectory)
        point_btn_layout.addWidget(clear_btn)

        points_layout.addLayout(point_btn_layout)
        points_group.setLayout(points_layout)
        layout.addWidget(points_group)

        # 軌跡參數
        params_group = QGroupBox("軌跡參數")
        params_layout = QGridLayout()

        # 運動類型
        params_layout.addWidget(QLabel("運動類型:"), 0, 0)
        self.motion_type = QComboBox()
        self.motion_type.addItems(["關節空間", "直線", "圓弧"])
        params_layout.addWidget(self.motion_type, 0, 1)

        # 速度
        params_layout.addWidget(QLabel("速度 (%):"), 1, 0)
        self.traj_speed = QSpinBox()
        self.traj_speed.setRange(1, 100)
        self.traj_speed.setValue(30)
        params_layout.addWidget(self.traj_speed, 1, 1)

        # 加速度
        params_layout.addWidget(QLabel("加速度 (%):"), 2, 0)
        self.traj_accel = QSpinBox()
        self.traj_accel.setRange(1, 100)
        self.traj_accel.setValue(20)
        params_layout.addWidget(self.traj_accel, 2, 1)

        # 混合半徑
        params_layout.addWidget(QLabel("混合半徑 (mm):"), 3, 0)
        self.blend_radius = QSpinBox()
        self.blend_radius.setRange(0, 100)
        self.blend_radius.setValue(10)
        params_layout.addWidget(self.blend_radius, 3, 1)

        params_group.setLayout(params_layout)
        layout.addWidget(params_group)

        # 執行控制
        exec_group = QGroupBox("執行控制")
        exec_layout = QVBoxLayout()

        # 執行按鈕
        exec_btn_layout = QHBoxLayout()

        run_btn = QPushButton("▶ 執行軌跡")
        run_btn.clicked.connect(self.run_trajectory)
        exec_btn_layout.addWidget(run_btn)

        pause_btn = QPushButton("⏸ 暫停")
        pause_btn.clicked.connect(self.pause_trajectory)
        exec_btn_layout.addWidget(pause_btn)

        stop_btn = QPushButton("⏹ 停止")
        stop_btn.clicked.connect(self.stop_trajectory)
        exec_btn_layout.addWidget(stop_btn)

        exec_layout.addLayout(exec_btn_layout)

        # 循環選項
        loop_layout = QHBoxLayout()
        self.loop_checkbox = QCheckBox("循環執行")
        loop_layout.addWidget(self.loop_checkbox)

        loop_layout.addWidget(QLabel("次數:"))
        self.loop_count = QSpinBox()
        self.loop_count.setRange(1, 999)
        self.loop_count.setValue(1)
        loop_layout.addWidget(self.loop_count)

        loop_layout.addStretch()
        exec_layout.addLayout(loop_layout)

        exec_group.setLayout(exec_layout)
        layout.addWidget(exec_group)

        layout.addStretch()
        widget.setLayout(layout)
        return widget

    def create_monitor_tab(self):
        """創建監控標籤頁"""
        widget = QWidget()
        layout = QVBoxLayout()

        # 系統資訊
        info_group = QGroupBox("系統資訊")
        info_layout = QGridLayout()

        self.monitor_labels = {}

        monitors = [
            ("連線狀態", "已連線", 0, 0),
            ("運行時間", "00:00:00", 0, 2),
            ("CPU 使用率", "0%", 1, 0),
            ("記憶體使用", "0 MB", 1, 2),
            ("目前速度", "0 mm/s", 2, 0),
            ("目前加速度", "0 mm/s²", 2, 2),
        ]

        for name, default, row, col in monitors:
            label = QLabel(f"{name}:")
            value = QLabel(default)
            value.setObjectName("value")
            self.monitor_labels[name] = value
            info_layout.addWidget(label, row, col)
            info_layout.addWidget(value, row, col + 1)

        info_group.setLayout(info_layout)
        layout.addWidget(info_group)

        # 錯誤日誌
        log_group = QGroupBox("系統日誌")
        log_layout = QVBoxLayout()

        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(200)
        log_layout.addWidget(self.log_text)

        # 日誌控制
        log_btn_layout = QHBoxLayout()

        clear_log_btn = QPushButton("清空日誌")
        clear_log_btn.clicked.connect(self.clear_log)
        log_btn_layout.addWidget(clear_log_btn)

        export_log_btn = QPushButton("匯出日誌")
        export_log_btn.clicked.connect(self.export_log)
        log_btn_layout.addWidget(export_log_btn)

        log_btn_layout.addStretch()
        log_layout.addLayout(log_btn_layout)

        log_group.setLayout(log_layout)
        layout.addWidget(log_group)

        # 警告區域
        warning_group = QGroupBox("警告")
        warning_layout = QVBoxLayout()

        self.warning_list = QListWidget()
        self.warning_list.setMaximumHeight(100)
        warning_layout.addWidget(self.warning_list)

        warning_group.setLayout(warning_layout)
        layout.addWidget(warning_group)

        layout.addStretch()
        widget.setLayout(layout)
        return widget

    # ========== 事件處理函數 ==========

    def on_joint_slider_changed(self, joint_idx, value):
        """關節滑桿值改變"""
        self.joint_labels[joint_idx].setText(f"{value}°")
        angles = [self.joint_sliders[i].value() for i in range(1, 7)]
        self.ctrl.set_joint_angles(angles)
        self.update_position_display()

    def fine_adjust_joint(self, joint_idx, delta):
        """微調關節角度"""
        slider = self.joint_sliders[joint_idx]
        new_value = slider.value() + delta
        min_val, max_val = JOINT_LIMITS[f'J{joint_idx}']
        new_value = max(min_val, min(max_val, new_value))
        slider.setValue(new_value)

    def go_home(self):
        """回到原點位置"""
        home_angles = [0, 0, 0, 0, 0, 0]
        self.ctrl.move_to_home()
        for i, angle in enumerate(home_angles, 1):
            self.joint_sliders[i].setValue(angle)

    def zero_all_joints(self):
        """所有關節歸零"""
        for i in range(1, 7):
            self.joint_sliders[i].setValue(0)

    def move_to_position(self):
        """移動到目標位置"""
        target = {
            'x': self.position_inputs['X'].value(),
            'y': self.position_inputs['Y'].value(),
            'z': self.position_inputs['Z'].value(),
            'rx': self.position_inputs['Rx'].value(),
            'ry': self.position_inputs['Ry'].value(),
            'rz': self.position_inputs['Rz'].value()
        }
        speed = self.speed_slider.value() / 100.0

        success = self.ctrl.move_to_position(target, speed)
        if success:
            self.add_log(f"移動到位置: {target}")
        else:
            self.add_warning("無法到達目標位置")

    def linear_move(self):
        """直線移動"""
        target = {
            'x': self.position_inputs['X'].value(),
            'y': self.position_inputs['Y'].value(),
            'z': self.position_inputs['Z'].value(),
            'rx': self.position_inputs['Rx'].value(),
            'ry': self.position_inputs['Ry'].value(),
            'rz': self.position_inputs['Rz'].value()
        }
        speed = self.speed_slider.value() / 100.0

        success = self.ctrl.linear_move(target, speed)
        if success:
            self.add_log(f"直線移動到: {target}")
        else:
            self.add_warning("直線路徑規劃失敗")

    def add_trajectory_point(self):
        """新增軌跡點"""
        current_pos = self.ctrl.get_current_position()
        if current_pos:
            point_str = f"P{len(self.trajectory_points)+1}: "
            point_str += f"X={current_pos['x']:.1f}, Y={current_pos['y']:.1f}, Z={current_pos['z']:.1f}"

            self.trajectory_points.append(current_pos)
            self.trajectory_list.addItem(point_str)
            self.add_log(f"新增軌跡點 {len(self.trajectory_points)}")

    def insert_trajectory_point(self):
        """插入軌跡點"""
        current_row = self.trajectory_list.currentRow()
        if current_row >= 0:
            current_pos = self.ctrl.get_current_position()
            if current_pos:
                self.trajectory_points.insert(current_row, current_pos)
                self.refresh_trajectory_list()
                self.add_log(f"插入軌跡點於位置 {current_row+1}")

    def delete_trajectory_point(self):
        """刪除選中的軌跡點"""
        current_row = self.trajectory_list.currentRow()
        if current_row >= 0:
            self.trajectory_points.pop(current_row)
            self.refresh_trajectory_list()
            self.add_log(f"刪除軌跡點 {current_row+1}")

    def clear_trajectory(self):
        """清空所有軌跡點"""
        reply = QMessageBox.question(self, '確認', '確定要清空所有軌跡點嗎？',
                                    QMessageBox.Yes | QMessageBox.No)
        if reply == QMessageBox.Yes:
            self.trajectory_points.clear()
            self.trajectory_list.clear()
            self.add_log("清空所有軌跡點")

    def refresh_trajectory_list(self):
        """刷新軌跡點列表顯示"""
        self.trajectory_list.clear()
        for i, point in enumerate(self.trajectory_points):
            point_str = f"P{i+1}: "
            point_str += f"X={point['x']:.1f}, Y={point['y']:.1f}, Z={point['z']:.1f}"
            self.trajectory_list.addItem(point_str)

    def run_trajectory(self):
        """執行軌跡"""
        if not self.trajectory_points:
            self.add_warning("沒有軌跡點")
            return

        params = {
            'motion_type': self.motion_type.currentText(),
            'speed': self.traj_speed.value() / 100.0,
            'acceleration': self.traj_accel.value() / 100.0,
            'blend_radius': self.blend_radius.value(),
            'loop': self.loop_checkbox.isChecked(),
            'loop_count': self.loop_count.value()
        }

        success = self.ctrl.execute_trajectory(self.trajectory_points, params)
        if success:
            self.add_log("開始執行軌跡")
            self.status_indicator.setText("● 執行中")
            self.status_indicator.setStyleSheet(f"color: {GUI_THEME['warning']}; font-size: 14px;")
        else:
            self.add_warning("軌跡執行失敗")

    def pause_trajectory(self):
        """暫停軌跡執行"""
        self.ctrl.pause_motion()
        self.add_log("軌跡暫停")
        self.status_indicator.setText("● 暫停")
        self.status_indicator.setStyleSheet(f"color: {GUI_THEME['warning']}; font-size: 14px;")

    def stop_trajectory(self):
        """停止軌跡執行"""
        self.ctrl.stop_motion()
        self.add_log("軌跡停止")
        self.status_indicator.setText("● 已停止")
        self.status_indicator.setStyleSheet(f"color: {GUI_THEME['danger']}; font-size: 14px;")

    def load_preset(self, item):
        """載入預設姿態"""
        preset_name = item.text()
        preset_data = self.preset_manager.load_preset(preset_name)

        if preset_data and 'joint_angles' in preset_data:
            angles = preset_data['joint_angles']
            for i, angle in enumerate(angles, 1):
                if i <= 6:
                    self.joint_sliders[i].setValue(int(angle))
            self.add_log(f"載入預設姿態: {preset_name}")

    def save_current_preset(self):
        """儲存當前姿態為預設"""
        name, ok = QInputDialog.getText(self, '儲存預設姿態', '請輸入預設名稱:')
        if ok and name:
            angles = [self.joint_sliders[i].value() for i in range(1, 7)]
            position = self.ctrl.get_current_position()

            preset_data = {
                'joint_angles': angles,
                'position': position,
                'timestamp': QDateTime.currentDateTime().toString()
            }

            self.preset_manager.save_preset(name, preset_data)
            self.preset_list.addItem(name)
            self.add_log(f"儲存預設姿態: {name}")

    def delete_preset(self):
        """刪除選中的預設姿態"""
        current_item = self.preset_list.currentItem()
        if current_item:
            preset_name = current_item.text()
            reply = QMessageBox.question(self, '確認刪除',
                                        f'確定要刪除預設姿態 "{preset_name}" 嗎？',
                                        QMessageBox.Yes | QMessageBox.No)
            if reply == QMessageBox.Yes:
                self.preset_manager.delete_preset(preset_name)
                self.preset_list.takeItem(self.preset_list.currentRow())
                self.add_log(f"刪除預設姿態: {preset_name}")

    def emergency_stop(self):
        """緊急停止"""
        self.ctrl.emergency_stop()
        self.add_warning("緊急停止已觸發！")
        self.status_indicator.setText("● 緊急停止")
        self.status_indicator.setStyleSheet(f"color: {GUI_THEME['danger']}; font-size: 14px;")

        QMessageBox.critical(self, "緊急停止",
                           "機器人已緊急停止！\n請檢查系統後重新啟動。")

    def update_display(self):
        """定時更新顯示"""
        self.update_position_display()
        self.update_monitor_info()

        if self.vtk_widget:
            angles = [self.joint_sliders[i].value() for i in range(1, 7)]
            self.vtk_widget.update_robot_pose(angles)

    def update_position_display(self):
        """更新位置顯示"""
        current_pos = self.ctrl.get_current_position()
        if current_pos:
            self.current_pos_labels['X'].setText(f"{current_pos['x']:.1f} mm")
            self.current_pos_labels['Y'].setText(f"{current_pos['y']:.1f} mm")
            self.current_pos_labels['Z'].setText(f"{current_pos['z']:.1f} mm")
            self.current_pos_labels['Rx'].setText(f"{current_pos['rx']:.1f}°")
            self.current_pos_labels['Ry'].setText(f"{current_pos['ry']:.1f}°")
            self.current_pos_labels['Rz'].setText(f"{current_pos['rz']:.1f}°")

    def update_monitor_info(self):
        """更新監控資訊"""
        status = self.ctrl.get_system_status()
        if status:
            self.monitor_labels["連線狀態"].setText(
                "已連線" if status.get('connected', False) else "未連線"
            )

            uptime = status.get('uptime', 0)
            hours = int(uptime // 3600)
            minutes = int((uptime % 3600) // 60)
            seconds = int(uptime % 60)
            self.monitor_labels["運行時間"].setText(f"{hours:02d}:{minutes:02d}:{seconds:02d}")

            self.monitor_labels["CPU 使用率"].setText(f"{status.get('cpu_usage', 0):.1f}%")
            self.monitor_labels["記憶體使用"].setText(f"{status.get('memory_usage', 0):.0f} MB")
            self.monitor_labels["目前速度"].setText(f"{status.get('current_speed', 0):.1f} mm/s")
            self.monitor_labels["目前加速度"].setText(f"{status.get('current_accel', 0):.1f} mm/s²")

    def add_log(self, message):
        """添加日誌訊息"""
        timestamp = QDateTime.currentDateTime().toString("HH:mm:ss")
        self.log_text.append(f"[{timestamp}] {message}")

        scrollbar = self.log_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def add_warning(self, message):
        """添加警告訊息"""
        timestamp = QDateTime.currentDateTime().toString("HH:mm:ss")
        warning_msg = f"[{timestamp}] ⚠ {message}"
        self.warning_list.addItem(warning_msg)

        self.add_log(f"⚠ 警告: {message}")

        while self.warning_list.count() > 10:
            self.warning_list.takeItem(0)

    def clear_log(self):
        """清空日誌"""
        reply = QMessageBox.question(self, '確認', '確定要清空日誌嗎？',
                                    QMessageBox.Yes | QMessageBox.No)
        if reply == QMessageBox.Yes:
            self.log_text.clear()
            self.add_log("日誌已清空")

    def export_log(self):
        """匯出日誌"""
        filename, _ = QFileDialog.getSaveFileName(
            self, "匯出日誌",
            f"robot_log_{QDateTime.currentDateTime().toString('yyyyMMdd_HHmmss')}.txt",
            "Text Files (*.txt)"
        )

        if filename:
            try:
                with open(filename, 'w', encoding='utf-8') as f:
                    f.write(self.log_text.toPlainText())
                self.add_log(f"日誌已匯出到: {filename}")
                QMessageBox.information(self, "成功", "日誌匯出成功！")
            except Exception as e:
                QMessageBox.critical(self, "錯誤", f"匯出失敗: {str(e)}")

    def closeEvent(self, event):
        """關閉視窗事件"""
        reply = QMessageBox.question(
            self, '確認退出',
            '確定要關閉機器人控制系統嗎？',
            QMessageBox.Yes | QMessageBox.No
        )

        if reply == QMessageBox.Yes:
            self.ctrl.stop_motion()
            self.update_timer.stop()

            if self.vtk_widget:
                self.vtk_widget.close()

            event.accept()
        else:
            event.ignore()