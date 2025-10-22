"""
gui/main_window.py
主視窗 - 重新設計的佈局（無分頁，三欄式設計）
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
    """主視窗 - 三欄式佈局"""

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
        self.current_pos_labels = {}
        self.monitor_labels = {}
        self.trajectory_points = []

        self.setWindowTitle("RA605-710-GC 六軸機械手臂再控制系統")
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
                font-size: 16px;
                font-weight: bold;
            }}
            QScrollArea {{
                border: none;
                background-color: transparent;
            }}
            QSpinBox, QDoubleSpinBox, QComboBox {{
                background-color: #1a1a1a;
                border: 1px solid {theme['border']};
                border-radius: 4px;
                padding: 4px;
                color: white;
            }}
            QSpinBox:focus, QDoubleSpinBox:focus, QComboBox:focus {{
                border: 1px solid {theme['primary']};
            }}
        """)

    def setup_ui(self):
        """建立使用者介面 - 三欄式佈局"""
        central = QWidget()
        self.setCentralWidget(central)

        main_layout = QVBoxLayout(central)
        main_layout.setSpacing(0)
        main_layout.setContentsMargins(0, 0, 0, 0)

        # 標題欄
        title_bar = self.create_title_bar()
        main_layout.addWidget(title_bar)

        # 主要內容區域
        content_layout = QHBoxLayout()
        content_layout.setSpacing(10)
        content_layout.setContentsMargins(10, 10, 10, 10)

        # 左側面板：位置控制 + 軌跡規劃
        left_panel = self.create_left_panel()
        left_panel.setMaximumWidth(420)
        left_panel.setMinimumWidth(420)
        content_layout.addWidget(left_panel)

        # 中間面板：3D視圖 + 監控
        middle_panel = self.create_middle_panel()
        content_layout.addWidget(middle_panel, stretch=3)

        # 右側面板：關節控制
        right_panel = self.create_right_panel()
        right_panel.setMaximumWidth(380)
        right_panel.setMinimumWidth(380)
        content_layout.addWidget(right_panel)

        main_layout.addLayout(content_layout)

        # 狀態欄
        self.statusBar().showMessage("已回到原點位置")

    def zero_all_joints(self):
        """所有關節歸零"""
        for i in range(1, 7):
            self.joint_sliders[i].setValue(0)
        self.statusBar().showMessage("所有關節已歸零")

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
            self.statusBar().showMessage(f"移動到位置: X={target['x']:.1f}, Y={target['y']:.1f}, Z={target['z']:.1f}")
        else:
            self.statusBar().showMessage("⚠ 無法到達目標位置")
            QMessageBox.warning(self, "警告", "目標位置超出工作範圍或違反關節限制")

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
            self.statusBar().showMessage(f"直線移動到: X={target['x']:.1f}, Y={target['y']:.1f}, Z={target['z']:.1f}")
        else:
            self.statusBar().showMessage("⚠ 直線路徑規劃失敗")
            QMessageBox.warning(self, "警告", "直線路徑中存在無法到達的點")

    def add_trajectory_point(self):
        """新增軌跡點"""
        current_pos = self.ctrl.get_current_position()
        if current_pos:
            point_str = f"P{len(self.trajectory_points)+1}: "
            point_str += f"X={current_pos['x']:.1f}, Y={current_pos['y']:.1f}, Z={current_pos['z']:.1f}"

            self.trajectory_points.append(current_pos)
            self.trajectory_list.addItem(point_str)
            self.statusBar().showMessage(f"新增軌跡點 P{len(self.trajectory_points)}")

    def insert_trajectory_point(self):
        """插入軌跡點"""
        current_row = self.trajectory_list.currentRow()
        if current_row >= 0:
            current_pos = self.ctrl.get_current_position()
            if current_pos:
                self.trajectory_points.insert(current_row, current_pos)
                self.refresh_trajectory_list()
                self.statusBar().showMessage(f"插入軌跡點於位置 {current_row+1}")
        else:
            QMessageBox.information(self, "提示", "請先選擇要插入的位置")

    def delete_trajectory_point(self):
        """刪除選中的軌跡點"""
        current_row = self.trajectory_list.currentRow()
        if current_row >= 0:
            self.trajectory_points.pop(current_row)
            self.refresh_trajectory_list()
            self.statusBar().showMessage(f"刪除軌跡點 P{current_row+1}")
        else:
            QMessageBox.information(self, "提示", "請先選擇要刪除的軌跡點")

    def clear_trajectory(self):
        """清空所有軌跡點"""
        if not self.trajectory_points:
            return

        reply = QMessageBox.question(self, '確認', '確定要清空所有軌跡點嗎？',
                                    QMessageBox.Yes | QMessageBox.No)
        if reply == QMessageBox.Yes:
            self.trajectory_points.clear()
            self.trajectory_list.clear()
            self.statusBar().showMessage("已清空所有軌跡點")

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
            QMessageBox.warning(self, "警告", "沒有軌跡點，請先新增軌跡點")
            return

        params = {
            'motion_type': self.motion_type.currentText(),
            'speed': self.traj_speed.value() / 100.0,
            'acceleration': self.traj_accel.value() / 100.0,
            'blend_radius': self.blend_radius.value()
        }

        success = self.ctrl.execute_trajectory(self.trajectory_points, params)
        if success:
            self.statusBar().showMessage(f"開始執行軌跡 ({len(self.trajectory_points)} 個點)")
            self.status_indicator.setText("● 執行中")
            self.status_indicator.setStyleSheet(f"color: {GUI_THEME['warning']}; font-size: 14px;")
        else:
            self.statusBar().showMessage("⚠ 軌跡執行失敗")
            QMessageBox.warning(self, "警告", "軌跡執行失敗，請檢查軌跡點是否有效")

    def pause_trajectory(self):
        """暫停軌跡執行"""
        self.ctrl.pause_motion()
        self.statusBar().showMessage("軌跡已暫停")
        self.status_indicator.setText("● 暫停")
        self.status_indicator.setStyleSheet(f"color: {GUI_THEME['warning']}; font-size: 14px;")

    def stop_trajectory(self):
        """停止軌跡執行"""
        self.ctrl.stop_motion()
        self.statusBar().showMessage("軌跡已停止")
        self.status_indicator.setText("● 已停止")
        self.status_indicator.setStyleSheet(f"color: {GUI_THEME['danger']}; font-size: 14px;")

    def emergency_stop(self):
        """緊急停止"""
        self.ctrl.emergency_stop()
        self.statusBar().showMessage("⚠ 緊急停止已觸發！")
        self.status_indicator.setText("● 緊急停止")
        self.status_indicator.setStyleSheet(f"color: {GUI_THEME['danger']}; font-size: 14px;")

        QMessageBox.critical(self, "緊急停止",
                           "機器人已緊急停止！\n請檢查系統後重新啟動。")

    def reset_camera_view(self):
        """重置相機視角"""
        if self.vtk_widget:
            self.vtk_widget.reset_camera()
            self.statusBar().showMessage("相機視角已重置")

    def set_camera_view(self, view_type):
        """設定相機視角"""
        if self.vtk_widget:
            self.vtk_widget.set_view(view_type)
            view_names = {
                'top': '俯視',
                'side': '側視',
                'front': '正視'
            }
            self.statusBar().showMessage(f"切換到{view_names.get(view_type, '')}視角")

    def update_display(self):
        """定時更新顯示"""
        try:
            # 更新位置顯示
            self.update_position_display()

            # 更新監控資訊
            self.update_monitor_info()

            # 更新 VTK 視覺化
            if self.vtk_widget:
                angles = [self.joint_sliders[i].value() for i in range(1, 7)]
                self.vtk_widget.update_robot_pose(angles)

        except Exception as e:
            # 靜默處理更新錯誤，避免影響主循環
            pass

    def update_position_display(self):
        """更新位置顯示"""
        try:
            current_pos = self.ctrl.get_current_position()
            if current_pos:
                self.current_pos_labels['X'].setText(f"{current_pos['x']:.1f} mm")
                self.current_pos_labels['Y'].setText(f"{current_pos['y']:.1f} mm")
                self.current_pos_labels['Z'].setText(f"{current_pos['z']:.1f} mm")
                self.current_pos_labels['Rx'].setText(f"{current_pos['rx']:.1f}°")
                self.current_pos_labels['Ry'].setText(f"{current_pos['ry']:.1f}°")
                self.current_pos_labels['Rz'].setText(f"{current_pos['rz']:.1f}°")
        except:
            pass

    def update_monitor_info(self):
        """更新監控資訊"""
        try:
            status = self.ctrl.get_system_status()
            if status:
                # 連線狀態
                self.monitor_labels["連線狀態"].setText(
                    "已連線" if status.get('connected', False) else "未連線"
                )

                # 運行時間
                uptime = status.get('uptime', 0)
                hours = int(uptime // 3600)
                minutes = int((uptime % 3600) // 60)
                seconds = int(uptime % 60)
                self.monitor_labels["運行時間"].setText(f"{hours:02d}:{minutes:02d}:{seconds:02d}")

                # CPU 使用率
                self.monitor_labels["CPU 使用率"].setText(f"{status.get('cpu_usage', 0):.1f}%")

                # 記憶體使用
                self.monitor_labels["記憶體使用"].setText(f"{status.get('memory_usage', 0):.0f} MB")

                # 目前速度
                self.monitor_labels["目前速度"].setText(f"{status.get('current_speed', 0):.1f} mm/s")

                # 目前加速度
                self.monitor_labels["目前加速度"].setText(f"{status.get('current_accel', 0):.1f} mm/s²")
        except:
            pass

    def closeEvent(self, event):
        """關閉視窗事件"""
        reply = QMessageBox.question(
            self, '確認退出',
            '確定要關閉機器人控制系統嗎？',
            QMessageBox.Yes | QMessageBox.No
        )

        if reply == QMessageBox.Yes:
            # 停止所有運動
            try:
                self.ctrl.stop_motion()
            except:
                pass

            # 停止更新定時器
            self.update_timer.stop()

            # 關閉 VTK Widget
            if self.vtk_widget:
                try:
                    self.vtk_widget.close()
                except:
                    pass

            event.accept()
        else:
            event.ignore()


# ========== 測試程式 ==========
if __name__ == "__main__":
    from PyQt5.QtWidgets import QApplication
    from core.controller import AnimationController
    import sys

    app = QApplication(sys.argv)
    app.setStyle('Fusion')

    # 創建控制器
    ctrl = AnimationController()

    # 創建主視窗
    window = RobotMainWindow(ctrl)
    window.show()

    sys.exit(app.exec_())showMessage("系統就緒")
        self.statusBar().setStyleSheet(f"color: {GUI_THEME['primary']};")

    def create_title_bar(self):
        """創建標題欄"""
        widget = QWidget()
        widget.setStyleSheet("""
            QWidget {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #2c3e50, stop:1 #34495e);
                border-bottom: 2px solid #3498db;
            }
        """)
        widget.setFixedHeight(60)

        layout = QHBoxLayout(widget)
        layout.setContentsMargins(20, 10, 20, 10)

        # 標題
        title = QLabel("RA605-710-GC 六軸機械手臂再控制系統")
        title.setStyleSheet("font-size: 20px; font-weight: bold; color: #3498db;")
        layout.addWidget(title)

        layout.addStretch()

        # 系統狀態
        self.status_indicator = QLabel("● 運行中")
        self.status_indicator.setStyleSheet(f"color: {GUI_THEME['success']}; font-size: 14px;")
        layout.addWidget(self.status_indicator)

        # 緊急停止按鈕
        emergency_btn = QPushButton("⏹ 緊急停止")
        emergency_btn.setObjectName("emergency")
        emergency_btn.setMinimumWidth(120)
        emergency_btn.setMinimumHeight(40)
        emergency_btn.clicked.connect(self.emergency_stop)
        layout.addWidget(emergency_btn)

        return widget

    def create_left_panel(self):
        """創建左側面板：位置控制 + 軌跡規劃"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setSpacing(10)
        layout.setContentsMargins(0, 0, 0, 0)

        # 位置控制（上半部）
        position_group = self.create_position_control()
        layout.addWidget(position_group)

        # 軌跡規劃（下半部）
        trajectory_group = self.create_trajectory_control()
        layout.addWidget(trajectory_group, stretch=1)

        return widget

    def create_position_control(self):
        """創建位置控制面板"""
        group = QGroupBox("📍 位置控制")
        group.setStyleSheet("""
            QGroupBox::title {
                color: #3498db;
                font-size: 14px;
            }
        """)
        layout = QVBoxLayout()

        # 當前末端位置
        current_label = QLabel("當前末端位置")
        current_label.setStyleSheet("color: #95a5a6; font-size: 11px; margin-top: 5px;")
        layout.addWidget(current_label)

        current_grid = QGridLayout()
        current_grid.setSpacing(8)

        positions = ['X', 'Y', 'Z', 'Rx', 'Ry', 'Rz']
        units = ['mm', 'mm', 'mm', '°', '°', '°']

        for i, (pos, unit) in enumerate(zip(positions, units)):
            frame = QFrame()
            frame.setStyleSheet("background-color: #1a1a1a; border-radius: 5px; padding: 8px;")
            frame_layout = QVBoxLayout(frame)
            frame_layout.setContentsMargins(8, 6, 8, 6)
            frame_layout.setSpacing(2)

            label = QLabel(f"{pos}:")
            label.setStyleSheet("color: #7f8c8d; font-size: 10px;")
            frame_layout.addWidget(label)

            value = QLabel(f"0.0 {unit}")
            value.setObjectName("position")
            value.setStyleSheet("color: #2ecc71; font-size: 14px; font-weight: bold;")
            self.current_pos_labels[pos] = value
            frame_layout.addWidget(value)

            row = i // 3
            col = i % 3
            current_grid.addWidget(frame, row, col)

        layout.addLayout(current_grid)

        # 目標位置
        target_label = QLabel("目標位置")
        target_label.setStyleSheet("color: #95a5a6; font-size: 11px; margin-top: 10px;")
        layout.addWidget(target_label)

        target_grid = QGridLayout()
        target_grid.setSpacing(6)

        for i, (pos, unit) in enumerate(zip(positions, units)):
            label = QLabel(f"{pos}:")
            label.setStyleSheet("color: #7f8c8d; font-size: 10px;")

            if unit == 'mm':
                spin = QDoubleSpinBox()
                spin.setRange(-1000, 1000)
                spin.setSingleStep(10)
                spin.setSuffix(" mm")
            else:
                spin = QDoubleSpinBox()
                spin.setRange(-180, 180)
                spin.setSingleStep(1)
                spin.setSuffix("°")

            spin.setMinimumHeight(28)
            self.position_inputs[pos] = spin

            row = i // 3
            col = (i % 3) * 2
            target_grid.addWidget(label, row, col)
            target_grid.addWidget(spin, row, col + 1)

        layout.addLayout(target_grid)

        # 速度控制
        speed_layout = QHBoxLayout()
        speed_label = QLabel("速度:")
        speed_label.setStyleSheet("color: #95a5a6; font-size: 11px;")
        speed_layout.addWidget(speed_label)

        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(1, 100)
        self.speed_slider.setValue(50)
        self.speed_slider.setMinimumHeight(20)
        speed_layout.addWidget(self.speed_slider)

        self.speed_label = QLabel("50%")
        self.speed_label.setObjectName("value")
        self.speed_label.setMinimumWidth(50)
        self.speed_slider.valueChanged.connect(lambda v: self.speed_label.setText(f"{v}%"))
        speed_layout.addWidget(self.speed_label)

        layout.addLayout(speed_layout)

        # 移動按鈕
        btn_layout = QHBoxLayout()
        btn_layout.setSpacing(8)

        move_btn = QPushButton("▶ 移動到目標")
        move_btn.setMinimumHeight(36)
        move_btn.clicked.connect(self.move_to_position)
        btn_layout.addWidget(move_btn)

        linear_btn = QPushButton("📐 直線移動")
        linear_btn.setMinimumHeight(36)
        linear_btn.clicked.connect(self.linear_move)
        btn_layout.addWidget(linear_btn)

        layout.addLayout(btn_layout)

        group.setLayout(layout)
        return group

    def create_trajectory_control(self):
        """創建軌跡規劃面板"""
        group = QGroupBox("🔄 軌跡規劃")
        group.setStyleSheet("""
            QGroupBox::title {
                color: #9b59b6;
                font-size: 14px;
            }
        """)
        layout = QVBoxLayout()

        # 軌跡點列表
        points_label = QLabel("軌跡點")
        points_label.setStyleSheet("color: #95a5a6; font-size: 11px;")
        layout.addWidget(points_label)

        self.trajectory_list = QListWidget()
        self.trajectory_list.setMaximumHeight(120)
        self.trajectory_list.setStyleSheet("""
            QListWidget {
                background-color: #1a1a1a;
                border: 1px solid #3d3d3d;
                border-radius: 5px;
                padding: 5px;
            }
            QListWidget::item {
                color: #95a5a6;
                padding: 4px;
                border-bottom: 1px solid #2d2d2d;
            }
            QListWidget::item:selected {
                background-color: #34495e;
                color: white;
            }
        """)
        layout.addWidget(self.trajectory_list)

        # 軌跡點操作按鈕
        point_btn_layout = QHBoxLayout()
        point_btn_layout.setSpacing(6)

        add_btn = QPushButton("➕")
        add_btn.setToolTip("新增當前點")
        add_btn.setMaximumWidth(50)
        add_btn.setMinimumHeight(32)
        add_btn.clicked.connect(self.add_trajectory_point)
        point_btn_layout.addWidget(add_btn)

        insert_btn = QPushButton("📝")
        insert_btn.setToolTip("插入")
        insert_btn.setMaximumWidth(50)
        insert_btn.setMinimumHeight(32)
        insert_btn.clicked.connect(self.insert_trajectory_point)
        point_btn_layout.addWidget(insert_btn)

        delete_btn = QPushButton("❌")
        delete_btn.setToolTip("刪除")
        delete_btn.setMaximumWidth(50)
        delete_btn.setMinimumHeight(32)
        delete_btn.clicked.connect(self.delete_trajectory_point)
        point_btn_layout.addWidget(delete_btn)

        clear_btn = QPushButton("🗑️")
        clear_btn.setToolTip("清空")
        clear_btn.setMaximumWidth(50)
        clear_btn.setMinimumHeight(32)
        clear_btn.clicked.connect(self.clear_trajectory)
        point_btn_layout.addWidget(clear_btn)

        layout.addLayout(point_btn_layout)

        # 執行參數
        params_label = QLabel("執行參數")
        params_label.setStyleSheet("color: #95a5a6; font-size: 11px; margin-top: 5px;")
        layout.addWidget(params_label)

        params_grid = QGridLayout()
        params_grid.setSpacing(6)

        # 運動類型
        params_grid.addWidget(QLabel("運動類型:"), 0, 0)
        self.motion_type = QComboBox()
        self.motion_type.addItems(["關節空間", "直線", "圓弧"])
        self.motion_type.setMinimumHeight(28)
        params_grid.addWidget(self.motion_type, 0, 1)

        # 速度
        params_grid.addWidget(QLabel("速度 (%):"), 1, 0)
        self.traj_speed = QSpinBox()
        self.traj_speed.setRange(1, 100)
        self.traj_speed.setValue(30)
        self.traj_speed.setMinimumHeight(28)
        params_grid.addWidget(self.traj_speed, 1, 1)

        # 加速度
        params_grid.addWidget(QLabel("加速度 (%):"), 2, 0)
        self.traj_accel = QSpinBox()
        self.traj_accel.setRange(1, 100)
        self.traj_accel.setValue(20)
        self.traj_accel.setMinimumHeight(28)
        params_grid.addWidget(self.traj_accel, 2, 1)

        # 混合半徑
        params_grid.addWidget(QLabel("混合半徑 (mm):"), 3, 0)
        self.blend_radius = QSpinBox()
        self.blend_radius.setRange(0, 100)
        self.blend_radius.setValue(10)
        self.blend_radius.setMinimumHeight(28)
        params_grid.addWidget(self.blend_radius, 3, 1)

        layout.addLayout(params_grid)

        # 執行控制按鈕
        exec_btn_layout = QHBoxLayout()
        exec_btn_layout.setSpacing(6)

        run_btn = QPushButton("▶ 執行")
        run_btn.setMinimumHeight(36)
        run_btn.clicked.connect(self.run_trajectory)
        exec_btn_layout.addWidget(run_btn)

        pause_btn = QPushButton("⏸ 暫停")
        pause_btn.setMinimumHeight(36)
        pause_btn.setStyleSheet("background-color: #e67e22;")
        pause_btn.clicked.connect(self.pause_trajectory)
        exec_btn_layout.addWidget(pause_btn)

        stop_btn = QPushButton("⏹ 停止")
        stop_btn.setMinimumHeight(36)
        stop_btn.setStyleSheet("background-color: #e74c3c;")
        stop_btn.clicked.connect(self.stop_trajectory)
        exec_btn_layout.addWidget(stop_btn)

        layout.addLayout(exec_btn_layout)

        layout.addStretch()
        group.setLayout(layout)
        return group

    def create_middle_panel(self):
        """創建中間面板：3D視圖 + 監控"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setSpacing(10)
        layout.setContentsMargins(0, 0, 0, 0)

        # 3D 視覺化
        viz_group = self.create_visualization_panel()
        layout.addWidget(viz_group, stretch=4)

        # 系統監控
        monitor_group = self.create_monitor_panel()
        layout.addWidget(monitor_group)

        return widget

    def create_visualization_panel(self):
        """創建 3D 視覺化面板"""
        group = QGroupBox("🎥 3D 視覺化")
        group.setStyleSheet("""
            QGroupBox::title {
                color: #95a5a6;
                font-size: 14px;
            }
        """)
        layout = QVBoxLayout()
        layout.setContentsMargins(5, 5, 5, 5)

        try:
            from visualization.vtk_view import VTKWidget

            self.vtk_widget = VTKWidget(self.ctrl)
            self.vtk_widget.setMinimumSize(600, 500)
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
                    padding: 40px;
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
        control_layout.setSpacing(8)

        reset_btn = QPushButton("🔄 重置")
        reset_btn.setMaximumWidth(80)
        reset_btn.setMinimumHeight(32)
        reset_btn.clicked.connect(self.reset_camera_view)
        control_layout.addWidget(reset_btn)

        top_btn = QPushButton("⬆ 俯視")
        top_btn.setMaximumWidth(80)
        top_btn.setMinimumHeight(32)
        top_btn.clicked.connect(lambda: self.set_camera_view("top"))
        control_layout.addWidget(top_btn)

        side_btn = QPushButton("➡ 側視")
        side_btn.setMaximumWidth(80)
        side_btn.setMinimumHeight(32)
        side_btn.clicked.connect(lambda: self.set_camera_view("side"))
        control_layout.addWidget(side_btn)

        front_btn = QPushButton("👁 正視")
        front_btn.setMaximumWidth(80)
        front_btn.setMinimumHeight(32)
        front_btn.clicked.connect(lambda: self.set_camera_view("front"))
        control_layout.addWidget(front_btn)

        control_layout.addStretch()
        layout.addLayout(control_layout)

        group.setLayout(layout)
        return group

    def create_monitor_panel(self):
        """創建監控面板"""
        group = QGroupBox("📊 系統監控")
        group.setStyleSheet("""
            QGroupBox::title {
                color: #1abc9c;
                font-size: 14px;
            }
        """)
        group.setMaximumHeight(120)

        layout = QVBoxLayout()

        grid = QGridLayout()
        grid.setSpacing(8)

        monitors = [
            ("連線狀態", "已連線", 0, 0),
            ("運行時間", "00:00:00", 0, 1),
            ("CPU 使用率", "0%", 0, 2),
            ("記憶體使用", "0 MB", 1, 0),
            ("目前速度", "0 mm/s", 1, 1),
            ("目前加速度", "0 mm/s²", 1, 2),
        ]

        for name, default, row, col in monitors:
            frame = QFrame()
            frame.setStyleSheet("background-color: #1a1a1a; border-radius: 5px; padding: 8px;")
            frame_layout = QVBoxLayout(frame)
            frame_layout.setContentsMargins(8, 6, 8, 6)
            frame_layout.setSpacing(2)

            label = QLabel(f"{name}")
            label.setStyleSheet("color: #7f8c8d; font-size: 10px;")
            frame_layout.addWidget(label)

            value = QLabel(default)
            value.setStyleSheet("color: #1abc9c; font-size: 13px; font-weight: bold;")
            self.monitor_labels[name] = value
            frame_layout.addWidget(value)

            grid.addWidget(frame, row, col)

        layout.addLayout(grid)
        group.setLayout(layout)
        return group

    def create_right_panel(self):
        """創建右側面板：關節控制"""
        group = QGroupBox("🎮 關節控制")
        group.setStyleSheet("""
            QGroupBox::title {
                color: #3498db;
                font-size: 14px;
            }
        """)

        layout = QVBoxLayout()

        # 關節滑桿區域
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)

        scroll_widget = QWidget()
        scroll_layout = QVBoxLayout(scroll_widget)
        scroll_layout.setContentsMargins(5, 5, 5, 5)
        scroll_layout.setSpacing(8)

        for i in range(1, 7):
            joint_frame = QFrame()
            joint_frame.setStyleSheet("""
                QFrame {
                    background-color: #1a1a1a;
                    border: 1px solid #3d3d3d;
                    border-radius: 8px;
                    padding: 10px;
                }
            """)
            joint_layout = QVBoxLayout(joint_frame)
            joint_layout.setContentsMargins(10, 8, 10, 8)
            joint_layout.setSpacing(6)

            # 標題和當前值
            header_layout = QHBoxLayout()
            title = QLabel(f"關節 {i}")
            title.setStyleSheet("font-weight: bold; font-size: 13px; color: #ecf0f1;")
            header_layout.addWidget(title)

            header_layout.addStretch()

            value_label = QLabel("0°")
            value_label.setObjectName("value")
            value_label.setStyleSheet("font-size: 18px; font-weight: bold; color: #3498db;")
            self.joint_labels[i] = value_label
            header_layout.addWidget(value_label)

            joint_layout.addLayout(header_layout)

            # 滑桿和範圍
            slider_layout = QHBoxLayout()
            slider_layout.setSpacing(8)

            min_val, max_val = JOINT_LIMITS[f'J{i}']

            min_label = QLabel(f"{min_val}°")
            min_label.setStyleSheet("color: #7f8c8d; font-size: 9px;")
            min_label.setMinimumWidth(35)
            min_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            slider_layout.addWidget(min_label)

            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(int(min_val))
            slider.setMaximum(int(max_val))
            slider.setValue(0)
            slider.setMinimumHeight(24)
            slider.valueChanged.connect(lambda v, idx=i: self.on_joint_slider_changed(idx, v))
            self.joint_sliders[i] = slider
            slider_layout.addWidget(slider)

            max_label = QLabel(f"{max_val}°")
            max_label.setStyleSheet("color: #7f8c8d; font-size: 9px;")
            max_label.setMinimumWidth(35)
            max_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            slider_layout.addWidget(max_label)

            joint_layout.addLayout(slider_layout)

            scroll_layout.addWidget(joint_frame)

        scroll_layout.addStretch()
        scroll.setWidget(scroll_widget)
        layout.addWidget(scroll)

        # 控制按鈕
        btn_layout = QHBoxLayout()
        btn_layout.setSpacing(8)

        home_btn = QPushButton("🏠 回原點")
        home_btn.setMinimumHeight(40)
        home_btn.clicked.connect(self.go_home)
        btn_layout.addWidget(home_btn)

        zero_btn = QPushButton("0️⃣ 歸零")
        zero_btn.setMinimumHeight(40)
        zero_btn.clicked.connect(self.zero_all_joints)
        btn_layout.addWidget(zero_btn)

        layout.addLayout(btn_layout)

        group.setLayout(layout)
        return group

    # ========== 事件處理函數 ==========

    def on_joint_slider_changed(self, joint_idx, value):
        """關節滑桿值改變"""
        self.joint_labels[joint_idx].setText(f"{value}°")
        angles = [self.joint_sliders[i].value() for i in range(1, 7)]
        self.ctrl.set_joint_angles(angles)
        self.update_position_display()

    def go_home(self):
        """回到原點位置"""
        home_angles = [0, 0, 0, 0, 0, 0]
        self.ctrl.move_to_home()
        for i, angle in enumerate(home_angles, 1):
            self.joint_sliders[i].setValue(angle)
        self.statusBar().