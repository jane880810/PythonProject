'''
PyQt5 機械手臂控制介面 - PyCharm 直接可執行版本
安裝: pip install PyQt5
執行: 直接點擊 PyCharm 的綠色運行按鈕
'''

from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
import sys
import threading
import time

# 模擬的 AnimationController（用於測試，實際使用時替換為您的真實控制器）
class MockController:
    def __init__(self):
        self.current_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.target_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.is_animating = False
        self.is_following_trajectory = False
        self.trajectory_index = 0
        self.trajectory_points = None
        self.skip_unreachable_points = True
        
    def set_target(self, idx, value):
        self.target_angles[idx] = value
        self.is_animating = True
        # 簡單模擬：直接設定為目標值
        self.current_angles[idx] = value
        
    def get_joint5_position(self):
        # 模擬返回末端位置
        return [0.345, 0.123, 0.823]
        
    def set_trajectory_speed(self, speed):
        print(f"設定速度: {speed}x")
        
    def start_trajectory(self, points):
        self.trajectory_points = points
        self.is_following_trajectory = True
        print(f"開始執行軌跡，共 {len(points)} 點")
        
    def stop_trajectory(self):
        self.is_following_trajectory = False
        print("停止軌跡")
        
    def reset_pose(self):
        for i in range(6):
            self.set_target(i, 0)
        print("重置姿態")


class RobotControlGUI(QMainWindow):
    def __init__(self, controller):
        super().__init__()
        self.ctrl = controller
        
        self.setWindowTitle("RA605-710-GC 機械手臂控制系統")
        self.setGeometry(100, 100, 1200, 800)
        self.apply_dark_theme()
        
        self.setup_ui()
        
        # 啟動定時器更新顯示
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_display)
        self.update_timer.start(50)
        
        print("✓ PyQt5 GUI 啟動成功！")
        
    def apply_dark_theme(self):
        """深色主題"""
        self.setStyleSheet("""
            QMainWindow {
                background-color: #1e1e1e;
            }
            QWidget {
                background-color: #2d2d2d;
                color: #ffffff;
                font-family: 'Microsoft JhengHei', 'Segoe UI', Arial;
                font-size: 12px;
            }
            QGroupBox {
                border: 2px solid #3d3d3d;
                border-radius: 8px;
                margin-top: 10px;
                padding: 15px;
                font-weight: bold;
                font-size: 13px;
            }
            QGroupBox::title {
                color: #4a9eff;
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }
            QPushButton {
                background-color: #0d7377;
                border: none;
                border-radius: 5px;
                padding: 10px;
                color: white;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #14a085;
            }
            QPushButton:pressed {
                background-color: #0a5f62;
            }
            QPushButton#emergency {
                background-color: #c0392b;
            }
            QPushButton#emergency:hover {
                background-color: #e74c3c;
            }
            QSlider::groove:horizontal {
                height: 8px;
                background: #3d3d3d;
                border-radius: 4px;
            }
            QSlider::handle:horizontal {
                background: #4a9eff;
                width: 18px;
                margin: -5px 0;
                border-radius: 9px;
            }
            QLabel#value {
                color: #4a9eff;
                font-size: 16px;
                font-weight: bold;
            }
            QLabel#position {
                color: #2ecc71;
                font-size: 20px;
                font-weight: bold;
            }
            QTextEdit {
                background-color: #1e1e1e;
                border: 1px solid #3d3d3d;
                border-radius: 5px;
                padding: 5px;
                font-family: 'Courier New';
                font-size: 11px;
                color: #ffffff;
            }
        """)
        
    def setup_ui(self):
        """建立介面"""
        central = QWidget()
        self.setCentralWidget(central)
        
        main_layout = QHBoxLayout(central)
        main_layout.setSpacing(10)
        main_layout.setContentsMargins(10, 10, 10, 10)
        
        # 左側：關節控制
        left_panel = self.create_joint_panel()
        main_layout.addWidget(left_panel, stretch=2)
        
        # 右側：控制面板
        right_panel = self.create_control_panel()
        main_layout.addWidget(right_panel, stretch=1)
        
        # 底部狀態欄
        self.statusBar().showMessage("就緒 | PyQt5 GUI 運行中")
        self.statusBar().setStyleSheet("""
            QStatusBar {
                background-color: #1e1e1e;
                color: #4a9eff;
                border-top: 1px solid #3d3d3d;
                font-weight: bold;
            }
        """)
        
    def create_joint_panel(self):
        """關節控制面板"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        # 標題
        title = QLabel("🎮 關節控制")
        title.setStyleSheet("font-size: 18px; font-weight: bold; color: #4a9eff; padding: 10px;")
        layout.addWidget(title)
        
        # 滾動區域
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_content = QWidget()
        scroll_layout = QVBoxLayout(scroll_content)
        
        joint_names = [
            "關節 1 - 基座旋轉",
            "關節 2 - 肩部俯仰",
            "關節 3 - 肘部俯仰",
            "關節 4 - 腕部旋轉",
            "關節 5 - 腕部俯仰",
            "關節 6 - 末端旋轉"
        ]
        
        limits = [
            (-165, 165), (-125, 85), (-55, 185),
            (-190, 190), (-25, 205), (-360, 360)
        ]
        
        self.joint_sliders = []
        self.joint_value_labels = []
        
        for i, (name, (min_val, max_val)) in enumerate(zip(joint_names, limits)):
            group = QGroupBox(name)
            group_layout = QVBoxLayout()
            
            # 數值顯示
            value_widget = QWidget()
            value_layout = QHBoxLayout(value_widget)
            value_layout.setContentsMargins(0, 0, 0, 0)
            
            value_layout.addWidget(QLabel("當前:"))
            
            value_label = QLabel("0.0°")
            value_label.setObjectName("value")
            value_layout.addWidget(value_label)
            self.joint_value_labels.append(value_label)
            
            value_layout.addStretch()
            
            range_label = QLabel(f"[{min_val}° ~ {max_val}°]")
            range_label.setStyleSheet("color: #888888; font-size: 10px;")
            value_layout.addWidget(range_label)
            
            group_layout.addWidget(value_widget)
            
            # 滑桿
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(min_val)
            slider.setMaximum(max_val)
            slider.setValue(0)
            slider.valueChanged.connect(
                lambda v, idx=i: self.on_joint_slider_change(idx, v)
            )
            self.joint_sliders.append(slider)
            group_layout.addWidget(slider)
            
            group.setLayout(group_layout)
            scroll_layout.addWidget(group)
        
        scroll.setWidget(scroll_content)
        layout.addWidget(scroll)
        
        # 快速按鈕
        btn_layout = QHBoxLayout()
        
        home_btn = QPushButton("🏠 Home")
        home_btn.clicked.connect(self.home_position)
        btn_layout.addWidget(home_btn)
        
        reset_btn = QPushButton("🔄 重置")
        reset_btn.clicked.connect(self.reset_pose)
        btn_layout.addWidget(reset_btn)
        
        stop_btn = QPushButton("⏹ 停止")
        stop_btn.setObjectName("emergency")
        stop_btn.clicked.connect(self.emergency_stop)
        btn_layout.addWidget(stop_btn)
        
        layout.addLayout(btn_layout)
        
        return widget
        
    def create_control_panel(self):
        """控制面板"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        # 當前位置
        pos_group = QGroupBox("📍 當前末端位置 (J5)")
        pos_layout = QVBoxLayout()
        
        self.pos_labels = {}
        colors = ["#e74c3c", "#2ecc71", "#3498db"]
        
        for axis, color in zip(['X', 'Y', 'Z'], colors):
            axis_widget = QWidget()
            axis_layout = QHBoxLayout(axis_widget)
            axis_layout.setContentsMargins(0, 0, 0, 0)
            
            axis_label = QLabel(f"{axis}:")
            axis_label.setStyleSheet(f"color: {color}; font-size: 14px; font-weight: bold;")
            axis_layout.addWidget(axis_label)
            
            value_label = QLabel("0.000 m")
            value_label.setObjectName("position")
            value_label.setStyleSheet(f"color: {color};")
            axis_layout.addWidget(value_label)
            axis_layout.addStretch()
            
            self.pos_labels[axis] = value_label
            pos_layout.addWidget(axis_widget)
        
        pos_group.setLayout(pos_layout)
        layout.addWidget(pos_group)
        
        # 目標位置
        target_group = QGroupBox("🎯 目標位置設定")
        target_layout = QVBoxLayout()
        
        self.target_sliders = {}
        for axis, default in [('X', 0.3), ('Y', 0.0), ('Z', 0.8)]:
            axis_widget = QWidget()
            axis_layout = QVBoxLayout(axis_widget)
            
            header = QWidget()
            header_layout = QHBoxLayout(header)
            header_layout.setContentsMargins(0, 0, 0, 0)
            
            header_layout.addWidget(QLabel(f"目標 {axis}:"))
            
            value_label = QLabel(f"{default:.2f} m")
            value_label.setObjectName("value")
            header_layout.addWidget(value_label)
            header_layout.addStretch()
            
            axis_layout.addWidget(header)
            
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(-700 if axis != 'Z' else 0)
            slider.setMaximum(700 if axis != 'Z' else 1000)
            slider.setValue(int(default * 1000))
            slider.valueChanged.connect(
                lambda v, lbl=value_label: lbl.setText(f"{v/1000:.2f} m")
            )
            axis_layout.addWidget(slider)
            
            self.target_sliders[axis] = slider
            target_layout.addWidget(axis_widget)
        
        target_group.setLayout(target_layout)
        layout.addWidget(target_group)
        
        # 軌跡控制
        traj_group = QGroupBox("🔄 圓弧軌跡控制")
        traj_layout = QVBoxLayout()
        
        # 速度
        speed_widget = QWidget()
        speed_layout = QVBoxLayout(speed_widget)
        
        speed_header = QWidget()
        speed_header_layout = QHBoxLayout(speed_header)
        speed_header_layout.addWidget(QLabel("執行速度:"))
        
        self.speed_label = QLabel("100x")
        self.speed_label.setObjectName("value")
        speed_header_layout.addWidget(self.speed_label)
        speed_header_layout.addStretch()
        
        speed_layout.addWidget(speed_header)
        
        speed_slider = QSlider(Qt.Horizontal)
        speed_slider.setRange(1, 1000)
        speed_slider.setValue(100)
        speed_slider.valueChanged.connect(self.on_speed_change)
        speed_layout.addWidget(speed_slider)
        
        traj_layout.addWidget(speed_widget)
        
        # 執行按鈕
        execute_btn = QPushButton("▶ 執行圓弧移動")
        execute_btn.clicked.connect(self.execute_arc)
        traj_layout.addWidget(execute_btn)
        
        stop_btn = QPushButton("⏹ 停止軌跡")
        stop_btn.setObjectName("emergency")
        stop_btn.clicked.connect(self.stop_trajectory)
        traj_layout.addWidget(stop_btn)
        
        traj_group.setLayout(traj_layout)
        layout.addWidget(traj_group)
        
        # 狀態顯示
        status_group = QGroupBox("📊 系統狀態")
        status_layout = QVBoxLayout()
        
        self.status_label = QLabel("準備就緒")
        self.status_label.setStyleSheet("color: #2ecc71; font-weight: bold; font-size: 14px;")
        status_layout.addWidget(self.status_label)
        
        status_group.setLayout(status_layout)
        layout.addWidget(status_group)
        
        # 控制台
        console_group = QGroupBox("📋 控制台輸出")
        console_layout = QVBoxLayout()
        
        self.console = QTextEdit()
        self.console.setReadOnly(True)
        self.console.setMaximumHeight(150)
        self.log("[系統] PyQt5 GUI 初始化完成")
        self.log("[系統] 準備就緒，等待指令...")
        
        console_layout.addWidget(self.console)
        console_group.setLayout(console_layout)
        layout.addWidget(console_group)
        
        layout.addStretch()
        
        return widget
    
    def log(self, message):
        """輸出日誌"""
        timestamp = time.strftime("%H:%M:%S")
        self.console.append(f"[{timestamp}] {message}")
        
    # ========== 事件處理 ==========
    
    def on_joint_slider_change(self, idx, value):
        """關節滑桿變更"""
        self.joint_value_labels[idx].setText(f"{value:.1f}°")
        self.ctrl.set_target(idx, float(value))
        self.log(f"關節 {idx+1} 移動到 {value:.1f}°")
        
    def on_speed_change(self, value):
        """速度變更"""
        self.speed_label.setText(f"{value}x")
        self.ctrl.set_trajectory_speed(value)
        
    def update_display(self):
        """定時更新顯示"""
        try:
            # 更新位置
            pos = self.ctrl.get_joint5_position()
            self.pos_labels['X'].setText(f"{pos[0]:.3f} m")
            self.pos_labels['Y'].setText(f"{pos[1]:.3f} m")
            self.pos_labels['Z'].setText(f"{pos[2]:.3f} m")
            
            # 更新關節角度
            for i, angle in enumerate(self.ctrl.current_angles):
                self.joint_value_labels[i].setText(f"{angle:.1f}°")
                self.joint_sliders[i].blockSignals(True)
                self.joint_sliders[i].setValue(int(angle))
                self.joint_sliders[i].blockSignals(False)
            
            # 更新狀態
            if self.ctrl.is_following_trajectory:
                self.status_label.setText("執行軌跡中...")
                self.status_label.setStyleSheet("color: #f39c12; font-weight: bold; font-size: 14px;")
            elif self.ctrl.is_animating:
                self.status_label.setText("移動中...")
                self.status_label.setStyleSheet("color: #3498db; font-weight: bold; font-size: 14px;")
            else:
                self.status_label.setText("準備就緒")
                self.status_label.setStyleSheet("color: #2ecc71; font-weight: bold; font-size: 14px;")
                
        except Exception as e:
            pass
            
    def execute_arc(self):
        """執行圓弧移動"""
        self.log("[軌跡] 開始執行圓弧移動")
        QMessageBox.information(self, "提示", "圓弧移動功能需要整合您的軌跡計算函數")
        
    def home_position(self):
        """回到原點"""
        self.log("[動作] 回到原點")
        for i in range(6):
            self.ctrl.set_target(i, 0)
            
    def reset_pose(self):
        """重置姿態"""
        self.log("[動作] 重置姿態")
        self.ctrl.reset_pose()
        
    def emergency_stop(self):
        """緊急停止"""
        self.log("[緊急] 緊急停止！")
        self.ctrl.is_animating = False
        self.status_label.setText("已停止")
        self.status_label.setStyleSheet("color: #e74c3c; font-weight: bold; font-size: 14px;")
        
    def stop_trajectory(self):
        """停止軌跡"""
        self.log("[軌跡] 停止執行")
        self.ctrl.stop_trajectory()
        
    def closeEvent(self, event):
        """關閉視窗"""
        reply = QMessageBox.question(
            self, '確認',
            "確定要關閉程式嗎？",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            self.log("[系統] 正在關閉...")
            event.accept()
        else:
            event.ignore()


# ========== 主程式 ==========

def main():
    """主程式入口 - 直接在 PyCharm 中執行"""
    print("=" * 60)
    print("RA605-710-GC 機械手臂控制系統")
    print("=" * 60)
    print("正在啟動 PyQt5 GUI...")
    print()
    
    # 創建控制器（實際使用時替換為您的真實控制器）
    ctrl = MockController()
    
    # 創建 Qt 應用程式
    app = QApplication(sys.argv)
    
    # 創建並顯示主視窗
    window = RobotControlGUI(ctrl)
    window.show()
    
    print("✓ GUI 啟動成功！")
    print("提示：關閉視窗即可退出程式")
    print()
    
    # 運行應用程式
    sys.exit(app.exec_())


# 如果直接運行此檔案
if __name__ == "__main__":
    main()