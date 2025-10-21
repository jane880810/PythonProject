"""
test_vtk.py
測試 VTK 是否正常工作（放在專案根目錄）
"""

from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QPushButton
from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
import vtk
import sys


class TestVTKWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("VTK 測試 - 如果看到紅色球體，說明 VTK 正常")
        self.setGeometry(100, 100, 800, 600)
        self.setStyleSheet("background-color: #2d2d2d; color: white;")

        # 中央 Widget
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)

        # 創建 VTK Widget
        try:
            self.vtkWidget = QVTKRenderWindowInteractor(self)
            layout.addWidget(self.vtkWidget)

            # 創建渲染器
            self.renderer = vtk.vtkRenderer()
            self.renderer.SetBackground(0.1, 0.1, 0.1)  # 深灰色背景

            # 創建一個紅色球體
            sphere = vtk.vtkSphereSource()
            sphere.SetRadius(1.0)
            sphere.SetThetaResolution(32)
            sphere.SetPhiResolution(32)

            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputConnection(sphere.GetOutputPort())

            actor = vtk.vtkActor()
            actor.SetMapper(mapper)
            actor.GetProperty().SetColor(1, 0, 0)  # 紅色

            self.renderer.AddActor(actor)

            # 添加座標軸
            axes = vtk.vtkAxesActor()
            axes.SetTotalLength(2, 2, 2)
            self.renderer.AddActor(axes)

            # 設定渲染窗口
            renderWindow = self.vtkWidget.GetRenderWindow()
            renderWindow.AddRenderer(self.renderer)

            # 初始化交互器
            interactor = self.vtkWidget.GetRenderWindow().GetInteractor()
            style = vtk.vtkInteractorStyleTrackballCamera()
            interactor.SetInteractorStyle(style)
            interactor.Initialize()

            print("=" * 60)
            print("✓ VTK 測試視窗創建成功！")
            print("=" * 60)
            print("如果您看到：")
            print("  - 一個紅色球體")
            print("  - 可以用滑鼠拖曳旋轉視角")
            print("  - 滾輪縮放")
            print("說明 VTK 工作完全正常！")
            print("=" * 60)

        except Exception as e:
            print(f"❌ VTK 創建失敗: {e}")
            import traceback
            traceback.print_exc()

            # 顯示錯誤
            from PyQt5.QtWidgets import QLabel
            error_label = QLabel(f"VTK 創建失敗:\n{str(e)}")
            error_label.setStyleSheet("color: red; font-size: 16px;")
            layout.addWidget(error_label)

        # 添加控制按鈕
        btn_layout = QVBoxLayout()

        info_label = QLabel("滑鼠操作：\n左鍵拖曳：旋轉\n滾輪：縮放\n中鍵/Shift+左鍵：平移")
        info_label.setStyleSheet("color: #4a9eff; font-size: 12px; padding: 10px;")
        btn_layout.addWidget(info_label)

        close_btn = QPushButton("關閉測試")
        close_btn.setStyleSheet("""
            QPushButton {
                background-color: #e74c3c;
                color: white;
                padding: 10px;
                border-radius: 5px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #c0392b;
            }
        """)
        close_btn.clicked.connect(self.close)
        btn_layout.addWidget(close_btn)

        layout.addLayout(btn_layout)


if __name__ == "__main__":
    print("\n正在啟動 VTK 測試...")
    print("請稍候...\n")

    app = QApplication(sys.argv)
    window = TestVTKWindow()
    window.show()
    sys.exit(app.exec_())