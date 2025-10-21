"""
test_vtk.py
測試 VTK 是否正常工作
"""

from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout
from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
import vtk
import sys


class TestWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("VTK 測試")
        self.setGeometry(100, 100, 800, 600)

        # 中央 Widget
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)

        # 創建 VTK Widget
        self.vtkWidget = QVTKRenderWindowInteractor(self)
        layout.addWidget(self.vtkWidget)

        # 創建渲染器
        self.renderer = vtk.vtkRenderer()
        self.renderer.SetBackground(0.1, 0.2, 0.3)

        # 創建一個簡單的球體
        sphere = vtk.vtkSphereSource()
        sphere.SetRadius(1.0)

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(sphere.GetOutputPort())

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(1, 0, 0)

        self.renderer.AddActor(actor)

        # 設定渲染窗口
        renderWindow = self.vtkWidget.GetRenderWindow()
        renderWindow.AddRenderer(self.renderer)

        # 初始化交互器
        self.vtkWidget.GetRenderWindow().GetInteractor().Initialize()

        print("✓ VTK 測試視窗創建成功！")
        print("  如果您看到一個紅色球體，說明 VTK 工作正常")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = TestWindow()
    window.show()
    sys.exit(app.exec_())