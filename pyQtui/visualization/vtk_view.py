"""
visualization/vtk_view.py
VTK 3D 視覺化模組 - 完美嵌入 Qt
"""

import vtk
from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
from PyQt5.QtWidgets import QWidget, QVBoxLayout
import numpy as np
from config.robot_config import MODEL_PATHS, ROBOT_DH_PARAMS


class VTKWidget(QWidget):
    """VTK 可嵌入 Qt 的 Widget"""

    def __init__(self, controller, parent=None):
        super().__init__(parent)
        self.ctrl = controller

        # VTK 組件
        self.vtkWidget = None
        self.renderer = None
        self.renderWindow = None
        self.interactor = None

        # 演員（Actors）字典
        self.actors = {}
        self.meshes = []

        # 從配置讀取參數
        self.S1 = ROBOT_DH_PARAMS['S1']
        self.S2 = ROBOT_DH_PARAMS['S2']
        self.L1 = ROBOT_DH_PARAMS['L1']
        self.L2 = ROBOT_DH_PARAMS['L2']
        self.L3 = ROBOT_DH_PARAMS['L3']
        self.L4 = ROBOT_DH_PARAMS['L4']

        # 末端標記
        self.joint5_marker = None

        self.setup_vtk()
        self.load_scene()

    def setup_vtk(self):
        """設定 VTK 渲染器"""
        # 創建佈局
        layout = QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)

        # 創建 VTK Widget
        self.vtkWidget = QVTKRenderWindowInteractor(self)
        layout.addWidget(self.vtkWidget)
        self.setLayout(layout)

        # 創建渲染器
        self.renderer = vtk.vtkRenderer()
        self.renderer.SetBackground(0.1, 0.1, 0.1)  # 深灰色背景

        # 創建渲染窗口
        self.renderWindow = self.vtkWidget.GetRenderWindow()
        self.renderWindow.AddRenderer(self.renderer)

        # 創建交互器
        self.interactor = self.vtkWidget.GetRenderWindow().GetInteractor()

        # 設定交互風格（可旋轉、縮放、平移）
        style = vtk.vtkInteractorStyleTrackballCamera()
        self.interactor.SetInteractorStyle(style)

        # 初始化
        self.interactor.Initialize()

        print("✓ VTK 渲染器初始化完成")

    def load_scene(self):
        """載入場景"""
        # 添加光源
        self.add_lights()

        # 載入機械手臂模型
        success = self.load_robot_models()

        if not success:
            # 使用簡化視覺化
            self.create_simple_robot()

        # 添加環境
        self.add_environment()

        # 創建末端標記
        self.create_joint5_marker()

        # 設定相機
        self.setup_camera()

        # 渲染
        self.renderWindow.Render()

    def add_lights(self):
        """添加光源"""
        # 主光源
        light1 = vtk.vtkLight()
        light1.SetPosition(2, 2, 3)
        light1.SetIntensity(1.0)
        light1.SetColor(1, 1, 1)
        self.renderer.AddLight(light1)

        # 補光
        light2 = vtk.vtkLight()
        light2.SetPosition(-2, -2, 1)
        light2.SetIntensity(0.5)
        self.renderer.AddLight(light2)

    def load_robot_models(self):
        """載入機械手臂 3D 模型"""
        base_path = MODEL_PATHS['base_path']
        model_files = MODEL_PATHS['models']

        print(f"正在從 {base_path} 載入模型...")

        for i, model_file in enumerate(model_files):
            full_path = base_path + model_file
            try:
                # 讀取 OBJ 檔案
                reader = vtk.vtkOBJReader()
                reader.SetFileName(full_path)
                reader.Update()

                # 創建 Mapper
                mapper = vtk.vtkPolyDataMapper()
                mapper.SetInputConnection(reader.GetOutputPort())

                # 創建 Actor
                actor = vtk.vtkActor()
                actor.SetMapper(mapper)

                # 設定顏色（根據部件設定不同顏色）
                if i == 0:
                    actor.GetProperty().SetColor(0.5, 0.5, 0.5)  # 基座：灰色
                else:
                    actor.GetProperty().SetColor(0.7, 0.7, 0.7)  # 其他：淺灰

                # 設定材質屬性
                actor.GetProperty().SetSpecular(0.3)
                actor.GetProperty().SetSpecularPower(20)
                actor.GetProperty().SetAmbient(0.2)
                actor.GetProperty().SetDiffuse(0.8)

                # 添加到渲染器
                self.renderer.AddActor(actor)

                # 儲存引用
                self.actors[f"part_{i}"] = actor

            except Exception as e:
                print(f"⚠ 無法載入 {model_file}: {e}")
                return False

        print(f"✓ 已載入 {len(self.actors)} 個模型")
        return True

    def create_simple_robot(self):
        """創建簡化的機械手臂視覺化"""
        print("使用簡化視覺化模式...")

        # 基座（圓柱）
        cylinder = vtk.vtkCylinderSource()
        cylinder.SetRadius(0.1)
        cylinder.SetHeight(0.2)
        cylinder.SetResolution(32)

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(cylinder.GetOutputPort())

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(0.5, 0.5, 0.5)

        self.renderer.AddActor(actor)
        self.actors["base"] = actor

        # 連桿 1
        link1 = vtk.vtkCylinderSource()
        link1.SetRadius(0.05)
        link1.SetHeight(self.L1)
        link1.SetResolution(16)

        mapper1 = vtk.vtkPolyDataMapper()
        mapper1.SetInputConnection(link1.GetOutputPort())

        actor1 = vtk.vtkActor()
        actor1.SetMapper(mapper1)
        actor1.GetProperty().SetColor(0.3, 0.6, 0.8)
        actor1.SetPosition(0, 0, self.L1 / 2)

        self.renderer.AddActor(actor1)
        self.actors["link1"] = actor1

    def create_joint5_marker(self):
        """創建關節 5 末端標記"""
        # 創建球體
        sphere = vtk.vtkSphereSource()
        sphere.SetRadius(0.08)
        sphere.SetThetaResolution(32)
        sphere.SetPhiResolution(32)

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(sphere.GetOutputPort())

        self.joint5_marker = vtk.vtkActor()
        self.joint5_marker.SetMapper(mapper)
        self.joint5_marker.GetProperty().SetColor(1.0, 0.0, 0.0)  # 紅色

        # 設定初始位置
        pos = self.ctrl.get_joint5_position()
        self.joint5_marker.SetPosition(pos[0], pos[1], pos[2])

        self.renderer.AddActor(self.joint5_marker)

    def update_joint5_marker(self):
        """更新關節 5 標記位置"""
        if self.joint5_marker:
            pos = self.ctrl.get_joint5_position()
            self.joint5_marker.SetPosition(pos[0], pos[1], pos[2])
            self.renderWindow.Render()

    def add_environment(self):
        """添加環境物件"""
        # 添加座標軸
        axes = vtk.vtkAxesActor()
        axes.SetTotalLength(0.3, 0.3, 0.3)
        axes.SetShaftTypeToCylinder()
        axes.SetCylinderRadius(0.02)
        self.renderer.AddActor(axes)

        # 添加網格地板
        self.add_grid()

    def add_grid(self, size=2.0, divisions=20):
        """添加網格地板"""
        # 創建平面
        plane = vtk.vtkPlaneSource()
        plane.SetOrigin(-size / 2, -size / 2, 0)
        plane.SetPoint1(size / 2, -size / 2, 0)
        plane.SetPoint2(-size / 2, size / 2, 0)
        plane.SetXResolution(divisions)
        plane.SetYResolution(divisions)

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(plane.GetOutputPort())

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetRepresentationToWireframe()
        actor.GetProperty().SetColor(0.3, 0.3, 0.3)
        actor.GetProperty().SetLineWidth(1)

        self.renderer.AddActor(actor)

    def setup_camera(self):
        """設定相機視角"""
        camera = self.renderer.GetActiveCamera()
        camera.SetPosition(1.5, 1.5, 1.5)
        camera.SetFocalPoint(0, 0, 0.5)
        camera.SetViewUp(0, 0, 1)
        self.renderer.ResetCamera()

    def reset_camera(self):
        """重置相機視角"""
        self.setup_camera()
        self.renderWindow.Render()

    def set_view(self, view_type):
        """設定視角"""
        camera = self.renderer.GetActiveCamera()

        if view_type == "top":
            camera.SetPosition(0, 0, 2)
            camera.SetFocalPoint(0, 0, 0)
            camera.SetViewUp(0, 1, 0)
        elif view_type == "side":
            camera.SetPosition(2, 0, 0.5)
            camera.SetFocalPoint(0, 0, 0.5)
            camera.SetViewUp(0, 0, 1)
        elif view_type == "front":
            camera.SetPosition(0, 2, 0.5)
            camera.SetFocalPoint(0, 0, 0.5)
            camera.SetViewUp(0, 0, 1)

        self.renderer.ResetCamera()
        self.renderWindow.Render()

    def render(self):
        """手動渲染"""
        if self.renderWindow:
            self.renderWindow.Render()