"""
visualization/vtk_view.py
VTK 3D 視覺化模組 - 完整運動學實現
"""

import vtk
from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
from PyQt5.QtWidgets import QWidget, QVBoxLayout
import numpy as np
from config.robot_config import MODEL_CONFIG, DH_PARAMS, INITIAL_TRANSLATIONS, ROTATION_CENTERS


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

        # 部件演員和組
        self.actors = {}
        self.assembly_groups = {}  # 用於層級變換

        # DH 參數
        self.S1 = DH_PARAMS['S1']
        self.S2 = DH_PARAMS['S2']
        self.L1 = DH_PARAMS['L1']
        self.L2 = DH_PARAMS['L2']
        self.L3 = DH_PARAMS['L3']
        self.L4 = DH_PARAMS['L4']

        # 末端標記
        self.joint5_marker = None

        # 當前關節角度
        self.current_angles = [0, 0, 0, 0, 0, 0]

        self.setup_vtk()
        self.load_scene()

    def setup_vtk(self):
        """設定 VTK 渲染器"""
        layout = QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)

        self.vtkWidget = QVTKRenderWindowInteractor(self)
        layout.addWidget(self.vtkWidget)
        self.setLayout(layout)

        self.renderer = vtk.vtkRenderer()
        self.renderer.SetBackground(0.1, 0.1, 0.1)

        self.renderWindow = self.vtkWidget.GetRenderWindow()
        self.renderWindow.AddRenderer(self.renderer)

        self.interactor = self.vtkWidget.GetRenderWindow().GetInteractor()
        style = vtk.vtkInteractorStyleTrackballCamera()
        self.interactor.SetInteractorStyle(style)
        self.interactor.Initialize()

        print("✓ VTK 渲染器初始化完成")

    def load_scene(self):
        """載入場景"""
        self.add_lights()

        success = self.load_robot_models()
        if not success:
            self.create_simple_robot()

        self.add_environment()
        self.create_joint5_marker()
        self.setup_camera()

        # 設定初始姿態
        self.update_robot_pose([0, 0, 0, 0, 0, 0])

        self.renderWindow.Render()

    def add_lights(self):
        """添加光源"""
        light1 = vtk.vtkLight()
        light1.SetPosition(2, 2, 3)
        light1.SetIntensity(1.0)
        self.renderer.AddLight(light1)

        light2 = vtk.vtkLight()
        light2.SetPosition(-2, -2, 1)
        light2.SetIntensity(0.5)
        self.renderer.AddLight(light2)

        light3 = vtk.vtkLight()
        light3.SetPosition(0, 0, 2)
        light3.SetIntensity(0.3)
        self.renderer.AddLight(light3)

    def load_robot_models(self):
        """載入機械手臂 3D 模型"""
        import os

        base_path = MODEL_CONFIG.get('obj_path', '/home/yahboom/Desktop/Obj/')

        print(f"正在從 {base_path} 載入模型...")

        loaded_count = 0

        for i in range(1, 9):
            model_file = f"p{i}.obj"
            full_path = os.path.join(base_path, model_file)

            if not os.path.exists(full_path):
                continue

            try:
                reader = vtk.vtkOBJReader()
                reader.SetFileName(full_path)
                reader.Update()

                mapper = vtk.vtkPolyDataMapper()
                mapper.SetInputConnection(reader.GetOutputPort())

                actor = vtk.vtkActor()
                actor.SetMapper(mapper)

                # 設定顏色
                if i == 1:
                    actor.GetProperty().SetColor(0.4, 0.4, 0.4)
                else:
                    actor.GetProperty().SetColor(0.65, 0.65, 0.65)

                actor.GetProperty().SetSpecular(0.3)
                actor.GetProperty().SetSpecularPower(20)
                actor.GetProperty().SetAmbient(0.2)
                actor.GetProperty().SetDiffuse(0.8)

                self.renderer.AddActor(actor)
                self.actors[f'p{i}'] = actor
                loaded_count += 1

            except Exception as e:
                print(f"  ✗ {model_file}: {e}")

        if loaded_count > 0:
            print(f"✓ 成功載入 {loaded_count} 個模型")
            return True
        return False

    def create_simple_robot(self):
        """簡化視覺化"""
        base = self.create_cylinder(0.1, 0.23, (0.5, 0.5, 0.5))
        self.renderer.AddActor(base)
        self.actors["p1"] = base

    def create_cylinder(self, radius, height, color):
        """創建圓柱體"""
        cylinder = vtk.vtkCylinderSource()
        cylinder.SetRadius(radius)
        cylinder.SetHeight(height)
        cylinder.SetResolution(32)

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(cylinder.GetOutputPort())

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(*color)

        return actor

    def create_joint5_marker(self):
        """創建末端標記"""
        sphere = vtk.vtkSphereSource()
        sphere.SetRadius(0.065)  # 放大到 1.3 倍 (0.05 * 1.3 = 0.065)
        sphere.SetThetaResolution(32)
        sphere.SetPhiResolution(32)

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(sphere.GetOutputPort())

        self.joint5_marker = vtk.vtkActor()
        self.joint5_marker.SetMapper(mapper)
        self.joint5_marker.GetProperty().SetColor(1.0, 0.0, 0.0)

        self.renderer.AddActor(self.joint5_marker)

    def matrix_to_vtk_transform(self, matrix_4x4):
        """將 4x4 numpy 矩陣轉換為 vtkTransform"""
        transform = vtk.vtkTransform()
        vtk_matrix = vtk.vtkMatrix4x4()

        for i in range(4):
            for j in range(4):
                vtk_matrix.SetElement(i, j, matrix_4x4[i, j])

        transform.SetMatrix(vtk_matrix)
        return transform

    def update_robot_pose(self, joint_angles):
        """
        更新機器人姿態 - 使用正確的 DH 變換
        Args:
            joint_angles: [θ1, θ2, θ3, θ4, θ5, θ6] (度)
        """
        self.current_angles = joint_angles

        # 先更新 controller 的角度，讓它計算正確的關節5位置
        try:
            self.ctrl.current_angles = joint_angles
        except:
            pass

        # 轉換為弧度
        theta = [np.deg2rad(a) for a in joint_angles]

        # ========== 構建變換矩陣 ==========

        # 基座變換 (theta1 繞 Z 軸)
        T1 = np.eye(4)
        c1, s1 = np.cos(theta[0]), np.sin(theta[0])
        T1[:3, :3] = np.array([
            [c1, -s1, 0],
            [s1,  c1, 0],
            [0,   0,  1]
        ])

        # 關節 2 變換 (theta2 繞 Y 軸，在 L1 高度)
        T2 = np.eye(4)
        T2[:3, 3] = [-self.S1, 0, self.L1]
        c2, s2 = np.cos(theta[1]), np.sin(theta[1])
        R2 = np.array([
            [ c2, 0, s2],
            [ 0,  1, 0],
            [-s2, 0, c2]
        ])
        T2[:3, :3] = R2

        # 關節 3 變換 (theta3 繞 Y 軸，在 L2 處，有 -90° 偏移)
        T3 = np.eye(4)
        T3[:3, 3] = [0, 0, self.L2]
        c3, s3 = np.cos(theta[2] - np.pi/2), np.sin(theta[2] - np.pi/2)
        R3 = np.array([
            [ c3, 0, s3],
            [ 0,  1, 0],
            [-s3, 0, c3]
        ])
        T3[:3, :3] = R3

        # 關節 4 變換 (theta4 繞 Z 軸，在 S2 處)
        T4 = np.eye(4)
        T4[:3, 3] = [self.S2, 0, 0]
        c4, s4 = np.cos(theta[3]), np.sin(theta[3])
        R4 = np.array([
            [c4, -s4, 0],
            [s4,  c4, 0],
            [0,   0,  1]
        ])
        T4[:3, :3] = R4

        # 關節 5 變換 (theta5 繞 Y 軸，在 L3 處，初始 -90°)
        T5 = np.eye(4)
        T5[:3, 3] = [0, 0, self.L3]
        c5, s5 = np.cos(theta[4] - np.pi/2), np.sin(theta[4] - np.pi/2)
        R5 = np.array([
            [ c5, 0, s5],
            [ 0,  1, 0],
            [-s5, 0, c5]
        ])
        T5[:3, :3] = R5

        # 關節 6 變換 (theta6 繞局部 Z 軸旋轉)
        T6 = np.eye(4)
        # 不添加位移，只有旋轉
        c6, s6 = np.cos(theta[5]), np.sin(theta[5])
        R6 = np.array([
            [c6, -s6, 0],
            [s6,  c6, 0],
            [0,   0,  1]
        ])
        T6[:3, :3] = R6

        # ========== 計算累積變換 ==========

        # P1 (基座) - 使用初始平移
        if 'p1' in self.actors:
            transform = vtk.vtkTransform()
            tx, ty, tz = INITIAL_TRANSLATIONS.get('p1', (0, 0, 0))
            transform.Translate(tx, ty, tz)
            self.actors['p1'].SetUserTransform(transform)

        # P2 - 受 theta1 影響
        if 'p2' in self.actors:
            T_p2 = T1.copy()
            tx, ty, tz = INITIAL_TRANSLATIONS.get('p2', (0, 0, 0.23))
            T_p2[:3, 3] += np.array([tx, ty, tz])
            self.actors['p2'].SetUserTransform(self.matrix_to_vtk_transform(T_p2))

        # P3 - 受 theta1, theta2 影響
        if 'p3' in self.actors:
            T_p3 = T1 @ T2
            tx, ty, tz = INITIAL_TRANSLATIONS.get('p3', (-0.03, 0, 0.375))
            # 在局部座標系中添加偏移
            offset = np.array([tx + self.S1, ty, tz - self.L1])
            T_p3[:3, 3] += T_p3[:3, :3] @ offset
            self.actors['p3'].SetUserTransform(self.matrix_to_vtk_transform(T_p3))

        # P4 - 受 theta1, theta2, theta3 影響
        if 'p4' in self.actors:
            T_p4 = T1 @ T2 @ T3
            tx, ty, tz = INITIAL_TRANSLATIONS.get('p4', (-0.03, 0, 0.715))
            offset = np.array([tx + self.S1, ty, tz - self.L1 - self.L2])
            T_p4[:3, 3] += T_p4[:3, :3] @ offset
            self.actors['p4'].SetUserTransform(self.matrix_to_vtk_transform(T_p4))

        # P5 - 受 theta1~4 影響
        if 'p5' in self.actors:
            T_p5 = T1 @ T2 @ T3 @ T4
            tx, ty, tz = INITIAL_TRANSLATIONS.get('p5', (0.01, 0, 0.81))
            offset = np.array([tx + self.S1 - self.S2, ty, tz - self.L1 - self.L2])
            T_p5[:3, 3] += T_p5[:3, :3] @ offset
            self.actors['p5'].SetUserTransform(self.matrix_to_vtk_transform(T_p5))

        # P6 - 受 theta1~5 影響
        if 'p6' in self.actors:
            T_p6 = T1 @ T2 @ T3 @ T4 @ T5
            tx, ty, tz = INITIAL_TRANSLATIONS.get('p6', (0.01, 0, 1.053))
            offset = np.array([tx + self.S1 - self.S2, ty, tz - self.L1 - self.L2 - self.L3])
            T_p6[:3, 3] += T_p6[:3, :3] @ offset
            self.actors['p6'].SetUserTransform(self.matrix_to_vtk_transform(T_p6))

        # P7 - 受 theta1~5 影響
        if 'p7' in self.actors:
            T_p7 = T1 @ T2 @ T3 @ T4 @ T5
            tx, ty, tz = INITIAL_TRANSLATIONS.get('p7', (0.01, 0, 1.12))
            offset = np.array([tx + self.S1 - self.S2, ty, tz - self.L1 - self.L2 - self.L3])
            T_p7[:3, 3] += T_p7[:3, :3] @ offset
            self.actors['p7'].SetUserTransform(self.matrix_to_vtk_transform(T_p7))

        # P8 (末端執行器) - 繞自身中心的局部 Z 軸旋轉
        if 'p8' in self.actors:
            # 計算到 P8 位置的變換（不含 theta6）
            T_p8_base = T1 @ T2 @ T3 @ T4 @ T5

            # 獲取 P8 的初始平移
            tx, ty, tz = INITIAL_TRANSLATIONS.get('p8', (0.01, 0, 1.1395))

            # 在局部座標系中的偏移（相對於關節5）
            local_offset = np.array([tx + self.S1 - self.S2, ty, tz - self.L1 - self.L2 - self.L3, 1])

            # 轉換到全局座標
            global_offset = T_p8_base @ local_offset

            # 構建最終變換：先移動到正確位置，然後繞自身 Z 軸旋轉
            T_p8_final = T_p8_base.copy()

            # 應用 theta6 旋轉（在局部 Z 軸）
            T_p8_final[:3, :3] = T_p8_final[:3, :3] @ T6[:3, :3]

            # 設定位置（使用計算出的全局偏移）
            T_p8_final[:3, 3] = global_offset[:3]

            self.actors['p8'].SetUserTransform(self.matrix_to_vtk_transform(T_p8_final))

        # 更新末端標記（關節5位置）
        # 構建到關節5的變換
        T_to_joint5 = T1 @ T2 @ T3 @ T4

        # 關節5在局部座標系的 (0, 0, L3) 位置
        joint5_local = np.array([0, 0, self.L3, 1])
        joint5_global = T_to_joint5 @ joint5_local

        self.joint5_marker.SetPosition(
            joint5_global[0],
            joint5_global[1],
            joint5_global[2]
        )

        self.renderWindow.Render()

    def add_environment(self):
        """添加環境"""
        axes = vtk.vtkAxesActor()
        axes.SetTotalLength(0.3, 0.3, 0.3)
        axes.SetShaftTypeToCylinder()
        axes.SetCylinderRadius(0.01)
        self.renderer.AddActor(axes)

        self.add_grid()

    def add_grid(self, size=2.0, divisions=20):
        """網格地板"""
        plane = vtk.vtkPlaneSource()
        plane.SetOrigin(-size/2, -size/2, 0)
        plane.SetPoint1(size/2, -size/2, 0)
        plane.SetPoint2(-size/2, size/2, 0)
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
        """設定相機"""
        camera = self.renderer.GetActiveCamera()
        camera.SetPosition(1.5, 1.5, 1.2)
        camera.SetFocalPoint(0, 0, 0.6)
        camera.SetViewUp(0, 0, 1)
        self.renderer.ResetCamera()

    def reset_camera(self):
        """重置相機"""
        self.setup_camera()
        self.renderWindow.Render()

    def set_view(self, view_type):
        """設定視角"""
        camera = self.renderer.GetActiveCamera()

        if view_type == "top":
            camera.SetPosition(0, 0, 2.5)
            camera.SetFocalPoint(0, 0, 0.5)
            camera.SetViewUp(0, 1, 0)
        elif view_type == "side":
            camera.SetPosition(2.5, 0, 0.8)
            camera.SetFocalPoint(0, 0, 0.8)
            camera.SetViewUp(0, 0, 1)
        elif view_type == "front":
            camera.SetPosition(0, 2.5, 0.8)
            camera.SetFocalPoint(0, 0, 0.8)
            camera.SetViewUp(0, 0, 1)

        self.renderer.ResetCamera()
        self.renderWindow.Render()

    def render(self):
        """渲染"""
        if self.renderWindow:
            self.renderWindow.Render()