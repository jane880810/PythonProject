'''
20250930
RA605-710-GC 六軸機械手臂控制系統
根據 DH 參數表更新參數名稱與數值
'''

import open3d as o3d
import numpy as np
import tkinter as tk
import threading
import time
import sys
from scipy.optimize import minimize


# ---------- MeshNode 類別 ----------
class MeshNode:
    def __init__(self, mesh, name=""):
        self.mesh = mesh
        self.name = name
        self.children = []

    def add_child(self, child_node):
        self.children.append(child_node)

    def transform(self, transformation, center):
        self.mesh.rotate(transformation[:3, :3], center=center)
        self.mesh.translate(transformation[:3, 3])
        for child in self.children:
            child.transform(transformation, center)

    def get_all_meshes(self):
        meshes = [self.mesh]
        for child in self.children:
            meshes.extend(child.get_all_meshes())
        return meshes


# ---------- 載入 OBJ 模型 ----------
paths = [rf"/home/yahboom/Desktop/Obj/p{i}.obj" for i in range(1, 9)]
meshes = [o3d.io.read_triangle_mesh(p) for p in paths]
if not all(m.has_triangles() for m in meshes):
    print("有模型未正確載入")
    exit()
for m in meshes:
    m.compute_vertex_normals()

# 初始平移（避免重疊）
meshes[1].translate((0, 0, 0.23))
meshes[2].translate((-0.03, 0, 0.38))
meshes[3].translate((-0.03, 0, 0.72))
meshes[4].translate((0.01, 0, 0.81))
meshes[5].translate((0.01, 0, 1.05))
meshes[6].translate((0.01, 0, 1.12))
meshes[7].translate((0.01, 0, 1.13))

# ---------- 建立父子層級結構 ----------
node2 = MeshNode(meshes[1], "p2.obj")
node3 = MeshNode(meshes[2], "p3.obj")
node4 = MeshNode(meshes[3], "p4.obj")
node5 = MeshNode(meshes[4], "p5.obj")
node6 = MeshNode(meshes[5], "p6.obj")
node7 = MeshNode(meshes[6], "p7.obj")
node8 = MeshNode(meshes[7], "p8.obj")

node8_group = node8
node7_group = node7
node7_group.add_child(node8_group)
node6_group = node6
node6_group.add_child(node7_group)
node5_group = node5
node5_group.add_child(node6_group)
node4_group = node4
node4_group.add_child(node5_group)
node3.add_child(node4_group)
node2.add_child(node3)

root_nodes = [MeshNode(meshes[0], "p1.obj"), node2]

# ---------- 初始旋轉 ----------
p5_top_center = [0.01, 0, 1.05]
R = o3d.geometry.get_rotation_matrix_from_axis_angle([0, np.deg2rad(-90), 0])
T = np.eye(4)
T[:3, :3] = R
node6_group.transform(T, p5_top_center)

p3_base_local_init = np.array([-0.03, 0, 0.38])
p3_length_vec_init = np.array([0, 0, 0.34])
p3_top_center = p3_base_local_init + p3_length_vec_init
R_p4_init = o3d.geometry.get_rotation_matrix_from_axis_angle([0, np.deg2rad(-90), 0])
T_p4_init = np.eye(4)
T_p4_init[:3, :3] = R_p4_init
node4_group.transform(T_p4_init, p3_top_center)

# ---------- 建立視窗 ----------
vis = o3d.visualization.Visualizer()
vis.create_window("RA605-710-GC 機械手臂控制", width=1000, height=1000)

for root in root_nodes:
    for m in root.get_all_meshes():
        vis.add_geometry(m)

# ---------- 添加座標軸和網格 ----------
world_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3, origin=[0.375, 0.3, 0])
vis.add_geometry(world_frame)


def create_enhanced_grid(size=2.0, major_step=0.5, minor_step=0.1):
    lines = []
    points = []
    colors = []
    major_num = int(size / major_step) + 1
    minor_num = int(size / minor_step) + 1

    for i in range(major_num):
        coord = -size / 2 + i * major_step
        points.extend([[coord, -size / 2, 0], [coord, size / 2, 0]])
        line_idx = len(points) - 2
        lines.append([line_idx, line_idx + 1])
        colors.append([0.4, 0.4, 0.4])
        points.extend([[-size / 2, coord, 0], [size / 2, coord, 0]])
        line_idx = len(points) - 2
        lines.append([line_idx, line_idx + 1])
        colors.append([0.4, 0.4, 0.4])

    for i in range(minor_num):
        coord = -size / 2 + i * minor_step
        if abs(coord % major_step) > 0.01:
            points.extend([[coord, -size / 2, 0], [coord, size / 2, 0]])
            line_idx = len(points) - 2
            lines.append([line_idx, line_idx + 1])
            colors.append([0.8, 0.8, 0.8])
            points.extend([[-size / 2, coord, 0], [size / 2, coord, 0]])
            line_idx = len(points) - 2
            lines.append([line_idx, line_idx + 1])
            colors.append([0.8, 0.8, 0.8])

    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)
    return line_set


enhanced_grid = create_enhanced_grid(size=2.0, major_step=0.5, minor_step=0.1)
vis.add_geometry(enhanced_grid)

base_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.08, origin=[0, 0, 0])
vis.add_geometry(base_frame)

origin_marker = o3d.geometry.TriangleMesh.create_sphere(radius=0.03)
origin_marker.translate([0, 0, 0.02])
origin_marker.paint_uniform_color([1.0, 1.0, 1.0])
vis.add_geometry(origin_marker)


# ---------- 動畫控制器 ----------
class AnimationController:
    def __init__(self):
        # ===== DH 參數（根據 RA605-710-GC 規格）=====
        # 表 3-1: 機構長度參數（單位：米）
        self.S1 = 0.030  # 30 mm
        self.S2 = 0.040  # 40 mm
        self.L1 = 0.375  # 375 mm
        self.L2 = 0.340  # 340 mm
        self.L3 = 0.338  # 338 mm
        self.L4 = 0.0865  # 86.5 mm

        # 打印 DH 參數
        print("\n" + "=" * 60)
        print("RA605-710-GC DH 參數設定")
        print("=" * 60)
        print(f"S1 = {self.S1 * 1000:.1f} mm")
        print(f"S2 = {self.S2 * 1000:.1f} mm")
        print(f"L1 = {self.L1 * 1000:.1f} mm")
        print(f"L2 = {self.L2 * 1000:.1f} mm")
        print(f"L3 = {self.L3 * 1000:.1f} mm")
        print(f"L4 = {self.L4 * 1000:.1f} mm")
        print("=" * 60 + "\n")

        # 當前角度和目標角度
        self.current_angles = [0, 0, 0, 0, 0, 0]
        self.target_angles = [0, 0, 0, 0, 0, 0]
        self.default_angles = [0, 0, 0, 0, 0, 0]

        # 動畫參數
        self.animation_speed = 0.12
        self.is_animating = False

        # 累積旋轉矩陣
        self.R_p2_total = np.eye(3)
        self.R_p3_total = np.eye(3)
        self.R_p4_total = o3d.geometry.get_rotation_matrix_from_axis_angle([0, np.deg2rad(-90), 0])
        self.R_p5_total = np.eye(3)
        self.R_p6_total = np.eye(3)
        self.R_p8_total = np.eye(3)

        # 幾何向量（基於實際3D模型，對應DH參數命名）
        # 注意：這些向量是根據實際3D模型的初始位置計算得出
        # 與DH參數表中的理論值可能略有差異，但能正確匹配實際模型
        self.p3_base_local = np.array([-0.03, 0, 0.38])  # P2到P3基座（≈-S1, 0, L1+offset）
        self.p3_length_vec = np.array([0, 0, 0.34])  # P3長度（≈L2）
        self.p4_length_vec = np.array([0.04, 0, 0.09])  # P4長度向量（含S2偏移）
        self.p5_length_vec = np.array([0, 0, 0.24])  # P5長度向量（≈L3部分）
        self.p6_length_vec = np.array([0, 0, 0.07])  # P6長度
        self.p7_length_vec = np.array([0, 0, 0.01])  # P7長度
        self.p8_center_offset = np.array([0, 0, -0.08])  # P8中心偏移（≈-L4）

        # 關節5標記
        self.joint5_marker = None

    def set_marker(self, marker):
        self.joint5_marker = marker

    def smooth_interpolate(self, current, target, speed):
        diff = target - current
        if abs(diff) < 0.1:
            return target
        return current + diff * speed

    def get_joint5_position(self):
        """計算關節 5 (P4-B 點) 的位置"""
        position = self.R_p2_total @ (
                self.p3_base_local +
                self.R_p3_total @ (
                        self.p3_length_vec +
                        self.R_p4_total @ (
                                self.p4_length_vec +
                                self.R_p5_total @ self.p5_length_vec
                        )
                )
        )
        return position

    def update_joint5_marker_position(self):
        if self.joint5_marker is None:
            return
        current_pos = self.get_joint5_position()
        vis.remove_geometry(self.joint5_marker, reset_bounding_box=False)
        self.joint5_marker = o3d.geometry.TriangleMesh.create_sphere(radius=0.08)
        self.joint5_marker.paint_uniform_color([1.0, 0.0, 0.0])
        self.joint5_marker.compute_vertex_normals()
        self.joint5_marker.translate(current_pos)
        vis.add_geometry(self.joint5_marker, reset_bounding_box=False)

    def get_all_joint_positions(self):
        """獲取所有關節位置（對應 DH 參數定義）"""
        positions = {}

        # Joint 1 (P1-A): 基座原點
        positions['Joint_1'] = np.array([0, 0, 0])

        # Joint 2 (P2-A): 第一關節後
        positions['Joint_2'] = self.R_p2_total @ self.p3_base_local

        # Joint 3 (P3-A): 第二關節後
        positions['Joint_3'] = self.R_p2_total @ (
                self.p3_base_local +
                self.R_p3_total @ self.p3_length_vec
        )

        # Joint 4 (P4-A): 第三關節後
        positions['Joint_4'] = self.R_p2_total @ (
                self.p3_base_local +
                self.R_p3_total @ (
                        self.p3_length_vec +
                        self.R_p4_total @ self.p4_length_vec
                )
        )

        # Joint 5 (P4-B): 第四關節後（紅點標記位置）
        positions['Joint_5'] = self.get_joint5_position()

        # Joint 6 (末端執行器): 第六關節
        p8_base = self.R_p2_total @ (
                self.p3_base_local +
                self.R_p3_total @ (
                        self.p3_length_vec +
                        self.R_p4_total @ (
                                self.p4_length_vec +
                                self.R_p5_total @ (
                                        self.p5_length_vec +
                                        self.R_p6_total @ (
                                                self.p6_length_vec + self.p7_length_vec
                                        )
                                )
                        )
                )
        )
        p8_center_offset_world = (
                self.R_p2_total @
                self.R_p3_total @
                self.R_p4_total @
                self.R_p5_total @
                self.R_p6_total @
                self.p8_center_offset
        )
        positions['Joint_6'] = p8_base + p8_center_offset_world

        return positions

    def update_joint(self, joint_idx, delta_angle):
        if abs(delta_angle) < 0.001:
            return

        delta_rad = np.deg2rad(delta_angle)

        if joint_idx == 0:  # 關節1 (Z軸)
            R = o3d.geometry.get_rotation_matrix_from_axis_angle([0, 0, delta_rad])
            self.R_p2_total[:] = R @ self.R_p2_total
            T = np.eye(4)
            T[:3, :3] = R
            node2.transform(T, center=[0, 0, 0])

        elif joint_idx == 1:  # 關節2 (Y軸)
            R_local = o3d.geometry.get_rotation_matrix_from_axis_angle([0, delta_rad, 0])
            self.R_p3_total[:] = R_local @ self.R_p3_total
            local_y = self.R_p2_total @ np.array([0, 1, 0])
            R_world = o3d.geometry.get_rotation_matrix_from_axis_angle(local_y * delta_rad)
            center = self.R_p2_total @ self.p3_base_local
            T = np.eye(4)
            T[:3, :3] = R_world
            node3.transform(T, center)

        elif joint_idx == 2:  # 關節3 (Y軸)
            R_local = o3d.geometry.get_rotation_matrix_from_axis_angle([0, delta_rad, 0])
            self.R_p4_total[:] = R_local @ self.R_p4_total
            local_y = self.R_p2_total @ np.array([0, 1, 0])
            R_world = o3d.geometry.get_rotation_matrix_from_axis_angle(local_y * delta_rad)
            center = self.R_p2_total @ (self.p3_base_local + self.R_p3_total @ self.p3_length_vec)
            T = np.eye(4)
            T[:3, :3] = R_world
            node4_group.transform(T, center)

        elif joint_idx == 3:  # 關節4 (Z軸)
            R_local = o3d.geometry.get_rotation_matrix_from_axis_angle([0, 0, delta_rad])
            self.R_p5_total[:] = R_local @ self.R_p5_total
            p4_world_z = self.R_p2_total @ self.R_p3_total @ self.R_p4_total @ np.array([0, 0, 1])
            R_world = o3d.geometry.get_rotation_matrix_from_axis_angle(p4_world_z * delta_rad)
            center = self.R_p2_total @ (self.p3_base_local + self.R_p3_total @ (
                    self.p3_length_vec + self.R_p4_total @ self.p4_length_vec))
            T = np.eye(4)
            T[:3, :3] = R_world
            node5_group.transform(T, center)

        elif joint_idx == 4:  # 關節5 (Y軸)
            R_local = o3d.geometry.get_rotation_matrix_from_axis_angle([0, delta_rad, 0])
            self.R_p6_total[:] = R_local @ self.R_p6_total
            p5_world_y = self.R_p2_total @ self.R_p3_total @ self.R_p4_total @ self.R_p5_total @ np.array([0, 1, 0])
            R_world = o3d.geometry.get_rotation_matrix_from_axis_angle(p5_world_y * delta_rad)
            center = self.R_p2_total @ (self.p3_base_local + self.R_p3_total @ (self.p3_length_vec + self.R_p4_total @ (
                    self.p4_length_vec + self.R_p5_total @ self.p5_length_vec)))
            T = np.eye(4)
            T[:3, :3] = R_world
            node6_group.transform(T, center)

        elif joint_idx == 5:  # 關節6 (X軸)
            R_local = o3d.geometry.get_rotation_matrix_from_axis_angle([delta_rad, 0, 0])
            self.R_p8_total[:] = R_local @ self.R_p8_total
            p8_world_x = self.R_p2_total @ self.R_p3_total @ self.R_p4_total @ self.R_p5_total @ self.R_p6_total @ np.array(
                [1, 0, 0])
            R_world = o3d.geometry.get_rotation_matrix_from_axis_angle(p8_world_x * delta_rad)
            p8_base = self.R_p2_total @ (self.p3_base_local + self.R_p3_total @ (
                    self.p3_length_vec + self.R_p4_total @ (self.p4_length_vec + self.R_p5_total @ (
                    self.p5_length_vec + self.R_p6_total @ (self.p6_length_vec + self.p7_length_vec)))))
            p8_center_offset_world = self.R_p2_total @ self.R_p3_total @ self.R_p4_total @ self.R_p5_total @ self.R_p6_total @ self.p8_center_offset
            center = p8_base + p8_center_offset_world
            T = np.eye(4)
            T[:3, :3] = R_world
            node8_group.transform(T, center)

        # 更新幾何
        if joint_idx == 0:
            for m in node2.get_all_meshes():
                vis.update_geometry(m)
        elif joint_idx == 1:
            for m in node3.get_all_meshes():
                vis.update_geometry(m)
        elif joint_idx == 2:
            for m in node4_group.get_all_meshes():
                vis.update_geometry(m)
        elif joint_idx == 3:
            for m in node5_group.get_all_meshes():
                vis.update_geometry(m)
        elif joint_idx == 4:
            for m in node6_group.get_all_meshes():
                vis.update_geometry(m)
        elif joint_idx == 5:
            for m in node8_group.get_all_meshes():
                vis.update_geometry(m)

        self.update_joint5_marker_position()

    def animate(self):
        if not self.is_animating:
            return
        moved = False
        for i in range(6):
            if abs(self.current_angles[i] - self.target_angles[i]) > 0.1:
                old_angle = self.current_angles[i]
                self.current_angles[i] = self.smooth_interpolate(
                    self.current_angles[i],
                    self.target_angles[i],
                    self.animation_speed
                )
                delta = self.current_angles[i] - old_angle
                self.update_joint(i, delta)
                moved = True
        if not moved:
            self.is_animating = False

    def set_target(self, joint_idx, angle):
        self.target_angles[joint_idx] = angle
        self.is_animating = True

    def reset_to_default(self):
        for i in range(6):
            self.target_angles[i] = self.default_angles[i]
        self.is_animating = True

    def calculate_forward_kinematics(self, angles):
        """正向運動學：使用實際幾何參數計算末端位置"""
        R_p2 = o3d.geometry.get_rotation_matrix_from_axis_angle([0, 0, np.deg2rad(angles[0])])
        R_p3 = o3d.geometry.get_rotation_matrix_from_axis_angle([0, np.deg2rad(angles[1]), 0])
        R_p4_base = o3d.geometry.get_rotation_matrix_from_axis_angle([0, np.deg2rad(-90), 0])
        R_p4 = o3d.geometry.get_rotation_matrix_from_axis_angle([0, np.deg2rad(angles[2]), 0]) @ R_p4_base
        R_p5 = o3d.geometry.get_rotation_matrix_from_axis_angle([0, 0, np.deg2rad(angles[3])])

        position = R_p2 @ (
                self.p3_base_local +
                R_p3 @ (
                        self.p3_length_vec +
                        R_p4 @ (
                                self.p4_length_vec +
                                R_p5 @ self.p5_length_vec
                        )
                )
        )
        return position

    def inverse_kinematics(self, target_pos, initial_guess=None):
        """逆向運動學：計算到達目標位置的關節角度"""
        if initial_guess is None:
            initial_guess = self.current_angles[:4]

        def objective(angles):
            current_pos = self.calculate_forward_kinematics(angles)
            error = np.linalg.norm(current_pos - target_pos)
            return error

        bounds = [
            (-165, 165),  # Joint 1: θ1
            (-125, 85),  # Joint 2: θ2
            (-55, 185),  # Joint 3: θ3
            (-190, 190)  # Joint 4: θ4
        ]

        result = minimize(
            objective,
            initial_guess,
            method='SLSQP',
            bounds=bounds,
            options={'ftol': 1e-6, 'maxiter': 200}
        )

        if result.success:
            final_pos = self.calculate_forward_kinematics(result.x)
            error = np.linalg.norm(final_pos - target_pos)
            return result.x, error, True
        else:
            return initial_guess, float('inf'), False

    def move_to_position(self, target_x, target_y, target_z):
        """IK 控制：移動到目標位置"""
        target_pos = np.array([target_x, target_y, target_z])
        angles, error, success = self.inverse_kinematics(target_pos)

        if success and error < 0.01:
            for i in range(4):
                self.set_target(i, angles[i])
            return True, error, angles
        else:
            return False, error, angles


anim_controller = AnimationController()
sliders = {}
exit_flag = False


# ---------- GUI 控制 ----------
def start_gui():
    def on_z_slider(value):
        anim_controller.set_target(0, int(value))

    def on_y_slider(value):
        anim_controller.set_target(1, int(value))

    def on_p4_y_slider(value):
        anim_controller.set_target(2, int(value))

    def on_p5_z_slider(value):
        anim_controller.set_target(3, int(value))

    def on_p6_y_slider(value):
        anim_controller.set_target(4, int(value))

    def on_p8_z_slider(value):
        anim_controller.set_target(5, int(value))

    def reset_pose():
        anim_controller.reset_to_default()
        for i, (slider_name, default_value) in enumerate([
            ('z_slider', 0), ('y_slider', 0), ('p4_slider', 0),
            ('p5_z_slider', 0), ('p6_y_slider', 0), ('p8_z_slider', 0)
        ]):
            if slider_name in sliders:
                sliders[slider_name].set(default_value)

    def exit_program():
        global exit_flag
        exit_flag = True
        root.quit()
        root.destroy()

    root = tk.Tk()
    root.title("RA605-710-GC 控制面板")
    root.configure(bg="#FFD9EC")

    # 關節控制
    tk.Label(root, text="Joint 1 (θ1) - Z軸旋轉", bg="#FFD9EC", font=("Arial", 9)).pack()
    z_slider = tk.Scale(root, from_=-165, to=165, orient=tk.HORIZONTAL, length=250, command=on_z_slider)
    z_slider.pack(padx=15, pady=5)
    sliders['z_slider'] = z_slider

    tk.Label(root, text="Joint 2 (θ2) - Y軸旋轉", bg="#FFD9EC", font=("Arial", 9)).pack()
    y_slider = tk.Scale(root, from_=-125, to=85, orient=tk.HORIZONTAL, length=250, command=on_y_slider)
    y_slider.pack(padx=15, pady=5)
    sliders['y_slider'] = y_slider

    tk.Label(root, text="Joint 3 (θ3) - Y軸旋轉", bg="#FFD9EC", font=("Arial", 9)).pack()
    p4_slider = tk.Scale(root, from_=-55, to=185, orient=tk.HORIZONTAL, length=250, command=on_p4_y_slider)
    p4_slider.set(0)
    p4_slider.pack(padx=15, pady=5)
    sliders['p4_slider'] = p4_slider

    tk.Label(root, text="Joint 4 (θ4) - Z軸旋轉", bg="#FFD9EC", font=("Arial", 9)).pack()
    p5_z_slider = tk.Scale(root, from_=-190, to=190, orient=tk.HORIZONTAL, length=250, command=on_p5_z_slider)
    p5_z_slider.pack(padx=15, pady=5)
    sliders['p5_z_slider'] = p5_z_slider

    tk.Label(root, text="Joint 5 (θ5) - Y軸旋轉", bg="#FFD9EC", font=("Arial", 9)).pack()
    p6_y_slider = tk.Scale(root, from_=-25, to=205, orient=tk.HORIZONTAL, length=250, command=on_p6_y_slider)
    p6_y_slider.pack(padx=15, pady=5)
    sliders['p6_y_slider'] = p6_y_slider

    tk.Label(root, text="Joint 6 (θ6) - X軸自轉", bg="#FFD9EC", font=("Arial", 9)).pack()
    p8_z_slider = tk.Scale(root, from_=-360, to=360, orient=tk.HORIZONTAL, length=250, command=on_p8_z_slider)
    p8_z_slider.pack(padx=15, pady=5)
    sliders['p8_z_slider'] = p8_z_slider

    tk.Label(root, text="動畫速度", bg="#FFD9EC", font=("Arial", 9)).pack()

    def on_speed_change(value):
        anim_controller.animation_speed = float(value) / 100.0

    speed_slider = tk.Scale(root, from_=5, to=30, orient=tk.HORIZONTAL, length=250, command=on_speed_change)
    speed_slider.set(12)
    speed_slider.pack(padx=15, pady=5)

    # 位置顯示
    position_frame = tk.Frame(root, bg="#E6F3FF", relief=tk.RIDGE, bd=2)
    position_frame.pack(pady=5, padx=15, fill=tk.BOTH)
    tk.Label(position_frame, text="Joint 5 (P4-B) 當前位置",
             font=("Arial", 10, "bold"), bg="#E6F3FF", fg="#FF0000").pack(pady=3)

    x_frame = tk.Frame(position_frame, bg="#E6F3FF")
    x_frame.pack(pady=1)
    tk.Label(x_frame, text="X: ", font=("Arial", 9), bg="#E6F3FF").pack(side=tk.LEFT)
    x_label = tk.Label(x_frame, text="0.000", font=("Arial", 9, "bold"),
                       bg="#FFFFFF", width=10, relief=tk.SUNKEN)
    x_label.pack(side=tk.LEFT, padx=3)

    y_frame = tk.Frame(position_frame, bg="#E6F3FF")
    y_frame.pack(pady=1)
    tk.Label(y_frame, text="Y: ", font=("Arial", 9), bg="#E6F3FF").pack(side=tk.LEFT)
    y_label = tk.Label(y_frame, text="0.000", font=("Arial", 9, "bold"),
                       bg="#FFFFFF", width=10, relief=tk.SUNKEN)
    y_label.pack(side=tk.LEFT, padx=3)

    z_frame = tk.Frame(position_frame, bg="#E6F3FF")
    z_frame.pack(pady=1)
    tk.Label(z_frame, text="Z: ", font=("Arial", 9), bg="#E6F3FF").pack(side=tk.LEFT)
    z_label = tk.Label(z_frame, text="0.000", font=("Arial", 9, "bold"),
                       bg="#FFFFFF", width=10, relief=tk.SUNKEN)
    z_label.pack(side=tk.LEFT, padx=3)

    def update_position_display():
        try:
            pos = anim_controller.get_joint5_position()
            x_label.config(text=f"{pos[0]:.4f}")
            y_label.config(text=f"{pos[1]:.4f}")
            z_label.config(text=f"{pos[2]:.4f}")
            root.after(50, update_position_display)
        except:
            pass

    update_position_display()

    # 目標位置輸入
    target_frame = tk.Frame(root, bg="#FFE6E6", relief=tk.RIDGE, bd=2)
    target_frame.pack(pady=5, padx=15, fill=tk.BOTH)
    tk.Label(target_frame, text="逆向運動學控制",
             font=("Arial", 10, "bold"), bg="#FFE6E6", fg="#CC0000").pack(pady=3)

    target_x_frame = tk.Frame(target_frame, bg="#FFE6E6")
    target_x_frame.pack(pady=1)
    tk.Label(target_x_frame, text="目標 X: ", font=("Arial", 9), bg="#FFE6E6").pack(side=tk.LEFT)
    target_x_entry = tk.Entry(target_x_frame, width=10, font=("Arial", 9))
    target_x_entry.insert(0, "0.0")
    target_x_entry.pack(side=tk.LEFT, padx=3)

    target_y_frame = tk.Frame(target_frame, bg="#FFE6E6")
    target_y_frame.pack(pady=1)
    tk.Label(target_y_frame, text="目標 Y: ", font=("Arial", 9), bg="#FFE6E6").pack(side=tk.LEFT)
    target_y_entry = tk.Entry(target_y_frame, width=10, font=("Arial", 9))
    target_y_entry.insert(0, "0.0")
    target_y_entry.pack(side=tk.LEFT, padx=3)

    target_z_frame = tk.Frame(target_frame, bg="#FFE6E6")
    target_z_frame.pack(pady=1)
    tk.Label(target_z_frame, text="目標 Z: ", font=("Arial", 9), bg="#FFE6E6").pack(side=tk.LEFT)
    target_z_entry = tk.Entry(target_z_frame, width=10, font=("Arial", 9))
    target_z_entry.insert(0, "1.0")
    target_z_entry.pack(side=tk.LEFT, padx=3)

    result_label = tk.Label(target_frame, text="", font=("Arial", 8), bg="#FFE6E6", fg="#006600")
    result_label.pack(pady=2)

    def move_to_target():
        try:
            target_x = float(target_x_entry.get())
            target_y = float(target_y_entry.get())
            target_z = float(target_z_entry.get())
            result_label.config(text="計算中...", fg="#0000CC")
            root.update()
            success, error, angles = anim_controller.move_to_position(target_x, target_y, target_z)
            if success:
                result_label.config(text=f"成功！誤差: {error * 1000:.2f}mm", fg="#006600")
                sliders['z_slider'].set(int(angles[0]))
                sliders['y_slider'].set(int(angles[1]))
                sliders['p4_slider'].set(int(angles[2]))
                sliders['p5_z_slider'].set(int(angles[3]))
            else:
                result_label.config(text=f"無法到達 (誤差: {error * 1000:.1f}mm)", fg="#CC0000")
        except ValueError:
            result_label.config(text="請輸入有效數值！", fg="#CC0000")
        except Exception as e:
            result_label.config(text=f"錯誤: {str(e)}", fg="#CC0000")

    move_button = tk.Button(target_frame, text="移動到目標位置", command=move_to_target,
                            font=("Arial", 9, "bold"), bg="#AADDFF", padx=10, pady=3)
    move_button.pack(pady=3)

    def set_current_as_target():
        pos = anim_controller.get_joint5_position()
        target_x_entry.delete(0, tk.END)
        target_x_entry.insert(0, f"{pos[0]:.4f}")
        target_y_entry.delete(0, tk.END)
        target_y_entry.insert(0, f"{pos[1]:.4f}")
        target_z_entry.delete(0, tk.END)
        target_z_entry.insert(0, f"{pos[2]:.4f}")
        result_label.config(text="已設定為當前位置", fg="#006600")

    set_current_button = tk.Button(target_frame, text="使用當前位置", command=set_current_as_target,
                                   font=("Arial", 8), bg="#DDDDDD", padx=5, pady=2)
    set_current_button.pack(pady=2)

    # 按鈕
    button_frame = tk.Frame(root, bg="#FFD9EC")
    button_frame.pack(pady=8)

    reset_button = tk.Button(button_frame, text="回復預設姿態", command=reset_pose,
                             font=("Arial", 9, "bold"), bg="#B8E6B8", padx=10, pady=5)
    reset_button.pack(side=tk.LEFT, padx=5)

    def print_all_positions():
        positions = anim_controller.get_all_joint_positions()
        print("\n" + "=" * 60)
        print("RA605-710-GC 當前關節位置")
        print("=" * 60)
        for joint_name, pos in positions.items():
            print(f"{joint_name}: X={pos[0]:.4f}m, Y={pos[1]:.4f}m, Z={pos[2]:.4f}m")
        print("=" * 60 + "\n")

    print_button = tk.Button(button_frame, text="打印關節位置", command=print_all_positions,
                             font=("Arial", 9, "bold"), bg="#FFE6CC", padx=10, pady=5)
    print_button.pack(side=tk.LEFT, padx=5)

    exit_button = tk.Button(button_frame, text="結束程式", command=exit_program,
                            font=("Arial", 9, "bold"), bg="#FFB6C1", padx=10, pady=5)
    exit_button.pack(side=tk.LEFT, padx=5)

    root.protocol("WM_DELETE_WINDOW", exit_program)
    root.mainloop()


# ---------- 主循環 ----------
def open3d_loop():
    global exit_flag
    initial_joint5_pos = anim_controller.get_joint5_position()
    joint5_marker = o3d.geometry.TriangleMesh.create_sphere(radius=0.08)
    joint5_marker.paint_uniform_color([1.0, 0.0, 0.0])
    joint5_marker.compute_vertex_normals()
    joint5_marker.translate(initial_joint5_pos)
    vis.add_geometry(joint5_marker)
    anim_controller.set_marker(joint5_marker)

    threading.Thread(target=start_gui, daemon=True).start()
    last_time = time.time()

    try:
        while True:
            if exit_flag:
                print("正在安全關閉程式...")
                break
            current_time = time.time()
            if current_time - last_time > 1.0 / 60.0:
                anim_controller.animate()
                last_time = current_time
            vis.poll_events()
            vis.update_renderer()
    except Exception as e:
        print(f"程式執行中發生錯誤：{e}")
    finally:
        try:
            vis.destroy_window()
        except:
            pass
        print("程式已安全關閉")
        sys.exit(0)


open3d_loop()