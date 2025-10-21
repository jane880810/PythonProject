'''
20250930
RA605-710-GC 六軸機械手臂控制系統
使用解析解逆向運動學（基於公式 3.13-3.21）- 修正版

座標系說明：
- 統一使用單一座標系：X, Y, Z
- GUI顯示座標系與3D視圖座標系相同
'''

import open3d as o3d
import numpy as np
import tkinter as tk
import threading
import time
import sys


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


# ---------- DH 參數定義 ----------
# RA605-710-GC 機械手臂 DH 參數
S1 = 0.030  # 30 mm
S2 = 0.040  # 40 mm
L1 = 0.375  # 375 mm
L2 = 0.340  # 340 mm
L3 = 0.338  # 338 mm
L4 = 0.0865  # 86.5 mm

print("\n" + "=" * 60)
print("RA605-710-GC DH 參數")
print("=" * 60)
print(f"S1 = {S1 * 1000:.1f} mm")
print(f"S2 = {S2 * 1000:.1f} mm")
print(f"L1 = {L1 * 1000:.1f} mm")
print(f"L2 = {L2 * 1000:.1f} mm")
print(f"L3 = {L3 * 1000:.1f} mm")
print(f"L4 = {L4 * 1000:.1f} mm")
print("=" * 60 + "\n")

# ---------- 載入 OBJ 模型 ----------
paths = [rf"/home/yahboom/Desktop/Obj/p{i}.obj" for i in range(1, 9)]
meshes = [o3d.io.read_triangle_mesh(p) for p in paths]
if not all(m.has_triangles() for m in meshes):
    print("有模型未正確載入")
    exit()
for m in meshes:
    m.compute_vertex_normals()

# 初始平移（保持原始值，用於正確顯示3D模型）
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

# ---------- 初始旋轉（使用DH參數計算旋轉中心）----------
# P5頂端中心（P6基座），用於P6-P8的初始旋轉
p5_top_center = [S2 - S1, 0, L1 + L2 + L3]
R = o3d.geometry.get_rotation_matrix_from_axis_angle([0, np.deg2rad(-90), 0])
T = np.eye(4)
T[:3, :3] = R
node6_group.transform(T, p5_top_center)

# P3頂端中心（P4基座），用於P4-P8的初始旋轉
p3_top_center = [-S1, 0, L1 + L2]
R_p4_init = o3d.geometry.get_rotation_matrix_from_axis_angle([0, np.deg2rad(-90), 0])
T_p4_init = np.eye(4)
T_p4_init[:3, :3] = R_p4_init
node4_group.transform(T_p4_init, p3_top_center)

# ---------- 建立視窗 ----------
vis = o3d.visualization.Visualizer()
vis.create_window("六軸手臂 3D 視圖", width=500, height=500)

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
        # ===== DH 參數（RA605-710-GC）=====
        self.S1 = S1
        self.S2 = S2
        self.L1 = L1
        self.L2 = L2
        self.L3 = L3
        self.L4 = L4

        # 關節角度
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

        # 幾何向量
        self.p3_base_local = np.array([-self.S1, 0, self.L1])
        self.p3_length_vec = np.array([0, 0, self.L2])
        self.p4_length_vec = np.array([self.S2, 0, 0])
        self.p5_length_vec = np.array([0, 0, self.L3])
        self.p6_length_vec = np.array([0, 0, 0.07])
        self.p7_length_vec = np.array([0, 0, 0.01])
        self.p8_center_offset = np.array([self.L4, 0, 0])

        # 關節5標記
        self.joint5_marker = None

    def set_marker(self, marker):
        self.joint5_marker = marker

    def smooth_interpolate(self, current, target, speed):
        diff = target - current
        if abs(diff) < 0.01:
            return target
        return current + diff * speed

    def get_joint5_position_internal(self):
        """計算關節5的位置（用於3D顯示）"""
        position_internal = self.R_p2_total @ (
                self.p3_base_local +
                self.R_p3_total @ (
                        self.p3_length_vec +
                        self.R_p4_total @ (
                                self.p4_length_vec +
                                self.R_p5_total @ self.p5_length_vec
                        )
                )
        )
        return position_internal

    def get_joint5_position(self):
        """計算關節5位置"""
        return self.get_joint5_position_internal()

    def update_joint5_marker_position(self):
        """更新關節5標記位置"""
        if self.joint5_marker is None:
            return

        current_pos_internal = self.get_joint5_position_internal()
        vis.remove_geometry(self.joint5_marker, reset_bounding_box=False)

        self.joint5_marker = o3d.geometry.TriangleMesh.create_sphere(radius=0.08)
        self.joint5_marker.paint_uniform_color([1.0, 0.0, 0.0])
        self.joint5_marker.compute_vertex_normals()
        self.joint5_marker.translate(current_pos_internal)

        vis.add_geometry(self.joint5_marker, reset_bounding_box=False)

    def get_all_joint_positions(self):
        positions = {}
        positions['Joint_1'] = np.array([0, 0, 0])
        positions['Joint_2'] = self.R_p2_total @ self.p3_base_local
        positions['Joint_3'] = self.R_p2_total @ (
                self.p3_base_local + self.R_p3_total @ self.p3_length_vec
        )
        positions['Joint_4'] = self.R_p2_total @ (
                self.p3_base_local + self.R_p3_total @ (
                self.p3_length_vec + self.R_p4_total @ self.p4_length_vec
        )
        )
        positions['Joint_5'] = self.get_joint5_position()
        p8_base = self.R_p2_total @ (
                self.p3_base_local + self.R_p3_total @ (
                self.p3_length_vec + self.R_p4_total @ (
                self.p4_length_vec + self.R_p5_total @ (
                self.p5_length_vec + self.R_p6_total @ (
                self.p6_length_vec + self.p7_length_vec
        )
        )
        )
        )
        )
        p8_center_offset_world = (
                self.R_p2_total @ self.R_p3_total @ self.R_p4_total @
                self.R_p5_total @ self.R_p6_total @ self.p8_center_offset
        )
        positions['Joint_6'] = p8_base + p8_center_offset_world
        return positions

    def update_joint(self, joint_idx, delta_angle):
        if abs(delta_angle) < 0.0001:
            return

        delta_rad = np.deg2rad(delta_angle)

        if joint_idx == 0:  # Joint 1
            R = o3d.geometry.get_rotation_matrix_from_axis_angle([0, 0, delta_rad])
            self.R_p2_total[:] = R @ self.R_p2_total
            T = np.eye(4)
            T[:3, :3] = R
            node2.transform(T, center=[0, 0, 0])

        elif joint_idx == 1:  # Joint 2
            R_local = o3d.geometry.get_rotation_matrix_from_axis_angle([0, delta_rad, 0])
            self.R_p3_total[:] = R_local @ self.R_p3_total
            local_y = self.R_p2_total @ np.array([0, 1, 0])
            R_world = o3d.geometry.get_rotation_matrix_from_axis_angle(local_y * delta_rad)
            center = self.R_p2_total @ self.p3_base_local
            T = np.eye(4)
            T[:3, :3] = R_world
            node3.transform(T, center)

        elif joint_idx == 2:  # Joint 3
            R_local = o3d.geometry.get_rotation_matrix_from_axis_angle([0, delta_rad, 0])
            self.R_p4_total[:] = R_local @ self.R_p4_total
            local_y = self.R_p2_total @ np.array([0, 1, 0])
            R_world = o3d.geometry.get_rotation_matrix_from_axis_angle(local_y * delta_rad)
            center = self.R_p2_total @ (self.p3_base_local + self.R_p3_total @ self.p3_length_vec)
            T = np.eye(4)
            T[:3, :3] = R_world
            node4_group.transform(T, center)

        elif joint_idx == 3:  # Joint 4
            R_local = o3d.geometry.get_rotation_matrix_from_axis_angle([0, 0, delta_rad])
            self.R_p5_total[:] = R_local @ self.R_p5_total
            p4_world_z = self.R_p2_total @ self.R_p3_total @ self.R_p4_total @ np.array([0, 0, 1])
            R_world = o3d.geometry.get_rotation_matrix_from_axis_angle(p4_world_z * delta_rad)
            center = self.R_p2_total @ (self.p3_base_local + self.R_p3_total @ (
                    self.p3_length_vec + self.R_p4_total @ self.p4_length_vec))
            T = np.eye(4)
            T[:3, :3] = R_world
            node5_group.transform(T, center)

        elif joint_idx == 4:  # Joint 5
            R_local = o3d.geometry.get_rotation_matrix_from_axis_angle([0, delta_rad, 0])
            self.R_p6_total[:] = R_local @ self.R_p6_total
            p5_world_y = self.R_p2_total @ self.R_p3_total @ self.R_p4_total @ self.R_p5_total @ np.array([0, 1, 0])
            R_world = o3d.geometry.get_rotation_matrix_from_axis_angle(p5_world_y * delta_rad)
            center = self.R_p2_total @ (self.p3_base_local + self.R_p3_total @ (self.p3_length_vec + self.R_p4_total @ (
                    self.p4_length_vec + self.R_p5_total @ self.p5_length_vec)))
            T = np.eye(4)
            T[:3, :3] = R_world
            node6_group.transform(T, center)


        elif joint_idx == 5:  # Joint 6

            R_local = o3d.geometry.get_rotation_matrix_from_axis_angle([delta_rad, 0, 0])

            self.R_p8_total[:] = R_local @ self.R_p8_total

            p8_world_x = self.R_p2_total @ self.R_p3_total @ self.R_p4_total @ self.R_p5_total @ self.R_p6_total @ np.array(

                [1, 0, 0])

            R_world = o3d.geometry.get_rotation_matrix_from_axis_angle(p8_world_x * delta_rad)

            # 計算 P8 基座位置（關節6的位置）

            center = self.R_p2_total @ (self.p3_base_local + self.R_p3_total @ (

                    self.p3_length_vec + self.R_p4_total @ (self.p4_length_vec + self.R_p5_total @ (

                    self.p5_length_vec + self.R_p6_total @ (self.p6_length_vec + self.p7_length_vec)))))

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
            if abs(self.current_angles[i] - self.target_angles[i]) > 0.01:
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
        """
        正向運動學：計算末端執行器位置
        """
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

    def inverse_kinematics_analytical(self, x4A, y4A, z4A):
        """
        嚴格按照公式 3.13-3.15 實現的逆向運動學（修正版）
        輸入: 目標點座標 (x4A, y4A, z4A) - 直接使用輸入座標
        輸出: (θ1, θ2, θ3, θ4) 或 None
        """
        try:
            # ===== 直接使用輸入座標，不進行轉換 =====
            x_model = x4A
            y_model = y4A
            z_model = z4A

            print(f"\n[IK Debug] 輸入座標: X={x4A:.4f}, Y={y4A:.4f}, Z={z4A:.4f}")

            # ===== 公式 3.13: θ1 =====
            # 修正：反轉方向以匹配機械手臂的實際座標系
            theta1 = np.arctan2(-y_model, -x_model)
            print(f"[IK Debug] θ1 = {np.rad2deg(theta1):.3f}°")

            # ===== 計算用於 θ2, θ3 的中間變數 =====
            # 水平距離
            r_xy = np.sqrt(y_model ** 2 + x_model ** 2)
            print(f"[IK Debug] 水平距離 r_xy = {r_xy:.4f}m")

            # ===== 公式 3.14: θ2 =====
            # 第一項：基礎角度 arctan((z - L1) / (r_xy - S1))
            z_offset = z_model - self.L1
            r_offset = r_xy - self.S1

            if abs(r_offset) < 1e-6:
                return None, "θ2計算：水平距離過小"

            alpha = np.arctan2(z_offset, r_offset)
            print(f"[IK Debug] α (基礎角) = {np.rad2deg(alpha):.3f}°")

            # 第二項：餘弦定理部分
            # 目標到P3基座的距離
            d_to_p3 = np.sqrt(r_offset ** 2 + z_offset ** 2)
            print(f"[IK Debug] 到P3距離 d = {d_to_p3:.4f}m")

            # P4-P5 連桿的有效長度（考慮90度初始旋轉）
            L_45 = np.sqrt(self.S2 ** 2 + self.L3 ** 2)
            print(f"[IK Debug] L45有效長度 = {L_45:.4f}m")

            # 檢查工作範圍
            max_reach = self.L2 + L_45
            min_reach = abs(self.L2 - L_45)
            if d_to_p3 > max_reach:
                return None, f"目標過遠 (d={d_to_p3:.3f}m > max={max_reach:.3f}m)"
            if d_to_p3 < min_reach:
                return None, f"目標過近 (d={d_to_p3:.3f}m < min={min_reach:.3f}m)"

            # 使用餘弦定理計算 β
            cos_beta = (self.L2 ** 2 + d_to_p3 ** 2 - L_45 ** 2) / (2 * self.L2 * d_to_p3)
            cos_beta = np.clip(cos_beta, -1.0, 1.0)
            beta = np.arccos(cos_beta)
            print(f"[IK Debug] β (肘部角) = {np.rad2deg(beta):.3f}°")

            # θ2 = -π/2 + α + β（公式3.14簡化版）
            theta2 = -np.pi / 2 + alpha + beta
            print(f"[IK Debug] θ2 = {np.rad2deg(theta2):.3f}°")

            # ===== 公式 3.15: θ3 =====
            # φ = arctan(S2/L3) - P4-P5的內部角度
            phi = np.arctan2(self.S2, self.L3)
            print(f"[IK Debug] φ (P4-P5內角) = {np.rad2deg(phi):.3f}°")

            # 使用餘弦定理計算 γ（P3-P4-P5的角度）
            cos_gamma = (self.L2 ** 2 + L_45 ** 2 - d_to_p3 ** 2) / (2 * self.L2 * L_45)
            cos_gamma = np.clip(cos_gamma, -1.0, 1.0)
            gamma = np.arccos(cos_gamma)
            print(f"[IK Debug] γ (連桿夾角) = {np.rad2deg(gamma):.3f}°")

            # θ3 = -π/2 - φ + γ（公式3.15）
            theta3 = -np.pi / 2 - phi + gamma
            print(f"[IK Debug] θ3 = {np.rad2deg(theta3):.3f}°")

            # ===== θ4 =====
            # 保持末端水平
            theta4 = 0

            # ===== 轉換為角度 =====
            angles = [
                np.rad2deg(theta1),
                np.rad2deg(theta2),
                np.rad2deg(theta3),
                np.rad2deg(theta4)
            ]

            print(
                f"[IK Debug] 最終角度: θ1={angles[0]:.3f}°, θ2={angles[1]:.3f}°, θ3={angles[2]:.3f}°, θ4={angles[3]:.3f}°")

            # ===== 檢查關節限制 =====
            limits = [(-165, 165), (-125, 85), (-55, 185), (-190, 190)]
            for i, (angle, (min_a, max_a)) in enumerate(zip(angles, limits)):
                if angle < min_a or angle > max_a:
                    return None, f"關節{i + 1}超出限制: {angle:.1f}° (範圍: {min_a}~{max_a}°)"

            return angles, "成功"

        except Exception as e:
            import traceback
            error_msg = f"計算錯誤: {str(e)}\n{traceback.format_exc()}"
            print(f"[IK Error] {error_msg}")
            return None, error_msg

    def move_to_position_analytical(self, target_x, target_y, target_z):
        """使用解析解IK移動到目標位置"""
        print(f"\n{'=' * 60}")
        print(f"開始IK計算 - 目標位置: X={target_x:.4f}, Y={target_y:.4f}, Z={target_z:.4f}")
        print(f"{'=' * 60}")

        result = self.inverse_kinematics_analytical(target_x, target_y, target_z)

        if result[0] is None:
            print(f"[IK 失敗] {result[1]}")
            return False, 0, None, result[1]

        angles = result[0]

        # 驗證正向運動學
        final_pos = self.calculate_forward_kinematics(angles)
        target_pos = np.array([target_x, target_y, target_z])
        error = np.linalg.norm(final_pos - target_pos)

        print(f"[FK驗證] 計算位置: X={final_pos[0]:.4f}, Y={final_pos[1]:.4f}, Z={final_pos[2]:.4f}")
        print(f"[FK驗證] 位置誤差: {error * 1000:.2f}mm")
        print(f"{'=' * 60}\n")

        if error < 0.01:  # 10mm 優秀
            for i in range(4):
                self.set_target(i, angles[i])
            return True, error, angles, "成功"
        elif error < 0.05:  # 50mm 可接受
            for i in range(4):
                self.set_target(i, angles[i])
            return True, error, angles, "成功（小誤差）"
        else:
            for i in range(4):
                self.set_target(i, angles[i])
            return False, error, angles, f"誤差較大: {error * 1000:.1f}mm"


anim_controller = AnimationController()
sliders = {}
exit_flag = False


# ---------- GUI 控制 ----------
def start_gui():
    angle_value_labels = {}

    def on_joint_slider_precise(joint_idx, value, key):
        angle = float(value)
        anim_controller.set_target(joint_idx, angle)
        angle_value_labels[key].config(text=f"{angle:.3f}°")

    def reset_pose():
        anim_controller.reset_to_default()
        for slider_name, default_value in [
            ('j1', 0), ('j2', 0), ('j3', 0), ('j4', 0), ('j5', 0), ('j6', 0)
        ]:
            if slider_name in sliders:
                sliders[slider_name].set(default_value)
                if slider_name in angle_value_labels:
                    angle_value_labels[slider_name].config(text=f"{default_value:.3f}°")

    def exit_program():
        global exit_flag
        exit_flag = True
        root.quit()
        root.destroy()

    root = tk.Tk()
    root.title("六軸手臂控制")
    root.configure(bg="#FFD9EC")

    # ========== 左側：關節控制 ==========
    left_frame = tk.Frame(root, bg="#FFD9EC")
    left_frame.pack(side=tk.LEFT, padx=10, pady=10, fill=tk.BOTH)

    tk.Label(left_frame, text="關節角度控制", font=("Arial", 12, "bold"),
             bg="#FFD9EC").pack(pady=5)

    joint_configs = [
        ("Joint 1 (θ1) - Z軸", -165, 165, 0, 'j1'),
        ("Joint 2 (θ2) - Y軸", -125, 85, 0, 'j2'),
        ("Joint 3 (θ3) - Y軸", -55, 185, 0, 'j3'),
        ("Joint 4 (θ4) - Z軸", -190, 190, 0, 'j4'),
        ("Joint 5 (θ5) - Y軸", -25, 205, 0, 'j5'),
        ("Joint 6 (θ6) - X軸", -360, 360, 0, 'j6')
    ]

    for i, (label, min_v, max_v, default, key) in enumerate(joint_configs):
        frame = tk.Frame(left_frame, bg="#FFD9EC")
        frame.pack(pady=3)

        header_frame = tk.Frame(frame, bg="#FFD9EC")
        header_frame.pack()
        tk.Label(header_frame, text=label, bg="#FFD9EC", font=("Arial", 9)).pack(side=tk.LEFT)
        value_label = tk.Label(header_frame, text=f"{default:.3f}°", bg="#FFFFFF",
                               font=("Arial", 9, "bold"), width=10, relief=tk.SUNKEN)
        value_label.pack(side=tk.LEFT, padx=5)
        angle_value_labels[key] = value_label

        slider = tk.Scale(frame, from_=min_v, to=max_v, orient=tk.HORIZONTAL,
                          length=250, resolution=0.001, showvalue=False,
                          command=lambda v, idx=i, k=key: on_joint_slider_precise(idx, v, k))
        slider.set(default)
        slider.pack()
        sliders[key] = slider

    # 動畫速度
    tk.Label(left_frame, text="動畫速度", bg="#FFD9EC", font=("Arial", 9)).pack(pady=(10, 0))

    def on_speed_change(value):
        anim_controller.animation_speed = float(value) / 100.0

    speed_slider = tk.Scale(left_frame, from_=5, to=30, orient=tk.HORIZONTAL,
                            length=250, command=on_speed_change)
    speed_slider.set(12)
    speed_slider.pack()

    # ========== 左側按鈕區 ==========
    def print_all_positions():
        positions = anim_controller.get_all_joint_positions()
        print("\n" + "=" * 60)
        print("RA605-710-GC 當前關節位置")
        print("=" * 60)
        for joint_name, pos in positions.items():
            print(f"{joint_name}: X={pos[0]:.4f}m, Y={pos[1]:.4f}m, Z={pos[2]:.4f}m")
        print("=" * 60 + "\n")

    left_button_frame = tk.Frame(left_frame, bg="#FFD9EC")
    left_button_frame.pack(pady=15)

    reset_button = tk.Button(left_button_frame, text="回復預設姿態", command=reset_pose,
                             font=("Arial", 9, "bold"), bg="#B8E6B8", padx=10, pady=5)
    reset_button.pack(pady=3)

    print_button = tk.Button(left_button_frame, text="打印關節位置", command=print_all_positions,
                             font=("Arial", 9, "bold"), bg="#FFE6CC", padx=10, pady=5)
    print_button.pack(pady=3)

    exit_button = tk.Button(left_button_frame, text="結束程式", command=exit_program,
                            font=("Arial", 9, "bold"), bg="#FFB6C1", padx=10, pady=5)
    exit_button.pack(pady=3)

    # ========== 右側：IK控制 ==========
    right_frame = tk.Frame(root, bg="#E6F3FF")
    right_frame.pack(side=tk.RIGHT, padx=10, pady=10, fill=tk.BOTH, expand=True)

    tk.Label(right_frame, text="逆向運動學控制",
             font=("Arial", 12, "bold"), bg="#E6F3FF", fg="#0000CC").pack(pady=5)

    # 當前位置顯示
    pos_frame = tk.Frame(right_frame, bg="#FFFFFF", relief=tk.RIDGE, bd=2)
    pos_frame.pack(pady=5, padx=10, fill=tk.X)
    tk.Label(pos_frame, text="當前 Joint 5 位置", font=("Arial", 10, "bold"),
             bg="#FFFFFF", fg="#FF0000").pack(pady=2)

    current_labels = {}
    for axis in ['X', 'Y', 'Z']:
        f = tk.Frame(pos_frame, bg="#FFFFFF")
        f.pack(pady=1)
        tk.Label(f, text=f"{axis}: ", font=("Arial", 9), bg="#FFFFFF").pack(side=tk.LEFT)
        label = tk.Label(f, text="0.000", font=("Arial", 9, "bold"),
                         bg="#F0F0F0", width=10, relief=tk.SUNKEN)
        label.pack(side=tk.LEFT, padx=3)
        current_labels[axis] = label

    def update_position_display():
        try:
            pos = anim_controller.get_joint5_position()
            current_labels['X'].config(text=f"{pos[0]:.4f}")
            current_labels['Y'].config(text=f"{pos[1]:.4f}")
            current_labels['Z'].config(text=f"{pos[2]:.4f}")
            root.after(50, update_position_display)
        except:
            pass

    update_position_display()

    # 目標位置輸入
    target_frame = tk.Frame(right_frame, bg="#FFE6E6", relief=tk.RIDGE, bd=2)
    target_frame.pack(pady=10, padx=10, fill=tk.BOTH, expand=True)

    tk.Label(target_frame, text="目標位置設定（滑桿控制）",
             font=("Arial", 11, "bold"), bg="#FFE6E6", fg="#CC0000").pack(pady=5)



    # X滑桿
    x_frame = tk.Frame(target_frame, bg="#FFE6E6")
    x_frame.pack(pady=5, fill=tk.X, padx=10)
    tk.Label(x_frame, text="目標 X (m)", font=("Arial", 9, "bold"),
             bg="#FFE6E6", width=12, anchor='w').pack(side=tk.LEFT)
    x_slider = tk.Scale(x_frame, from_=-0.7, to=0.7, resolution=0.001,
                        orient=tk.HORIZONTAL, length=200, bg="#FFCCCC")
    x_slider.set(0.0)
    x_slider.pack(side=tk.LEFT, padx=5)
    x_value_label = tk.Label(x_frame, text="0.000", font=("Arial", 9, "bold"),
                             bg="#FFFFFF", width=8, relief=tk.SUNKEN)
    x_value_label.pack(side=tk.LEFT, padx=3)

    def update_x_label(val):
        x_value_label.config(text=f"{float(val):.3f}")

    x_slider.config(command=update_x_label)

    # Y滑桿
    y_frame = tk.Frame(target_frame, bg="#FFE6E6")
    y_frame.pack(pady=5, fill=tk.X, padx=10)
    tk.Label(y_frame, text="目標 Y (m)", font=("Arial", 9, "bold"),
             bg="#FFE6E6", width=12, anchor='w').pack(side=tk.LEFT)
    y_slider = tk.Scale(y_frame, from_=-0.7, to=0.7, resolution=0.001,
                        orient=tk.HORIZONTAL, length=200, bg="#FFCCCC")
    y_slider.set(0.0)
    y_slider.pack(side=tk.LEFT, padx=5)
    y_value_label = tk.Label(y_frame, text="0.000", font=("Arial", 9, "bold"),
                             bg="#FFFFFF", width=8, relief=tk.SUNKEN)
    y_value_label.pack(side=tk.LEFT, padx=3)

    def update_y_label(val):
        y_value_label.config(text=f"{float(val):.3f}")

    y_slider.config(command=update_y_label)

    # Z滑桿
    z_frame = tk.Frame(target_frame, bg="#FFE6E6")
    z_frame.pack(pady=5, fill=tk.X, padx=10)
    tk.Label(z_frame, text="目標 Z (m)", font=("Arial", 9, "bold"),
             bg="#FFE6E6", width=12, anchor='w').pack(side=tk.LEFT)
    z_slider = tk.Scale(z_frame, from_=0.0, to=1.0, resolution=0.001,
                        orient=tk.HORIZONTAL, length=200, bg="#FFCCCC")
    z_slider.set(1.0)
    z_slider.pack(side=tk.LEFT, padx=5)
    z_value_label = tk.Label(z_frame, text="1.000", font=("Arial", 9, "bold"),
                             bg="#FFFFFF", width=8, relief=tk.SUNKEN)
    z_value_label.pack(side=tk.LEFT, padx=3)

    def update_z_label(val):
        z_value_label.config(text=f"{float(val):.3f}")

    z_slider.config(command=update_z_label)

    # 結果顯示
    result_label = tk.Label(target_frame, text="", font=("Arial", 9),
                            bg="#FFE6E6", fg="#006600", wraplength=400)
    result_label.pack(pady=5)

    # 移動按鈕
    def move_to_target():
        target_x = x_slider.get()
        target_y = y_slider.get()
        target_z = z_slider.get()

        result_label.config(text="計算中...", fg="#0000CC")
        root.update()

        success, error, angles, message = anim_controller.move_to_position_analytical(
            target_x, target_y, target_z
        )

        if success:
            if error < 0.01:
                color = "#006600"
                status = "✓"
            elif error < 0.05:
                color = "#009900"
                status = "✓"
            else:
                color = "#CC6600"
                status = "⚠"

            result_label.config(
                text=f"{status} {message}\n誤差: {error * 1000:.2f}mm\n"
                     f"θ1={angles[0]:.3f}° θ2={angles[1]:.3f}° "
                     f"θ3={angles[2]:.3f}° θ4={angles[3]:.3f}°",
                fg=color
            )
            sliders['j1'].set(angles[0])
            sliders['j2'].set(angles[1])
            sliders['j3'].set(angles[2])
            sliders['j4'].set(angles[3])
            angle_value_labels['j1'].config(text=f"{angles[0]:.3f}°")
            angle_value_labels['j2'].config(text=f"{angles[1]:.3f}°")
            angle_value_labels['j3'].config(text=f"{angles[2]:.3f}°")
            angle_value_labels['j4'].config(text=f"{angles[3]:.3f}°")
        else:
            result_label.config(text=f"✗ {message}", fg="#CC0000")

    btn_frame = tk.Frame(target_frame, bg="#FFE6E6")
    btn_frame.pack(pady=10)

    move_button = tk.Button(btn_frame, text="移動到目標位置", command=move_to_target,
                            font=("Arial", 10, "bold"), bg="#66CC66", fg="#FFFFFF",
                            padx=15, pady=8, relief=tk.RAISED, bd=3)
    move_button.pack(side=tk.LEFT, padx=5)

    def set_current_as_target():
        pos = anim_controller.get_joint5_position()
        x_slider.set(pos[0])
        y_slider.set(pos[1])
        z_slider.set(pos[2])
        result_label.config(text="已設定為當前位置", fg="#006600")

    current_button = tk.Button(btn_frame, text="使用當前位置", command=set_current_as_target,
                               font=("Arial", 9), bg="#DDDDDD", padx=10, pady=5)
    current_button.pack(side=tk.LEFT, padx=5)

    root.protocol("WM_DELETE_WINDOW", exit_program)
    root.mainloop()


# ---------- 主循環 ----------
def open3d_loop():
    global exit_flag

    initial_joint5_pos_internal = anim_controller.get_joint5_position_internal()

    joint5_marker = o3d.geometry.TriangleMesh.create_sphere(radius=0.08)
    joint5_marker.paint_uniform_color([1.0, 0.0, 0.0])
    joint5_marker.compute_vertex_normals()
    joint5_marker.translate(initial_joint5_pos_internal)
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