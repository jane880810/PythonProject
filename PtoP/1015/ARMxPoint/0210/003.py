'''
20251119
Open3D 視窗(顯示 3D 機械手臂)
Tkinter 控制介面(增強版 GUI)
新增功能:
1. 關節動畫速度調節
2. 軌跡點跳躍拉桿(一次跳多個點)
3. 已刪除終端機輸出
4. GUI更新頻率改為20Hz
5. 視窗並排顯示
6. 主迴圈更新頻率改為 100Hz
7. Joint5 紅球改為平移更新,不重建
8. 執行圓弧移動時保存1000點座標到日期命名的文字檔
9. 綠色弧形軌跡在執行完點的移動後依序清除
10. 關節1、2、3改為拉桿控制
11. Y預設值改為0.3
12. 修改為移動後顯示殘影軌跡,每個點在出現後10秒單獨消失
13. 第二次執行時不清除現有殘影,舊殘影繼續依時間自動消失
14. 紅球位置改為關節5末端點(P7末端,L4向下垂直XY平面),紅球尺寸縮小50%
15. 修正P7旋轉,確保P7永遠垂直XY平面,並正確計算關節5角度
16. 目標位置預設值改為x0.2 y0.2 z0.7
17. 動畫倍速預設改為10倍
'''

import open3d as o3d
import numpy as np
import tkinter as tk
from tkinter import ttk, messagebox, simpledialog
import threading
import time
import sys
import json
import os
from datetime import datetime


# ---------- 圓弧軌跡計算 ----------

def compute_arc_with_auto_center(A, B, radius_scale=2.0, num_points=1000):
    A = np.array(A, dtype=float)
    B = np.array(B, dtype=float)
    AB = B - A
    M = (A + B) / 2

    z_ref = np.array([0, 0, 1])
    if np.allclose(np.cross(AB, z_ref), 0):
        z_ref = np.array([1, 0, 0])

    dir_vec = np.cross(AB, z_ref)
    dir_vec = dir_vec / np.linalg.norm(dir_vec)

    half_len = np.linalg.norm(AB) / 2
    h = half_len * radius_scale
    C = M + dir_vec * h

    v1 = A - C
    v2 = B - C
    normal = np.cross(v1, v2)
    normal = normal / np.linalg.norm(normal)

    v1 = v1 / np.linalg.norm(v1)
    v2_proj = v2 - np.dot(v2, v1) * v1
    v2_proj = v2_proj / np.linalg.norm(v2_proj)

    angle = np.arccos(np.clip(np.dot(v1, v2 / np.linalg.norm(v2)), -1, 1))
    theta = np.linspace(0, angle, num_points)
    arc = [C + (np.cos(t) * v1 + np.sin(t) * v2_proj) * np.linalg.norm(A - C) for t in theta]

    return np.array(arc), A, B, C


# ---------- MeshNode ----------

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


# ---------- DH 參數 ----------

S1 = 0.030
S2 = 0.040
L1 = 0.375
L2 = 0.340
L3 = 0.338
L4 = 0.0865

print("\n" + "=" * 60)
print("RA605-710-GC")
print("=" * 60)
print(f"S1={S1 * 1000:.1f} S2={S2 * 1000:.1f} L1={L1 * 1000:.1f}")
print(f"L2={L2 * 1000:.1f} L3={L3 * 1000:.1f} L4={L4 * 1000:.1f}")
print("=" * 60 + "\n")

# ---------- 載入模型 ----------

#paths = [rf"/home/yahboom/Desktop/Obj/p{i}.obj" for i in range(1, 9)]


paths = [rf"/home/test/桌面/Obj/p{i}.obj" for i in range(1, 9)]
# paths = [rf"C:\Users\Administrator\OneDrive - Ming Chuan University\Desktop\Obj\p{i}.obj" for i in range(1, 9)]

meshes = [o3d.io.read_triangle_mesh(p) for p in paths]
if not all(m.has_triangles() for m in meshes):
    print("模型載入失敗")
    exit()
for m in meshes:
    m.compute_vertex_normals()

meshes[1].translate((0, 0, 0.23))
meshes[2].translate((-0.03, 0, 0.375))
meshes[3].translate((-0.03, 0, 0.715))
meshes[4].translate((0.01, 0, 0.81))
meshes[5].translate((0.01, 0, 1.053))
meshes[6].translate((0.01, 0, 1.12))
meshes[7].translate((0.01, 0, 1.1395))

# ---------- 建立結構 ----------

node2 = MeshNode(meshes[1], "p2")
node3 = MeshNode(meshes[2], "p3")
node4 = MeshNode(meshes[3], "p4")
node5 = MeshNode(meshes[4], "p5")
node6 = MeshNode(meshes[5], "p6")
node7 = MeshNode(meshes[6], "p7")
node8 = MeshNode(meshes[7], "p8")

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

root_nodes = [MeshNode(meshes[0], "p1"), node2]

# ---------- 初始旋轉 ----------

p5_top_center = [S2 - S1, 0, L1 + L2 + L3]
R = o3d.geometry.get_rotation_matrix_from_axis_angle([0, np.deg2rad(-90), 0])
T = np.eye(4)
T[:3, :3] = R
node6_group.transform(T, p5_top_center)

p3_top_center = [-S1, 0, L1 + L2]
R_p4_init = o3d.geometry.get_rotation_matrix_from_axis_angle([0, np.deg2rad(-90), 0])
T_p4_init = np.eye(4)
T_p4_init[:3, :3] = R_p4_init
node4_group.transform(T_p4_init, p3_top_center)

# ---------- Open3D 視窗(右側) ----------

vis = o3d.visualization.Visualizer()
vis.create_window("Robot Arm", width=500, height=500, left=870, top=50)

for root in root_nodes:
    for m in root.get_all_meshes():
        vis.add_geometry(m)

world_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3, origin=[0.375, 0.3, 0])
vis.add_geometry(world_frame)


def create_grid(size=2.0, step=0.5):
    lines, points, colors = [], [], []
    num = int(size / step) + 1
    for i in range(num):
        coord = -size / 2 + i * step
        points.extend([[coord, -size / 2, 0], [coord, size / 2, 0]])
        idx = len(points) - 2
        lines.append([idx, idx + 1])
        colors.append([0.5, 0.5, 0.5])
        points.extend([[-size / 2, coord, 0], [size / 2, coord, 0]])
        idx = len(points) - 2
        lines.append([idx, idx + 1])
        colors.append([0.5, 0.5, 0.5])

    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)
    return line_set


grid = create_grid()
vis.add_geometry(grid)

base_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.08, origin=[0, 0, 0])
vis.add_geometry(base_frame)


# ---------- 控制器 ----------

class AnimationController:
    def __init__(self):
        self.S1, self.S2 = S1, S2
        self.L1, self.L2, self.L3, self.L4 = L1, L2, L3, L4

        self.current_angles = [0, 0, 0, 0, 0, 0]
        self.target_angles = [0, 0, 0, 0, 0, 0]
        self.animation_speed = 0.12
        self.is_animating = False
        self.joint_scales = []

        self.R_p2_total = np.eye(3)
        self.R_p3_total = np.eye(3)
        self.R_p4_total = o3d.geometry.get_rotation_matrix_from_axis_angle([0, np.deg2rad(-90), 0])
        self.R_p5_total = np.eye(3)
        self.R_p6_total = np.eye(3)
        self.R_p8_total = np.eye(3)

        self.p3_base_local = np.array([-self.S1, 0, self.L1])
        self.p3_length_vec = np.array([0, 0, self.L2])
        self.p4_length_vec = np.array([self.S2, 0, 0])
        self.p5_length_vec = np.array([0, 0, self.L3])
        self.p6_length_vec = np.array([0, 0, 0.067])
        self.p7_length_vec = np.array([0, 0, -0.067])
        self.p8_center_offset = np.array([self.L4, 0, 0])

        self.joint5_marker = None
        self.joint5_pos = None

        self.trajectory_line = None
        self.trajectory_points = None
        self.is_following_trajectory = False
        self.trajectory_index = 0
        self.trajectory_delay = 0.005
        self.trajectory_step = 10  # 預設改為10倍
        self.last_trajectory_time = 0
        self.skip_unreachable_points = True
        self.skipped_points_count = 0
        self.status_callback = None

        # 殘影軌跡相關變量
        self.trail_line = None  # 殘影軌跡線
        self.trail_points = []  # 記錄已走過的點和時間戳 [(point, timestamp), ...]

    def set_marker(self, marker):
        self.joint5_marker = marker
        self.joint5_pos = np.array(marker.get_center(), dtype=float)

    def set_status_callback(self, callback):
        """設置狀態更新回調函數"""
        self.status_callback = callback

    def log_status(self, message):
        """記錄狀態訊息(只發送到GUI,不輸出到終端機)"""
        if self.status_callback:
            self.status_callback(message)

    def update_scales(self):
        """更新GUI滑桿顯示當前角度"""
        if not self.joint_scales:
            return
        for i, scale in enumerate(self.joint_scales):
            if scale:
                scale.config(command=lambda v: None)
                scale.set(self.current_angles[i])
                scale.config(command=self._make_scale_command(i))

    def _make_scale_command(self, idx):
        """創建滑桿命令回調(正確捕獲索引)"""
        return lambda v: self.set_target(idx, float(v))

    def get_joint5_position(self):
        """計算關節5末端點位置(P7末端,L4向下垂直XY平面)"""
        # 計算關節5基座位置
        joint5_base = self.R_p2_total @ (
                self.p3_base_local +
                self.R_p3_total @ (
                        self.p3_length_vec +
                        self.R_p4_total @ (
                                self.p4_length_vec +
                                self.R_p5_total @ self.p5_length_vec
                        )
                )
        )

        # P7(L4)永遠向下垂直XY平面,不受關節4、5旋轉影響
        l4_vector = np.array([0, 0, -self.L4])

        # 關節5末端點(P7末端) = 關節5基座 + L4向量(向下)
        joint5_end = joint5_base + l4_vector

        return joint5_end

    def update_joint5_marker(self):
        """更新關節5標記位置:改為平移,不重建"""
        if self.joint5_marker is None:
            return

        new_pos = self.get_joint5_position()
        if self.joint5_pos is None:
            self.joint5_pos = new_pos.copy()

        delta = new_pos - self.joint5_pos
        self.joint5_marker.translate(delta)
        self.joint5_pos = new_pos.copy()

        vis.update_geometry(self.joint5_marker)

    def update_joint(self, joint_idx, delta_angle):
        if abs(delta_angle) < 0.0001:
            return
        delta_rad = np.deg2rad(delta_angle)

        if joint_idx == 0:
            R = o3d.geometry.get_rotation_matrix_from_axis_angle([0, 0, delta_rad])
            self.R_p2_total[:] = R @ self.R_p2_total
            T = np.eye(4)
            T[:3, :3] = R
            node2.transform(T, [0, 0, 0])
            for m in node2.get_all_meshes():
                vis.update_geometry(m)

        elif joint_idx == 1:
            R_local = o3d.geometry.get_rotation_matrix_from_axis_angle([0, delta_rad, 0])
            self.R_p3_total[:] = R_local @ self.R_p3_total
            local_y = self.R_p2_total @ np.array([0, 1, 0])
            R_world = o3d.geometry.get_rotation_matrix_from_axis_angle(local_y * delta_rad)
            center = self.R_p2_total @ self.p3_base_local
            T = np.eye(4)
            T[:3, :3] = R_world
            node3.transform(T, center)
            for m in node3.get_all_meshes():
                vis.update_geometry(m)

        elif joint_idx == 2:
            R_local = o3d.geometry.get_rotation_matrix_from_axis_angle([0, delta_rad, 0])
            self.R_p4_total[:] = R_local @ self.R_p4_total
            local_y = self.R_p2_total @ np.array([0, 1, 0])
            R_world = o3d.geometry.get_rotation_matrix_from_axis_angle(local_y * delta_rad)
            center = self.R_p2_total @ (self.p3_base_local + self.R_p3_total @ self.p3_length_vec)
            T = np.eye(4)
            T[:3, :3] = R_world
            node4_group.transform(T, center)
            for m in node4_group.get_all_meshes():
                vis.update_geometry(m)

        elif joint_idx == 3:
            R_local = o3d.geometry.get_rotation_matrix_from_axis_angle([0, 0, delta_rad])
            self.R_p5_total[:] = R_local @ self.R_p5_total
            p4_z = self.R_p2_total @ self.R_p3_total @ self.R_p4_total @ np.array([0, 0, 1])
            R_world = o3d.geometry.get_rotation_matrix_from_axis_angle(p4_z * delta_rad)
            center = self.R_p2_total @ (self.p3_base_local + self.R_p3_total @ (
                    self.p3_length_vec + self.R_p4_total @ self.p4_length_vec))
            T = np.eye(4)
            T[:3, :3] = R_world
            node5_group.transform(T, center)
            for m in node5_group.get_all_meshes():
                vis.update_geometry(m)

        elif joint_idx == 4:
            R_local = o3d.geometry.get_rotation_matrix_from_axis_angle([0, delta_rad, 0])
            self.R_p6_total[:] = R_local @ self.R_p6_total
            p5_y = self.R_p2_total @ self.R_p3_total @ self.R_p4_total @ self.R_p5_total @ np.array([0, 1, 0])
            R_world = o3d.geometry.get_rotation_matrix_from_axis_angle(p5_y * delta_rad)
            center = self.R_p2_total @ (self.p3_base_local + self.R_p3_total @ (
                    self.p3_length_vec + self.R_p4_total @ (
                    self.p4_length_vec + self.R_p5_total @ self.p5_length_vec)))
            T = np.eye(4)
            T[:3, :3] = R_world
            node6_group.transform(T, center)
            for m in node6_group.get_all_meshes():
                vis.update_geometry(m)

        elif joint_idx == 5:
            R_local = o3d.geometry.get_rotation_matrix_from_axis_angle([delta_rad, 0, 0])
            self.R_p8_total[:] = R_local @ self.R_p8_total
            p8_x = self.R_p2_total @ self.R_p3_total @ self.R_p4_total @ self.R_p5_total @ self.R_p6_total @ np.array(
                [1, 0, 0])
            R_world = o3d.geometry.get_rotation_matrix_from_axis_angle(p8_x * delta_rad)
            p8_base = self.R_p2_total @ (
                    self.p3_base_local + self.R_p3_total @ (
                    self.p3_length_vec + self.R_p4_total @ (
                    self.p4_length_vec + self.R_p5_total @ (
                    self.p5_length_vec + self.R_p6_total @ (
                    self.p6_length_vec + self.p7_length_vec)))))
            p8_offset = self.R_p2_total @ self.R_p3_total @ self.R_p4_total @ self.R_p5_total @ self.R_p6_total @ self.p8_center_offset
            center = p8_base + p8_offset
            T = np.eye(4)
            T[:3, :3] = R_world
            node8_group.transform(T, center)
            for m in node8_group.get_all_meshes():
                vis.update_geometry(m)

        self.update_joint5_marker()
        self.update_scales()

    def animate(self):
        # 持續更新殘影(會自動移除超過10秒的點)
        if len(self.trail_points) > 0:
            self.update_trail()

        if self.is_following_trajectory:
            current_time = time.time()
            if current_time - self.last_trajectory_time >= self.trajectory_delay:
                self.last_trajectory_time = current_time
                self.move_next_point()
            return

        if not self.is_animating:
            return
        moved = False
        for i in range(6):
            if abs(self.current_angles[i] - self.target_angles[i]) > 0.01:
                old = self.current_angles[i]
                diff = self.target_angles[i] - old
                self.current_angles[i] = old + diff * self.animation_speed
                self.update_joint(i, self.current_angles[i] - old)
                moved = True
        if not moved:
            self.is_animating = False

    def set_target(self, joint_idx, angle):
        self.target_angles[joint_idx] = angle
        self.is_animating = True

    def fk(self, angles):
        """正向運動學:計算關節5末端點位置(P7末端,L4向下垂直XY平面)"""
        R2 = o3d.geometry.get_rotation_matrix_from_axis_angle([0, 0, np.deg2rad(angles[0])])
        R3 = o3d.geometry.get_rotation_matrix_from_axis_angle([0, np.deg2rad(angles[1]), 0])
        R4b = o3d.geometry.get_rotation_matrix_from_axis_angle([0, np.deg2rad(-90), 0])
        R4 = o3d.geometry.get_rotation_matrix_from_axis_angle([0, np.deg2rad(angles[2]), 0]) @ R4b
        R5 = o3d.geometry.get_rotation_matrix_from_axis_angle([0, 0, np.deg2rad(angles[3])])

        # 關節5基座位置
        joint5_base = R2 @ (self.p3_base_local + R3 @ (
                self.p3_length_vec + R4 @ (self.p4_length_vec + R5 @ self.p5_length_vec)))

        # P7(L4)永遠向下垂直XY平面
        l4_vector = np.array([0, 0, -self.L4])
        joint5_end = joint5_base + l4_vector

        return joint5_end

    def ik(self, x, y, z):
        """逆向運動學:從關節5末端點(P7末端)反推關節角度,包括關節5"""
        try:
            # 扣除L4長度(向下),得到關節5基座目標位置
            z_base = z + self.L4

            theta1 = np.arctan2(-y, -x)
            r_xy = np.sqrt(y ** 2 + x ** 2)
            z_off = z_base - self.L1
            r_off = r_xy - self.S1
            if abs(r_off) < 1e-6:
                return None, "err"
            alpha = np.arctan2(z_off, r_off)
            d = np.sqrt(r_off ** 2 + z_off ** 2)
            L45 = np.sqrt(self.S2 ** 2 + self.L3 ** 2)
            if d > self.L2 + L45 or d < abs(self.L2 - L45):
                return None, "reach"
            cb = np.clip((self.L2 ** 2 + d ** 2 - L45 ** 2) / (2 * self.L2 * d), -1, 1)
            beta = np.arccos(cb)
            theta2 = -np.pi / 2 + alpha + beta
            phi = np.arctan2(self.S2, self.L3)
            cg = np.clip((self.L2 ** 2 + L45 ** 2 - d ** 2) / (2 * self.L2 * L45), -1, 1)
            gamma = np.arccos(cg)
            theta3 = -np.pi / 2 - phi + gamma

            # 計算關節5角度,使P7保持垂直
            # theta5 = -(theta2 + theta3) 使P7垂直
            theta5 = -(theta2 + theta3)

            angles = [np.rad2deg(theta1), np.rad2deg(theta2), np.rad2deg(theta3), 0, np.rad2deg(theta5)]
            limits = [(-165, 165), (-125, 85), (-55, 185), (-190, 190), (-25, 205)]
            for i, (a, (mn, mx)) in enumerate(zip(angles, limits)):
                if a < mn or a > mx:
                    return None, "limit"
            return angles, "ok"
        except:
            return None, "err"

    def move_to(self, x, y, z):
        res = self.ik(x, y, z)
        if res[0] is None:
            return False, res[1]
        angles = res[0]
        for i in range(5):  # 包括關節5
            self.set_target(i, angles[i])
        return True, "ok"

    def move_to_instant(self, x, y, z):
        """直接移動到目標位置,不使用動畫"""
        res = self.ik(x, y, z)
        if res[0] is None:
            return False, res[1]
        angles = res[0]

        for i in range(5):  # 包括關節5
            delta = angles[i] - self.current_angles[i]
            self.current_angles[i] = angles[i]
            self.target_angles[i] = angles[i]
            if abs(delta) > 0.0001:
                self.update_joint(i, delta)

        return True, "ok"

    def update_trail(self):
        """更新殘影軌跡顯示 - 自動移除超過10秒的點"""
        # 先清除過期的點(超過10秒)
        current_time = time.time()
        self.trail_points = [(pt, ts) for pt, ts in self.trail_points if current_time - ts <= 10]

        # 如果點數少於2個,清除軌跡線
        if len(self.trail_points) < 2:
            if self.trail_line:
                vis.remove_geometry(self.trail_line, reset_bounding_box=False)
                self.trail_line = None
            return

        # 移除舊的殘影線
        if self.trail_line:
            vis.remove_geometry(self.trail_line, reset_bounding_box=False)

        # 創建新的殘影線(只使用未過期的點)
        points_only = [pt for pt, ts in self.trail_points]
        pts = o3d.utility.Vector3dVector(points_only)
        lns = [[i, i + 1] for i in range(len(points_only) - 1)]
        ls = o3d.geometry.LineSet()
        ls.points = pts
        ls.lines = o3d.utility.Vector2iVector(lns)
        ls.paint_uniform_color([0, 1, 0])  # 綠色
        self.trail_line = ls
        vis.add_geometry(ls, reset_bounding_box=False)

    def clear_trail(self):
        """清除殘影軌跡"""
        if self.trail_line:
            vis.remove_geometry(self.trail_line, reset_bounding_box=False)
            self.trail_line = None
        self.trail_points = []

    def show_trajectory(self, points, start_index=0):
        """顯示軌跡線(從start_index開始) - 此方法已不再使用於執行中顯示"""
        if self.trajectory_line:
            vis.remove_geometry(self.trajectory_line, reset_bounding_box=False)

        if start_index >= len(points) - 1:
            self.trajectory_line = None
            return

        remaining_points = points[start_index:]
        pts = o3d.utility.Vector3dVector(remaining_points)
        lns = [[i, i + 1] for i in range(len(remaining_points) - 1)]
        ls = o3d.geometry.LineSet()
        ls.points = pts
        ls.lines = o3d.utility.Vector2iVector(lns)
        ls.paint_uniform_color([0, 1, 0])
        self.trajectory_line = ls
        vis.add_geometry(ls, reset_bounding_box=False)

    def clear_trajectory_markers(self):
        """清除軌跡線"""
        if self.trajectory_line:
            vis.remove_geometry(self.trajectory_line, reset_bounding_box=False)
            self.trajectory_line = None

    def start_trajectory(self, points):
        self.trajectory_points = points
        self.trajectory_index = 0
        self.skipped_points_count = 0
        self.is_following_trajectory = True
        self.last_trajectory_time = time.time()

        # 不清除現有的殘影,讓它們繼續依時間自動消失
        # 舊的殘影會在 update_trail() 中自動移除(超過10秒的點)

        self.move_next_point()

    def move_next_point(self):
        if not self.is_following_trajectory or self.trajectory_points is None:
            return

        # 修正:應該是 > 而不是 >=,確保最後一點也能執行
        if self.trajectory_index > len(self.trajectory_points) - 1:
            self.is_following_trajectory = False
            self.skipped_points_count = 0
            # 軌跡完成,殘影會自動依時間消失
            return

        pt = self.trajectory_points[self.trajectory_index]
        ok, msg = self.move_to_instant(pt[0], pt[1], pt[2])

        if ok:
            # 記錄當前點和時間戳到殘影列表
            current_time = time.time()
            self.trail_points.append(([pt[0], pt[1], pt[2]], current_time))

            # 更新殘影軌跡顯示
            self.update_trail()

            self.trajectory_index += self.trajectory_step
        else:
            if self.skip_unreachable_points:
                if self.skipped_points_count > len(self.trajectory_points) * 0.2:
                    self.is_following_trajectory = False
                    self.skipped_points_count = 0
                    # 跳過太多點,結束執行
                    return

                self.skipped_points_count += 1
                first_skip_index = self.trajectory_index
                self.trajectory_index += self.trajectory_step

                consecutive_skips = 1
                found_reachable = False

                while self.trajectory_index <= len(self.trajectory_points) - 1:  # 修正:改為 <=
                    if self.skipped_points_count > len(self.trajectory_points) * 0.2:
                        self.is_following_trajectory = False
                        self.skipped_points_count = 0
                        # 跳過太多點,結束執行
                        return

                    pt_next = self.trajectory_points[self.trajectory_index]
                    ok_next, msg_next = self.move_to_instant(pt_next[0], pt_next[1], pt_next[2])

                    if ok_next:
                        found_reachable = True

                        # 記錄當前點和時間戳到殘影列表
                        current_time = time.time()
                        self.trail_points.append(([pt_next[0], pt_next[1], pt_next[2]], current_time))

                        # 更新殘影軌跡顯示
                        self.update_trail()

                        self.trajectory_index += self.trajectory_step
                        break
                    else:
                        self.skipped_points_count += 1
                        self.trajectory_index += self.trajectory_step
                        consecutive_skips += 1

                if not found_reachable:
                    self.is_following_trajectory = False
                    self.skipped_points_count = 0
                    # 無法找到可達點,結束執行
            else:
                self.is_following_trajectory = False
                self.skipped_points_count = 0
                # 不跳過點,結束執行

    def set_trajectory_speed(self, speed):
        """設定軌跡速度(調整延遲時間)"""
        self.trajectory_delay = 0.005 * (1000 - speed) / 999

    def set_animation_speed(self, speed):
        """設定關節動畫速度"""
        self.animation_speed = speed / 100.0

    def set_trajectory_step(self, step):
        """設定軌跡點跳躍步數"""
        self.trajectory_step = max(1, int(step))

    def stop_trajectory(self):
        self.is_following_trajectory = False
        self.skipped_points_count = 0
        self.clear_trajectory_markers()

    def reset_pose(self):
        """重置所有關節到預設姿態(0度)"""
        if self.is_following_trajectory:
            self.stop_trajectory()

        for i in range(6):
            self.set_target(i, 0)

        for i, scale in enumerate(self.joint_scales):
            if scale:
                scale.config(command=lambda v: None)
                scale.set(0)
                scale.config(command=self._make_scale_command(i))


ctrl = AnimationController()
exit_flag = False


# ---------- 增強版 GUI ----------

class RobotControlGUI:
    def __init__(self, root, controller):
        self.root = root
        self.ctrl = controller
        self.root.title("RA605-710-GC 六軸機械手臂控制系統 (增強版)")
        self.root.geometry("850x750+0+50")

        self.setup_ui()
        self.ctrl.set_status_callback(self.append_status)
        self.update_display()

    def setup_ui(self):
        """建立主要介面"""
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)

        self.create_joint_control(main_frame)
        self.create_position_control(main_frame)
        self.create_system_control(main_frame)

    def create_joint_control(self, parent):
        """關節控制面板 - 使用拉桿控制"""
        frame = ttk.LabelFrame(parent, text="關節控制", padding="10")
        frame.grid(row=0, column=0, padx=5, pady=5, sticky=(tk.N, tk.S))

        joint_config = [
            ("關節 1 (Base)", -165, 165),
            ("關節 2 (Shoulder)", -125, 85),
            ("關節 3 (Elbow)", -55, 185),
            ("關節 4 (Wrist Z)", -190, 190),
            ("關節 5 (Wrist Y)", -25, 205),
            ("關節 6 (Wrist X)", -360, 360)
        ]

        self.joint_labels = []
        self.ctrl.joint_scales = []

        for i, (name, min_val, max_val) in enumerate(joint_config):
            # 標題列
            header_frame = ttk.Frame(frame)
            header_frame.grid(row=i * 2, column=0, sticky=tk.W, pady=(10 if i > 0 else 0, 5))

            ttk.Label(header_frame, text=name, font=('Arial', 10, 'bold')).pack(side=tk.LEFT)

            value_label = ttk.Label(header_frame, text="0.0°",
                                    foreground="blue", font=('Arial', 10, 'bold'))
            value_label.pack(side=tk.RIGHT)
            self.joint_labels.append(value_label)

            # 拉桿控制列
            control_frame = ttk.Frame(frame)
            control_frame.grid(row=i * 2 + 1, column=0, sticky=(tk.W, tk.E), padx=5)

            scale = ttk.Scale(control_frame, from_=min_val, to=max_val,
                              orient=tk.HORIZONTAL, command=self.ctrl._make_scale_command(i))
            scale.set(0)
            scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 10))
            self.ctrl.joint_scales.append(scale)

            range_label = ttk.Label(control_frame,
                                    text=f"[{min_val}° ~ {max_val}°]",
                                    font=('Arial', 8), foreground='gray')
            range_label.pack(side=tk.LEFT)

        # 動畫速度控制
        anim_frame = ttk.LabelFrame(frame, text="動畫速度", padding="5")
        anim_frame.grid(row=len(joint_config) * 2, column=0, pady=10, sticky=(tk.W, tk.E))

        speed_control_frame = ttk.Frame(anim_frame)
        speed_control_frame.pack(fill=tk.X, pady=2)

        ttk.Label(speed_control_frame, text="關節速度:").pack(side=tk.LEFT)

        self.anim_speed_scale = ttk.Scale(speed_control_frame, from_=1, to=100,
                                          orient=tk.HORIZONTAL)
        self.anim_speed_scale.set(12)
        self.anim_speed_scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)

        self.anim_speed_label = ttk.Label(speed_control_frame, text="12%", width=6)
        self.anim_speed_label.pack(side=tk.LEFT)

        self.anim_speed_scale.configure(command=self.on_anim_speed_change)

        # 重置按鈕
        btn_frame = ttk.Frame(frame)
        btn_frame.grid(row=len(joint_config) * 2 + 1, column=0, pady=10)

        ttk.Button(btn_frame, text="重置姿態",
                   command=self.reset_pose).pack(fill=tk.X, pady=2)

    def create_position_control(self, parent):
        """位置控制與軌跡規劃面板"""
        frame = ttk.Frame(parent)
        frame.grid(row=0, column=1, padx=5, pady=5, sticky=(tk.N, tk.S, tk.W, tk.E))

        # 當前位置顯示
        current_frame = ttk.LabelFrame(frame, text="當前末端位置 (J5)", padding="10")
        current_frame.pack(fill=tk.X, pady=(0, 10))

        pos_frame = ttk.Frame(current_frame)
        pos_frame.pack()

        self.current_labels = {}
        for i, axis in enumerate(['X', 'Y', 'Z']):
            ttk.Label(pos_frame, text=f"{axis}:", font=('Arial', 10, 'bold')).grid(
                row=0, column=i * 2, padx=5)
            label = ttk.Label(pos_frame, text="0.000 m",
                              font=('Courier', 11), foreground='green')
            label.grid(row=0, column=i * 2 + 1, padx=5)
            self.current_labels[axis] = label

        # 目標位置設定
        target_frame = ttk.LabelFrame(frame, text="目標位置設定", padding="10")
        target_frame.pack(fill=tk.X, pady=(0, 10))

        self.target_scales = {}
        target_config = [
            ('X', -0.7, 0.7, 0.2),  # 預設改為0.2
            ('Y', -0.7, 0.7, 0.2),  # 預設改為0.2
            ('Z', 0.0, 1.0, 0.7)    # 預設改為0.7
        ]

        for axis, min_val, max_val, default in target_config:
            axis_frame = ttk.Frame(target_frame)
            axis_frame.pack(fill=tk.X, pady=2)

            ttk.Label(axis_frame, text=f"{axis}:", width=3).pack(side=tk.LEFT)

            scale = ttk.Scale(axis_frame, from_=min_val, to=max_val,
                              orient=tk.HORIZONTAL)
            scale.set(default)
            scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)

            value_label = ttk.Label(axis_frame, text=f"{default:.2f}", width=6)
            value_label.pack(side=tk.LEFT)

            scale.configure(command=lambda v, lbl=value_label: lbl.configure(
                text=f"{float(v):.2f}"))

            self.target_scales[axis] = scale

        # 圓弧軌跡控制
        traj_frame = ttk.LabelFrame(frame, text="圓弧軌跡控制", padding="10")
        traj_frame.pack(fill=tk.BOTH, expand=True)

        # 圓弧係數
        radius_frame = ttk.Frame(traj_frame)
        radius_frame.pack(fill=tk.X, pady=5)

        ttk.Label(radius_frame, text="圓弧係數:").pack(side=tk.LEFT)
        self.radius_scale = ttk.Scale(radius_frame, from_=0.5, to=5.0,
                                      orient=tk.HORIZONTAL)
        self.radius_scale.set(2.0)
        self.radius_scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)

        self.radius_label = ttk.Label(radius_frame, text="2.00", width=5)
        self.radius_label.pack(side=tk.LEFT)
        self.radius_scale.configure(command=lambda v: self.radius_label.configure(
            text=f"{float(v):.2f}"))

        # 軌跡速度
        speed_frame = ttk.Frame(traj_frame)
        speed_frame.pack(fill=tk.X, pady=5)

        ttk.Label(speed_frame, text="軌跡速度:").pack(side=tk.LEFT)

        self.speed_scale = ttk.Scale(speed_frame, from_=1, to=1000,
                                     orient=tk.HORIZONTAL)
        self.speed_scale.set(100)
        self.speed_scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)

        self.speed_label = ttk.Label(speed_frame, text="100x", width=8)
        self.speed_label.pack(side=tk.LEFT)

        self.speed_scale.configure(command=self.on_speed_change)

        # 動畫倍速
        step_frame = ttk.Frame(traj_frame)
        step_frame.pack(fill=tk.X, pady=5)

        ttk.Label(step_frame, text="動畫倍速:").pack(side=tk.LEFT)

        self.step_scale = ttk.Scale(step_frame, from_=1, to=50,
                                    orient=tk.HORIZONTAL)
        self.step_scale.set(10)  # 預設改為10倍
        self.step_scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)

        self.step_label = ttk.Label(step_frame, text="10倍", width=8)  # 顯示改為10倍
        self.step_label.pack(side=tk.LEFT)

        self.step_scale.configure(command=self.on_step_change)

        # 選項
        option_frame = ttk.Frame(traj_frame)
        option_frame.pack(fill=tk.X, pady=5)

        self.skip_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(option_frame, text="自動跳過無法到達的點",
                        variable=self.skip_var,
                        command=self.toggle_skip).pack(anchor=tk.W)

        # 狀態顯示
        status_frame = ttk.Frame(traj_frame)
        status_frame.pack(fill=tk.BOTH, expand=True, pady=5)

        status_scrollbar = ttk.Scrollbar(status_frame)
        status_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        self.status_text = tk.Text(status_frame, height=8, width=40,
                                   font=('Courier', 10),
                                   state='disabled',
                                   wrap=tk.WORD,
                                   yscrollcommand=status_scrollbar.set,
                                   spacing1=2,
                                   spacing2=1,
                                   spacing3=2,
                                   padx=5,
                                   pady=5)
        self.status_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        status_scrollbar.config(command=self.status_text.yview)
        self.update_status("準備就緒")

        # 控制按鈕
        btn_frame = ttk.Frame(traj_frame)
        btn_frame.pack(fill=tk.X, pady=5)

        ttk.Button(btn_frame, text="執行圓弧移動",
                   command=self.execute_arc).pack(side=tk.LEFT, padx=2)
        ttk.Button(btn_frame, text="停止",
                   command=self.stop_trajectory).pack(side=tk.LEFT, padx=2)
        ttk.Button(btn_frame, text="重置軌跡",
                   command=self.reset_trajectory).pack(side=tk.LEFT, padx=2)

    def create_system_control(self, parent):
        """系統控制面板"""
        frame = ttk.Frame(parent)
        frame.grid(row=1, column=0, columnspan=3, pady=10, sticky=(tk.W, tk.E))

        ttk.Button(frame, text="關閉程式",
                   command=self.quit_program).pack(side=tk.RIGHT, padx=5)

        self.info_label = ttk.Label(frame, text="RA605-710-GC 六軸機械手臂 (增強版)",
                                    font=('Arial', 9))
        self.info_label.pack(side=tk.LEFT, padx=5)

    def on_anim_speed_change(self, value):
        """動畫速度變更"""
        speed = int(float(value))
        self.ctrl.set_animation_speed(speed)
        if hasattr(self, 'anim_speed_label'):
            self.anim_speed_label.configure(text=f"{speed}%")

    def on_speed_change(self, value):
        """軌跡速度變更"""
        speed = int(float(value))
        self.ctrl.set_trajectory_speed(speed)
        if hasattr(self, 'speed_label'):
            delay_ms = (0.005 * (1000 - speed) / 999) * 1000

            if delay_ms < 0.001:
                delay_str = "最高速"
            else:
                delay_str = f"{delay_ms:.2f}ms"

            self.speed_label.configure(text=f"{speed}x ({delay_str})")

    def on_step_change(self, value):
        """動畫倍速變更"""
        step = int(float(value))
        self.ctrl.set_trajectory_step(step)
        if hasattr(self, 'step_label'):
            self.step_label.configure(text=f"{step}倍")

    def toggle_skip(self):
        """切換跳過選項"""
        self.ctrl.skip_unreachable_points = self.skip_var.get()

    def execute_arc(self):
        """執行圓弧移動"""
        try:
            A = self.ctrl.get_joint5_position()
            B = [self.target_scales['X'].get(),
                 self.target_scales['Y'].get(),
                 self.target_scales['Z'].get()]

            radius = self.radius_scale.get()

            self.update_status("計算軌跡中...")
            self.root.update()

            arc, _, _, C = compute_arc_with_auto_center(A, B, radius, 1000)

            # 保存軌跡座標到文字檔
            today = datetime.now().strftime("%Y%m%d")
            filename = f"{today}.txt"
            script_dir = os.path.dirname(os.path.abspath(__file__))
            filepath = os.path.join(script_dir, filename)

            with open(filepath, 'w') as f:
                for i, point in enumerate(arc, start=1):
                    f.write(
                        f"{i} {point[0] * 1000:.3f} {point[1] * 1000:.3f} {point[2] * 1000:.3f} 0 0 -1\n")

            length = np.sum(np.sqrt(np.sum(np.diff(arc, axis=0) ** 2, axis=1)))
            step = self.ctrl.trajectory_step

            self.update_status(
                f"軌跡長度: {length:.3f}m\n總點數: {len(arc)}\n倍速: {step}倍\n已存檔: {filename}\n執行中...")
            self.ctrl.start_trajectory(arc)

        except Exception as e:
            messagebox.showerror("錯誤", f"執行失敗: {str(e)}")

    def stop_trajectory(self):
        """停止軌跡"""
        self.ctrl.stop_trajectory()
        self.update_status("已停止")

    def reset_trajectory(self):
        """重置軌跡"""
        self.ctrl.stop_trajectory()
        self.ctrl.trajectory_index = 0
        self.ctrl.clear_trajectory_markers()
        self.update_status("準備就緒")

    def reset_pose(self):
        """重置姿態"""
        self.ctrl.reset_pose()

    def update_status(self, message):
        """更新狀態文字"""
        self.status_text.config(state='normal')
        self.status_text.delete(1.0, tk.END)
        self.status_text.insert(1.0, message)
        self.status_text.config(state='disabled')

    def append_status(self, message):
        """追加狀態訊息(保留歷史記錄)"""
        self.status_text.config(state='normal')
        self.status_text.insert(tk.END, message + "\n")
        self.status_text.see(tk.END)
        lines = int(self.status_text.index('end-1c').split('.')[0])
        if lines > 100:
            self.status_text.delete(1.0, f"{lines - 100}.0")
        self.status_text.config(state='disabled')

    def update_display(self):
        """更新顯示 - 20Hz更新頻率"""
        try:
            pos = self.ctrl.get_joint5_position()
            for i, axis in enumerate(['X', 'Y', 'Z']):
                self.current_labels[axis].configure(text=f"{pos[i]:.3f} m")

            for i, angle in enumerate(self.ctrl.current_angles):
                self.joint_labels[i].configure(text=f"{angle:.1f}°")

            if self.ctrl.is_following_trajectory and self.ctrl.trajectory_points is not None:
                total = len(self.ctrl.trajectory_points)
                current = self.ctrl.trajectory_index
                step = self.ctrl.trajectory_step
                progress = (current / total * 100) if total > 0 else 0
                status_msg = f"執行中... {current}/{total} ({progress:.1f}%)\n倍速: {step}倍"
                self.update_status(status_msg)

            self.root.after(50, self.update_display)
        except:
            pass

    def quit_program(self):
        """關閉程式"""
        if messagebox.askyesno("確認", "確定要關閉程式嗎?"):
            global exit_flag
            exit_flag = True
            self.root.quit()
            self.root.destroy()


# ---------- GUI 啟動函數 ----------

def start_gui():
    """啟動增強版 GUI"""
    root = tk.Tk()
    gui = RobotControlGUI(root, ctrl)
    root.mainloop()


# ---------- Main ----------

def main_loop():
    global exit_flag
    pos = ctrl.get_joint5_position()
    # 紅球尺寸縮小50%: 原本0.08 -> 0.04
    marker = o3d.geometry.TriangleMesh.create_sphere(radius=0.04)
    marker.paint_uniform_color([1.0, 0.0, 0.0])
    marker.compute_vertex_normals()
    marker.translate(pos)
    vis.add_geometry(marker)
    ctrl.set_marker(marker)

    gui_thread = threading.Thread(target=start_gui, daemon=False)
    gui_thread.start()

    last = time.time()

    try:
        while not exit_flag:
            now = time.time()
            if now - last > 1 / 100:
                ctrl.animate()
                last = now

            vis.poll_events()
            vis.update_renderer()
    except:
        pass
    finally:
        try:
            vis.destroy_window()
        except:
            pass
        sys.exit(0)


if __name__ == "__main__":
    main_loop()