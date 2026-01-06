'''
20251119 - 障礙物避障版本
Open3D 視窗(顯示 3D 機械手臂)
Tkinter 控制介面(增強版 GUI + 障礙物功能)

新增功能:
1. 障礙物設定與管理
2. 長方體障礙物可視化
3. 碰撞檢測功能
4. 障礙物列表顯示
5. 軌跡執行時自動避障

原有功能:
1. 關節動畫速度調節
2. 軌跡點跳躍拉桿(一次跳多個點)
3. GUI更新頻率20Hz
4. 視窗並排顯示
5. 主迴圈更新頻率 100Hz
6. Joint5 紅球平移更新
7. 執行圓弧移動時保存1000點座標到日期命名的文字檔
8. 移動後顯示殘影軌跡,每個點在出現後10秒單獨消失
9. 第二次執行時不清除現有殘影,舊殘影繼續依時間自動消失
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


# ---------- 手臂連桿包圍盒類別 ----------

def create_bbox_from_mesh(mesh, name="", expansion=0.01):
    """
    從 mesh 自動創建貼合的包圍盒
    mesh: Open3D TriangleMesh
    name: 包圍盒名稱
    expansion: 擴展尺寸(公尺),用於安全餘量
    """
    # 獲取 mesh 的包圍盒
    bbox_points = np.asarray(mesh.vertices)
    min_bound = bbox_points.min(axis=0)
    max_bound = bbox_points.max(axis=0)

    # 計算中心和尺寸
    center = (min_bound + max_bound) / 2
    size = max_bound - min_bound + expansion * 2  # 加上安全餘量

    return center, size


class ArmSegmentBoundingBox:
    """手臂連桿包圍盒 - 用於碰撞檢測"""

    def __init__(self, center, size, name="", color=[0.0, 1.0, 1.0, 1.0]):
        """
        初始化手臂連桿包圍盒
        center: [x, y, z] 中心位置 (公尺)
        size: [length, width, height] 尺寸 (公尺)
        name: 連桿名稱
        color: RGB顏色 (青色線框)
        """
        self.initial_center = np.array(center, dtype=float)
        self.current_center = np.array(center, dtype=float)
        self.size = np.array(size, dtype=float)
        self.name = name if name else f"Segment_{id(self)}"
        self.color = color[:3]  # RGB only for Open3D - 青色 [0, 1, 1]
        self.mesh = None
        self.R_total = np.eye(3)  # 累積的旋轉矩陣
        self.create_mesh()

    def create_mesh(self):
        """創建包圍盒的線框網格"""
        # 先創建實體包圍盒
        box = o3d.geometry.TriangleMesh.create_box(
            width=self.size[0],
            height=self.size[1],
            depth=self.size[2]
        )
        # 移動到中心位置
        box.translate(self.current_center - self.size / 2)

        # 轉換為線框 LineSet
        self.mesh = o3d.geometry.LineSet.create_from_triangle_mesh(box)

        # 設定青色線條
        self.mesh.paint_uniform_color(self.color)

    def transform(self, transformation, center):
        """
        應用變換矩陣
        transformation: 4x4 變換矩陣
        center: 旋轉中心點
        """
        # 更新累積旋轉矩陣
        R = transformation[:3, :3]
        self.R_total = R @ self.R_total

        # 應用旋轉和平移
        self.mesh.rotate(R, center=center)
        self.mesh.translate(transformation[:3, 3])

        # 更新當前中心位置
        self.current_center = R @ (self.current_center - center) + center + transformation[:3, 3]

    def check_collision_with_point(self, point):
        """
        檢查點是否與包圍盒碰撞
        使用 OBB (Oriented Bounding Box) 檢測
        point: [x, y, z] 檢查點座標
        返回: True 如果碰撞, False 如果安全
        """
        point = np.array(point)

        # 將點轉換到包圍盒的局部座標系
        # 使用逆旋轉矩陣將點轉換到包圍盒的局部空間
        R_inv = self.R_total.T  # 旋轉矩陣的逆 = 轉置
        local_point = R_inv @ (point - self.current_center)

        # 在局部座標系中,包圍盒是軸對齊的
        half_size = self.size / 2

        # 檢查點是否在局部AABB內
        return np.all(np.abs(local_point) <= half_size)

    def update_mesh(self):
        """更新包圍盒網格(當參數改變時)"""
        # 創建新的實體包圍盒
        box = o3d.geometry.TriangleMesh.create_box(
            width=self.size[0],
            height=self.size[1],
            depth=self.size[2]
        )
        box.translate(self.current_center - self.size / 2)

        # 轉換為線框
        line_set = o3d.geometry.LineSet.create_from_triangle_mesh(box)
        line_set.paint_uniform_color(self.color)

        # 更新 mesh (實際上是 LineSet)
        self.mesh.points = line_set.points
        self.mesh.lines = line_set.lines
        self.mesh.colors = line_set.colors


class ArmBoundingBoxManager:
    """手臂包圍盒管理器"""

    def __init__(self, visualizer):
        self.bounding_boxes = []
        self.vis = visualizer

    def add_bounding_box(self, center, size, name=""):
        """新增手臂連桿包圍盒"""
        bbox = ArmSegmentBoundingBox(center, size, name)
        self.bounding_boxes.append(bbox)
        self.vis.add_geometry(bbox.mesh)
        return bbox

    def check_collision_with_point(self, point):
        """
        檢查點是否與任何手臂連桿碰撞
        返回: (is_collision, segment_name)
        """
        for bbox in self.bounding_boxes:
            if bbox.check_collision_with_point(point):
                return True, bbox.name
        return False, None

    def get_all_meshes(self):
        """獲取所有包圍盒網格"""
        return [bbox.mesh for bbox in self.bounding_boxes]


# ---------- 障礙物類別 ----------

class Obstacle:
    """長方體障礙物類別"""

    def __init__(self, center, size, name=""):
        """
        初始化障礙物
        center: [x, y, z] 中心位置 (公尺)
        size: [length, width, height] 尺寸 (公尺)
        """
        self.center = np.array(center, dtype=float)
        self.size = np.array(size, dtype=float)
        self.name = name if name else f"障礙物_{id(self)}"
        self.mesh = None
        self.create_mesh()

    def create_mesh(self):
        """創建障礙物的3D網格"""
        self.mesh = o3d.geometry.TriangleMesh.create_box(
            width=self.size[0],
            height=self.size[1],
            depth=self.size[2]
        )
        # 移動到中心位置
        self.mesh.translate(self.center - self.size / 2)
        # 設定半透明青色
        self.mesh.paint_uniform_color([0, 0.7, 0.7])
        self.mesh.compute_vertex_normals()

    def check_collision(self, point):
        """
        檢查點是否與障礙物碰撞
        point: [x, y, z] 檢查點座標
        返回: True 如果碰撞, False 如果安全
        """
        point = np.array(point)
        half_size = self.size / 2
        min_bound = self.center - half_size
        max_bound = self.center + half_size

        # 檢查點是否在長方體內
        return np.all(point >= min_bound) and np.all(point <= max_bound)

    def update_mesh(self):
        """更新障礙物網格(當參數改變時)"""
        self.mesh.clear()
        box = o3d.geometry.TriangleMesh.create_box(
            width=self.size[0],
            height=self.size[1],
            depth=self.size[2]
        )
        box.translate(self.center - self.size / 2)
        box.paint_uniform_color([0, 0.7, 0.7])
        box.compute_vertex_normals()
        self.mesh.vertices = box.vertices
        self.mesh.triangles = box.triangles
        self.mesh.vertex_normals = box.vertex_normals
        self.mesh.vertex_colors = box.vertex_colors

    def __str__(self):
        return f"{self.name}: 中心({self.center[0]:.2f}, {self.center[1]:.2f}, {self.center[2]:.2f}) 尺寸({self.size[0]:.2f}×{self.size[1]:.2f}×{self.size[2]:.2f})"


class ObstacleManager:
    """障礙物管理器"""

    def __init__(self, visualizer):
        self.obstacles = []
        self.vis = visualizer

    def add_obstacle(self, center, size, name=""):
        """新增障礙物"""
        obstacle = Obstacle(center, size, name)
        self.obstacles.append(obstacle)
        self.vis.add_geometry(obstacle.mesh)
        return obstacle

    def remove_obstacle(self, index):
        """移除指定索引的障礙物"""
        if 0 <= index < len(self.obstacles):
            obstacle = self.obstacles.pop(index)
            self.vis.remove_geometry(obstacle.mesh, reset_bounding_box=False)
            return True
        return False

    def clear_all(self):
        """清除所有障礙物"""
        for obstacle in self.obstacles:
            self.vis.remove_geometry(obstacle.mesh, reset_bounding_box=False)
        self.obstacles.clear()

    def check_collision(self, point):
        """
        檢查點是否與任何障礙物碰撞
        返回: (is_collision, obstacle_name)
        """
        for obstacle in self.obstacles:
            if obstacle.check_collision(point):
                return True, obstacle.name
        return False, None

    def get_obstacle_list(self):
        """獲取障礙物列表資訊"""
        return [str(obs) for obs in self.obstacles]

    def update_obstacle(self, index, center=None, size=None):
        """更新障礙物參數"""
        if 0 <= index < len(self.obstacles):
            obstacle = self.obstacles[index]
            if center is not None:
                obstacle.center = np.array(center, dtype=float)
            if size is not None:
                obstacle.size = np.array(size, dtype=float)
            obstacle.update_mesh()
            self.vis.update_geometry(obstacle.mesh)
            return True
        return False


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
    def __init__(self, mesh, name="", bounding_box=None):
        self.mesh = mesh
        self.name = name
        self.children = []
        self.bounding_box = bounding_box  # 對應的包圍盒

    def add_child(self, child_node):
        self.children.append(child_node)

    def set_bounding_box(self, bbox):
        """設定對應的包圍盒"""
        self.bounding_box = bbox

    def transform(self, transformation, center):
        self.mesh.rotate(transformation[:3, :3], center=center)
        self.mesh.translate(transformation[:3, 3])

        # 同步更新包圍盒
        if self.bounding_box is not None:
            self.bounding_box.transform(transformation, center)
            vis.update_geometry(self.bounding_box.mesh)

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
print("RA605-710-GC 六軸機械手臂控制系統 (障礙物避障版)")
print("=" * 60)
print(f"S1={S1 * 1000:.1f} S2={S2 * 1000:.1f} L1={L1 * 1000:.1f}")
print(f"L2={L2 * 1000:.1f} L3={L3 * 1000:.1f} L4={L4 * 1000:.1f}")
print("=" * 60 + "\n")

# ---------- 載入模型 ----------

# paths = [rf"/home/yahboom/Desktop/Obj/p{i}.obj" for i in range(1, 9)]
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
vis.create_window("Robot Arm with Obstacles", width=500, height=500, left=870, top=50)

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

# ---------- 初始化障礙物管理器 ----------
obstacle_manager = ObstacleManager(vis)

# ---------- 初始化手臂包圍盒管理器 ----------
# 在所有初始旋轉完成後才創建包圍盒
arm_bbox_manager = ArmBoundingBoxManager(vis)

# 自動從 mesh 創建包圍盒
mesh_bbox_pairs = [
    (meshes[1], "Link1_p2", 0.01),  # p2
    (meshes[2], "Link2_p3", 0.01),  # p3
    (meshes[3], "Link3_p4", 0.01),  # p4
    (meshes[4], "Link4_p5", 0.01),  # p5
    (meshes[5], "Link5_p6", 0.01),  # p6
    (meshes[6], "Link6_p7", 0.01),  # p7
    # 不包含 p8 (末端執行器)
]

arm_bboxes = []
# 跳過 p1 (底座不需要包圍盒)
arm_bboxes.append(None)  # placeholder for p1

for mesh, name, expansion in mesh_bbox_pairs:
    center, size = create_bbox_from_mesh(mesh, name, expansion)
    bbox = arm_bbox_manager.add_bounding_box(center, size, name)
    arm_bboxes.append(bbox)

# 將包圍盒綁定到對應的節點
if len(arm_bboxes) >= 7:
    # p1 跳過 (index 0)

    # p2 - 跟隨 node2 旋轉 (Z軸) - index 1
    node2.set_bounding_box(arm_bboxes[1])

    # p3 - 跟隨 node3 旋轉 (Y軸,shoulder) - index 2
    node3.set_bounding_box(arm_bboxes[2])

    # p4 - 跟隨 node4 旋轉 (Y軸,elbow) - index 3
    node4.set_bounding_box(arm_bboxes[3])

    # p5 - 跟隨 node5 旋轉 (Z軸,wrist roll) - index 4
    node5.set_bounding_box(arm_bboxes[4])

    # p6 - 跟隨 node6 旋轉 (Y軸,wrist pitch) - index 5
    node6.set_bounding_box(arm_bboxes[5])

    # p7 - 跟隨 node7 旋轉 (X軸,wrist yaw) - index 6
    node7.set_bounding_box(arm_bboxes[6])

    # p8 不需要包圍盒


# ---------- 控制器 ----------

class AnimationController:
    def __init__(self, obstacle_manager, arm_bbox_manager):
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
        self.trajectory_step = 1
        self.last_trajectory_time = 0
        self.skip_unreachable_points = True
        self.skipped_points_count = 0
        self.collision_count = 0  # 碰撞計數
        self.arm_collision_count = 0  # 手臂自碰撞計數
        self.status_callback = None

        # 殘影軌跡相關變量
        self.trail_line = None
        self.trail_points = []

        # 障礙物管理器
        self.obstacle_manager = obstacle_manager

        # 手臂包圍盒管理器
        self.arm_bbox_manager = arm_bbox_manager

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
        return self.R_p2_total @ (
                self.p3_base_local +
                self.R_p3_total @ (
                        self.p3_length_vec +
                        self.R_p4_total @ (
                                self.p4_length_vec +
                                self.R_p5_total @ self.p5_length_vec
                        )
                )
        )

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
        # 持續更新殘影（會自動移除超過10秒的點）
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
        R2 = o3d.geometry.get_rotation_matrix_from_axis_angle([0, 0, np.deg2rad(angles[0])])
        R3 = o3d.geometry.get_rotation_matrix_from_axis_angle([0, np.deg2rad(angles[1]), 0])
        R4b = o3d.geometry.get_rotation_matrix_from_axis_angle([0, np.deg2rad(-90), 0])
        R4 = o3d.geometry.get_rotation_matrix_from_axis_angle([0, np.deg2rad(angles[2]), 0]) @ R4b
        R5 = o3d.geometry.get_rotation_matrix_from_axis_angle([0, 0, np.deg2rad(angles[3])])
        return R2 @ (self.p3_base_local + R3 @ (
                self.p3_length_vec + R4 @ (self.p4_length_vec + R5 @ self.p5_length_vec)))

    def ik(self, x, y, z):
        try:
            theta1 = np.arctan2(-y, -x)
            r_xy = np.sqrt(y ** 2 + x ** 2)
            z_off = z - self.L1
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
            angles = [np.rad2deg(theta1), np.rad2deg(theta2), np.rad2deg(theta3), 0]
            limits = [(-165, 165), (-125, 85), (-55, 185), (-190, 190)]
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
        for i in range(4):
            self.set_target(i, angles[i])
        return True, "ok"

    def move_to_instant(self, x, y, z):
        """直接移動到目標位置,不使用動畫"""
        res = self.ik(x, y, z)
        if res[0] is None:
            return False, res[1]
        angles = res[0]

        for i in range(4):
            delta = angles[i] - self.current_angles[i]
            self.current_angles[i] = angles[i]
            self.target_angles[i] = angles[i]
            if abs(delta) > 0.0001:
                self.update_joint(i, delta)

        return True, "ok"

    def update_trail(self):
        """更新殘影軌跡顯示 - 自動移除超過10秒的點"""
        current_time = time.time()
        self.trail_points = [(pt, ts) for pt, ts in self.trail_points if current_time - ts <= 10]

        if len(self.trail_points) < 2:
            if self.trail_line:
                vis.remove_geometry(self.trail_line, reset_bounding_box=False)
                self.trail_line = None
            return

        if self.trail_line:
            vis.remove_geometry(self.trail_line, reset_bounding_box=False)

        points_only = [pt for pt, ts in self.trail_points]
        pts = o3d.utility.Vector3dVector(points_only)
        lns = [[i, i + 1] for i in range(len(points_only) - 1)]
        ls = o3d.geometry.LineSet()
        ls.points = pts
        ls.lines = o3d.utility.Vector2iVector(lns)
        ls.paint_uniform_color([0, 1, 0])
        self.trail_line = ls
        vis.add_geometry(ls, reset_bounding_box=False)

    def clear_trail(self):
        """清除殘影軌跡"""
        if self.trail_line:
            vis.remove_geometry(self.trail_line, reset_bounding_box=False)
            self.trail_line = None
        self.trail_points = []

    def show_trajectory(self, points, start_index=0):
        """顯示軌跡線(從start_index開始)"""
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
        self.collision_count = 0  # 重置環境碰撞計數
        self.arm_collision_count = 0  # 重置手臂碰撞計數
        self.is_following_trajectory = True
        self.last_trajectory_time = time.time()
        self.move_next_point()

    def move_next_point(self):
        """移動到下一個軌跡點(含障礙物檢測和手臂自碰撞檢測)"""
        if not self.is_following_trajectory or self.trajectory_points is None:
            return

        if self.trajectory_index >= len(self.trajectory_points):
            self.is_following_trajectory = False
            self.skipped_points_count = 0
            self.collision_count = 0
            self.arm_collision_count = 0
            self.log_status(f"軌跡執行完成!")
            return

        pt = self.trajectory_points[self.trajectory_index]

        # 檢查是否與環境障礙物碰撞
        is_collision, obstacle_name = self.obstacle_manager.check_collision(pt)

        if is_collision:
            # 發現與環境障礙物碰撞,跳過此點
            self.collision_count += 1
            self.log_status(f"警告: 點 {self.trajectory_index} 與 {obstacle_name} 碰撞,已跳過")
            self.trajectory_index += self.trajectory_step

            # 如果碰撞次數過多,停止執行
            if self.collision_count > len(self.trajectory_points) * 0.3:
                self.is_following_trajectory = False
                self.log_status(f"錯誤: 環境碰撞點過多({self.collision_count}),已停止執行")
                return
            return

        # 檢查是否與手臂本身碰撞
        is_arm_collision, arm_segment_name = self.arm_bbox_manager.check_collision_with_point(pt)

        if is_arm_collision:
            # 發現與手臂自身碰撞,跳過此點
            self.arm_collision_count += 1
            self.log_status(f"警告: 點 {self.trajectory_index} 與手臂 {arm_segment_name} 碰撞,已跳過")
            self.trajectory_index += self.trajectory_step

            # 如果手臂自碰撞次數過多,停止執行
            if self.arm_collision_count > len(self.trajectory_points) * 0.2:
                self.is_following_trajectory = False
                self.log_status(f"錯誤: 手臂自碰撞點過多({self.arm_collision_count}),已停止執行")
                return
            return

        # 嘗試移動到目標點
        ok, msg = self.move_to_instant(pt[0], pt[1], pt[2])

        if ok:
            # 成功移動,記錄殘影
            current_time = time.time()
            self.trail_points.append(([pt[0], pt[1], pt[2]], current_time))
            self.update_trail()
            self.trajectory_index += self.trajectory_step
        else:
            # 移動失敗(超出範圍或其他錯誤)
            if self.skip_unreachable_points:
                if self.skipped_points_count > len(self.trajectory_points) * 0.2:
                    self.is_following_trajectory = False
                    self.skipped_points_count = 0
                    self.log_status(f"錯誤: 跳過點數過多,已停止執行")
                    return

                self.skipped_points_count += 1
                self.trajectory_index += self.trajectory_step

                consecutive_skips = 1
                found_reachable = False

                while self.trajectory_index < len(self.trajectory_points):
                    if self.skipped_points_count > len(self.trajectory_points) * 0.2:
                        self.is_following_trajectory = False
                        self.skipped_points_count = 0
                        self.log_status(f"錯誤: 跳過點數過多,已停止執行")
                        return

                    pt_next = self.trajectory_points[self.trajectory_index]

                    # 檢查下一個點是否與環境障礙物碰撞
                    is_collision_next, obstacle_name_next = self.obstacle_manager.check_collision(pt_next)
                    if is_collision_next:
                        self.collision_count += 1
                        self.trajectory_index += self.trajectory_step
                        continue

                    # 檢查下一個點是否與手臂碰撞
                    is_arm_collision_next, arm_segment_name_next = self.arm_bbox_manager.check_collision_with_point(
                        pt_next)
                    if is_arm_collision_next:
                        self.arm_collision_count += 1
                        self.trajectory_index += self.trajectory_step
                        continue

                    ok_next, msg_next = self.move_to_instant(pt_next[0], pt_next[1], pt_next[2])

                    if ok_next:
                        found_reachable = True
                        current_time = time.time()
                        self.trail_points.append(([pt_next[0], pt_next[1], pt_next[2]], current_time))
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
                    self.log_status(f"錯誤: 無法找到可達點,已停止執行")
            else:
                self.is_following_trajectory = False
                self.skipped_points_count = 0
                self.log_status(f"錯誤: 遇到無法到達的點,已停止執行")

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
        self.collision_count = 0
        self.arm_collision_count = 0
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


ctrl = AnimationController(obstacle_manager, arm_bbox_manager)
exit_flag = False


# ---------- 增強版 GUI (含障礙物控制) ----------

class RobotControlGUI:
    def __init__(self, root, controller, obstacle_manager):
        self.root = root
        self.ctrl = controller
        self.obstacle_mgr = obstacle_manager
        self.root.title("RA605-710-GC 六軸機械手臂控制系統 (障礙物避障版)")
        self.root.geometry("850x950+0+50")

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
        self.create_obstacle_control(main_frame)  # 新增障礙物控制面板
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
            header_frame = ttk.Frame(frame)
            header_frame.grid(row=i * 2, column=0, sticky=tk.W, pady=(10 if i > 0 else 5, 5))

            ttk.Label(header_frame, text=name, font=('Arial', 10, 'bold')).pack(side=tk.LEFT, pady=2)

            value_label = ttk.Label(header_frame, text="0.0°",
                                    foreground="blue", font=('Arial', 10, 'bold'))
            value_label.pack(side=tk.RIGHT, pady=3, padx=5)
            self.joint_labels.append(value_label)

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
        self.anim_speed_label.pack(side=tk.LEFT, pady=3)

        self.anim_speed_scale.configure(command=self.on_anim_speed_change)

        btn_frame = ttk.Frame(frame)
        btn_frame.grid(row=len(joint_config) * 2 + 1, column=0, pady=10)

        ttk.Button(btn_frame, text="重置姿態",
                   command=self.reset_pose).pack(fill=tk.X, pady=2)

    def create_position_control(self, parent):
        """位置控制與軌跡規劃面板"""
        frame = ttk.Frame(parent)
        frame.grid(row=0, column=1, padx=5, pady=5, sticky=(tk.N, tk.S, tk.W, tk.E))

        current_frame = ttk.LabelFrame(frame, text="當前末端位置 (J5)", padding="10")
        current_frame.pack(fill=tk.X, pady=(0, 10))

        pos_frame = ttk.Frame(current_frame)
        pos_frame.pack()

        self.current_labels = {}
        for i, axis in enumerate(['X', 'Y', 'Z']):
            ttk.Label(pos_frame, text=f"{axis}:", font=('Arial', 10, 'bold')).grid(
                row=0, column=i * 2, padx=5, sticky=tk.W)
            label = ttk.Label(pos_frame, text="0.000 m",
                              font=('Courier', 11), foreground='green')
            label.grid(row=0, column=i * 2 + 1, padx=5, sticky=tk.W, pady=3)
            self.current_labels[axis] = label

        target_frame = ttk.LabelFrame(frame, text="目標位置設定", padding="10")
        target_frame.pack(fill=tk.X, pady=(0, 10))

        self.target_scales = {}
        target_config = [
            ('X', -0.7, 0.7, 0.3),
            ('Y', -0.7, 0.7, 0.3),
            ('Z', 0.0, 1.0, 0.8)
        ]

        for axis, min_val, max_val, default in target_config:
            axis_frame = ttk.Frame(target_frame)
            axis_frame.pack(fill=tk.X, pady=3)

            ttk.Label(axis_frame, text=f"{axis}:", width=3).pack(side=tk.LEFT)

            scale = ttk.Scale(axis_frame, from_=min_val, to=max_val,
                              orient=tk.HORIZONTAL)
            scale.set(default)
            scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)

            value_label = ttk.Label(axis_frame, text=f"{default:.2f}", width=6)
            value_label.pack(side=tk.LEFT, pady=3)

            scale.configure(command=lambda v, lbl=value_label: lbl.configure(
                text=f"{float(v):.2f}"))

            self.target_scales[axis] = scale

        traj_frame = ttk.LabelFrame(frame, text="圓弧軌跡控制", padding="10")
        traj_frame.pack(fill=tk.BOTH, expand=True)

        radius_frame = ttk.Frame(traj_frame)
        radius_frame.pack(fill=tk.X, pady=5)

        ttk.Label(radius_frame, text="圓弧係數:").pack(side=tk.LEFT)
        self.radius_scale = ttk.Scale(radius_frame, from_=0.5, to=5.0,
                                      orient=tk.HORIZONTAL)
        self.radius_scale.set(2.0)
        self.radius_scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)

        self.radius_label = ttk.Label(radius_frame, text="2.00", width=5)
        self.radius_label.pack(side=tk.LEFT, pady=3)
        self.radius_scale.configure(command=lambda v: self.radius_label.configure(
            text=f"{float(v):.2f}"))

        speed_frame = ttk.Frame(traj_frame)
        speed_frame.pack(fill=tk.X, pady=5)

        ttk.Label(speed_frame, text="軌跡速度:").pack(side=tk.LEFT)

        self.speed_scale = ttk.Scale(speed_frame, from_=1, to=1000,
                                     orient=tk.HORIZONTAL)
        self.speed_scale.set(100)
        self.speed_scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)

        self.speed_label = ttk.Label(speed_frame, text="100x", width=8)
        self.speed_label.pack(side=tk.LEFT, pady=3)

        self.speed_scale.configure(command=self.on_speed_change)

        step_frame = ttk.Frame(traj_frame)
        step_frame.pack(fill=tk.X, pady=5)

        ttk.Label(step_frame, text="動畫倍速:").pack(side=tk.LEFT)

        self.step_scale = ttk.Scale(step_frame, from_=1, to=50,
                                    orient=tk.HORIZONTAL)
        self.step_scale.set(1)
        self.step_scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)

        self.step_label = ttk.Label(step_frame, text="1倍", width=8)
        self.step_label.pack(side=tk.LEFT, pady=3)

        self.step_scale.configure(command=self.on_step_change)

        option_frame = ttk.Frame(traj_frame)
        option_frame.pack(fill=tk.X, pady=5)

        self.skip_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(option_frame, text="自動跳過無法到達的點",
                        variable=self.skip_var,
                        command=self.toggle_skip).pack(anchor=tk.W)

        status_frame = ttk.Frame(traj_frame)
        status_frame.pack(fill=tk.BOTH, expand=True, pady=5)

        status_scrollbar = ttk.Scrollbar(status_frame)
        status_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        self.status_text = tk.Text(status_frame, height=6, width=40,
                                   font=('Courier', 9),
                                   state='disabled',
                                   wrap=tk.WORD,
                                   yscrollcommand=status_scrollbar.set,
                                   spacing1=3,
                                   spacing2=2,
                                   spacing3=3,
                                   padx=5,
                                   pady=5)
        self.status_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        status_scrollbar.config(command=self.status_text.yview)
        self.update_status("準備就緒")

        btn_frame = ttk.Frame(traj_frame)
        btn_frame.pack(fill=tk.X, pady=5)

        ttk.Button(btn_frame, text="執行圓弧移動",
                   command=self.execute_arc).pack(side=tk.LEFT, padx=2)
        ttk.Button(btn_frame, text="停止",
                   command=self.stop_trajectory).pack(side=tk.LEFT, padx=2)
        ttk.Button(btn_frame, text="重置軌跡",
                   command=self.reset_trajectory).pack(side=tk.LEFT, padx=2)

    def create_obstacle_control(self, parent):
        """障礙物控制面板"""
        frame = ttk.LabelFrame(parent, text="障礙物管理", padding="10")
        frame.grid(row=1, column=0, columnspan=2, padx=5, pady=5, sticky=(tk.W, tk.E, tk.N, tk.S))

        # 左側:參數設定
        left_frame = ttk.Frame(frame)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 10))

        # 中心位置
        center_frame = ttk.LabelFrame(left_frame, text="中心位置 (公尺)", padding="5")
        center_frame.pack(fill=tk.X, pady=5)

        self.obs_center_scales = {}
        for axis, default in [('X', 0.30), ('Y', 0.30), ('Z', 0.65)]:
            axis_frame = ttk.Frame(center_frame)
            axis_frame.pack(fill=tk.X, pady=3)

            ttk.Label(axis_frame, text=f"{axis}:", width=3).pack(side=tk.LEFT)

            scale = ttk.Scale(axis_frame, from_=-0.7, to=0.7 if axis != 'Z' else 1.0,
                              orient=tk.HORIZONTAL)
            scale.set(default)
            scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)

            value_label = ttk.Label(axis_frame, text=f"{default:.2f}", width=6)
            value_label.pack(side=tk.LEFT, pady=3)

            scale.configure(command=lambda v, lbl=value_label: lbl.configure(
                text=f"{float(v):.2f}"))

            self.obs_center_scales[axis] = scale

        # 尺寸
        size_frame = ttk.LabelFrame(left_frame, text="尺寸 (公尺)", padding="5")
        size_frame.pack(fill=tk.X, pady=5)

        self.obs_size_scales = {}
        for dim, default in [('長', 0.20), ('寬', 0.20), ('高', 0.30)]:
            dim_frame = ttk.Frame(size_frame)
            dim_frame.pack(fill=tk.X, pady=3)

            ttk.Label(dim_frame, text=f"{dim}:", width=3).pack(side=tk.LEFT)

            scale = ttk.Scale(dim_frame, from_=0.05, to=0.5,
                              orient=tk.HORIZONTAL)
            scale.set(default)
            scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)

            value_label = ttk.Label(dim_frame, text=f"{default:.2f}", width=6)
            value_label.pack(side=tk.LEFT, pady=3)

            scale.configure(command=lambda v, lbl=value_label: lbl.configure(
                text=f"{float(v):.2f}"))

            self.obs_size_scales[dim] = scale

        # 按鈕
        btn_frame = ttk.Frame(left_frame)
        btn_frame.pack(fill=tk.X, pady=5)

        ttk.Button(btn_frame, text="新增障礙物",
                   command=self.add_obstacle).pack(side=tk.LEFT, padx=2)
        ttk.Button(btn_frame, text="刪除選中",
                   command=self.remove_obstacle).pack(side=tk.LEFT, padx=2)
        ttk.Button(btn_frame, text="清除全部",
                   command=self.clear_obstacles).pack(side=tk.LEFT, padx=2)

        # 右側:障礙物列表
        right_frame = ttk.Frame(frame)
        right_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        list_label = ttk.Label(right_frame, text="當前障礙物列表:", font=('Arial', 10, 'bold'))
        list_label.pack(anchor=tk.W)

        list_scroll = ttk.Scrollbar(right_frame)
        list_scroll.pack(side=tk.RIGHT, fill=tk.Y)

        self.obstacle_listbox = tk.Listbox(right_frame, height=8,
                                           font=('Courier', 8),
                                           yscrollcommand=list_scroll.set,
                                           selectmode=tk.SINGLE)
        self.obstacle_listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, pady=2)
        list_scroll.config(command=self.obstacle_listbox.yview)

    def add_obstacle(self):
        """新增障礙物"""
        try:
            center = [
                self.obs_center_scales['X'].get(),
                self.obs_center_scales['Y'].get(),
                self.obs_center_scales['Z'].get()
            ]
            size = [
                self.obs_size_scales['長'].get(),
                self.obs_size_scales['寬'].get(),
                self.obs_size_scales['高'].get()
            ]

            obstacle = self.obstacle_mgr.add_obstacle(center, size)
            self.update_obstacle_list()
            self.append_status(f"已新增: {obstacle.name}")

        except Exception as e:
            messagebox.showerror("錯誤", f"新增障礙物失敗: {str(e)}")

    def remove_obstacle(self):
        """刪除選中的障礙物"""
        selection = self.obstacle_listbox.curselection()
        if not selection:
            messagebox.showwarning("提示", "請先選擇要刪除的障礙物")
            return

        index = selection[0]
        if self.obstacle_mgr.remove_obstacle(index):
            self.update_obstacle_list()
            self.append_status(f"已刪除障礙物 #{index + 1}")
        else:
            messagebox.showerror("錯誤", "刪除失敗")

    def clear_obstacles(self):
        """清除所有障礙物"""
        if messagebox.askyesno("確認", "確定要清除所有障礙物嗎?"):
            self.obstacle_mgr.clear_all()
            self.update_obstacle_list()
            self.append_status("已清除所有障礙物")

    def update_obstacle_list(self):
        """更新障礙物列表顯示"""
        self.obstacle_listbox.delete(0, tk.END)
        for i, obs_str in enumerate(self.obstacle_mgr.get_obstacle_list(), 1):
            self.obstacle_listbox.insert(tk.END, f"{i}. {obs_str}")

    def create_system_control(self, parent):
        """系統控制面板"""
        frame = ttk.Frame(parent)
        frame.grid(row=2, column=0, columnspan=3, pady=10, sticky=(tk.W, tk.E))

        ttk.Button(frame, text="關閉程式",
                   command=self.quit_program).pack(side=tk.RIGHT, padx=5)

        self.info_label = ttk.Label(frame, text="RA605-710-GC 六軸機械手臂 (障礙物避障版)",
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
                env_collision = self.ctrl.collision_count
                arm_collision = self.ctrl.arm_collision_count
                status_msg = f"執行中... {current}/{total} ({progress:.1f}%)\n倍速: {step}倍\n環境碰撞: {env_collision}\n手臂碰撞: {arm_collision}"
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
    gui = RobotControlGUI(root, ctrl, obstacle_manager)
    root.mainloop()


# ---------- Main ----------

def main_loop():
    global exit_flag
    pos = ctrl.get_joint5_position()
    marker = o3d.geometry.TriangleMesh.create_sphere(radius=0.08)
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