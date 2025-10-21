'''
20250930
起始位置為政成姿態
新增：回復預設姿態按鈕
新增：結束程式按鈕
新增：關節5位置顯示
新增：打印所有關節位置功能
新增：在Open3D中用紅點顯示關節5位置（已修正位置計算）

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

# 重新組織節點結構，將P8作為獨立的關節
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

# ---------- 初始讓 p6~p8 旋轉 90 度 ----------
p5_top_center = [0.01, 0, 1.05]
R = o3d.geometry.get_rotation_matrix_from_axis_angle([0, np.deg2rad(-90), 0])
T = np.eye(4)
T[:3, :3] = R
node6_group.transform(T, p5_top_center)

# ---------- 初始讓 p4~p8 旋轉 90 度----------
# 計算 P3 的頂端位置作為旋轉中心
p3_base_local_init = np.array([-0.03, 0, 0.38])
p3_length_vec_init = np.array([0, 0, 0.34])
p3_top_center = p3_base_local_init + p3_length_vec_init  # [-0.03, 0, 0.72]

# 旋轉 -90 度（與 p6~p8 一致）
R_p4_init = o3d.geometry.get_rotation_matrix_from_axis_angle([0, np.deg2rad(-90), 0])
T_p4_init = np.eye(4)
T_p4_init[:3, :3] = R_p4_init
node4_group.transform(T_p4_init, p3_top_center)

# ---------- 建立視窗 ----------
vis = o3d.visualization.Visualizer()
vis.create_window("機械手臂3d圖", width=1000, height=1000)

# 添加所有mesh到視窗
for root in root_nodes:
    for m in root.get_all_meshes():
        vis.add_geometry(m)

# ---------- 添加座標軸和網格 ----------
# 1. 建立世界座標系軸線（更靠近原點）
world_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3, origin=[0.375, 0.3, 0])
vis.add_geometry(world_frame)


# 2. 建立增強網格線
def create_enhanced_grid(size=2.0, major_step=0.5, minor_step=0.1):
    """建立增強網格線"""
    lines = []
    points = []
    colors = []

    # 建立X-Y平面的網格線
    major_num = int(size / major_step) + 1
    minor_num = int(size / minor_step) + 1

    # 主要網格線（每0.5m，較粗較暗）
    for i in range(major_num):
        coord = -size / 2 + i * major_step

        # X方向的主要線
        points.extend([[coord, -size / 2, 0], [coord, size / 2, 0]])
        line_idx = len(points) - 2
        lines.append([line_idx, line_idx + 1])
        colors.append([0.4, 0.4, 0.4])  # 深灰色

        # Y方向的主要線
        points.extend([[-size / 2, coord, 0], [size / 2, coord, 0]])
        line_idx = len(points) - 2
        lines.append([line_idx, line_idx + 1])
        colors.append([0.4, 0.4, 0.4])  # 深灰色

    # 次要網格線（每0.1m，較細較淡）
    for i in range(minor_num):
        coord = -size / 2 + i * minor_step

        # 跳過已經有主要線的位置
        if abs(coord % major_step) > 0.01:
            # X方向的次要線
            points.extend([[coord, -size / 2, 0], [coord, size / 2, 0]])
            line_idx = len(points) - 2
            lines.append([line_idx, line_idx + 1])
            colors.append([0.8, 0.8, 0.8])  # 淺灰色

            # Y方向的次要線
            points.extend([[-size / 2, coord, 0], [size / 2, coord, 0]])
            line_idx = len(points) - 2
            lines.append([line_idx, line_idx + 1])
            colors.append([0.8, 0.8, 0.8])  # 淺灰色

    # 建立LineSet
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)

    return line_set


# 添加增強網格到視窗
enhanced_grid = create_enhanced_grid(size=2.0, major_step=0.5, minor_step=0.1)
vis.add_geometry(enhanced_grid)

# 3. 在機械手臂基座處添加小座標軸
base_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.08, origin=[0, 0, 0])
vis.add_geometry(base_frame)

# 4. 添加原點標記
origin_marker = o3d.geometry.TriangleMesh.create_sphere(radius=0.03)
origin_marker.translate([0, 0, 0.02])
origin_marker.paint_uniform_color([1.0, 1.0, 1.0])  # 白色原點
vis.add_geometry(origin_marker)


# ---------- 動畫系統 ----------
class AnimationController:
    def __init__(self):
        # 當前角度和目標角度
        self.current_angles = [0, 0, 0, 0, 0, 0]
        self.target_angles = [0, 0, 0, 0, 0, 0]

        # 預設姿態（可以根據需要調整）
        self.default_angles = [0, 0, 0, 0, 0, 0]

        # 動畫參數
        self.animation_speed = 0.12  # 動畫速度（0.05=很慢, 0.3=很快）
        self.is_animating = False

        # 累積旋轉矩陣（考慮p4~p8初始旋轉-90度）
        self.R_p2_total = np.eye(3)
        self.R_p3_total = np.eye(3)
        self.R_p4_total = o3d.geometry.get_rotation_matrix_from_axis_angle([0, np.deg2rad(-90), 0])  # p4~p8初始-90度
        self.R_p5_total = np.eye(3)
        self.R_p6_total = np.eye(3)
        self.R_p8_total = np.eye(3)

        # 幾何設定
        self.p3_base_local = np.array([-0.03, 0, 0.38])
        self.p3_length_vec = np.array([0, 0, 0.34])
        self.p4_length_vec = np.array([0.04, 0, 0.09])
        self.p5_length_vec = np.array([0, 0, 0.24])
        self.p6_length_vec = np.array([0, 0, 0.07])
        self.p7_length_vec = np.array([0, 0, 0.01])
        self.p8_center_offset = np.array([0, 0, -0.08])

        # 關節5標記（稍後設置）
        self.joint5_marker = None

    def set_marker(self, marker):
        """設置關節5標記的引用"""
        self.joint5_marker = marker

    def smooth_interpolate(self, current, target, speed):
        """平滑插值函數"""
        diff = target - current
        if abs(diff) < 0.1:
            return target
        return current + diff * speed

    def get_joint5_position(self):
        """獲取關節5（p6關節）的當前世界座標位置"""
        # 計算p6關節的旋轉中心（這就是關節5的位置）
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
        """更新關節5標記到當前位置"""
        if self.joint5_marker is None:
            return

        # 獲取當前關節5位置
        current_pos = self.get_joint5_position()

        # 重新創建球體在新位置
        # 先移除舊的標記
        vis.remove_geometry(self.joint5_marker, reset_bounding_box=False)

        # 創建新的標記在正確位置
        self.joint5_marker = o3d.geometry.TriangleMesh.create_sphere(radius=0.08)
        self.joint5_marker.paint_uniform_color([1.0, 0.0, 0.0])  # 紅色
        self.joint5_marker.compute_vertex_normals()
        self.joint5_marker.translate(current_pos)

        # 添加回視窗
        vis.add_geometry(self.joint5_marker, reset_bounding_box=False)

    def get_all_joint_positions(self):
        """獲取所有關節的當前位置"""
        positions = {}

        # 關節1 (p2基座)
        positions['joint1'] = np.array([0, 0, 0])

        # 關節2 (p3基座)
        positions['joint2'] = self.R_p2_total @ self.p3_base_local

        # 關節3 (p4基座)
        positions['joint3'] = self.R_p2_total @ (
                self.p3_base_local +
                self.R_p3_total @ self.p3_length_vec
        )

        # 關節4 (p5基座)
        positions['joint4'] = self.R_p2_total @ (
                self.p3_base_local +
                self.R_p3_total @ (
                        self.p3_length_vec +
                        self.R_p4_total @ self.p4_length_vec
                )
        )

        # 關節5 (p6基座)
        positions['joint5'] = self.get_joint5_position()

        # 關節6 (p8中心)
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
        positions['joint6'] = p8_base + p8_center_offset_world

        return positions

    def update_joint(self, joint_idx, delta_angle):
        """更新指定關節"""
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

        elif joint_idx == 5:  # 關節6 (X軸自轉)
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

        # 更新所有相關的mesh
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

        # 更新關節5標記位置
        self.update_joint5_marker_position()

    def animate(self):
        """執行動畫更新"""
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
        """設定目標角度"""
        self.target_angles[joint_idx] = angle
        self.is_animating = True

    def reset_to_default(self):
        """重設到預設姿態"""
        for i in range(6):
            self.target_angles[i] = self.default_angles[i]
        self.is_animating = True


# 創建全域動畫控制器
anim_controller = AnimationController()

# 全域變數來儲存slider引用，用於重設
sliders = {}

# 全域退出標記
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
        """回復預設姿態"""
        anim_controller.reset_to_default()
        # 同時重設所有slider到預設位置
        for i, (slider_name, default_value) in enumerate([
            ('z_slider', 0),
            ('y_slider', 0),
            ('p4_slider', 0),
            ('p5_z_slider', 0),
            ('p6_y_slider', 0),
            ('p8_z_slider', 0)
        ]):
            if slider_name in sliders:
                sliders[slider_name].set(default_value)

    def exit_program():
        """結束程式"""
        global exit_flag
        exit_flag = True
        root.quit()
        root.destroy()

    root = tk.Tk()
    root.title("手臂關節控制面板 - 流暢動畫")
    root.configure(bg="#FFD9EC")

    # 關節1控制
    tk.Label(root, text="關節1（p2~p8）繞Z軸", bg="#FFD9EC", font=("Arial", 9)).pack()
    z_slider = tk.Scale(root, from_=-165, to=165, orient=tk.HORIZONTAL, length=250, command=on_z_slider)
    z_slider.pack(padx=15, pady=5)
    sliders['z_slider'] = z_slider

    # 關節2控制
    tk.Label(root, text="關節2（p3~p8 跟隨）繞Y軸", bg="#FFD9EC", font=("Arial", 9)).pack()
    y_slider = tk.Scale(root, from_=-125, to=85, orient=tk.HORIZONTAL, length=250, command=on_y_slider)
    y_slider.pack(padx=15, pady=5)
    sliders['y_slider'] = y_slider

    # 關節3控制
    tk.Label(root, text="關節3（p4~p8 跟隨）繞Y軸", bg="#FFD9EC", font=("Arial", 9)).pack()
    p4_slider = tk.Scale(root, from_=-55, to=180, orient=tk.HORIZONTAL, length=250, command=on_p4_y_slider)
    p4_slider.set(0)
    p4_slider.pack(padx=15, pady=5)
    sliders['p4_slider'] = p4_slider

    # 關節4控制
    tk.Label(root, text="關節4（p5~p8）繞P4的Z軸", bg="#FFD9EC", font=("Arial", 9)).pack()
    p5_z_slider = tk.Scale(root, from_=-190, to=190, orient=tk.HORIZONTAL, length=250, command=on_p5_z_slider)
    p5_z_slider.pack(padx=15, pady=5)
    sliders['p5_z_slider'] = p5_z_slider

    # 關節5控制
    tk.Label(root, text="關節5（p6~p8）繞P5的Y軸", bg="#FFD9EC", font=("Arial", 9)).pack()
    p6_y_slider = tk.Scale(root, from_=-25, to=205, orient=tk.HORIZONTAL, length=250, command=on_p6_y_slider)
    p6_y_slider.pack(padx=15, pady=5)
    sliders['p6_y_slider'] = p6_y_slider

    # 關節6控制
    tk.Label(root, text="關節6（p8）以自身為原點X軸自轉", bg="#FFD9EC", font=("Arial", 9)).pack()
    p8_z_slider = tk.Scale(root, from_=-360, to=360, orient=tk.HORIZONTAL, length=250, command=on_p8_z_slider,
                           resolution=1)
    p8_z_slider.pack(padx=15, pady=5)
    sliders['p8_z_slider'] = p8_z_slider

    # 動畫速度控制
    tk.Label(root, text="動畫速度", bg="#FFD9EC", font=("Arial", 9)).pack()

    def on_speed_change(value):
        anim_controller.animation_speed = float(value) / 100.0

    speed_slider = tk.Scale(root, from_=5, to=30, orient=tk.HORIZONTAL, length=250, command=on_speed_change)
    speed_slider.set(12)  # 預設值12%
    speed_slider.pack(padx=15, pady=5)

    # ===== 位置顯示區域 =====
    position_frame = tk.Frame(root, bg="#E6F3FF", relief=tk.RIDGE, bd=2)
    position_frame.pack(pady=5, padx=15, fill=tk.BOTH)

    tk.Label(position_frame, text="關節5 (P6) 當前位置 - 紅點標記",
             font=("Arial", 10, "bold"), bg="#E6F3FF", fg="#FF0000").pack(pady=3)

    # X座標顯示
    x_frame = tk.Frame(position_frame, bg="#E6F3FF")
    x_frame.pack(pady=1)
    tk.Label(x_frame, text="X: ", font=("Arial", 9), bg="#E6F3FF").pack(side=tk.LEFT)
    x_label = tk.Label(x_frame, text="0.000", font=("Arial", 9, "bold"),
                       bg="#FFFFFF", width=10, relief=tk.SUNKEN)
    x_label.pack(side=tk.LEFT, padx=3)

    # Y座標顯示
    y_frame = tk.Frame(position_frame, bg="#E6F3FF")
    y_frame.pack(pady=1)
    tk.Label(y_frame, text="Y: ", font=("Arial", 9), bg="#E6F3FF").pack(side=tk.LEFT)
    y_label = tk.Label(y_frame, text="0.000", font=("Arial", 9, "bold"),
                       bg="#FFFFFF", width=10, relief=tk.SUNKEN)
    y_label.pack(side=tk.LEFT, padx=3)

    # Z座標顯示
    z_frame = tk.Frame(position_frame, bg="#E6F3FF")
    z_frame.pack(pady=1)
    tk.Label(z_frame, text="Z: ", font=("Arial", 9), bg="#E6F3FF").pack(side=tk.LEFT)
    z_label = tk.Label(z_frame, text="0.000", font=("Arial", 9, "bold"),
                       bg="#FFFFFF", width=10, relief=tk.SUNKEN)
    z_label.pack(side=tk.LEFT, padx=3)

    # 更新位置顯示的函數
    def update_position_display():
        """定期更新關節5的位置顯示"""
        try:
            pos = anim_controller.get_joint5_position()
            x_label.config(text=f"{pos[0]:.4f}")
            y_label.config(text=f"{pos[1]:.4f}")
            z_label.config(text=f"{pos[2]:.4f}")
            root.after(50, update_position_display)  # 每50ms更新一次
        except:
            pass

    # 啟動位置更新
    update_position_display()

    # 按鈕框架
    button_frame = tk.Frame(root, bg="#FFD9EC")
    button_frame.pack(pady=8)

    # 回復預設姿態按鈕
    reset_button = tk.Button(button_frame, text="回復預設姿態", command=reset_pose,
                             font=("Arial", 9, "bold"), bg="#B8E6B8", fg="#000000",
                             padx=10, pady=5, relief=tk.RAISED, bd=2)
    reset_button.pack(side=tk.LEFT, padx=5)

    # 打印所有關節位置的按鈕
    def print_all_positions():
        """在終端機打印所有關節位置"""
        positions = anim_controller.get_all_joint_positions()
        print("\n" + "=" * 50)
        print("當前所有關節位置：")
        print("=" * 50)
        for joint_name, pos in positions.items():
            print(f"{joint_name}: X={pos[0]:.4f}, Y={pos[1]:.4f}, Z={pos[2]:.4f}")
        print("=" * 50 + "\n")

    print_button = tk.Button(button_frame, text="打印所有關節位置",
                             command=print_all_positions,
                             font=("Arial", 9, "bold"), bg="#FFE6CC", fg="#000000",
                             padx=10, pady=5, relief=tk.RAISED, bd=2)
    print_button.pack(side=tk.LEFT, padx=5)

    # 結束程式按鈕
    exit_button = tk.Button(button_frame, text="結束程式", command=exit_program,
                            font=("Arial", 9, "bold"), bg="#FFB6C1", fg="#000000",
                            padx=10, pady=5, relief=tk.RAISED, bd=2)
    exit_button.pack(side=tk.LEFT, padx=5)

    # 設置視窗關閉事件
    root.protocol("WM_DELETE_WINDOW", exit_program)

    root.mainloop()


# ---------- 主循環 ----------
def open3d_loop():
    global exit_flag

    # 計算關節5的初始位置（考慮初始旋轉）
    initial_joint5_pos = anim_controller.get_joint5_position()

    # 創建關節5標記在正確的初始位置
    joint5_marker = o3d.geometry.TriangleMesh.create_sphere(radius=0.08)
    joint5_marker.paint_uniform_color([1.0, 0.0, 0.0])  # 紅色
    joint5_marker.compute_vertex_normals()
    joint5_marker.translate(initial_joint5_pos)
    vis.add_geometry(joint5_marker)

    # 將標記設置到控制器
    anim_controller.set_marker(joint5_marker)

    # 啟動GUI線程
    threading.Thread(target=start_gui, daemon=True).start()

    last_time = time.time()

    try:
        while True:
            # 檢查退出標記
            if exit_flag:
                print("正在安全關閉程式...")
                break

            current_time = time.time()
            # 控制動畫幀率（約60FPS）
            if current_time - last_time > 1.0 / 60.0:
                anim_controller.animate()
                last_time = current_time

            vis.poll_events()
            vis.update_renderer()

    except Exception as e:
        print(f"程式執行中發生錯誤：{e}")

    finally:
        # 安全關閉Open3D視窗
        try:
            vis.destroy_window()
        except:
            pass
        print("程式已安全關閉")
        sys.exit(0)


open3d_loop()