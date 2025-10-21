'''
20250701
關節123完成
包含拉桿控制
修改：讓p4~p8模組起始位置旋轉90度
'''


import open3d as o3d
import numpy as np
import tkinter as tk
import threading


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

node6_group = MeshNode(meshes[5], "p6.obj")
node6_group.add_child(MeshNode(meshes[6], "p7.obj"))
node6_group.add_child(MeshNode(meshes[7], "p8.obj"))

node4_group = node4
node4_group.add_child(node5)
node4_group.add_child(node6_group)

node3.add_child(node4_group)
node2.add_child(node3)

root_nodes = [MeshNode(meshes[0], "p1.obj"), node2]

# ---------- 初始讓 p6~p8 旋轉 90 度 ----------
p5_top_center = [0.01, 0, 1.05]
R = o3d.geometry.get_rotation_matrix_from_axis_angle([0, np.deg2rad(-90), 0])
T = np.eye(4)
T[:3, :3] = R
node6_group.transform(T, p5_top_center)

# ---------- 初始讓 p4~p8 旋轉 90 度 ----------
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
vis.create_window("機械手臂3d圖", width=700, height=720)
for root in root_nodes:
    for m in root.get_all_meshes():
        vis.add_geometry(m)


# ---------- GUI 控制 ----------
def start_gui():
    prev_z_angle = [0]
    prev_y_angle = [0]
    prev_p4_angle = [0]

    R_p2_total = np.eye(3)
    R_p3_total = np.eye(3)

    # 幾何設定
    p3_base_local = np.array([-0.03, 0, 0.38])  # P3 起點（相對於 p2）
    p3_length_vec = np.array([0, 0, 0.34])  # P3 模組長度（朝向 P4）

    def on_z_slider(value):
        new_angle = int(value)
        delta = new_angle - prev_z_angle[0]
        prev_z_angle[0] = new_angle

        delta_rad = np.deg2rad(delta)
        R = o3d.geometry.get_rotation_matrix_from_axis_angle([0, 0, delta_rad])
        R_p2_total[:] = R @ R_p2_total

        T = np.eye(4)
        T[:3, :3] = R
        node2.transform(T, center=[0, 0, 0])

        for m in node2.get_all_meshes():
            vis.update_geometry(m)

    def on_y_slider(value):
        new_angle = int(value)
        delta = new_angle - prev_y_angle[0]
        prev_y_angle[0] = new_angle

        delta_rad = np.deg2rad(delta)

        #  修正：在P2局部坐標系中累積P3的旋轉
        R_local = o3d.geometry.get_rotation_matrix_from_axis_angle([0, delta_rad, 0])
        R_p3_total[:] = R_local @ R_p3_total

        # 將局部旋轉轉換到世界坐標系進行實際旋轉
        local_y = R_p2_total @ np.array([0, 1, 0])
        R_world = o3d.geometry.get_rotation_matrix_from_axis_angle(local_y * delta_rad)

        # 旋轉中心為 p3 起點（根據 p2 姿態）
        center = R_p2_total @ p3_base_local

        T = np.eye(4)
        T[:3, :3] = R_world
        node3.transform(T, center)

        for m in node3.get_all_meshes():
            vis.update_geometry(m)

    def on_p4_y_slider(value):
        new_angle = int(value)
        delta = new_angle - prev_p4_angle[0]
        prev_p4_angle[0] = new_angle

        delta_rad = np.deg2rad(delta)

        # 使用P2坐標系的Y軸作為旋轉軸
        local_y = R_p2_total @ np.array([0, 1, 0])

        #  修正：P4軸心跟隨P3末端移動
        # 旋轉中心 = P3起點 + P3旋轉後的長度向量（都在P2坐標系中計算，再轉到世界坐標系）
        center = R_p2_total @ (p3_base_local + R_p3_total @ p3_length_vec)

        R = o3d.geometry.get_rotation_matrix_from_axis_angle(local_y * delta_rad)
        T = np.eye(4)
        T[:3, :3] = R
        node4_group.transform(T, center)

        for m in node4_group.get_all_meshes():
            vis.update_geometry(m)

    root = tk.Tk()
    root.title("手臂關節控制面板")
    root.configure(bg="#FFD9EC")  # 背景色


    tk.Label(root, text="關節1（p2~p8）").pack()
    z_slider = tk.Scale(root, from_=-180, to=180, orient=tk.HORIZONTAL, length=300, command=on_z_slider)
    z_slider.pack(padx=20, pady=10)

    tk.Label(root, text="關節2（p3~p8 跟隨）").pack()
    y_slider = tk.Scale(root, from_=-120, to=90, orient=tk.HORIZONTAL, length=300, command=on_y_slider)
    y_slider.pack(padx=20, pady=10)

    tk.Label(root, text="關節3（p4~p8 跟隨）").pack()
    p4_slider = tk.Scale(root, from_=-120, to=120, orient=tk.HORIZONTAL, length=300, command=on_p4_y_slider)
    p4_slider.pack(padx=20, pady=10)

    root.mainloop()


# ---------- 非阻塞刷新 ----------
def open3d_loop():
    while True:
        vis.poll_events()
        vis.update_renderer()


threading.Thread(target=start_gui, daemon=True).start()
open3d_loop()