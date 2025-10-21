'''
20251015 - 圓弧軌跡運動 (GUI版)
RA605-710-GC 六軸機械手臂控制系統
功能：滑桿設定目標點，從當前位置以圓弧移動到目標點（弧度=2.0）
速度範圍：1x (5ms延遲) - 1000x (0延遲)
'''

import open3d as o3d
import numpy as np
import tkinter as tk
import threading
import time
import sys


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
paths = [rf"/home/yahboom/Desktop/Obj/p{i}.obj" for i in range(1, 9)]
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

# ---------- Open3D 視窗 ----------
vis = o3d.visualization.Visualizer()
vis.create_window("Robot Arm", width=500, height=500)

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
        self.animation_speed = 0.12  # 固定的關節動畫速度
        self.is_animating = False

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
        self.trajectory_line = None
        self.trajectory_points = None
        self.is_following_trajectory = False
        self.trajectory_index = 0
        self.trajectory_delay = 0.005  # 點與點之間的延遲時間（秒），預設5ms (速度1x)，最大速度時為0
        self.last_trajectory_time = 0  # 上次移動到軌跡點的時間
        self.skip_unreachable_points = True  # 是否跳過無法到達的點
        self.skipped_points_count = 0  # 跳過的點數量

    def set_marker(self, marker):
        self.joint5_marker = marker

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
        if self.joint5_marker is None:
            return
        pos = self.get_joint5_position()
        vis.remove_geometry(self.joint5_marker, reset_bounding_box=False)
        self.joint5_marker = o3d.geometry.TriangleMesh.create_sphere(radius=0.08)
        self.joint5_marker.paint_uniform_color([1.0, 0.0, 0.0])
        self.joint5_marker.compute_vertex_normals()
        self.joint5_marker.translate(pos)
        vis.add_geometry(self.joint5_marker, reset_bounding_box=False)

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

    def animate(self):
        # 處理軌跡跟踪（不使用動畫）
        if self.is_following_trajectory:
            current_time = time.time()
            if current_time - self.last_trajectory_time >= self.trajectory_delay:
                self.last_trajectory_time = current_time
                self.move_next_point()
            return

        # 處理手動關節控制（使用動畫）
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
        """直接移動到目標位置，不使用動畫"""
        res = self.ik(x, y, z)
        if res[0] is None:
            return False, res[1]
        angles = res[0]

        # 直接設置角度，不使用動畫
        for i in range(4):
            delta = angles[i] - self.current_angles[i]
            self.current_angles[i] = angles[i]
            self.target_angles[i] = angles[i]
            if abs(delta) > 0.0001:
                self.update_joint(i, delta)

        return True, "ok"

    def show_trajectory(self, points):
        if self.trajectory_line:
            vis.remove_geometry(self.trajectory_line, reset_bounding_box=False)
        pts = o3d.utility.Vector3dVector(points)
        lns = [[i, i + 1] for i in range(len(points) - 1)]
        ls = o3d.geometry.LineSet()
        ls.points = pts
        ls.lines = o3d.utility.Vector2iVector(lns)
        ls.paint_uniform_color([0, 1, 0])
        self.trajectory_line = ls
        vis.add_geometry(ls, reset_bounding_box=False)

    def start_trajectory(self, points):
        self.trajectory_points = points
        self.trajectory_index = 0
        self.skipped_points_count = 0
        self.is_following_trajectory = True
        self.last_trajectory_time = time.time()
        self.show_trajectory(points)
        self.move_next_point()

    def move_next_point(self):
        if not self.is_following_trajectory or self.trajectory_points is None:
            return
        if self.trajectory_index >= len(self.trajectory_points):
            if self.skipped_points_count > 0:
                print(
                    f"\n✓ Trajectory Complete! Points: {len(self.trajectory_points)}, Skipped: {self.skipped_points_count}")
            else:
                print(f"\n✓ Trajectory Complete! Total points: {len(self.trajectory_points)}")
            self.is_following_trajectory = False
            self.skipped_points_count = 0
            return

        pt = self.trajectory_points[self.trajectory_index]
        ok, msg = self.move_to_instant(pt[0], pt[1], pt[2])  # 使用瞬間移動

        if ok:
            self.trajectory_index += 1
            if self.trajectory_index % 100 == 0:
                delay_ms = self.trajectory_delay * 1000
                if delay_ms < 0.001:
                    delay_str = "No delay"
                else:
                    delay_str = f"{delay_ms:.2f}ms"
                print(f"Progress: {self.trajectory_index}/{len(self.trajectory_points)} (Delay: {delay_str})")
        else:
            if self.skip_unreachable_points:
                if self.skipped_points_count > len(self.trajectory_points) * 0.2:
                    print(
                        f"✗ Too many unreachable points ({self.skipped_points_count}/{len(self.trajectory_points)}). Stopping trajectory.")
                    self.is_following_trajectory = False
                    self.skipped_points_count = 0
                    return

                self.skipped_points_count += 1
                first_skip_index = self.trajectory_index
                self.trajectory_index += 1

                print(f"⚠ Skipping unreachable point {first_skip_index}/{len(self.trajectory_points)}: {msg}")
                print(f"  Target: ({pt[0]:.3f}, {pt[1]:.3f}, {pt[2]:.3f})")

                consecutive_skips = 1
                found_reachable = False

                while self.trajectory_index < len(self.trajectory_points):
                    if self.skipped_points_count > len(self.trajectory_points) * 0.2:
                        print(
                            f"✗ Too many unreachable points ({self.skipped_points_count}/{len(self.trajectory_points)}). Stopping.")
                        self.is_following_trajectory = False
                        self.skipped_points_count = 0
                        return

                    pt_next = self.trajectory_points[self.trajectory_index]
                    ok_next, msg_next = self.move_to_instant(pt_next[0], pt_next[1], pt_next[2])  # 使用瞬間移動

                    if ok_next:
                        found_reachable = True
                        self.trajectory_index += 1
                        print(
                            f"✓ Found reachable point after skipping {consecutive_skips} points (now at {self.trajectory_index}/{len(self.trajectory_points)})")
                        break
                    else:
                        self.skipped_points_count += 1
                        self.trajectory_index += 1
                        consecutive_skips += 1

                        if consecutive_skips % 10 == 0:
                            print(
                                f"⚠ Skipped {consecutive_skips} consecutive points... (now at {self.trajectory_index}/{len(self.trajectory_points)})")

                if not found_reachable:
                    print(f"⚠ Reached end of trajectory. Total skipped: {self.skipped_points_count}")
                    self.is_following_trajectory = False
                    self.skipped_points_count = 0

            else:
                print(f"✗ Failed at point {self.trajectory_index}/{len(self.trajectory_points)}: {msg}")
                print(f"  Target: ({pt[0]:.3f}, {pt[1]:.3f}, {pt[2]:.3f})")
                current_pos = self.get_joint5_position()
                print(f"  Current: ({current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f})")
                self.is_following_trajectory = False
                self.skipped_points_count = 0

    def set_trajectory_speed(self, speed):
        """設定軌跡速度（調整延遲時間）
        speed: 1-1000，數字越大速度越快
        延遲時間範圍: 5ms(speed=1) 到 0ms(speed=1000)
        """
        # 線性插值：從 5ms 降到 0ms
        self.trajectory_delay = 0.005 * (1000 - speed) / 999

    def stop_trajectory(self):
        self.is_following_trajectory = False
        self.skipped_points_count = 0


ctrl = AnimationController()
exit_flag = False


# ---------- GUI ----------
def start_gui():
    def update_target_label():
        try:
            pos = ctrl.get_joint5_position()
            cur_x.config(text=f"{pos[0]:.3f}")
            cur_y.config(text=f"{pos[1]:.3f}")
            cur_z.config(text=f"{pos[2]:.3f}")
            root.after(100, update_target_label)
        except:
            pass

    def move_arc():
        try:
            A = ctrl.get_joint5_position()
            B = [tx.get(), ty.get(), tz.get()]

            status.config(text="Calculating...")
            root.update()

            arc, _, _, C = compute_arc_with_auto_center(A, B, 2.0, 1000)
            length = np.sum(np.sqrt(np.sum(np.diff(arc, axis=0) ** 2, axis=1)))

            current_speed = int(traj_speed.get())
            delay_ms = (0.005 * (1000 - current_speed) / 999) * 1000

            if delay_ms < 0.001:
                delay_str = "No delay"
            elif delay_ms >= 1:
                delay_str = f"{delay_ms:.2f}ms"
            else:
                delay_str = f"{delay_ms:.3f}ms"

            status.config(text=f"Arc: {length:.3f}m\nRunning... ({delay_str}/point)")
            root.update()

            ctrl.start_trajectory(arc)
        except Exception as e:
            status.config(text=f"Error: {e}")

    def stop():
        ctrl.stop_trajectory()
        status.config(text="Stopped")

    def reset_trajectory():
        """重置軌跡，允許重新開始"""
        ctrl.stop_trajectory()
        ctrl.trajectory_index = 0
        ctrl.skipped_points_count = 0
        status.config(text="Ready")

    def quit_prog():
        global exit_flag
        exit_flag = True
        root.quit()
        root.destroy()

    root = tk.Tk()
    root.title("Robot Control")

    # Left: Joints
    lf = tk.Frame(root)
    lf.pack(side=tk.LEFT, padx=5, pady=5)

    tk.Label(lf, text="Joint Control").pack()

    jcfg = [("J1", -165, 165), ("J2", -125, 85), ("J3", -55, 185),
            ("J4", -190, 190), ("J5", -25, 205), ("J6", -360, 360)]

    for i, (name, mn, mx) in enumerate(jcfg):
        f = tk.Frame(lf)
        f.pack(pady=1)
        tk.Label(f, text=name, width=3).pack(side=tk.LEFT)
        s = tk.Scale(f, from_=mn, to=mx, orient=tk.HORIZONTAL, length=180,
                     command=lambda v, idx=i: ctrl.set_target(idx, float(v)))
        s.set(0)
        s.pack(side=tk.LEFT)

    tk.Button(lf, text="Quit", command=quit_prog, width=15).pack(pady=10)

    # Right: Target
    rf = tk.Frame(root)
    rf.pack(side=tk.RIGHT, padx=5, pady=5)

    # Current
    cf = tk.Frame(rf, relief=tk.RIDGE, bd=1)
    cf.pack(pady=5, fill=tk.X)
    tk.Label(cf, text="Current J5").pack()

    cur_x = tk.Label(cf, text="0.000", width=8, relief=tk.SUNKEN)
    cur_x.pack()
    cur_y = tk.Label(cf, text="0.000", width=8, relief=tk.SUNKEN)
    cur_y.pack()
    cur_z = tk.Label(cf, text="0.000", width=8, relief=tk.SUNKEN)
    cur_z.pack()

    update_target_label()

    # Target
    tf = tk.Frame(rf, relief=tk.RIDGE, bd=1)
    tf.pack(pady=5, fill=tk.BOTH, expand=True)
    tk.Label(tf, text="Target Position").pack()
    tk.Label(tf, text="(Arc Radius=2.0)", font=("Arial", 8)).pack()

    tx = tk.Scale(tf, from_=-0.7, to=0.7, resolution=0.01, orient=tk.HORIZONTAL,
                  length=180, label="X")
    tx.set(0.3)
    tx.pack()

    ty = tk.Scale(tf, from_=-0.7, to=0.7, resolution=0.01, orient=tk.HORIZONTAL,
                  length=180, label="Y")
    ty.set(0.0)
    ty.pack()

    tz = tk.Scale(tf, from_=0.0, to=1.0, resolution=0.01, orient=tk.HORIZONTAL,
                  length=180, label="Z")
    tz.set(0.8)
    tz.pack()

    # 軌跡速度滑桿
    tk.Label(tf, text="").pack(pady=2)

    def update_traj_speed(v):
        speed = float(v)
        ctrl.set_trajectory_speed(speed)
        delay_ms = (0.005 * (1000 - speed) / 999) * 1000

        if delay_ms < 0.001:
            delay_str = "No delay"
        elif delay_ms >= 1:
            delay_str = f"{delay_ms:.2f}ms"
        else:
            delay_str = f"{delay_ms:.3f}ms"

        speed_label.config(text=f"Speed: {int(speed)}x ({delay_str})")

        if ctrl.is_following_trajectory:
            current_status = status.cget("text")
            if "Running" in current_status:
                lines = current_status.split('\n')
                if len(lines) >= 1:
                    status.config(text=f"{lines[0]}\nRunning... ({delay_str}/point)")

    speed_frame = tk.Frame(tf)
    speed_frame.pack(fill=tk.X, padx=5)

    speed_label = tk.Label(speed_frame, text="Speed: 100x (4.50ms)", width=30)
    speed_label.pack()

    traj_speed = tk.Scale(speed_frame, from_=1, to=1000, resolution=1,
                          orient=tk.HORIZONTAL, length=180,
                          command=update_traj_speed, showvalue=False)
    traj_speed.set(100)
    traj_speed.pack()

    update_traj_speed(100)

    tk.Label(speed_frame, text="(Range: 1-1000x, Max=No delay)", font=("Arial", 7)).pack()

    # 跳過無法到達點的選項
    skip_frame = tk.Frame(tf)
    skip_frame.pack(fill=tk.X, padx=5, pady=5)

    skip_var = tk.BooleanVar(value=True)

    def toggle_skip():
        ctrl.skip_unreachable_points = skip_var.get()
        if skip_var.get():
            skip_status.config(text="✓ Auto-skip unreachable points")
        else:
            skip_status.config(text="✗ Stop on unreachable points")

    skip_check = tk.Checkbutton(skip_frame, text="Auto-skip unreachable",
                                variable=skip_var, command=toggle_skip)
    skip_check.pack()

    skip_status = tk.Label(skip_frame, text="✓ Auto-skip unreachable points",
                           font=("Arial", 7), fg="green")
    skip_status.pack()

    status = tk.Label(tf, text="Ready", wraplength=180)
    status.pack(pady=5)

    bf = tk.Frame(tf)
    bf.pack(pady=5)
    tk.Button(bf, text="Move (Arc)", command=move_arc, width=10).pack(side=tk.LEFT, padx=2)
    tk.Button(bf, text="Stop", command=stop, width=8).pack(side=tk.LEFT, padx=2)
    tk.Button(bf, text="Reset", command=reset_trajectory, width=8).pack(side=tk.LEFT, padx=2)

    root.protocol("WM_DELETE_WINDOW", quit_prog)
    root.mainloop()


# ---------- Main ----------
def main_loop():
    global exit_flag
    pos = ctrl.get_joint5_position()
    marker = o3d.geometry.TriangleMesh.create_sphere(radius=0.08)
    marker.paint_uniform_color([1, 0, 0])
    marker.compute_vertex_normals()
    marker.translate(pos)
    vis.add_geometry(marker)
    ctrl.set_marker(marker)

    threading.Thread(target=start_gui, daemon=True).start()
    last = time.time()

    try:
        while True:
            if exit_flag:
                break
            now = time.time()
            if now - last > 1 / 60:
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


main_loop()