'''
RA605-710-GC
PyBullet + Tkinter 版 (Linux 修正版)
- Tkinter GUI 在主執行緒
- PyBullet 模擬迴圈在背景執行緒
'''

import numpy as np
import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time
import sys

import pybullet as p
import pybullet_data

# ========= 幾何與圓弧函式 =========

def compute_arc_with_auto_center(A, B, radius_scale=2.0, num_points=1000):
    A = np.array(A, dtype=float)
    B = np.array(B, dtype=float)
    AB = B - A
    M = (A + B) / 2.0

    z_ref = np.array([0, 0, 1])
    if np.allclose(np.cross(AB, z_ref), 0):
        z_ref = np.array([1, 0, 0])

    dir_vec = np.cross(AB, z_ref)
    dir_vec = dir_vec / np.linalg.norm(dir_vec)

    half_len = np.linalg.norm(AB) / 2.0
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


S1 = 0.030
S2 = 0.040
L1 = 0.375
L2 = 0.340
L3 = 0.338
L4 = 0.0865

print("\n" + "=" * 60)
print("RA605-710-GC (PyBullet 版)")
print("=" * 60)
print(f"S1={S1 * 1000:.1f} S2={S2 * 1000:.1f} L1={L1 * 1000:.1f}")
print(f"L2={L2 * 1000:.1f} L3={L3 * 1000:.1f} L4={L4 * 1000:.1f}")
print("=" * 60 + "\n")

exit_flag = False


# ========= 控制器 =========

class AnimationController:
    def __init__(self):
        self.S1, self.S2 = S1, S2
        self.L1, self.L2, self.L3, self.L4 = L1, L2, L3, L4

        self.current_angles = [0, 0, 0, 0, 0, 0]
        self.target_angles = [0, 0, 0, 0, 0, 0]
        self.animation_speed = 0.12
        self.is_animating = False
        self.joint_scales = []

        self.p3_base_local = np.array([-self.S1, 0, self.L1])
        self.p3_length_vec = np.array([0, 0, self.L2])
        self.p4_length_vec = np.array([self.S2, 0, 0])
        self.p5_length_vec = np.array([0, 0, self.L3])

        self.joint5_marker = None
        self.trajectory_line_ids = []

        self.trajectory_points = None
        self.is_following_trajectory = False
        self.trajectory_index = 0
        self.trajectory_delay = 0.005
        self.trajectory_step = 1
        self.last_trajectory_time = 0
        self.skip_unreachable_points = True
        self.skipped_points_count = 0

        self.status_callback = None

    # ---- PyBullet ----
    def set_marker(self, marker_body_id):
        self.joint5_marker = marker_body_id
        self.update_joint5_marker()

    def set_status_callback(self, cb):
        self.status_callback = cb

    def log_status(self, msg):
        if self.status_callback:
            self.status_callback(msg)

    # ---- GUI joint scale ----
    def update_scales(self):
        if not self.joint_scales:
            return
        for i, scale in enumerate(self.joint_scales):
            if scale:
                scale.config(command=lambda v: None)
                scale.set(self.current_angles[i])
                scale.config(command=self._make_scale_command(i))

    def _make_scale_command(self, idx):
        return lambda v: self.set_target(idx, float(v))

    # ---- FK / J5 ----
    @staticmethod
    def _rotz(t):
        c, s = np.cos(t), np.sin(t)
        return np.array([[c, -s, 0],
                         [s,  c, 0],
                         [0,  0, 1]])

    @staticmethod
    def _roty(t):
        c, s = np.cos(t), np.sin(t)
        return np.array([[ c, 0,  s],
                         [ 0, 1,  0],
                         [-s, 0,  c]])

    def fk(self, ang_deg):
        a, b, c, d = map(np.deg2rad, ang_deg[:4])
        R2 = self._rotz(a)
        R3 = self._roty(b)
        R4b = self._roty(np.deg2rad(-90))
        R4 = self._roty(c) @ R4b
        R5 = self._rotz(d)

        pos = R2 @ (self.p3_base_local +
                    R3 @ (self.p3_length_vec +
                          R4 @ (self.p4_length_vec +
                                R5 @ self.p5_length_vec)))
        return pos

    def get_joint5_position(self):
        return self fk(self.current_angles)

    def update_joint5_marker(self):
        if self.joint5_marker is None:
            return
        pos = self.get_joint5_position()
        p.resetBasePositionAndOrientation(self.joint5_marker,
                                          posObj=pos.tolist(),
                                          ornObj=[0, 0, 0, 1])

    # ---- Joint / 動畫 ----
    def update_joint(self, joint_idx, delta_angle):
        if abs(delta_angle) < 1e-4:
            return
        self.update_joint5_marker()
        self.update_scales()

    def animate(self):
        # 軌跡模式
        if self.is_following_trajectory:
            now = time.time()
            if now - self.last_trajectory_time >= self.trajectory_delay:
                self.last_trajectory_time = now
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

    def set_target(self, idx, angle):
        self.target_angles[idx] = angle
        self.is_animating = True

    # ---- IK / 點到點 ----
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

            angles = [np.rad2deg(theta1), np.rad2deg(theta2),
                      np.rad2deg(theta3), 0]

            limits = [(-165, 165), (-125, 85), (-55, 185), (-190, 190)]
            for a, (mn, mx) in zip(angles, limits):
                if a < mn or a > mx:
                    return None, "limit"
            return angles, "ok"
        except:
            return None, "err"

    def move_to_instant(self, x, y, z):
        res = self.ik(x, y, z)
        if res[0] is None:
            return False, res[1]
        angles = res[0]
        for i in range(4):
            delta = angles[i] - self.current_angles[i]
            self.current_angles[i] = angles[i]
            self.target_angles[i] = angles[i]
            if abs(delta) > 1e-4:
                self.update_joint(i, delta)
        return True, "ok"

    # ---- 軌跡顯示 ----
    def show_trajectory(self, pts):
        self.clear_trajectory_markers()
        self.trajectory_line_ids = []
        pts = np.asarray(pts)
        for i in range(len(pts) - 1):
            lid = p.addUserDebugLine(pts[i].tolist(), pts[i+1].tolist(),
                                     lineColorRGB=[0, 1, 0], lineWidth=2,
                                     lifeTime=0)
            self.trajectory_line_ids.append(lid)

    def clear_trajectory_markers(self):
        for lid in self.trajectory_line_ids:
            p.removeUserDebugItem(lid)
        self.trajectory_line_ids = []

    # ---- 軌跡執行 ----
    def start_trajectory(self, points):
        self.trajectory_points = points
        self.trajectory_index = 0
        self.skipped_points_count = 0
        self.is_following_trajectory = True
        self.last_trajectory_time = time.time()
        self.show_trajectory(points)
        self.move_next_point()

    def move_next_point(self):
        if (not self.is_following_trajectory or
                self.trajectory_points is None):
            return

        if self.trajectory_index >= len(self.trajectory_points):
            self.is_following_trajectory = False
            self.skipped_points_count = 0
            self.clear_trajectory_markers()
            return

        pt = self.trajectory_points[self.trajectory_index]
        ok, msg = self.move_to_instant(pt[0], pt[1], pt[2])

        if ok:
            self.trajectory_index += self.trajectory_step
        else:
            if self.skip_unreachable_points:
                if self.skipped_points_count > len(self.trajectory_points) * 0.2:
                    self.is_following_trajectory = False
                    self.skipped_points_count = 0
                    self.clear_trajectory_markers()
                    return
                self.skipped_points_count += 1
                self.trajectory_index += self.trajectory_step
            else:
                self.is_following_trajectory = False
                self.skipped_points_count = 0
                self.clear_trajectory_markers()

    # ---- 其他 ----
    def set_trajectory_speed(self, speed):
        self.trajectory_delay = 0.005 * (1000 - speed) / 999.0

    def set_animation_speed(self, speed):
        self.animation_speed = speed / 100.0

    def set_trajectory_step(self, step):
        self.trajectory_step = max(1, int(step))

    def stop_trajectory(self):
        self.is_following_trajectory = False
        self.skipped_points_count = 0
        self.clear_trajectory_markers()

    def reset_pose(self):
        if self.is_following_trajectory:
            self.stop_trajectory()
        for i in range(6):
            self.current_angles[i] = 0
            self.target_angles[i] = 0
        self.update_joint5_marker()
        for i, s in enumerate(self.joint_scales):
            if s:
                s.config(command=lambda v: None)
                s.set(0)
                s.config(command=self._make_scale_command(i))


ctrl = AnimationController()


# ========= Tkinter GUI =========

class RobotControlGUI:
    def __init__(self, root, controller):
        self.root = root
        self.ctrl = controller
        self.root.title("RA605-710-GC 六軸機械手臂 (PyBullet)")
        self.root.geometry("850x750+0+50")

        self.setup_ui()
        self.ctrl.set_status_callback(self.append_status)
        self.update_display()

    # ---- Joint 設定 ----
    def set_joint_angle(self, joint_idx, entry):
        try:
            angle = float(entry.get())
            joint_limits = [(-165, 165), (-125, 85), (-55, 185)]
            mn, mx = joint_limits[joint_idx]
            if angle < mn or angle > mx:
                messagebox.showwarning(
                    "警告", f"角度超出範圍!\n允許範圍: [{mn}° ~ {mx}°]"
                )
                return
            self.ctrl.set_target(joint_idx, angle)
            entry.delete(0, tk.END)
        except ValueError:
            messagebox.showerror("錯誤", "請輸入有效的數字!")

    def setup_ui(self):
        main = ttk.Frame(self.root, padding="10")
        main.grid(row=0, column=0, sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main.columnconfigure(1, weight=1)

        self.create_joint_control(main)
        self.create_position_control(main)
        self.create_system_control(main)

    def create_joint_control(self, parent):
        frame = ttk.LabelFrame(parent, text="關節控制", padding="10")
        frame.grid(row=0, column=0, padx=5, pady=5, sticky="ns")

        joint_config = [
            ("關節 1 (Base)", -165, 165, True),
            ("關節 2 (Shoulder)", -125, 85, True),
            ("關節 3 (Elbow)", -55, 185, True),
            ("關節 4 (Wrist Z)", -190, 190, False),
            ("關節 5 (Wrist Y)", -25, 205, False),
            ("關節 6 (Wrist X)", -360, 360, False)
        ]

        self.joint_labels = []
        self.joint_entries = []
        joint_scales_ref = []

        for i, (name, mn, mx, has_input) in enumerate(joint_config):
            header = ttk.Frame(frame)
            header.grid(row=i*2, column=0, sticky="w", pady=(10 if i>0 else 0, 5))
            ttk.Label(header, text=name, font=('Arial',10,'bold')).pack(side="left")
            val_lbl = ttk.Label(header, text="0.0°",
                                foreground="blue", font=('Arial',10,'bold'))
            val_lbl.pack(side="right")
            self.joint_labels.append(val_lbl)

            ctrl_frame = ttk.Frame(frame)
            ctrl_frame.grid(row=i*2+1, column=0, sticky="ew", padx=5)

            if has_input:
                inner = ttk.Frame(ctrl_frame)
                inner.pack(fill="x")
                ttk.Label(inner,text="輸入角度:").pack(side="left", padx=(0,5))
                entry = ttk.Entry(inner, width=10)
                entry.pack(side="left", padx=5)
                self.joint_entries.append(entry)
                ttk.Button(inner, text="設定",
                           command=lambda idx=i,e=entry:self.set_joint_angle(idx,e)
                           ).pack(side="left", padx=5)
                ttk.Label(inner, text=f"[{mn}° ~ {mx}°]",
                          font=('Arial',8), foreground="gray"
                          ).pack(side="left", padx=(10,0))
            else:
                self.joint_entries.append(None)
                ttk.Label(ctrl_frame, text=f"範圍: [{mn}° ~ {mx}°]",
                          font=('Arial',8), foreground="gray"
                          ).pack(anchor="w")
            joint_scales_ref.append(None)  # 保留接口

        self.ctrl.joint_scales = joint_scales_ref

        anim_frame = ttk.LabelFrame(frame, text="動畫速度", padding="5")
        anim_frame.grid(row=len(joint_config)*2, column=0, pady=10, sticky="ew")

        spf = ttk.Frame(anim_frame)
        spf.pack(fill="x", pady=2)
        ttk.Label(spf, text="關節速度:").pack(side="left")
        self.anim_speed_scale = ttk.Scale(spf, from_=1, to=100, orient="horizontal")
        self.anim_speed_scale.set(12)
        self.anim_speed_scale.pack(side="left", fill="x", expand=True, padx=5)
        self.anim_speed_label = ttk.Label(spf, text="12%", width=6)
        self.anim_speed_label.pack(side="left")
        self.anim_speed_scale.configure(command=self.on_anim_speed_change)

        btnf = ttk.Frame(frame)
        btnf.grid(row=len(joint_config)*2+1, column=0, pady=10)
        ttk.Button(btnf, text="重置姿態",
                   command=self.reset_pose).pack(fill="x", pady=2)

    def create_position_control(self, parent):
        frame = ttk.Frame(parent)
        frame.grid(row=0, column=1, padx=5, pady=5, sticky="nsew")

        # 當前 J5 位置
        cur = ttk.LabelFrame(frame, text="當前末端位置 (J5)", padding="10")
        cur.pack(fill="x", pady=(0,10))
        pf = ttk.Frame(cur); pf.pack()
        self.current_labels = {}
        for i, ax in enumerate(["X","Y","Z"]):
            ttk.Label(pf, text=f"{ax}:", font=('Arial',10,'bold')).grid(row=0, column=i*2, padx=5)
            lbl = ttk.Label(pf, text="0.000 m", font=('Courier',11), foreground="green")
            lbl.grid(row=0, column=i*2+1, padx=5)
            self.current_labels[ax] = lbl

        # 目標 XYZ
        tgt = ttk.LabelFrame(frame, text="目標位置設定", padding="10")
        tgt.pack(fill="x", pady=(0,10))
        self.target_scales = {}
        cfg = [('X',-0.7,0.7,0.3),('Y',-0.7,0.7,0.0),('Z',0.0,1.0,0.8)]
        for ax, mn, mx, dv in cfg:
            af = ttk.Frame(tgt); af.pack(fill="x", pady=2)
            ttk.Label(af, text=f"{ax}:", width=3).pack(side="left")
            sc = ttk.Scale(af, from_=mn, to=mx, orient="horizontal")
            sc.set(dv); sc.pack(side="left", fill="x", expand=True, padx=5)
            vl = ttk.Label(af, text=f"{dv:.2f}", width=6); vl.pack(side="left")
            sc.configure(command=lambda v,l=vl:l.configure(text=f"{float(v):.2f}"))
            self.target_scales[ax] = sc

        # 軌跡控制
        traj = ttk.LabelFrame(frame, text="圓弧軌跡控制", padding="10")
        traj.pack(fill="both", expand=True)

        rf = ttk.Frame(traj); rf.pack(fill="x", pady=5)
        ttk.Label(rf, text="圓弧係數:").pack(side="left")
        self.radius_scale = ttk.Scale(rf, from_=0.5, to=5.0, orient="horizontal")
        self.radius_scale.set(2.0)
        self.radius_scale.pack(side="left", fill="x", expand=True, padx=5)
        self.radius_label = ttk.Label(rf, text="2.00", width=5); self.radius_label.pack(side="left")
        self.radius_scale.configure(command=lambda v:self.radius_label.configure(
            text=f"{float(v):.2f}"))

        sf = ttk.Frame(traj); sf.pack(fill="x", pady=5)
        ttk.Label(sf, text="軌跡速度:").pack(side="left")
        self.speed_scale = ttk.Scale(sf, from_=1, to=1000, orient="horizontal")
        self.speed_scale.set(100)
        self.speed_scale.pack(side="left", fill="x", expand=True, padx=5)
        self.speed_label = ttk.Label(sf, text="100x", width=12); self.speed_label.pack(side="left")
        self.speed_scale.configure(command=self.on_speed_change)

        stf = ttk.Frame(traj); stf.pack(fill="x", pady=5)
        ttk.Label(stf, text="動畫倍速:").pack(side="left")
        self.step_scale = ttk.Scale(stf, from_=1, to=50, orient="horizontal")
        self.step_scale.set(1)
        self.step_scale.pack(side="left", fill="x", expand=True, padx=5)
        self.step_label = ttk.Label(stf, text="1倍", width=8); self.step_label.pack(side="left")
        self.step_scale.configure(command=self.on_step_change)

        of = ttk.Frame(traj); of.pack(fill="x", pady=5)
        self.skip_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(of, text="自動跳過無法到達的點",
                        variable=self.skip_var,
                        command=self.toggle_skip).pack(anchor="w")

        # 狀態窗
        sfm = ttk.Frame(traj); sfm.pack(fill="both",expand=True,pady=5)
        sb = ttk.Scrollbar(sfm); sb.pack(side="right",fill="y")
        self.status_text = tk.Text(sfm,height=8,width=40,font=('Courier',10),
                                   state='disabled',wrap=tk.WORD,
                                   yscrollcommand=sb.set,spacing1=2,spacing2=1,
                                   spacing3=2,padx=5,pady=5)
        self.status_text.pack(side="left",fill="both",expand=True)
        sb.config(command=self.status_text.yview)
        self.update_status("準備就緒")

        bf = ttk.Frame(traj); bf.pack(fill="x", pady=5)
        ttk.Button(bf,text="執行圓弧移動",
                   command=self.execute_arc).pack(side="left",padx=2)
        ttk.Button(bf,text="停止",
                   command=self.stop_trajectory).pack(side="left",padx=2)
        ttk.Button(bf,text="重置軌跡",
                   command=self.reset_trajectory).pack(side="left",padx=2)

    def create_system_control(self, parent):
        frame = ttk.Frame(parent)
        frame.grid(row=1,column=0,columnspan=2,pady=10,sticky="ew")
        ttk.Button(frame,text="關閉程式",
                   command=self.quit_program).pack(side="right",padx=5)
        ttk.Label(frame,text="RA605-710-GC 六軸機械手臂 (PyBullet)",
                  font=('Arial',9)).pack(side="left",padx=5)

    # ---- 事件 ----
    def on_anim_speed_change(self,val):
        sp = int(float(val))
        self.ctrl.set_animation_speed(sp)
        self.anim_speed_label.configure(text=f"{sp}%")

    def on_speed_change(self,val):
        sp = int(float(val))
        self.ctrl.set_trajectory_speed(sp)
        delay_ms = (0.005*(1000-sp)/999.0)*1000.0
        s = "最高速" if delay_ms < 0.001 else f"{delay_ms:.2f}ms"
        self.speed_label.configure(text=f"{sp}x ({s})")

    def on_step_change(self,val):
        st = int(float(val))
        self.ctrl.set_trajectory_step(st)
        self.step_label.configure(text=f"{st}倍")

    def toggle_skip(self):
        self.ctrl.skip_unreachable_points = self.skip_var.get()

    def execute_arc(self):
        try:
            A = self.ctrl.get_joint5_position()
            B = [self.target_scales['X'].get(),
                 self.target_scales['Y'].get(),
                 self.target_scales['Z'].get()]
            r = self.radius_scale.get()
            self.update_status("計算軌跡中...")
            self.root.update()
            arc,_,_,_ = compute_arc_with_auto_center(A,B,r,1000)
            length = np.sum(np.sqrt(np.sum(np.diff(arc,axis=0)**2,axis=1)))
            st = self.ctrl.trajectory_step
            self.update_status(
                f"軌跡長度: {length:.3f}m\n總點數: {len(arc)}\n倍速: {st}倍\n執行中...")
            self.ctrl.start_trajectory(arc)
        except Exception as e:
            messagebox.showerror("錯誤",f"執行失敗: {e}")

    def stop_trajectory(self):
        self.ctrl.stop_trajectory()
        self.update_status("已停止")

    def reset_trajectory(self):
        self.ctrl.stop_trajectory()
        self.ctrl.trajectory_index = 0
        self.ctrl.clear_trajectory_markers()
        self.update_status("準備就緒")

    def reset_pose(self):
        self.ctrl.reset_pose()

    def update_status(self,msg):
        self.status_text.config(state='normal')
        self.status_text.delete(1.0,tk.END)
        self.status_text.insert(1.0,msg)
        self.status_text.config(state='disabled')

    def append_status(self,msg):
        self.status_text.config(state='normal')
        self.status_text.insert(tk.END,msg+"\n")
        self.status_text.see(tk.END)
        lines = int(self.status_text.index('end-1c').split('.')[0])
        if lines > 100:
            self.status_text.delete(1.0,f"{lines-100}.0")
        self.status_text.config(state='disabled')

    def update_display(self):
        try:
            pos = self.ctrl.get_joint5_position()
            for i,ax in enumerate(['X','Y','Z']):
                self.current_labels[ax].configure(text=f"{pos[i]:.3f} m")
            for i,ang in enumerate(self.ctrl.current_angles):
                self.joint_labels[i].configure(text=f"{ang:.1f}°")
            if self.ctrl.is_following_trajectory and self.ctrl.trajectory_points is not None:
                total = len(self.ctrl.trajectory_points)
                cur = self.ctrl.trajectory_index
                st = self.ctrl.trajectory_step
                prog = (cur/total*100) if total>0 else 0
                self.update_status(
                    f"執行中... {cur}/{total} ({prog:.1f}%)\n倍速: {st}倍")
        except:
            pass
        self.root.after(1000,self.update_display)

    def quit_program(self):
        global exit_flag
        if messagebox.askyesno("確認","確定要關閉程式嗎?"):
            exit_flag = True
            self.root.quit()
            self.root.destroy()


# ========= PyBullet 迴圈（背景執行緒） =========

def bullet_loop():
    global exit_flag

    cid = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0,0,-9.8)
    p.loadURDF("plane.urdf")

    start_pos = ctrl.get_joint5_position()
    vs = p.createVisualShape(p.GEOM_SPHERE,
                             radius=0.08,
                             rgbaColor=[1,0,0,1])
    marker_id = p.createMultiBody(baseMass=0,
                                  baseVisualShapeIndex=vs,
                                  basePosition=start_pos.tolist())
    ctrl.set_marker(marker_id)

    last = time.time()
    try:
        while not exit_flag:
            now = time.time()
            if now - last > 1.0/100.0:
                ctrl.animate()
                last = now
            p.stepSimulation()
            time.sleep(1.0/240.0)
    except:
        pass
    finally:
        p.disconnect()


# ========= main =========

if __name__ == "__main__":
    # 開啟 PyBullet 背景執行緒
    t = threading.Thread(target=bullet_loop, daemon=True)
    t.start()

    # 主執行緒跑 Tkinter GUI
    root = tk.Tk()
    gui = RobotControlGUI(root, ctrl)
    root.mainloop()

    exit_flag = True
    t.join()
    sys.exit(0)
