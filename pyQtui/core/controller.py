"""
core/controller.py 123456
機械手臂控制器
"""

import numpy as np
import time

# 從配置檔案匯入參數
from config.robot_config import ROBOT_DH_PARAMS


class AnimationController:
    """機械手臂動畫控制器"""

    def __init__(self):
        # 從配置讀取 DH 參數
        self.S1 = ROBOT_DH_PARAMS['S1']
        self.S2 = ROBOT_DH_PARAMS['S2']
        self.L1 = ROBOT_DH_PARAMS['L1']
        self.L2 = ROBOT_DH_PARAMS['L2']
        self.L3 = ROBOT_DH_PARAMS['L3']
        self.L4 = ROBOT_DH_PARAMS['L4']

        self.current_angles = [0, 0, 0, 0, 0, 0]
        self.target_angles = [0, 0, 0, 0, 0, 0]
        self.animation_speed = 0.12
        self.is_animating = False
        self.joint_scales = []  # GUI 滑桿引用

        # 旋轉矩陣（用於 3D 視覺化，目前簡化）
        self.R_p2_total = np.eye(3)
        self.R_p3_total = np.eye(3)
        self.R_p4_total = np.eye(3)
        self.R_p5_total = np.eye(3)
        self.R_p6_total = np.eye(3)
        self.R_p8_total = np.eye(3)

        # 局部座標向量
        self.p3_base_local = np.array([-self.S1, 0, self.L1])
        self.p3_length_vec = np.array([0, 0, self.L2])
        self.p4_length_vec = np.array([self.S2, 0, 0])
        self.p5_length_vec = np.array([0, 0, self.L3])
        self.p6_length_vec = np.array([0, 0, 0.067])
        self.p7_length_vec = np.array([0, 0, -0.067])
        self.p8_center_offset = np.array([self.L4, 0, 0])

        # 軌跡相關
        self.joint5_marker = None
        self.trajectory_line = None
        self.trajectory_points = None
        self.is_following_trajectory = False
        self.trajectory_index = 0
        self.trajectory_delay = 0.005
        self.last_trajectory_time = 0
        self.skip_unreachable_points = True
        self.skipped_points_count = 0

        print("✓ AnimationController 初始化完成")

    def set_marker(self, marker):
        """設定關節 5 標記（用於 3D 視覺化）"""
        self.joint5_marker = marker

    def get_joint5_position(self):
        """取得關節 5（末端）位置"""
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
        """更新關節 5 標記（3D 視覺化用）"""
        # 這部分需要 Open3D 和 vis 物件
        # 暫時註解掉，如需使用請整合 Open3D
        pass

    def update_joint(self, joint_idx, delta_angle):
        """更新關節角度（3D 視覺化用）"""
        # 這部分需要 Open3D 的 node 物件
        # 暫時簡化，只更新角度
        if abs(delta_angle) < 0.0001:
            return

        # 這裡可以加入實際的機械手臂通訊程式碼
        # 例如：發送命令到實體機械手臂
        pass

    def animate(self):
        """動畫更新循環"""
        # 處理軌跡跟踪
        if self.is_following_trajectory:
            current_time = time.time()
            if current_time - self.last_trajectory_time >= self.trajectory_delay:
                self.last_trajectory_time = current_time
                self.move_next_point()
            return

        # 處理手動關節控制
        if not self.is_animating:
            return

        moved = False
        for i in range(6):
            if abs(self.current_angles[i] - self.target_angles[i]) > 0.01:
                old = self.current_angles[i]
                diff = self.target_angles[i] - old
                self.current_angles[i] = old + diff * self.animation_speed
                # self.update_joint(i, self.current_angles[i] - old)
                moved = True

        if not moved:
            self.is_animating = False

    def set_target(self, joint_idx, angle):
        """設定目標角度"""
        self.target_angles[joint_idx] = angle
        self.is_animating = True

    def fk(self, angles):
        """正向運動學"""
        # 使用簡化版本，不依賴 Open3D
        theta1 = np.deg2rad(angles[0])
        theta2 = np.deg2rad(angles[1])
        theta3 = np.deg2rad(angles[2])
        theta4 = np.deg2rad(angles[3])

        # 簡化的 FK 計算
        x = (self.L2 + self.L3) * np.cos(theta2 + theta3) * np.cos(theta1)
        y = (self.L2 + self.L3) * np.cos(theta2 + theta3) * np.sin(theta1)
        z = self.L1 + (self.L2 + self.L3) * np.sin(theta2 + theta3)

        return np.array([x, y, z])

    def ik(self, x, y, z):
        """逆向運動學"""
        try:
            theta1 = np.arctan2(-y, -x)
            r_xy = np.sqrt(y**2 + x**2)
            z_off = z - self.L1
            r_off = r_xy - self.S1

            if abs(r_off) < 1e-6:
                return None, "err"

            alpha = np.arctan2(z_off, r_off)
            d = np.sqrt(r_off**2 + z_off**2)
            L45 = np.sqrt(self.S2**2 + self.L3**2)

            if d > self.L2 + L45 or d < abs(self.L2 - L45):
                return None, "reach"

            cb = np.clip((self.L2**2 + d**2 - L45**2) / (2*self.L2*d), -1, 1)
            beta = np.arccos(cb)
            theta2 = -np.pi/2 + alpha + beta

            phi = np.arctan2(self.S2, self.L3)
            cg = np.clip((self.L2**2 + L45**2 - d**2) / (2*self.L2*L45), -1, 1)
            gamma = np.arccos(cg)
            theta3 = -np.pi/2 - phi + gamma

            angles = [np.rad2deg(theta1), np.rad2deg(theta2), np.rad2deg(theta3), 0]

            # 檢查關節限制
            limits = [(-165, 165), (-125, 85), (-55, 185), (-190, 190)]
            for i, (a, (mn, mx)) in enumerate(zip(angles, limits)):
                if a < mn or a > mx:
                    return None, "limit"

            return angles, "ok"

        except Exception as e:
            return None, "err"

    def move_to(self, x, y, z):
        """移動到指定位置（使用動畫）"""
        res = self.ik(x, y, z)
        if res[0] is None:
            return False, res[1]

        angles = res[0]
        for i in range(4):
            self.set_target(i, angles[i])

        return True, "ok"

    def move_to_instant(self, x, y, z):
        """直接移動到目標位置（不使用動畫）"""
        res = self.ik(x, y, z)
        if res[0] is None:
            return False, res[1]

        angles = res[0]

        # 直接設置角度
        for i in range(4):
            delta = angles[i] - self.current_angles[i]
            self.current_angles[i] = angles[i]
            self.target_angles[i] = angles[i]

        return True, "ok"

    def show_trajectory(self, points):
        """顯示軌跡（3D 視覺化用）"""
        # 需要 Open3D，暫時跳過
        pass

    def start_trajectory(self, points):
        """開始執行軌跡"""
        self.trajectory_points = points
        self.trajectory_index = 0
        self.skipped_points_count = 0
        self.is_following_trajectory = True
        self.last_trajectory_time = time.time()
        # self.show_trajectory(points)
        print(f"✓ 開始執行軌跡，共 {len(points)} 點")
        self.move_next_point()

    def move_next_point(self):
        """移動到軌跡的下一個點"""
        if not self.is_following_trajectory or self.trajectory_points is None:
            return

        if self.trajectory_index >= len(self.trajectory_points):
            if self.skipped_points_count > 0:
                print(f"\n✓ 軌跡完成! 總點數: {len(self.trajectory_points)}, 跳過: {self.skipped_points_count}")
            else:
                print(f"\n✓ 軌跡完成! 總點數: {len(self.trajectory_points)}")
            self.is_following_trajectory = False
            self.skipped_points_count = 0
            return

        pt = self.trajectory_points[self.trajectory_index]
        ok, msg = self.move_to_instant(pt[0], pt[1], pt[2])

        if ok:
            self.trajectory_index += 1
            if self.trajectory_index % 100 == 0:
                print(f"進度: {self.trajectory_index}/{len(self.trajectory_points)}")
        else:
            if self.skip_unreachable_points:
                self.skipped_points_count += 1
                self.trajectory_index += 1

                if self.skipped_points_count > len(self.trajectory_points) * 0.2:
                    print(f"✗ 太多無法到達的點 ({self.skipped_points_count}/{len(self.trajectory_points)})")
                    self.is_following_trajectory = False
                    self.skipped_points_count = 0
            else:
                print(f"✗ 失敗於點 {self.trajectory_index}: {msg}")
                self.is_following_trajectory = False

    def set_trajectory_speed(self, speed):
        """設定軌跡速度
        speed: 1-1000，數字越大速度越快
        """
        self.trajectory_delay = 0.005 * (1000 - speed) / 999
        print(f"軌跡速度: {speed}x")

    def stop_trajectory(self):
        """停止軌跡執行"""
        self.is_following_trajectory = False
        self.skipped_points_count = 0
        print("軌跡已停止")

    def reset_pose(self):
        """重置所有關節到 0 度"""
        for i in range(6):
            self.set_target(i, 0)
        print("姿態已重置")