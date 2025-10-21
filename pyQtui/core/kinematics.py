"""
core/kinematics.py
運動學計算模組 - 正向運動學與逆向運動學
RA605-710-GC 六軸機械手臂
"""

import numpy as np
from config.robot_config import DH_PARAMS, JOINT_LIMITS

class Kinematics:
    """運動學計算類別"""

    def __init__(self):
        """初始化運動學參數"""
        self.S1 = DH_PARAMS['S1']
        self.S2 = DH_PARAMS['S2']
        self.L1 = DH_PARAMS['L1']
        self.L2 = DH_PARAMS['L2']
        self.L3 = DH_PARAMS['L3']
        self.L4 = DH_PARAMS['L4']

        # 幾何向量
        self.p3_base_local = np.array([-self.S1, 0, self.L1])
        self.p3_length_vec = np.array([0, 0, self.L2])
        self.p4_length_vec = np.array([self.S2, 0, 0])
        self.p5_length_vec = np.array([0, 0, self.L3])
        self.p6_length_vec = np.array([0, 0, 0.067])
        self.p7_length_vec = np.array([0, 0, -0.067])
        self.p8_center_offset = np.array([self.L4, 0, 0])

        print("✓ Kinematics 模組初始化完成")

    def forward_kinematics(self, joint_angles):
        """
        正向運動學：計算末端執行器位置
        Args:
            joint_angles: [θ1, θ2, θ3, θ4, θ5, θ6] (度)
        Returns:
            position: {'x', 'y', 'z', 'rx', 'ry', 'rz'}
        """
        # 轉換為弧度
        angles_rad = [np.deg2rad(a) for a in joint_angles]

        # 計算旋轉矩陣
        R_p2 = self._rotation_matrix_z(angles_rad[0])
        R_p3 = self._rotation_matrix_y(angles_rad[1])
        R_p4_base = self._rotation_matrix_y(np.deg2rad(-90))
        R_p4 = self._rotation_matrix_y(angles_rad[2]) @ R_p4_base
        R_p5 = self._rotation_matrix_z(angles_rad[3])

        # 計算位置
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

        # 計算姿態（簡化版）
        total_rotation = R_p2 @ R_p3 @ R_p4 @ R_p5
        rx, ry, rz = self._rotation_matrix_to_euler(total_rotation)

        return {
            'x': position[0] * 1000,  # 轉換為 mm
            'y': position[1] * 1000,
            'z': position[2] * 1000,
            'rx': np.rad2deg(rx),
            'ry': np.rad2deg(ry),
            'rz': np.rad2deg(rz)
        }

    def inverse_kinematics(self, target_position):
        """
        逆向運動學：計算關節角度（解析解）
        Args:
            target_position: {'x', 'y', 'z'} (mm)
        Returns:
            joint_angles: [θ1, θ2, θ3, θ4] 或 None
        """
        # 轉換為米
        x = target_position['x'] / 1000.0
        y = target_position['y'] / 1000.0
        z = target_position['z'] / 1000.0

        try:
            # θ1 計算
            theta1 = np.arctan2(-y, -x)

            # 水平距離
            r_xy = np.sqrt(y**2 + x**2)

            # θ2 計算
            z_offset = z - self.L1
            r_offset = r_xy - self.S1

            if abs(r_offset) < 1e-6:
                return None

            alpha = np.arctan2(z_offset, r_offset)
            d_to_p3 = np.sqrt(r_offset**2 + z_offset**2)

            # P4-P5 連桿的有效長度
            L_45 = np.sqrt(self.S2**2 + self.L3**2)

            # 檢查工作範圍
            max_reach = self.L2 + L_45
            min_reach = abs(self.L2 - L_45)
            if d_to_p3 > max_reach or d_to_p3 < min_reach:
                return None

            # 使用餘弦定理
            cos_beta = (self.L2**2 + d_to_p3**2 - L_45**2) / (2 * self.L2 * d_to_p3)
            cos_beta = np.clip(cos_beta, -1.0, 1.0)
            beta = np.arccos(cos_beta)

            theta2 = -np.pi/2 + alpha + beta

            # θ3 計算
            phi = np.arctan2(self.S2, self.L3)
            cos_gamma = (self.L2**2 + L_45**2 - d_to_p3**2) / (2 * self.L2 * L_45)
            cos_gamma = np.clip(cos_gamma, -1.0, 1.0)
            gamma = np.arccos(cos_gamma)

            theta3 = -np.pi/2 - phi + gamma

            # θ4 - 保持末端水平
            theta4 = 0

            # 轉換為角度
            angles = [
                np.rad2deg(theta1),
                np.rad2deg(theta2),
                np.rad2deg(theta3),
                np.rad2deg(theta4),
                0,  # θ5
                0   # θ6
            ]

            # 檢查關節限制
            if isinstance(JOINT_LIMITS, dict):
                limits = [JOINT_LIMITS[f'J{i+1}'] for i in range(6)]
            else:
                limits = JOINT_LIMITS

            for i, (angle, (min_a, max_a)) in enumerate(zip(angles, limits)):
                if angle < min_a or angle > max_a:
                    return None

            return angles

        except Exception as e:
            print(f"IK 計算錯誤: {e}")
            return None

    def _rotation_matrix_x(self, angle):
        """X軸旋轉矩陣"""
        c, s = np.cos(angle), np.sin(angle)
        return np.array([
            [1, 0, 0],
            [0, c, -s],
            [0, s, c]
        ])

    def _rotation_matrix_y(self, angle):
        """Y軸旋轉矩陣"""
        c, s = np.cos(angle), np.sin(angle)
        return np.array([
            [c, 0, s],
            [0, 1, 0],
            [-s, 0, c]
        ])

    def _rotation_matrix_z(self, angle):
        """Z軸旋轉矩陣"""
        c, s = np.cos(angle), np.sin(angle)
        return np.array([
            [c, -s, 0],
            [s, c, 0],
            [0, 0, 1]
        ])

    def _rotation_matrix_to_euler(self, R):
        """旋轉矩陣轉歐拉角 (XYZ)"""
        sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)

        singular = sy < 1e-6

        if not singular:
            x = np.arctan2(R[2, 1], R[2, 2])
            y = np.arctan2(-R[2, 0], sy)
            z = np.arctan2(R[1, 0], R[0, 0])
        else:
            x = np.arctan2(-R[1, 2], R[1, 1])
            y = np.arctan2(-R[2, 0], sy)
            z = 0

        return x, y, z