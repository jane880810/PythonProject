"""
core/trajectory.py
軌跡規劃模組
"""

import numpy as np
from config.robot_config import TRAJECTORY_CONFIG

class TrajectoryPlanner:
    """軌跡規劃器"""

    def __init__(self):
        """初始化軌跡規劃器"""
        self.config = TRAJECTORY_CONFIG
        print("✓ TrajectoryPlanner 模組初始化完成")

    def plan_joint_trajectory(self, start_angles, end_angles, duration):
        """
        關節空間軌跡規劃
        Args:
            start_angles: 起始關節角度
            end_angles: 目標關節角度
            duration: 運動時間 (秒)
        Returns:
            trajectory: 時間序列軌跡點
        """
        steps = int(duration * self.config['control_frequency'])
        trajectory = []

        for i in range(steps + 1):
            t = i / steps
            # 5次多項式插值
            s = self._quintic_polynomial(t)

            angles = []
            for start, end in zip(start_angles, end_angles):
                angle = start + (end - start) * s
                angles.append(angle)

            trajectory.append({
                'time': t * duration,
                'angles': angles
            })

        return trajectory

    def plan_linear_trajectory(self, start_pos, end_pos, duration):
        """
        直線軌跡規劃
        Args:
            start_pos: 起始位置 {'x', 'y', 'z'}
            end_pos: 目標位置 {'x', 'y', 'z'}
            duration: 運動時間
        Returns:
            trajectory: 位置序列
        """
        steps = int(duration * self.config['control_frequency'])
        trajectory = []

        for i in range(steps + 1):
            t = i / steps
            s = self._quintic_polynomial(t)

            pos = {
                'x': start_pos['x'] + (end_pos['x'] - start_pos['x']) * s,
                'y': start_pos['y'] + (end_pos['y'] - start_pos['y']) * s,
                'z': start_pos['z'] + (end_pos['z'] - start_pos['z']) * s,
            }

            trajectory.append({
                'time': t * duration,
                'position': pos
            })

        return trajectory

    def _quintic_polynomial(self, t):
        """
        5次多項式插值（平滑加減速）
        s(t) = 10t³ - 15t⁴ + 6t⁵
        """
        return 10 * t**3 - 15 * t**4 + 6 * t**5

    def blend_trajectory(self, points, blend_radius):
        """
        軌跡混合（圓弧過渡）
        Args:
            points: 軌跡點列表
            blend_radius: 混合半徑 (mm)
        Returns:
            blended_trajectory: 混合後的軌跡
        """
        if len(points) < 3:
            return points

        blended = [points[0]]  # 起點

        for i in range(1, len(points) - 1):
            prev = points[i - 1]
            curr = points[i]
            next_point = points[i + 1]

            # 計算混合點
            blend_points = self._create_blend(prev, curr, next_point, blend_radius)
            blended.extend(blend_points)

        blended.append(points[-1])  # 終點

        return blended

    def _create_blend(self, p1, p2, p3, radius):
        """創建圓弧混合"""
        # 簡化版：線性混合
        blend_points = []
        steps = 10

        for i in range(steps):
            t = i / (steps - 1)
            if t < 0.5:
                # 從 p1 到 p2
                s = t * 2
                blend_points.append(self._interpolate(p1, p2, s))
            else:
                # 從 p2 到 p3
                s = (t - 0.5) * 2
                blend_points.append(self._interpolate(p2, p3, s))

        return blend_points

    def _interpolate(self, p1, p2, t):
        """線性插值"""
        result = {}
        for key in p1.keys():
            if isinstance(p1[key], (int, float)):
                result[key] = p1[key] + (p2[key] - p1[key]) * t
        return result