"""
config/robot_config.py
機器人配置檔案
RA605-710-GC 六軸機械手臂參數
"""

import numpy as np

# ========== DH 參數 ==========
# RA605-710-GC 機械手臂 DH 參數
DH_PARAMS = {
    'S1': 0.030,    # 30 mm
    'S2': 0.040,    # 40 mm
    'L1': 0.375,    # 375 mm
    'L2': 0.340,    # 340 mm
    'L3': 0.338,    # 338 mm
    'L4': 0.0865,   # 86.5 mm
}

# ========== 關節限制 ==========
# 格式: {'J1': (min, max), 'J2': (min, max), ...}
JOINT_LIMITS = {
    'J1': (-165, 165),   # 關節1 Z軸旋轉
    'J2': (-125, 85),    # 關節2 Y軸旋轉
    'J3': (-55, 185),    # 關節3 Y軸旋轉
    'J4': (-190, 190),   # 關節4 Z軸旋轉
    'J5': (-25, 205),    # 關節5 Y軸旋轉
    'J6': (-360, 360),   # 關節6 X軸旋轉
}

# 也提供 list 格式方便索引（0-based）
JOINT_LIMITS_LIST = [
    (-165, 165),   # Joint 0 (J1)
    (-125, 85),    # Joint 1 (J2)
    (-55, 185),    # Joint 2 (J3)
    (-190, 190),   # Joint 3 (J4)
    (-25, 205),    # Joint 4 (J5)
    (-360, 360),   # Joint 5 (J6)
]

# ========== 速度與加速度限制 ==========
VELOCITY_LIMITS = {
    'joint_max': 180,      # 度/秒
    'cartesian_max': 1000, # mm/秒
}

ACCELERATION_LIMITS = {
    'joint_max': 360,      # 度/秒²
    'cartesian_max': 2000, # mm/秒²
}

# ========== GUI 主題配置 ==========
GUI_THEME = {
    'background': '#1a1a1a',
    'panel': '#2d2d2d',
    'border': '#3d3d3d',
    'primary': '#3498db',
    'success': '#2ecc71',
    'warning': '#f39c12',
    'danger': '#e74c3c',
    'text': '#ecf0f1',
}

# ========== 軌跡規劃配置 ==========
TRAJECTORY_CONFIG = {
    'default_speed': 0.3,         # 30%
    'default_acceleration': 0.2,   # 20%
    'default_blend_radius': 10,    # mm
    'interpolation_steps': 100,    # 插值點數
    'control_frequency': 50,       # Hz
}

# ========== 模型路徑配置 ==========
'''
MODEL_CONFIG = {
    'obj_path': '/home/yahboom/Desktop/Obj/',
    'model_count': 8,
    'model_prefix': 'p',
    'model_extension': '.obj',
}
'''
MODEL_CONFIG = {
    'obj_path': '/home/test/桌面/Obj/',
    'model_count': 8,
    'model_prefix': 'p',
    'model_extension': '.obj',
}


# ========== 初始平移量（根據你的數據）==========
INITIAL_TRANSLATIONS = {
    'p2': (0, 0, 0.23),
    'p3': (-0.03, 0, 0.375),
    'p4': (-0.03, 0, 0.715),
    'p5': (0.01, 0, 0.81),
    'p6': (0.01, 0, 1.053),
    'p7': (0.01, 0, 1.12),
    'p8': (0.01, 0, 1.1395),
}

# ========== 旋轉中心配置 ==========
ROTATION_CENTERS = {
    'p5_top': lambda: [DH_PARAMS['S2'] - DH_PARAMS['S1'], 0,
                       DH_PARAMS['L1'] + DH_PARAMS['L2'] + DH_PARAMS['L3']],
    'p3_top': lambda: [-DH_PARAMS['S1'], 0,
                       DH_PARAMS['L1'] + DH_PARAMS['L2']],
}

# ========== 安全配置 ==========
SAFETY_CONFIG = {
    'emergency_stop_decel': 5000,  # mm/s² 緊急停止減速度
    'collision_threshold': 50,      # N 碰撞檢測閾值
    'workspace_limits': {
        'x': (-700, 700),   # mm
        'y': (-700, 700),   # mm
        'z': (0, 1000),     # mm
    }
}

# ========== 預設姿態 ==========
PRESET_POSES = {
    'home': [0, 0, 0, 0, 0, 0],
    'vertical': [0, -90, 90, 0, 0, 0],
    'horizontal': [0, 0, 0, 0, -90, 0],
    'folded': [0, -45, 135, 0, -90, 0],
}

# ========== 通訊配置 ==========
COMMUNICATION_CONFIG = {
    'serial_port': '/dev/ttyUSB0',
    'baud_rate': 115200,
    'timeout': 1.0,
    'retry_times': 3,
}

# ========== 日誌配置 ==========
LOG_CONFIG = {
    'log_level': 'INFO',
    'log_file': 'robot_control.log',
    'max_file_size': 10 * 1024 * 1024,  # 10 MB
    'backup_count': 5,
}

# ========== 輔助函數 ==========
def get_joint_limit(joint_index):
    """
    獲取關節限制
    Args:
        joint_index: 0-5 或 1-6
    Returns:
        (min, max) tuple
    """
    if isinstance(joint_index, int):
        if 0 <= joint_index < 6:
            return JOINT_LIMITS_LIST[joint_index]
        elif 1 <= joint_index <= 6:
            return JOINT_LIMITS[f'J{joint_index}']
    elif isinstance(joint_index, str):
        return JOINT_LIMITS.get(joint_index, (-180, 180))

    raise ValueError(f"Invalid joint index: {joint_index}")

def is_within_limits(joint_index, angle):
    """
    檢查角度是否在限制範圍內
    Args:
        joint_index: 關節索引
        angle: 角度值（度）
    Returns:
        bool
    """
    min_val, max_val = get_joint_limit(joint_index)
    return min_val <= angle <= max_val

def is_position_safe(x, y, z):
    """
    檢查位置是否在安全工作空間內
    Args:
        x, y, z: 位置座標 (mm)
    Returns:
        bool
    """
    limits = SAFETY_CONFIG['workspace_limits']
    return (limits['x'][0] <= x <= limits['x'][1] and
            limits['y'][0] <= y <= limits['y'][1] and
            limits['z'][0] <= z <= limits['z'][1])

def print_config():
    """打印配置資訊"""
    print("\n" + "=" * 60)
    print("RA605-710-GC 配置資訊")
    print("=" * 60)

    print("\nDH 參數:")
    for key, value in DH_PARAMS.items():
        print(f"  {key} = {value * 1000:.1f} mm")

    print("\n關節限制:")
    for joint, (min_val, max_val) in JOINT_LIMITS.items():
        print(f"  {joint}: {min_val}° ~ {max_val}°")

    print("\n工作空間限制:")
    for axis, (min_val, max_val) in SAFETY_CONFIG['workspace_limits'].items():
        print(f"  {axis}: {min_val} ~ {max_val} mm")

    print("=" * 60 + "\n")

if __name__ == "__main__":
    print_config()