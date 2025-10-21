"""
config/__init__.py
配置模組初始化檔案
"""

# 從 robot_config 匯入所有需要的配置
from .robot_config import (
    DH_PARAMS,              # DH 參數
    JOINT_LIMITS,           # 關節限制
    GUI_THEME,              # GUI 主題
    TRAJECTORY_CONFIG,      # 軌跡配置
    VELOCITY_LIMITS,        # 速度限制
    ACCELERATION_LIMITS,    # 加速度限制
    SAFETY_CONFIG,          # 安全配置
    PRESET_POSES,           # 預設姿態
    MODEL_CONFIG,           # 模型配置
)

# 匯出所有配置
__all__ = [
    'DH_PARAMS',
    'JOINT_LIMITS',
    'GUI_THEME',
    'TRAJECTORY_CONFIG',
    'VELOCITY_LIMITS',
    'ACCELERATION_LIMITS',
    'SAFETY_CONFIG',
    'PRESET_POSES',
    'MODEL_CONFIG',
]