# ========== config/__init__.py ==========
"""
config/__init__.py
配置模組初始化
"""

from .robot_config import (
    ROBOT_DH_PARAMS,
    JOINT_LIMITS,
    MODEL_PATHS,
    DEFAULT_POSES,
    TRAJECTORY_CONFIG,
    GUI_THEME
)

__all__ = [
    'ROBOT_DH_PARAMS',
    'JOINT_LIMITS',
    'MODEL_PATHS',
    'DEFAULT_POSES',
    'TRAJECTORY_CONFIG',
    'GUI_THEME'
]
