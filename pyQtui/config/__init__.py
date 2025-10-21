# ============= 5. config/__init__.py =============
"""
config/__init__.py
配置模組初始化檔案
"""

# 從 robot_config 匯入所有需要的配置
from .robot_config import (
    DH_PARAMS,           # DH 參數
    JOINT_LIMITS,        # 關節限制
    WORKSPACE,           # 工作空間
    GUI_THEME,           # GUI 主題
    TRAJECTORY_CONFIG    # 軌跡配置
)

# 匯出所有配置
__all__ = [
    'DH_PARAMS',
    'JOINT_LIMITS',
    'WORKSPACE',
    'GUI_THEME',
    'TRAJECTORY_CONFIG'
]