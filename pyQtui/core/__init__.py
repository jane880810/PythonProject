# ============= 1. core/__init__.py =============
"""
core/__init__.py
核心模組初始化檔案
"""

# 匯入核心功能類別
from .controller import AnimationController
from .kinematics import Kinematics
from .trajectory import TrajectoryPlanner

__all__ = [
    'AnimationController',
    'Kinematics',
    'TrajectoryPlanner'
]
