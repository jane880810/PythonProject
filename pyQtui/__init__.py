# ============= 3. visualization/__init__.py =============
"""
visualization/__init__.py
視覺化模組初始化檔案
"""

# VTK 視覺化是選用的
try:
    from .vtk_view import VTKWidget
    __all__ = ['VTKWidget']
except ImportError:
    # 如果 VTK 沒有安裝，不要中斷程式
    print("警告: VTK 未安裝，3D 視覺化功能將不可用")
    __all__ = []