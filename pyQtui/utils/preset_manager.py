"""
utils/preset_manager.py
預設姿態管理器
"""

import json
import os
from datetime import datetime


class PresetManager:
    """預設姿態管理器"""

    def __init__(self, preset_file='presets.json'):
        """
        初始化預設管理器
        Args:
            preset_file: 預設檔案名稱
        """
        # 預設檔案路徑
        self.preset_dir = os.path.join(os.path.dirname(__file__), '..', 'data')
        self.preset_file = os.path.join(self.preset_dir, preset_file)

        # 確保資料目錄存在
        if not os.path.exists(self.preset_dir):
            os.makedirs(self.preset_dir)

        # 載入預設
        self.presets = self.load_all_presets()

        # 如果沒有預設，創建一些內建預設
        if not self.presets:
            self.create_default_presets()

        print("✓ PresetManager 初始化完成")

    def create_default_presets(self):
        """創建內建預設姿態"""
        default_presets = {
            '原點': {
                'joint_angles': [0, 0, 0, 0, 0, 0],
                'position': {'x': 0, 'y': 0, 'z': 0, 'rx': 0, 'ry': 0, 'rz': 0},
                'description': '機械手臂原點姿態',
                'timestamp': datetime.now().isoformat()
            },
            '垂直': {
                'joint_angles': [0, -90, 90, 0, 0, 0],
                'position': {'x': 0, 'y': 0, 'z': 0, 'rx': 0, 'ry': 0, 'rz': 0},
                'description': '垂直向上姿態',
                'timestamp': datetime.now().isoformat()
            },
            '水平': {
                'joint_angles': [0, 0, 0, 0, -90, 0],
                'position': {'x': 0, 'y': 0, 'z': 0, 'rx': 0, 'ry': 0, 'rz': 0},
                'description': '水平伸展姿態',
                'timestamp': datetime.now().isoformat()
            },
            '收納': {
                'joint_angles': [0, -45, 135, 0, -90, 0],
                'position': {'x': 0, 'y': 0, 'z': 0, 'rx': 0, 'ry': 0, 'rz': 0},
                'description': '收納姿態',
                'timestamp': datetime.now().isoformat()
            }
        }

        self.presets = default_presets
        self.save_all_presets()
        print("✓ 已創建內建預設姿態")

    def load_all_presets(self):
        """載入所有預設"""
        if not os.path.exists(self.preset_file):
            return {}

        try:
            with open(self.preset_file, 'r', encoding='utf-8') as f:
                presets = json.load(f)
            print(f"✓ 已載入 {len(presets)} 個預設姿態")
            return presets
        except Exception as e:
            print(f"⚠ 載入預設失敗: {e}")
            return {}

    def save_all_presets(self):
        """儲存所有預設"""
        try:
            with open(self.preset_file, 'w', encoding='utf-8') as f:
                json.dump(self.presets, f, indent=2, ensure_ascii=False)
            return True
        except Exception as e:
            print(f"✗ 儲存預設失敗: {e}")
            return False

    def save_preset(self, name, data):
        """
        儲存預設姿態
        Args:
            name: 預設名稱
            data: 預設資料字典，應包含:
                - joint_angles: 關節角度列表
                - position: 位置字典 (可選)
                - description: 描述 (可選)
        Returns:
            bool: 是否成功
        """
        # 添加時間戳記
        if 'timestamp' not in data:
            data['timestamp'] = datetime.now().isoformat()

        # 儲存到記憶體
        self.presets[name] = data

        # 寫入檔案
        success = self.save_all_presets()

        if success:
            print(f"✓ 已儲存預設: {name}")

        return success

    def load_preset(self, name):
        """
        載入預設姿態
        Args:
            name: 預設名稱
        Returns:
            dict: 預設資料，如果不存在返回 None
        """
        preset = self.presets.get(name)

        if preset:
            print(f"✓ 已載入預設: {name}")
        else:
            print(f"⚠ 預設不存在: {name}")

        return preset

    def delete_preset(self, name):
        """
        刪除預設姿態
        Args:
            name: 預設名稱
        Returns:
            bool: 是否成功
        """
        if name not in self.presets:
            print(f"⚠ 預設不存在: {name}")
            return False

        # 從記憶體刪除
        del self.presets[name]

        # 更新檔案
        success = self.save_all_presets()

        if success:
            print(f"✓ 已刪除預設: {name}")

        return success

    def get_all_preset_names(self):
        """
        取得所有預設名稱
        Returns:
            list: 預設名稱列表
        """
        return list(self.presets.keys())

    def get_preset_count(self):
        """
        取得預設數量
        Returns:
            int: 預設數量
        """
        return len(self.presets)

    def rename_preset(self, old_name, new_name):
        """
        重新命名預設
        Args:
            old_name: 舊名稱
            new_name: 新名稱
        Returns:
            bool: 是否成功
        """
        if old_name not in self.presets:
            print(f"⚠ 預設不存在: {old_name}")
            return False

        if new_name in self.presets:
            print(f"⚠ 預設名稱已存在: {new_name}")
            return False

        # 複製資料
        self.presets[new_name] = self.presets[old_name]

        # 刪除舊資料
        del self.presets[old_name]

        # 儲存
        success = self.save_all_presets()

        if success:
            print(f"✓ 已重新命名: {old_name} -> {new_name}")

        return success

    def export_presets(self, export_file):
        """
        匯出預設到檔案
        Args:
            export_file: 匯出檔案路徑
        Returns:
            bool: 是否成功
        """
        try:
            with open(export_file, 'w', encoding='utf-8') as f:
                json.dump(self.presets, f, indent=2, ensure_ascii=False)
            print(f"✓ 已匯出預設到: {export_file}")
            return True
        except Exception as e:
            print(f"✗ 匯出失敗: {e}")
            return False

    def import_presets(self, import_file, overwrite=False):
        """
        從檔案匯入預設
        Args:
            import_file: 匯入檔案路徑
            overwrite: 是否覆蓋現有預設
        Returns:
            bool: 是否成功
        """
        try:
            with open(import_file, 'r', encoding='utf-8') as f:
                imported_presets = json.load(f)

            if overwrite:
                # 完全覆蓋
                self.presets = imported_presets
            else:
                # 合併（保留現有的，只添加新的）
                for name, data in imported_presets.items():
                    if name not in self.presets:
                        self.presets[name] = data

            # 儲存
            success = self.save_all_presets()

            if success:
                print(f"✓ 已匯入 {len(imported_presets)} 個預設")

            return success

        except Exception as e:
            print(f"✗ 匯入失敗: {e}")
            return False

    def clear_all_presets(self):
        """
        清空所有預設（謹慎使用！）
        Returns:
            bool: 是否成功
        """
        self.presets = {}
        success = self.save_all_presets()

        if success:
            print("✓ 已清空所有預設")

        return success


# ========== 測試程式 ==========

if __name__ == "__main__":
    print("=" * 60)
    print("PresetManager 測試")
    print("=" * 60)

    # 創建管理器
    manager = PresetManager()

    # 顯示所有預設
    print(f"\n預設數量: {manager.get_preset_count()}")
    print("預設列表:")
    for name in manager.get_all_preset_names():
        print(f"  - {name}")

    # 新增自訂預設
    print("\n新增自訂預設...")
    test_preset = {
        'joint_angles': [45, -30, 60, 90, -45, 180],
        'position': {'x': 100, 'y': 200, 'z': 300, 'rx': 0, 'ry': 0, 'rz': 0},
        'description': '測試姿態'
    }
    manager.save_preset('測試姿態', test_preset)

    # 載入預設
    print("\n載入預設...")
    loaded = manager.load_preset('測試姿態')
    if loaded:
        print(f"關節角度: {loaded['joint_angles']}")

    # 刪除預設
    print("\n刪除預設...")
    manager.delete_preset('測試姿態')

    print("\n" + "=" * 60)
    print("測試完成！")
    print("=" * 60)