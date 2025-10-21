"""
utils/preset_manager.py
姿態管理模組
"""

import json
import os
from datetime import datetime
from config.robot_config import DEFAULT_POSES


class PresetManager:
    """姿態管理器"""

    def __init__(self, filename='robot_presets.json'):
        self.filename = filename
        self.presets = self.load_presets()

    def load_presets(self):
        """載入姿態檔案"""
        if os.path.exists(self.filename):
            try:
                with open(self.filename, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                    # 合併預設姿態
                    return {**DEFAULT_POSES, **data}
            except Exception as e:
                print(f"載入姿態失敗: {e}")
                return DEFAULT_POSES.copy()
        return DEFAULT_POSES.copy()

    def save_presets(self):
        """儲存姿態檔案"""
        try:
            # 只儲存使用者自訂的姿態（排除預設姿態）
            user_presets = {
                k: v for k, v in self.presets.items()
                if k not in DEFAULT_POSES
            }

            with open(self.filename, 'w', encoding='utf-8') as f:
                json.dump(user_presets, f, indent=2, ensure_ascii=False)
            return True
        except Exception as e:
            print(f"儲存姿態失敗: {e}")
            return False

    def add_preset(self, name, angles, metadata=None):
        """
        新增姿態

        Args:
            name: str, 姿態名稱
            angles: list, 關節角度
            metadata: dict, 額外資訊（描述、創建時間等）
        """
        if name in DEFAULT_POSES:
            raise ValueError(f"'{name}' 是預設姿態，無法覆寫")

        preset_data = {
            'angles': angles,
            'created_at': datetime.now().isoformat(),
        }

        if metadata:
            preset_data.update(metadata)

        self.presets[name] = angles
        return self.save_presets()

    def get_preset(self, name):
        """取得姿態"""
        return self.presets.get(name)

    def delete_preset(self, name):
        """刪除姿態"""
        if name in DEFAULT_POSES:
            raise ValueError(f"'{name}' 是預設姿態，無法刪除")

        if name in self.presets:
            del self.presets[name]
            return self.save_presets()
        return False

    def rename_preset(self, old_name, new_name):
        """重新命名姿態"""
        if old_name in DEFAULT_POSES:
            raise ValueError(f"'{old_name}' 是預設姿態，無法重新命名")

        if new_name in DEFAULT_POSES:
            raise ValueError(f"'{new_name}' 是預設姿態名稱")

        if old_name in self.presets:
            self.presets[new_name] = self.presets.pop(old_name)
            return self.save_presets()
        return False

    def get_all_preset_names(self):
        """取得所有姿態名稱"""
        return list(self.presets.keys())

    def export_presets(self, filename):
        """匯出姿態到檔案"""
        try:
            with open(filename, 'w', encoding='utf-8') as f:
                json.dump(self.presets, f, indent=2, ensure_ascii=False)
            return True
        except Exception as e:
            print(f"匯出失敗: {e}")
            return False

    def import_presets(self, filename, merge=True):
        """
        匯入姿態

        Args:
            filename: 檔案路徑
            merge: bool, True=合併, False=覆蓋
        """
        try:
            with open(filename, 'r', encoding='utf-8') as f:
                imported = json.load(f)

            if merge:
                self.presets.update(imported)
            else:
                self.presets = {**DEFAULT_POSES, **imported}

            return self.save_presets()
        except Exception as e:
            print(f"匯入失敗: {e}")
            return False

    def is_default_preset(self, name):
        """檢查是否為預設姿態"""
        return name in DEFAULT_POSES

    def search_presets(self, keyword):
        """搜尋姿態"""
        keyword = keyword.lower()
        return [
            name for name in self.presets.keys()
            if keyword in name.lower()
        ]