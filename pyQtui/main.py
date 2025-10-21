"""
main.py
主程式入口 - VTK 嵌入版本
"""

import sys
import os
from PyQt5.QtWidgets import QApplication
from PyQt5.QtGui import QIcon

# 添加專案根目錄到 Python 路徑
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, project_root)

# 匯入控制器和GUI
from core.controller import AnimationController
from gui.main_window import RobotMainWindow


def setup_logging():
    """設置日誌系統"""
    import logging
    from datetime import datetime

    # 確保日誌目錄存在
    log_dir = os.path.join(project_root, 'logs')
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)

    # 設置日誌檔案名稱
    log_file = os.path.join(log_dir, f'robot_control_{datetime.now():%Y%m%d_%H%M%S}.log')

    # 配置日誌
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler(log_file, encoding='utf-8'),
            logging.StreamHandler()
        ]
    )

    return logging.getLogger(__name__)


def check_dependencies():
    """檢查必要的依賴套件"""
    required_packages = {
        'PyQt5': 'PyQt5',
        'numpy': 'numpy',
        'vtk': 'vtk (選用，用於3D視覺化)',
    }

    missing_packages = []

    for package, description in required_packages.items():
        try:
            __import__(package)
            print(f"  ✓ {description}")
        except ImportError:
            if package == 'vtk':  # VTK 是選用的
                print(f"  ⚠ {description} - 3D視覺化將不可用")
            else:
                missing_packages.append(description)
                print(f"  ✗ {description}")

    if missing_packages:
        print("\n請安裝缺少的套件：")
        print(f"  pip install {' '.join([p.split()[0] for p in missing_packages])}")
        return False

    return True


def main():
    """主程式"""
    print("=" * 60)
    print("RA605-710-GC 六軸機械手臂控制系統")
    print("=" * 60)
    print()

    # 設置日誌
    logger = setup_logging()
    logger.info("系統啟動")

    # 檢查依賴
    print("檢查系統依賴...")
    if not check_dependencies():
        print("\n系統缺少必要的依賴套件，請先安裝。")
        input("按 Enter 鍵退出...")
        return 1
    print()

    try:
        # 初始化控制器
        print("正在初始化控制器...")
        ctrl = AnimationController()
        print("✓ 控制器初始化完成")
        logger.info("控制器初始化成功")
        print()

        # 創建 Qt 應用程式
        print("正在啟動圖形介面...")
        app = QApplication(sys.argv)
        app.setApplicationName("RA605-710-GC 控制系統")
        app.setOrganizationName("Robot Control")

        # 設置應用程式圖標（如果存在）
        icon_path = os.path.join(project_root, 'assets', 'robot_icon.png')
        if os.path.exists(icon_path):
            app.setWindowIcon(QIcon(icon_path))

        # 設置樣式（選用）
        app.setStyle('Fusion')  # 使用 Fusion 風格

        # 創建並顯示主視窗（VTK 已嵌入其中）
        window = RobotMainWindow(ctrl)
        window.show()

        print()
        print("=" * 60)
        print("✓ 系統啟動完成！")
        print("=" * 60)
        print()
        print("系統功能：")
        print("  ✓ 3D 視覺化（VTK 嵌入式）")
        print("  ✓ 關節控制（6軸獨立控制）")
        print("  ✓ 位置控制（正/逆向運動學）")
        print("  ✓ 軌跡規劃（點對點、直線、圓弧）")
        print("  ✓ 姿態管理（預設姿態儲存/載入）")
        print("  ✓ 系統監控（即時狀態顯示）")
        print("  ✓ 緊急停止（安全保護）")
        print("=" * 60)
        print()
        print("提示：")
        print("  • 雙擊預設姿態可快速載入")
        print("  • 使用滑桿或微調按鈕控制關節")
        print("  • 支援軌跡點拖放排序")
        print("  • 按 F11 切換全螢幕模式")
        print("=" * 60)
        print()

        logger.info("GUI 啟動成功")

        # 運行應用程式
        exit_code = app.exec_()

        # 程式結束
        print()
        print("正在關閉系統...")
        logger.info(f"系統正常關閉，退出碼: {exit_code}")
        print("✓ 系統已安全關閉")

        return exit_code

    except Exception as e:
        print()
        print("=" * 60)
        print("✗ 系統啟動失敗！")
        print(f"錯誤訊息: {str(e)}")
        print("=" * 60)

        logger.error(f"系統啟動失敗: {str(e)}", exc_info=True)

        # 如果是匯入錯誤，提供更詳細的說明
        if isinstance(e, ImportError):
            print("\n可能的原因：")
            print("  1. 檔案路徑不正確")
            print("  2. 模組檔案不存在")
            print("  3. 循環匯入問題")
            print("\n請檢查以下檔案是否存在：")
            print(f"  • {os.path.join(project_root, 'core', 'controller.py')}")
            print(f"  • {os.path.join(project_root, 'gui', 'main_window.py')}")

        print("\n詳細錯誤資訊已記錄到日誌檔案。")
        input("按 Enter 鍵退出...")
        return 1


def run_with_error_handler():
    """帶錯誤處理的執行包裝器"""
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n\n使用者中斷程式執行")
        sys.exit(0)
    except SystemExit:
        raise
    except Exception as e:
        print(f"\n未預期的錯誤: {str(e)}")
        import traceback
        traceback.print_exc()
        input("\n按 Enter 鍵退出...")
        sys.exit(1)


if __name__ == "__main__":
    # 設置環境變數（如果需要）
    os.environ['QT_AUTO_SCREEN_SCALE_FACTOR'] = '1'  # 自動縮放

    # 執行主程式
    run_with_error_handler()