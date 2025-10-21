#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
20250916
繁體中文
HP60C 深度相機顯示 - PyCharm相容版本
解決cv_bridge庫載入問題

"""

import sys
import os
import subprocess
import ctypes
import numpy as np
import cv2


# ========== 環境設定 ==========
def setup_ros_environment():
    """設定ROS環境"""
    print("正在設定ROS環境...")

    # 設定環境變數
    os.environ['ROS_MASTER_URI'] = 'http://192.168.40.128:11311'
    os.environ['ROS_IP'] = '192.168.40.128'

    # 設定日誌設定以消除警告
    os.environ['ROSCONSOLE_CONFIG_FILE'] = '/opt/ros/noetic/etc/ros/rosconsole.config'

    # 新增ROS路徑到Python path
    ros_paths = [
        "/opt/ros/noetic/lib/python3/dist-packages",
        "/home/yahboom/ascam_ws/devel/lib/python3/dist-packages"
    ]

    for path in ros_paths:
        if os.path.exists(path) and path not in sys.path:
            sys.path.insert(0, path)
            print(f"已新增路徑: {path}")

    # 設定庫路徑
    lib_paths = [
        "/opt/ros/noetic/lib",
        "/usr/lib/x86_64-linux-gnu"
    ]

    current_ld_path = os.environ.get('LD_LIBRARY_PATH', '')
    new_ld_path = ':'.join(lib_paths)
    if current_ld_path:
        new_ld_path = new_ld_path + ':' + current_ld_path
    os.environ['LD_LIBRARY_PATH'] = new_ld_path

    print("ROS環境設定完成")


def load_shared_libraries():
    """預先載入必要的共享庫"""
    try:
        # 嘗試載入cv_bridge相關庫
        lib_paths = [
            '/opt/ros/noetic/lib/libcv_bridge.so',
            '/opt/ros/noetic/lib/libopencv_core.so',
            '/opt/ros/noetic/lib/libopencv_imgproc.so'
        ]

        for lib_path in lib_paths:
            if os.path.exists(lib_path):
                try:
                    ctypes.CDLL(lib_path)
                    print(f"成功載入庫: {lib_path}")
                except Exception as e:
                    print(f"庫載入警告 {lib_path}: {e}")
    except Exception as e:
        print(f"庫載入過程出現問題: {e}")


# 設定環境
setup_ros_environment()
load_shared_libraries()

# ========== 匯入ROS模組 ==========
try:
    import rospy
    from sensor_msgs.msg import Image

    print("ROS模組匯入成功")

    # 嘗試匯入cv_bridge，如果失敗則使用手動轉換
    try:
        from cv_bridge import CvBridge

        USE_CV_BRIDGE = True
        print("cv_bridge匯入成功")
    except Exception as e:
        print(f"cv_bridge匯入失敗: {e}")
        print("將使用手動圖像轉換")
        USE_CV_BRIDGE = False

except ImportError as e:
    print(f"ROS模組匯入失敗: {e}")
    sys.exit(1)


# ========== 相機顯示類別 ==========
class PyCharmHP60CViewer:
    def __init__(self):
        print("初始化PyCharm HP60C相機顯示器...")

        # 初始化ROS節點
        rospy.init_node('pycharm_hp60c_viewer', anonymous=True)

        # 初始化cv_bridge（如果可用）
        if USE_CV_BRIDGE:
            self.bridge = CvBridge()

        # 圖像計數器
        self.frame_count = 0
        self.save_next_frame = False

        # 訂閱RGB圖像話題
        self.image_sub = rospy.Subscriber(
            '/ascamera_hp60c/rgb0/image',
            Image,
            self.image_callback,
            queue_size=1
        )

        print("PyCharm HP60C顯示器初始化完成")
        print("操作說明:")
        print("- 按 'q' 或 ESC 退出")
        print("- 按 's' 儲存截圖")
        print("- 按 'i' 顯示圖像資訊")

    def manual_image_convert(self, msg):
        """手動轉換ROS Image到OpenCV格式"""
        if msg.encoding == "bgr8":
            # BGR8格式：每像素3位元組
            np_arr = np.frombuffer(msg.data, dtype=np.uint8)
            cv_image = np_arr.reshape((msg.height, msg.width, 3))
            return cv_image
        elif msg.encoding == "rgb8":
            # RGB8格式：轉換為BGR
            np_arr = np.frombuffer(msg.data, dtype=np.uint8)
            rgb_image = np_arr.reshape((msg.height, msg.width, 3))
            cv_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
            return cv_image
        else:
            print(f"不支援的圖像編碼: {msg.encoding}")
            return None

    def image_callback(self, msg):
        """處理圖像回調函數"""
        try:
            self.frame_count += 1

            # 轉換圖像
            if USE_CV_BRIDGE:
                try:
                    cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                    conversion_method = "cv_bridge"
                except Exception as e:
                    print(f"cv_bridge轉換失敗，使用手動轉換: {e}")
                    cv_image = self.manual_image_convert(msg)
                    conversion_method = "manual"
            else:
                cv_image = self.manual_image_convert(msg)
                conversion_method = "manual"

            if cv_image is None:
                return

            # 新增資訊覆蓋層
            self.add_info_overlay(cv_image, msg, conversion_method)

            # 顯示圖像
            cv2.imshow('PyCharm HP60C Camera', cv_image)

            # 自動儲存第一幀用於驗證
            if self.frame_count == 1:
                cv2.imwrite('/tmp/pycharm_hp60c_first_frame.jpg', cv_image)
                print("已儲存第一幀到 /tmp/pycharm_hp60c_first_frame.jpg")

            # 處理按鍵輸入
            self.handle_keyboard(cv_image, msg)

        except Exception as e:
            print(f"圖像處理錯誤: {e}")
            import traceback
            traceback.print_exc()

    def add_info_overlay(self, image, msg, conversion_method):
        """新增資訊覆蓋層"""
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        color = (0, 255, 0)
        thickness = 2

        # 資訊文字
        info_lines = [
            f"PyCharm HP60C Camera",
            f"Size: {msg.width}x{msg.height}",
            f"Encoding: {msg.encoding}",
            f"Method: {conversion_method}",
            f"Frame: {self.frame_count}",
            f"Press 'q' to exit, 's' to save"
        ]

        # 繪製半透明背景
        overlay = image.copy()
        cv2.rectangle(overlay, (10, 10), (350, 160), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, image, 0.3, 0, image)

        # 繪製文字
        for i, line in enumerate(info_lines):
            y_pos = 30 + i * 20
            cv2.putText(image, line, (20, y_pos), font, font_scale, color, thickness)

    def handle_keyboard(self, cv_image, msg):
        """處理鍵盤輸入"""
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q') or key == 27:  # 'q' 或 ESC
            print("退出程式...")
            self.shutdown()
        elif key == ord('s'):  # 儲存截圖
            self.save_screenshot(cv_image)
        elif key == ord('i'):  # 顯示圖像資訊
            self.show_image_info(cv_image, msg)

    def save_screenshot(self, cv_image):
        """儲存截圖"""
        timestamp = rospy.Time.now().to_sec()
        filename = f'/tmp/pycharm_hp60c_screenshot_{timestamp:.0f}.jpg'

        if cv2.imwrite(filename, cv_image):
            print(f"截圖已儲存: {filename}")
        else:
            print("截圖儲存失敗!")

    def show_image_info(self, cv_image, msg):
        """顯示詳細圖像資訊"""
        print("\n" + "=" * 50)
        print("圖像詳細資訊:")
        print(f"  ROS訊息尺寸: {msg.width} x {msg.height}")
        print(f"  OpenCV形狀: {cv_image.shape}")
        print(f"  編碼格式: {msg.encoding}")
        print(f"  資料型別: {cv_image.dtype}")
        print(f"  像素值範圍: {cv_image.min()} - {cv_image.max()}")
        print(f"  記憶體大小: {cv_image.nbytes} bytes")
        print(f"  目前幀數: {self.frame_count}")
        print("=" * 50 + "\n")

    def shutdown(self):
        """安全關閉程式"""
        cv2.destroyAllWindows()
        rospy.signal_shutdown("使用者關閉")


# ========== 主程式 ==========
def main():
    """主程式進入點"""
    try:
        print("啟動PyCharm HP60C相機顯示程式...")

        # 檢查ROS連線
        try:
            rospy.get_master().getPid()
            print("ROS Master連線正常")
        except:
            print("無法連線到ROS Master!")
            print("請確保:")
            print("1. roscore正在執行")
            print("2. 相機驅動程式已啟動")
            return

        # 建立並執行顯示器
        viewer = PyCharmHP60CViewer()

        print("程式執行中，等待圖像資料...")
        print("在PyCharm中成功執行HP60C相機顯示!")

        # 保持程式執行
        rospy.spin()

    except rospy.ROSInterruptException:
        print("ROS連線中斷")
    except KeyboardInterrupt:
        print("偵測到Ctrl+C")
    except Exception as e:
        print(f"程式錯誤: {e}")
        import traceback
        traceback.print_exc()
    finally:
        cv2.destroyAllWindows()
        print("程式安全關閉")


if __name__ == '__main__':
    main()