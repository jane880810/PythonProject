#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
20250916
深度圖
HP60C 深度相機深度顯示 - PyCharm相容版本
顯示彩色深度畫面，不同距離顯示不同顏色

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
        print("將使用手動深度圖像轉換")
        USE_CV_BRIDGE = False

except ImportError as e:
    print(f"ROS模組匯入失敗: {e}")
    sys.exit(1)


# ========== 深度相機顯示類別 ==========
class PyCharmHP60CDepthViewer:
    def __init__(self):
        print("初始化PyCharm HP60C深度相機顯示器...")

        # 初始化ROS節點
        rospy.init_node('pycharm_hp60c_depth_viewer', anonymous=True)

        # 初始化cv_bridge（如果可用）
        if USE_CV_BRIDGE:
            self.bridge = CvBridge()

        # 圖像計數器
        self.frame_count = 0

        # 深度範圍設定（單位：毫米）- 根據HP60C實際有效範圍調整
        self.min_depth = 500  # 最小深度 50cm
        self.max_depth = 4000  # 最大深度 400cm

        # 訂閱深度圖像話題
        self.depth_sub = rospy.Subscriber(
            '/ascamera_hp60c/depth0/image_raw',
            Image,
            self.depth_callback,
            queue_size=1
        )

        print("PyCharm HP60C深度顯示器初始化完成")
        print("操作說明:")
        print("- 按 'q' 或 ESC 退出")
        print("- 按 's' 儲存深度圖")
        print("- 按 'i' 顯示深度資訊")
        print("- 按 '+' 增加最大深度範圍")
        print("- 按 '-' 減少最大深度範圍")

    def manual_depth_convert(self, msg):
        """手動轉換ROS深度圖像到OpenCV格式"""
        if msg.encoding == "mono16" or msg.encoding == "16UC1":
            # 16位深度圖像：每像素2位元組
            np_arr = np.frombuffer(msg.data, dtype=np.uint16)
            depth_image = np_arr.reshape((msg.height, msg.width))
            return depth_image
        elif msg.encoding == "32FC1":
            # 32位浮點深度圖像：每像素4位元組
            np_arr = np.frombuffer(msg.data, dtype=np.float32)
            depth_image = np_arr.reshape((msg.height, msg.width))
            # 轉換為毫米單位（如果原本是米）
            if np.max(depth_image) < 10:  # 假設如果最大值小於10，則單位是米
                depth_image = depth_image * 1000
            return depth_image.astype(np.uint16)
        else:
            print(f"不支援的深度編碼: {msg.encoding}")
            return None

    def colorize_depth(self, depth_image):
        """將深度圖像轉換為彩色視覺化"""
        # 建立彩色深度圖像
        colored_depth = np.zeros((depth_image.shape[0], depth_image.shape[1], 3), dtype=np.uint8)

        # 過濾無效深度值（0或過大的值）
        valid_mask = (depth_image > 0) & (depth_image < 65535)

        if np.any(valid_mask):
            # 正規化深度值到0-255範圍
            valid_depths = depth_image[valid_mask]

            # 限制深度範圍
            clipped_depths = np.clip(valid_depths, self.min_depth, self.max_depth)

            # 正規化到0-255範圍
            normalized_depths = ((clipped_depths - self.min_depth) /
                                 (self.max_depth - self.min_depth) * 255).astype(np.uint8)

            # 應用顏色映射（COLORMAP_JET：藍色=近，紅色=遠）
            colored_pixels = cv2.applyColorMap(normalized_depths.reshape(-1, 1), cv2.COLORMAP_JET)
            colored_depth[valid_mask] = colored_pixels.reshape(-1, 3)

            # 無效區域設為黑色
            colored_depth[~valid_mask] = [0, 0, 0]

        return colored_depth

    def depth_callback(self, msg):
        """處理深度圖像回調函數"""
        try:
            self.frame_count += 1

            # 轉換深度圖像 - 優化cv_bridge處理邏輯
            if USE_CV_BRIDGE:
                try:
                    # 直接使用訊息的原始編碼，避免不必要的轉換
                    if msg.encoding in ["mono16", "16UC1"]:
                        # 對於16位深度圖像，直接使用原始編碼
                        depth_image = self.bridge.imgmsg_to_cv2(msg, msg.encoding)
                        if msg.encoding == "16UC1":
                            # 16UC1實際上就是mono16，確保格式正確
                            depth_image = depth_image.astype(np.uint16)
                    elif msg.encoding == "32FC1":
                        # 32位浮點深度圖像
                        depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
                        # 轉換為毫米並轉為uint16
                        if np.max(depth_image) < 10:
                            depth_image = depth_image * 1000
                        depth_image = depth_image.astype(np.uint16)
                    else:
                        # 其他格式嘗試轉換為mono16
                        depth_image = self.bridge.imgmsg_to_cv2(msg, "mono16")
                    conversion_method = "cv_bridge"

                except Exception as e:
                    print(f"cv_bridge轉換失敗，使用手動轉換: {e}")
                    depth_image = self.manual_depth_convert(msg)
                    conversion_method = "manual"
            else:
                depth_image = self.manual_depth_convert(msg)
                conversion_method = "manual"

            if depth_image is None:
                print(f"深度圖像轉換失敗，編碼格式: {msg.encoding}")
                return

            # 轉換為彩色深度圖像
            colored_depth = self.colorize_depth(depth_image)

            # 新增資訊覆蓋層
            self.add_info_overlay(colored_depth, msg, conversion_method, depth_image)

            # 顯示深度圖像
            cv2.imshow('PyCharm HP60C Depth Camera', colored_depth)

            # 自動儲存第一幀用於驗證
            if self.frame_count == 1:
                cv2.imwrite('/tmp/pycharm_hp60c_first_depth.jpg', colored_depth)
                print("已儲存第一幀深度圖到 /tmp/pycharm_hp60c_first_depth.jpg")
                print(f"深度圖像編碼: {msg.encoding}, 形狀: {depth_image.shape}")

            # 處理按鍵輸入
            self.handle_keyboard(colored_depth, depth_image, msg)

        except Exception as e:
            print(f"深度圖像處理錯誤: {e}")
            import traceback
            traceback.print_exc()

    def add_info_overlay(self, image, msg, conversion_method, depth_image):
        """新增資訊覆蓋層"""
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        color = (255, 255, 255)  # 白色文字在深度圖上更清楚
        thickness = 2

        # 計算深度統計資訊
        valid_depths = depth_image[depth_image > 0]
        if len(valid_depths) > 0:
            min_valid = np.min(valid_depths)
            max_valid = np.max(valid_depths)
            mean_depth = np.mean(valid_depths)
        else:
            min_valid = max_valid = mean_depth = 0

        # 資訊文字
        info_lines = [
            f"PyCharm HP60C Depth Camera",
            f"Size: {msg.width}x{msg.height}",
            f"Encoding: {msg.encoding}",
            f"Method: {conversion_method}",
            f"Frame: {self.frame_count}",
            f"Range: {self.min_depth}-{self.max_depth}mm",
            f"Depth: {min_valid:.0f}-{max_valid:.0f}mm",
            f"Mean: {mean_depth:.0f}mm",
            f"Press 'q':exit, 's':save, '+/-':range"
        ]

        # 繪製半透明背景
        overlay = image.copy()
        cv2.rectangle(overlay, (10, 10), (420, 200), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, image, 0.3, 0, image)

        # 繪製文字
        for i, line in enumerate(info_lines):
            y_pos = 30 + i * 20
            cv2.putText(image, line, (20, y_pos), font, font_scale, color, thickness)

        # 繪製深度顏色條
        self.draw_depth_colorbar(image)

    def draw_depth_colorbar(self, image):
        """繪製深度顏色條"""
        # 建立顏色條
        colorbar_width = 20
        colorbar_height = 150
        x_start = image.shape[1] - colorbar_width - 20
        y_start = 30

        # 建立深度值陣列
        depth_values = np.linspace(0, 255, colorbar_height).astype(np.uint8)
        colorbar = cv2.applyColorMap(depth_values.reshape(-1, 1), cv2.COLORMAP_JET)
        colorbar = colorbar.reshape(colorbar_height, 1, 3)
        colorbar = np.repeat(colorbar, colorbar_width, axis=1)

        # 繪製顏色條
        image[y_start:y_start + colorbar_height, x_start:x_start + colorbar_width] = colorbar

        # 添加標籤
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.4
        color = (255, 255, 255)
        thickness = 1

        # 近距離標籤（藍色）
        cv2.putText(image, f"{self.min_depth}mm",
                    (x_start - 60, y_start + 15), font, font_scale, color, thickness)
        cv2.putText(image, "Near",
                    (x_start - 40, y_start + 30), font, font_scale, color, thickness)

        # 遠距離標籤（紅色）
        cv2.putText(image, f"{self.max_depth}mm",
                    (x_start - 60, y_start + colorbar_height - 5), font, font_scale, color, thickness)
        cv2.putText(image, "Far",
                    (x_start - 30, y_start + colorbar_height + 15), font, font_scale, color, thickness)

    def handle_keyboard(self, colored_depth, depth_image, msg):
        """處理鍵盤輸入"""
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q') or key == 27:  # 'q' 或 ESC
            print("退出程式...")
            self.shutdown()
        elif key == ord('s'):  # 儲存深度圖
            self.save_depth_image(colored_depth, depth_image)
        elif key == ord('i'):  # 顯示深度資訊
            self.show_depth_info(depth_image, msg)
        elif key == ord('+') or key == ord('='):  # 增加最大深度範圍
            self.max_depth = min(self.max_depth + 500, 10000)
            print(f"最大深度範圍調整為: {self.max_depth}mm")
        elif key == ord('-'):  # 減少最大深度範圍
            self.max_depth = max(self.max_depth - 500, self.min_depth + 500)
            print(f"最大深度範圍調整為: {self.max_depth}mm")

    def save_depth_image(self, colored_depth, depth_image):
        """儲存深度圖像"""
        timestamp = rospy.Time.now().to_sec()

        # 儲存彩色深度圖
        color_filename = f'/tmp/pycharm_hp60c_depth_color_{timestamp:.0f}.jpg'
        if cv2.imwrite(color_filename, colored_depth):
            print(f"彩色深度圖已儲存: {color_filename}")

        # 儲存原始深度資料
        raw_filename = f'/tmp/pycharm_hp60c_depth_raw_{timestamp:.0f}.png'
        if cv2.imwrite(raw_filename, depth_image):
            print(f"原始深度資料已儲存: {raw_filename}")

    def show_depth_info(self, depth_image, msg):
        """顯示詳細深度資訊"""
        valid_depths = depth_image[depth_image > 0]

        print("\n" + "=" * 50)
        print("深度圖像詳細資訊:")
        print(f"  ROS訊息尺寸: {msg.width} x {msg.height}")
        print(f"  OpenCV形狀: {depth_image.shape}")
        print(f"  編碼格式: {msg.encoding}")
        print(f"  資料型別: {depth_image.dtype}")
        print(f"  設定深度範圍: {self.min_depth}mm - {self.max_depth}mm")

        if len(valid_depths) > 0:
            print(f"  有效像素數: {len(valid_depths)} / {depth_image.size}")
            print(f"  實際深度範圍: {np.min(valid_depths):.0f}mm - {np.max(valid_depths):.0f}mm")
            print(f"  平均深度: {np.mean(valid_depths):.0f}mm")
            print(f"  中位數深度: {np.median(valid_depths):.0f}mm")
            print(f"  深度標準差: {np.std(valid_depths):.0f}mm")
        else:
            print("  無有效深度資料")

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
        print("啟動PyCharm HP60C深度相機顯示程式...")

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

        # 建立並執行深度顯示器
        viewer = PyCharmHP60CDepthViewer()

        print("程式執行中，等待深度圖像資料...")
        print("在PyCharm中成功執行HP60C深度相機顯示!")
        print("顏色說明：藍色=近距離，綠色=中距離，紅色=遠距離")

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