#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
20250916
3種濾波比較與切換
HP60C 最終工作版本 - 修復視窗顯示錯誤
包含雙邊濾波和邊緣增強功能

"""

import sys
import os
import numpy as np
import cv2

# 設定ROS環境
ros_paths = [
    "/opt/ros/noetic/lib/python3/dist-packages",
    "/home/yahboom/ascam_ws/devel/lib/python3/dist-packages"
]

for path in ros_paths:
    if os.path.exists(path) and path not in sys.path:
        sys.path.insert(0, path)

os.environ['ROS_MASTER_URI'] = 'http://192.168.40.128:11311'
os.environ['ROS_IP'] = '192.168.40.128'
os.environ['DISPLAY'] = ':0.0'

try:
    import rospy
    from sensor_msgs.msg import Image

    print("ROS模組載入成功")
except ImportError as e:
    print(f"ROS模組載入失敗: {e}")
    sys.exit(1)


class FinalWorkingViewer:
    def __init__(self):
        print("初始化HP60C最終工作版本...")

        rospy.init_node('final_working_viewer', anonymous=True)
        self.frame_count = 0
        self.filter_mode = 0  # 0=原始, 1=雙邊濾波, 2=邊緣增強
        self.window_created = False
        self.window_name = 'HP60C 深度相機 - 濾波增強版'

        # 找深度話題
        topics = rospy.get_published_topics()
        depth_topics = [topic for topic, _ in topics if 'depth' in topic.lower() and 'image' in topic]

        if not depth_topics:
            print("未找到深度話題！")
            return

        self.depth_topic = depth_topics[0]
        print(f"使用話題: {self.depth_topic}")

        # 訂閱深度圖像
        self.depth_sub = rospy.Subscriber(
            self.depth_topic,
            Image,
            self.depth_callback,
            queue_size=1
        )

        print("最終工作版本初始化完成")
        print("操作說明:")
        print("- 按 1: 原始深度圖")
        print("- 按 2: 雙邊濾波（保邊去雜訊）")
        print("- 按 3: 邊緣增強（增加清晰度）")
        print("- 按 s: 儲存當前圖像")
        print("- 按 q: 退出程式")

    def apply_bilateral_filter(self, depth_image):
        """雙邊濾波 - 保持邊緣清晰，減少雜訊"""
        try:
            valid_mask = (depth_image > 500) & (depth_image < 4000)

            if not np.any(valid_mask):
                return depth_image.copy()

            # 轉換為8位元進行濾波
            depth_8bit = np.zeros_like(depth_image, dtype=np.uint8)
            valid_depths = depth_image[valid_mask]
            clipped = np.clip(valid_depths, 500, 4000)
            norm_values = ((clipped - 500) / 3500 * 255).astype(np.uint8)
            depth_8bit[valid_mask] = norm_values

            # 應用雙邊濾波
            filtered_8bit = cv2.bilateralFilter(depth_8bit, 9, 50, 50)

            # 轉換回16位元
            filtered_depth = depth_image.copy()
            filtered_values = (filtered_8bit[valid_mask].astype(np.float32) / 255 * 3500 + 500).astype(np.uint16)
            filtered_depth[valid_mask] = filtered_values

            return filtered_depth

        except Exception as e:
            print(f"雙邊濾波錯誤: {e}")
            return depth_image.copy()

    def apply_edge_enhancement(self, depth_image):
        """邊緣增強 - 增加邊緣清晰度"""
        try:
            valid_mask = (depth_image > 500) & (depth_image < 4000)

            if not np.any(valid_mask):
                return depth_image.copy()

            # 轉換為浮點數進行處理
            depth_float = depth_image.astype(np.float32)
            depth_float[~valid_mask] = 0

            # 高斯模糊
            blurred = cv2.GaussianBlur(depth_float, (5, 5), 0)

            # 非銳化遮罩：原圖 + 1.5 * (原圖 - 模糊圖)
            enhanced = depth_float + 1.5 * (depth_float - blurred)

            # 限制範圍並轉換回整數
            enhanced = np.clip(enhanced, 0, 65535)
            enhanced_depth = enhanced.astype(np.uint16)
            enhanced_depth[~valid_mask] = 0

            return enhanced_depth

        except Exception as e:
            print(f"邊緣增強錯誤: {e}")
            return depth_image.copy()

    def colorize_depth(self, depth_image):
        """轉換為彩色顯示"""
        try:
            height, width = depth_image.shape
            colored = np.zeros((height, width, 3), dtype=np.uint8)

            valid_mask = (depth_image > 500) & (depth_image < 4000)

            if np.any(valid_mask):
                # 正規化深度值
                valid_depths = depth_image[valid_mask]
                clipped = np.clip(valid_depths, 500, 4000)
                normalized = ((clipped - 500) / 3500 * 255).astype(np.uint8)

                # 應用顏色映射
                colored_pixels = cv2.applyColorMap(normalized.reshape(-1, 1), cv2.COLORMAP_JET)
                colored[valid_mask] = colored_pixels.reshape(-1, 3)

            return colored

        except Exception as e:
            print(f"顏色轉換錯誤: {e}")
            return np.zeros((depth_image.shape[0], depth_image.shape[1], 3), dtype=np.uint8)

    def safe_display(self, image):
        """安全的顯示方法"""
        try:
            # 只在第一次建立視窗
            if not self.window_created:
                cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)
                cv2.moveWindow(self.window_name, 100, 100)
                self.window_created = True
                print(f"視窗已建立: {self.window_name}")

            # 顯示圖像
            cv2.imshow(self.window_name, image)

        except Exception as e:
            print(f"顯示錯誤: {e}")

    def add_info_overlay(self, image):
        """添加資訊覆蓋層"""
        try:
            font = cv2.FONT_HERSHEY_SIMPLEX

            # 模式名稱
            mode_names = ["原始深度圖", "雙邊濾波", "邊緣增強"]
            current_mode = mode_names[self.filter_mode] if self.filter_mode < len(mode_names) else "未知模式"

            info_lines = [
                f"HP60C 深度相機濾波增強版",
                f"模式: {current_mode}",
                f"幀數: {self.frame_count}",
                f"操作: 1=原始 2=濾波 3=增強 s=儲存 q=退出"
            ]

            # 繪製半透明背景
            overlay = image.copy()
            cv2.rectangle(overlay, (10, 10), (500, 110), (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.7, image, 0.3, 0, image)

            # 繪製文字
            for i, line in enumerate(info_lines):
                y_pos = 30 + i * 20
                cv2.putText(image, line, (20, y_pos), font, 0.6, (255, 255, 255), 2)

        except Exception as e:
            print(f"資訊覆蓋錯誤: {e}")

    def depth_callback(self, msg):
        """處理深度圖像回調"""
        try:
            self.frame_count += 1

            # 轉換深度數據
            if msg.encoding in ["mono16", "16UC1"]:
                np_arr = np.frombuffer(msg.data, dtype=np.uint16)
                depth_image = np_arr.reshape((msg.height, msg.width))
            else:
                print(f"不支援的編碼: {msg.encoding}")
                return

            # 根據模式處理深度圖像
            if self.filter_mode == 1:
                processed_depth = self.apply_bilateral_filter(depth_image)
            elif self.filter_mode == 2:
                processed_depth = self.apply_edge_enhancement(depth_image)
            else:
                processed_depth = depth_image.copy()

            # 轉換為彩色
            colored_depth = self.colorize_depth(processed_depth)

            # 添加資訊
            self.add_info_overlay(colored_depth)

            # 安全顯示
            self.safe_display(colored_depth)

            # 處理按鍵
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                print("退出程式...")
                self.shutdown()
            elif key == ord('1'):
                self.filter_mode = 0
                print("切換到原始深度圖模式")
            elif key == ord('2'):
                self.filter_mode = 1
                print("切換到雙邊濾波模式（保邊去雜訊）")
            elif key == ord('3'):
                self.filter_mode = 2
                print("切換到邊緣增強模式（增加清晰度）")
            elif key == ord('s'):
                self.save_image(colored_depth, processed_depth)

        except Exception as e:
            print(f"處理錯誤: {e}")
            import traceback
            traceback.print_exc()

    def save_image(self, colored_depth, raw_depth):
        """儲存圖像"""
        try:
            import time
            timestamp = time.time()
            mode_names = ["original", "bilateral", "enhanced"]
            mode_name = mode_names[self.filter_mode] if self.filter_mode < len(mode_names) else "unknown"

            # 儲存彩色圖像
            color_filename = f'/tmp/hp60c_{mode_name}_color_{timestamp:.0f}.jpg'
            if cv2.imwrite(color_filename, colored_depth):
                print(f"彩色圖像已儲存: {color_filename}")

            # 儲存原始深度資料
            raw_filename = f'/tmp/hp60c_{mode_name}_raw_{timestamp:.0f}.png'
            if cv2.imwrite(raw_filename, raw_depth):
                print(f"原始深度資料已儲存: {raw_filename}")

        except Exception as e:
            print(f"儲存錯誤: {e}")

    def shutdown(self):
        """安全關閉"""
        try:
            if self.window_created:
                cv2.destroyWindow(self.window_name)
            cv2.destroyAllWindows()
        except:
            pass
        rospy.signal_shutdown("使用者關閉")


def main():
    try:
        print("啟動HP60C最終工作版本...")

        # 檢查ROS連線
        try:
            rospy.get_master().getPid()
            print("ROS Master連線正常")
        except:
            print("無法連線到ROS Master!")
            return

        # 建立最終顯示器
        viewer = FinalWorkingViewer()

        print("\n程式執行中...")
        print("根據你的需求建議:")
        print("- 模式2（雙邊濾波）：保持邊緣清晰，減少雜訊")
        print("- 模式3（邊緣增強）：增加邊緣銳利度和對比度")

        # 保持程式執行
        rospy.spin()

    except rospy.ROSInterruptException:
        print("ROS連線中斷")
    except KeyboardInterrupt:
        print("偵測到Ctrl+C")
    except Exception as e:
        print(f"程式錯誤: {e}")
    finally:
        cv2.destroyAllWindows()
        print("程式安全關閉")


if __name__ == '__main__':
    main()