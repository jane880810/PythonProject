#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
20250916
簡單濾波測試
HP60C 簡化調試版本
專門用於診斷連接問題
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

try:
    import rospy
    from sensor_msgs.msg import Image

    print("✅ ROS模組載入成功")
except ImportError as e:
    print(f"❌ ROS模組載入失敗: {e}")
    sys.exit(1)


class SimpleDebugViewer:
    def __init__(self):
        print("🔍 初始化簡化調試版本...")

        rospy.init_node('simple_debug_viewer', anonymous=True)
        self.frame_count = 0
        self.data_received = False

        # 檢查可用話題
        print("📡 檢查可用話題...")
        topics = rospy.get_published_topics()
        depth_topics = [topic for topic, _ in topics if 'depth' in topic.lower() and 'image' in topic]

        print("找到的深度話題:")
        for topic in depth_topics:
            print(f"  {topic}")

        if not depth_topics:
            print("❌ 未找到深度話題！請確認相機驅動是否啟動")
            return

        # 使用找到的第一個深度話題
        self.depth_topic = depth_topics[0]
        print(f"📺 使用話題: {self.depth_topic}")

        # 訂閱深度圖像
        self.depth_sub = rospy.Subscriber(
            self.depth_topic,
            Image,
            self.debug_callback,
            queue_size=1
        )

        print("⏳ 等待深度數據...")
        print("如果10秒內看不到 '收到數據' 訊息，表示相機驅動有問題")

    def debug_callback(self, msg):
        """調試回調函數"""
        try:
            self.frame_count += 1
            self.data_received = True

            print(f"✅ 收到深度數據 #{self.frame_count}")
            print(f"   尺寸: {msg.width}x{msg.height}")
            print(f"   編碼: {msg.encoding}")
            print(f"   數據長度: {len(msg.data)} bytes")

            # 手動轉換數據（避免cv_bridge問題）
            if msg.encoding in ["mono16", "16UC1"]:
                np_arr = np.frombuffer(msg.data, dtype=np.uint16)
                depth_image = np_arr.reshape((msg.height, msg.width))

                print(f"   深度範圍: {np.min(depth_image)} - {np.max(depth_image)}")

                # 建立簡單的視覺化
                valid_mask = (depth_image > 500) & (depth_image < 4000)

                if np.any(valid_mask):
                    # 正規化並轉換為彩色
                    normalized = np.zeros_like(depth_image, dtype=np.uint8)
                    valid_depths = depth_image[valid_mask]
                    clipped = np.clip(valid_depths, 500, 4000)
                    norm_values = ((clipped - 500) / 3500 * 255).astype(np.uint8)
                    normalized[valid_mask] = norm_values

                    # 應用顏色映射
                    colored = cv2.applyColorMap(normalized, cv2.COLORMAP_JET)

                    # 添加簡單資訊
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    cv2.putText(colored, f"Frame: {self.frame_count}", (10, 30),
                                font, 0.7, (255, 255, 255), 2)
                    cv2.putText(colored, f"Valid pixels: {np.sum(valid_mask)}", (10, 60),
                                font, 0.7, (255, 255, 255), 2)
                    cv2.putText(colored, "Press 'q' to quit", (10, 90),
                                font, 0.7, (255, 255, 255), 2)

                    # 顯示圖像
                    cv2.imshow('HP60C Debug - Depth', colored)

                    # 只在前幾幀儲存測試圖像
                    if self.frame_count <= 3:
                        test_file = f'/tmp/debug_frame_{self.frame_count}.jpg'
                        cv2.imwrite(test_file, colored)
                        print(f"   已儲存測試圖像: {test_file}")
                else:
                    print("   ⚠️ 警告: 沒有有效的深度數據")
                    # 顯示黑色圖像，但有文字說明
                    black_img = np.zeros((msg.height, msg.width, 3), dtype=np.uint8)
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    cv2.putText(black_img, "No valid depth data", (50, msg.height // 2),
                                font, 1, (0, 0, 255), 2)
                    cv2.imshow('HP60C Debug - Depth', black_img)
            else:
                print(f"   ⚠️ 不支援的編碼格式: {msg.encoding}")

            # 處理按鍵
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                print("退出調試程式...")
                rospy.signal_shutdown("使用者退出")
                cv2.destroyAllWindows()

        except Exception as e:
            print(f"❌ 處理錯誤: {e}")
            import traceback
            traceback.print_exc()


def main():
    try:
        print("🚀 啟動HP60C調試程式...")

        # 檢查ROS Master
        try:
            rospy.get_master().getPid()
            print("✅ ROS Master連線正常")
        except:
            print("❌ 無法連線到ROS Master!")
            print("請確認:")
            print("1. roscore 是否在運行")
            print("2. ROS_MASTER_URI 是否正確")
            return

        # 建立調試檢視器
        viewer = SimpleDebugViewer()

        # 設置10秒超時檢查
        start_time = rospy.Time.now()

        print("程式運行中...")

        # 自定義spin循環以檢查數據接收
        rate = rospy.Rate(10)  # 10Hz
        timeout_warned = False

        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            elapsed = (current_time - start_time).to_sec()

            # 10秒後如果沒收到數據，給出警告
            if elapsed > 10 and not viewer.data_received and not timeout_warned:
                print("\n⚠️ 超過10秒未收到深度數據!")
                print("可能的問題:")
                print("1. 相機驅動未啟動: roslaunch ascamera hp60c.launch")
                print("2. 話題名稱不正確")
                print("3. 相機硬體連接問題")
                timeout_warned = True

            rate.sleep()

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
        print("調試程式關閉")


if __name__ == '__main__':
    main()