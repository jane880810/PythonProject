#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
20250916
繁體中文
HP60C 深度相機人體姿態檢測 - PyCharm相容版本
整合MediaPipe姿態檢測功能

"""

import sys
import os
import subprocess
import ctypes
import numpy as np
import cv2
import mediapipe as mp


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


# ========== 姿態檢測器類別 ==========
class PoseDetector:
    def __init__(self, static_image_mode=False, model_complexity=1, smooth_landmarks=True,
                 min_detection_confidence=0.5, min_tracking_confidence=0.5):
        """
        初始化姿態檢測器

        參數:
        - static_image_mode: 是否為靜態圖像模式
        - model_complexity: 模型複雜度 (0, 1, 2)
        - smooth_landmarks: 是否平滑關鍵點
        - min_detection_confidence: 最小檢測置信度
        - min_tracking_confidence: 最小追蹤置信度
        """
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(
            static_image_mode=static_image_mode,
            model_complexity=model_complexity,
            smooth_landmarks=smooth_landmarks,
            min_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence
        )
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles

    def detect_pose(self, image):
        """
        檢測圖像中的人體姿態

        參數:
        - image: 輸入圖像 (BGR格式)

        返回:
        - image: 繪製了骨架的圖像
        - results: 檢測結果
        """
        # 轉換顏色格式 BGR -> RGB
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # 進行姿態檢測
        results = self.pose.process(image_rgb)

        # 手動繪製簡化的骨架
        if results.pose_landmarks:
            self.draw_simplified_pose(image, results.pose_landmarks)

        return image, results

    def draw_simplified_pose(self, image, landmarks):
        """
        繪製簡化的人體骨架，頭部用圓形代替
        """
        h, w, _ = image.shape

        # 獲取關鍵點座標
        points = {}
        for idx, landmark in enumerate(landmarks.landmark):
            x = int(landmark.x * w)
            y = int(landmark.y * h)
            if landmark.visibility > 0.5:  # 只繪製可見的點
                points[idx] = (x, y)

        # 線條和圓形的顏色和粗細
        line_color = (0, 255, 0)  # 綠色
        circle_color = (0, 255, 255)  # 黃色
        line_thickness = 2
        circle_thickness = 2

        # 繪製頭部（用圓形代替面部關鍵點）
        head_points = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]  # 面部關鍵點
        head_center_x, head_center_y = 0, 0
        valid_head_points = 0

        for point_idx in head_points:
            if point_idx in points:
                head_center_x += points[point_idx][0]
                head_center_y += points[point_idx][1]
                valid_head_points += 1

        if valid_head_points > 0:
            head_center_x //= valid_head_points
            head_center_y //= valid_head_points
            # 計算頭部半徑（基於耳朵間距離）
            radius = 30  # 預設半徑
            if 7 in points and 8 in points:  # 左右耳
                ear_distance = abs(points[7][0] - points[8][0])
                radius = max(20, ear_distance // 2)

            # 繪製頭部圓形
            cv2.circle(image, (head_center_x, head_center_y), radius, circle_color, circle_thickness)

        # 定義身體連接線（排除面部）
        body_connections = [
            # 肩膀到軀幹
            (11, 12),  # 左肩到右肩
            (11, 23),  # 左肩到左臀
            (12, 24),  # 右肩到右臀
            (23, 24),  # 左臀到右臀

            # 左臂
            (11, 13),  # 左肩到左肘
            (13, 15),  # 左肘到左手腕

            # 右臂
            (12, 14),  # 右肩到右肘
            (14, 16),  # 右肘到右手腕

            # 左腿
            (23, 25),  # 左臀到左膝
            (25, 27),  # 左膝到左腳踝

            # 右腿
            (24, 26),  # 右臀到右膝
            (26, 28),  # 右膝到右腳踝

            # 腳部
            (27, 29),  # 左腳踝到左腳跟
            (29, 31),  # 左腳跟到左腳趾
            (28, 30),  # 右腳踝到右腳跟
            (30, 32),  # 右腳跟到右腳趾
        ]

        # 繪製身體連接線
        for connection in body_connections:
            start_idx, end_idx = connection
            if start_idx in points and end_idx in points:
                cv2.line(image, points[start_idx], points[end_idx], line_color, line_thickness)

        # 繪製關鍵關節點（排除面部）
        joint_points = [11, 12, 13, 14, 15, 16, 23, 24, 25, 26, 27, 28]
        for point_idx in joint_points:
            if point_idx in points:
                cv2.circle(image, points[point_idx], 4, line_color, -1)

    def get_landmarks_coordinates(self, results, image_width, image_height):
        """
        獲取關鍵點座標

        參數:
        - results: 檢測結果
        - image_width: 圖像寬度
        - image_height: 圖像高度

        返回:
        - landmarks: 關鍵點座標列表
        """
        landmarks = []
        if results.pose_landmarks:
            for landmark in results.pose_landmarks.landmark:
                x = int(landmark.x * image_width)
                y = int(landmark.y * image_height)
                z = landmark.z
                visibility = landmark.visibility
                landmarks.append([x, y, z, visibility])
        return landmarks


# ========== 相機姿態檢測系統類別 ==========
class HP60CPoseDetectionSystem:
    def __init__(self):
        print("初始化HP60C相機人體姿態檢測系統...")

        # 初始化ROS節點
        rospy.init_node('hp60c_pose_detection', anonymous=True)

        # 初始化cv_bridge（如果可用）
        if USE_CV_BRIDGE:
            self.bridge = CvBridge()

        # 初始化姿態檢測器
        self.pose_detector = PoseDetector(
            model_complexity=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )

        # 計數器和狀態
        self.frame_count = 0
        self.pose_detected_count = 0
        self.enable_pose_detection = True
        self.save_next_frame = False

        # 訂閱RGB圖像話題
        self.image_sub = rospy.Subscriber(
            '/ascamera_hp60c/rgb0/image',
            Image,
            self.image_callback,
            queue_size=1
        )

        print("HP60C姿態檢測系統初始化完成")
        print("操作說明:")
        print("- 按 'q' 或 ESC 退出")
        print("- 按 's' 儲存截圖")
        print("- 按 'p' 開關姿態檢測")
        print("- 按 'i' 顯示系統資訊（僅終端機）")
        print("- 按 'r' 重設統計資料")
        print("- 畫面已簡化：頭部顯示為圓形，移除所有文字資訊")

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

            # 複製原始圖像用於姿態檢測
            processed_image = cv_image.copy()
            pose_results = None

            # 執行姿態檢測
            if self.enable_pose_detection:
                processed_image, pose_results = self.pose_detector.detect_pose(processed_image)

                # 統計檢測到的姿態
                if pose_results and pose_results.pose_landmarks:
                    self.pose_detected_count += 1

            # 新增資訊覆蓋層
            self.add_info_overlay(processed_image, msg, conversion_method, pose_results)

            # 顯示圖像
            cv2.imshow('HP60C Pose Detection System', processed_image)

            # 自動儲存第一幀用於驗證
            if self.frame_count == 1:
                cv2.imwrite('/tmp/hp60c_pose_first_frame.jpg', processed_image)
                print("已儲存第一幀到 /tmp/hp60c_pose_first_frame.jpg")

            # 處理按鍵輸入
            self.handle_keyboard(processed_image, msg, pose_results)

        except Exception as e:
            print(f"圖像處理錯誤: {e}")
            import traceback
            traceback.print_exc()

    def add_info_overlay(self, image, msg, conversion_method, pose_results):
        """
        不顯示任何資訊覆蓋層（已移除所有文字顯示）
        """
        # 移除所有文字資訊顯示
        pass

    def handle_keyboard(self, cv_image, msg, pose_results):
        """處理鍵盤輸入"""
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q') or key == 27:  # 'q' 或 ESC
            print("退出程式...")
            self.shutdown()
        elif key == ord('s'):  # 儲存截圖
            self.save_screenshot(cv_image)
        elif key == ord('p'):  # 開關姿態檢測
            self.toggle_pose_detection()
        elif key == ord('i'):  # 顯示系統資訊
            self.show_system_info(cv_image, msg, pose_results)
        elif key == ord('r'):  # 重設統計資料
            self.reset_statistics()

    def save_screenshot(self, cv_image):
        """儲存截圖"""
        timestamp = rospy.Time.now().to_sec()
        filename = f'/tmp/hp60c_pose_screenshot_{timestamp:.0f}.jpg'

        if cv2.imwrite(filename, cv_image):
            print(f"截圖已儲存: {filename}")
        else:
            print("截圖儲存失敗!")

    def toggle_pose_detection(self):
        """開關姿態檢測功能"""
        self.enable_pose_detection = not self.enable_pose_detection
        status = "開啟" if self.enable_pose_detection else "關閉"
        print(f"姿態檢測功能已{status}")

    def reset_statistics(self):
        """重設統計資料"""
        self.frame_count = 0
        self.pose_detected_count = 0
        print("統計資料已重設")

    def show_system_info(self, cv_image, msg, pose_results):
        """顯示詳細系統資訊（僅在終端機顯示）"""
        print("\n" + "=" * 60)
        print("HP60C姿態檢測系統詳細資訊:")
        print(f"  相機解析度: {msg.width} x {msg.height}")
        print(f"  OpenCV圖像形狀: {cv_image.shape}")
        print(f"  圖像編碼格式: {msg.encoding}")
        print(f"  資料型別: {cv_image.dtype}")
        print(f"  像素值範圍: {cv_image.min()} - {cv_image.max()}")
        print(f"  記憶體大小: {cv_image.nbytes} bytes")
        print(f"  總處理幀數: {self.frame_count}")
        print(f"  檢測到姿態幀數: {self.pose_detected_count}")

        detection_rate = (self.pose_detected_count / self.frame_count * 100) if self.frame_count > 0 else 0
        print(f"  姿態檢測率: {detection_rate:.2f}%")
        print(f"  姿態檢測狀態: {'開啟' if self.enable_pose_detection else '關閉'}")

        if pose_results and pose_results.pose_landmarks:
            landmarks = self.pose_detector.get_landmarks_coordinates(
                pose_results, cv_image.shape[1], cv_image.shape[0]
            )
            print(f"  目前檢測關鍵點數: {len(landmarks)}")
            print(f"  關鍵點可見度平均: {np.mean([lm[3] for lm in landmarks]):.3f}")
        else:
            print("  目前未檢測到姿態")

        print("  顯示模式: 簡化模式（頭部圓形，無文字覆蓋）")
        print("=" * 60 + "\n")

    def shutdown(self):
        """安全關閉程式"""
        print(f"\n系統關閉統計:")
        print(f"  總處理幀數: {self.frame_count}")
        print(f"  檢測到姿態幀數: {self.pose_detected_count}")
        detection_rate = (self.pose_detected_count / self.frame_count * 100) if self.frame_count > 0 else 0
        print(f"  整體檢測率: {detection_rate:.2f}%")

        cv2.destroyAllWindows()
        rospy.signal_shutdown("使用者關閉")


# ========== 主程式 ==========
def main():
    """主程式進入點"""
    try:
        print("啟動HP60C相機人體姿態檢測系統...")

        # 檢查ROS連線
        try:
            rospy.get_master().getPid()
            print("ROS Master連線正常")
        except:
            print("無法連線到ROS Master!")
            print("請確保:")
            print("1. roscore正在執行")
            print("2. HP60C相機驅動程式已啟動")
            print("3. 相機話題 '/ascamera_hp60c/rgb0/image' 正在發布")
            return

        # 建立並執行姿態檢測系統
        pose_system = HP60CPoseDetectionSystem()

        print("系統執行中，等待相機資料...")
        print("HP60C相機人體姿態檢測系統啟動成功!")

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