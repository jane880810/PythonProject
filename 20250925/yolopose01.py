#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
20250916
繁體中文
HP60C 深度相機YOLO人體姿態檢測 - PyCharm相容版本
使用YOLOv8-pose進行姿態檢測

"""

import sys
import os
import subprocess
import ctypes
import numpy as np
import cv2
from ultralytics import YOLO


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


# ========== YOLO姿態檢測器類別 ==========
class YOLOPoseDetector:
    def __init__(self, model_path="yolov8n-pose.pt", conf_threshold=0.5):
        """
        初始化YOLO姿態檢測器

        參數:
        - model_path: YOLO模型路徑
        - conf_threshold: 置信度閾值
        """
        print("正在載入YOLO姿態檢測模型...")
        try:
            self.model = YOLO(model_path)
            self.conf_threshold = conf_threshold
            print(f"YOLO模型載入成功: {model_path}")
        except Exception as e:
            print(f"YOLO模型載入失敗: {e}")
            print("嘗試下載預設模型...")
            self.model = YOLO("yolov8n-pose.pt")  # 自動下載
            self.conf_threshold = conf_threshold

        # YOLO pose關鍵點索引 (17個關鍵點)
        self.keypoint_names = [
            "nose",  # 0
            "left_eye",  # 1
            "right_eye",  # 2
            "left_ear",  # 3
            "right_ear",  # 4
            "left_shoulder",  # 5
            "right_shoulder",  # 6
            "left_elbow",  # 7
            "right_elbow",  # 8
            "left_wrist",  # 9
            "right_wrist",  # 10
            "left_hip",  # 11
            "right_hip",  # 12
            "left_knee",  # 13
            "right_knee",  # 14
            "left_ankle",  # 15
            "right_ankle"  # 16
        ]

    def detect_pose(self, image):
        """
        檢測圖像中的人體姿態

        參數:
        - image: 輸入圖像 (BGR格式)

        返回:
        - image: 繪製了骨架的圖像
        - results: 檢測結果
        """
        try:
            # 使用YOLO進行檢測
            results = self.model(image, conf=self.conf_threshold, verbose=False)

            # 繪製簡化的骨架
            if results and len(results) > 0:
                self.draw_simplified_poses(image, results[0])

            return image, results
        except Exception as e:
            print(f"YOLO檢測錯誤: {e}")
            return image, None

    def draw_simplified_poses(self, image, result):
        """
        繪製簡化的人體骨架，頭部用圓形代替
        """
        if result.keypoints is None:
            return

        # 線條和圓形的顏色和粗細
        line_color = (0, 255, 0)  # 綠色
        circle_color = (0, 255, 255)  # 黃色
        line_thickness = 2
        circle_thickness = 2

        # 遍歷每個檢測到的人
        for keypoints in result.keypoints.data:
            if keypoints is None:
                continue

            # 轉換關鍵點格式
            points = {}
            for i, (x, y, conf) in enumerate(keypoints):
                if conf > 0.5:  # 置信度閾值
                    points[i] = (int(x), int(y))

            # 繪製頭部圓形
            self.draw_head_circle(image, points, circle_color, circle_thickness)

            # 繪製身體骨架
            self.draw_body_skeleton(image, points, line_color, line_thickness)

    def draw_head_circle(self, image, points, color, thickness):
        """繪製頭部圓形"""
        # 頭部關鍵點：鼻子、眼睛、耳朵
        head_points = [0, 1, 2, 3, 4]  # nose, left_eye, right_eye, left_ear, right_ear

        head_center_x, head_center_y = 0, 0
        valid_points = 0

        for point_idx in head_points:
            if point_idx in points:
                head_center_x += points[point_idx][0]
                head_center_y += points[point_idx][1]
                valid_points += 1

        if valid_points > 0:
            head_center_x //= valid_points
            head_center_y //= valid_points

            # 計算頭部半徑
            radius = 30  # 預設半徑

            # 基於耳朵間距離計算半徑
            if 3 in points and 4 in points:  # left_ear and right_ear
                ear_distance = abs(points[3][0] - points[4][0])
                radius = max(20, ear_distance // 2)
            elif 1 in points and 2 in points:  # left_eye and right_eye
                eye_distance = abs(points[1][0] - points[2][0])
                radius = max(15, eye_distance)

            # 繪製頭部圓形
            cv2.circle(image, (head_center_x, head_center_y), radius, color, thickness)

    def draw_body_skeleton(self, image, points, color, thickness):
        """繪製身體骨架線條"""
        # 定義身體連接線 (YOLO pose格式)
        connections = [
            # 軀幹
            (5, 6),  # left_shoulder - right_shoulder
            (5, 11),  # left_shoulder - left_hip
            (6, 12),  # right_shoulder - right_hip
            (11, 12),  # left_hip - right_hip

            # 左臂
            (5, 7),  # left_shoulder - left_elbow
            (7, 9),  # left_elbow - left_wrist

            # 右臂
            (6, 8),  # right_shoulder - right_elbow
            (8, 10),  # right_elbow - right_wrist

            # 左腿
            (11, 13),  # left_hip - left_knee
            (13, 15),  # left_knee - left_ankle

            # 右腿
            (12, 14),  # right_hip - right_knee
            (14, 16),  # right_knee - right_ankle
        ]

        # 繪製連接線
        for start_idx, end_idx in connections:
            if start_idx in points and end_idx in points:
                cv2.line(image, points[start_idx], points[end_idx], color, thickness)

        # 繪製關鍵關節點
        joint_points = [5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16]
        for point_idx in joint_points:
            if point_idx in points:
                cv2.circle(image, points[point_idx], 4, color, -1)

    def get_detection_count(self, results):
        """獲取檢測到的人數"""
        if results and len(results) > 0 and results[0].keypoints is not None:
            return len(results[0].keypoints.data)
        return 0


# ========== 相機姿態檢測系統類別 ==========
class HP60CYOLOPoseSystem:
    def __init__(self):
        print("初始化HP60C相機YOLO姿態檢測系統...")

        # 初始化ROS節點
        rospy.init_node('hp60c_yolo_pose_detection', anonymous=True)

        # 初始化cv_bridge（如果可用）
        if USE_CV_BRIDGE:
            self.bridge = CvBridge()

        # 初始化YOLO姿態檢測器
        self.pose_detector = YOLOPoseDetector(
            model_path="yolov8n-pose.pt",
            conf_threshold=0.5
        )

        # 計數器和狀態
        self.frame_count = 0
        self.pose_detected_count = 0
        self.total_persons_detected = 0
        self.enable_pose_detection = True

        # 訂閱RGB圖像話題
        self.image_sub = rospy.Subscriber(
            '/ascamera_hp60c/rgb0/image',
            Image,
            self.image_callback,
            queue_size=1
        )

        print("HP60C YOLO姿態檢測系統初始化完成")
        print("操作說明:")
        print("- 按 'q' 或 ESC 退出")
        print("- 按 's' 儲存截圖")
        print("- 按 'p' 開關姿態檢測")
        print("- 按 'i' 顯示系統資訊（僅終端機）")
        print("- 按 'r' 重設統計資料")
        print("- 使用YOLO檢測，支援多人檢測，頭部顯示為圓形")

    def manual_image_convert(self, msg):
        """手動轉換ROS Image到OpenCV格式"""
        if msg.encoding == "bgr8":
            np_arr = np.frombuffer(msg.data, dtype=np.uint8)
            cv_image = np_arr.reshape((msg.height, msg.width, 3))
            return cv_image
        elif msg.encoding == "rgb8":
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

            # 執行YOLO姿態檢測
            if self.enable_pose_detection:
                processed_image, pose_results = self.pose_detector.detect_pose(processed_image)

                # 統計檢測結果
                person_count = self.pose_detector.get_detection_count(pose_results)
                if person_count > 0:
                    self.pose_detected_count += 1
                    self.total_persons_detected += person_count

            # 顯示圖像（不顯示任何文字）
            cv2.imshow('HP60C YOLO Pose Detection', processed_image)

            # 自動儲存第一幀
            if self.frame_count == 1:
                cv2.imwrite('/tmp/hp60c_yolo_first_frame.jpg', processed_image)
                print("已儲存第一幀到 /tmp/hp60c_yolo_first_frame.jpg")

            # 處理按鍵輸入
            self.handle_keyboard(processed_image, msg, pose_results)

        except Exception as e:
            print(f"圖像處理錯誤: {e}")
            import traceback
            traceback.print_exc()

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
        filename = f'/tmp/hp60c_yolo_screenshot_{timestamp:.0f}.jpg'

        if cv2.imwrite(filename, cv_image):
            print(f"截圖已儲存: {filename}")
        else:
            print("截圖儲存失敗!")

    def toggle_pose_detection(self):
        """開關姿態檢測功能"""
        self.enable_pose_detection = not self.enable_pose_detection
        status = "開啟" if self.enable_pose_detection else "關閉"
        print(f"YOLO姿態檢測功能已{status}")

    def reset_statistics(self):
        """重設統計資料"""
        self.frame_count = 0
        self.pose_detected_count = 0
        self.total_persons_detected = 0
        print("統計資料已重設")

    def show_system_info(self, cv_image, msg, pose_results):
        """顯示詳細系統資訊（僅在終端機顯示）"""
        print("\n" + "=" * 60)
        print("HP60C YOLO姿態檢測系統詳細資訊:")
        print(f"  相機解析度: {msg.width} x {msg.height}")
        print(f"  OpenCV圖像形狀: {cv_image.shape}")
        print(f"  圖像編碼格式: {msg.encoding}")
        print(f"  總處理幀數: {self.frame_count}")
        print(f"  檢測到姿態幀數: {self.pose_detected_count}")
        print(f"  總檢測人數: {self.total_persons_detected}")

        detection_rate = (self.pose_detected_count / self.frame_count * 100) if self.frame_count > 0 else 0
        avg_persons = (self.total_persons_detected / self.pose_detected_count) if self.pose_detected_count > 0 else 0

        print(f"  姿態檢測率: {detection_rate:.2f}%")
        print(f"  平均每幀人數: {avg_persons:.2f}")
        print(f"  檢測狀態: {'開啟' if self.enable_pose_detection else '關閉'}")
        print(f"  使用模型: YOLO v8 Pose")

        if pose_results:
            current_persons = self.pose_detector.get_detection_count(pose_results)
            print(f"  目前檢測人數: {current_persons}")
        else:
            print("  目前未檢測到人體")

        print(f"  顯示模式: 簡化模式（頭部圓形，無文字覆蓋）")
        print("=" * 60 + "\n")

    def shutdown(self):
        """安全關閉程式"""
        print(f"\nYOLO系統關閉統計:")
        print(f"  總處理幀數: {self.frame_count}")
        print(f"  檢測到姿態幀數: {self.pose_detected_count}")
        print(f"  總檢測人數: {self.total_persons_detected}")
        detection_rate = (self.pose_detected_count / self.frame_count * 100) if self.frame_count > 0 else 0
        print(f"  整體檢測率: {detection_rate:.2f}%")

        cv2.destroyAllWindows()
        rospy.signal_shutdown("使用者關閉")


# ========== 主程式 ==========
def main():
    """主程式進入點"""
    try:
        print("啟動HP60C相機YOLO人體姿態檢測系統...")

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

        # 建立並執行YOLO姿態檢測系統
        yolo_system = HP60CYOLOPoseSystem()

        print("系統執行中，等待相機資料...")
        print("HP60C YOLO姿態檢測系統啟動成功!")

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