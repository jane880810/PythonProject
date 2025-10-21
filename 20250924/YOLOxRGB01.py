#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
20250916
RGB相機+yolo人物辨識
HP60C 深度相機顯示 - PyCharm相容版本 + YOLOv8人物辨識
解決cv_bridge庫載入問題，加入YOLOv8人物檢測功能

"""

import sys
import os
import subprocess
import ctypes
import numpy as np
import cv2

# YOLOv8相關導入
try:
    from ultralytics import YOLO

    YOLO_AVAILABLE = True
    print("YOLOv8庫載入成功")
except ImportError as e:
    print(f"YOLOv8庫載入失敗: {e}")
    print("請執行: pip install ultralytics")
    YOLO_AVAILABLE = False


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

        # 初始化YOLOv8模型
        self.yolo_model = None
        if YOLO_AVAILABLE:
            try:
                print("正在載入YOLOv8模型...")
                # 使用預訓練的YOLOv8n模型（較小且快速）
                self.yolo_model = YOLO('yolov8n.pt')
                print("YOLOv8模型載入成功")
            except Exception as e:
                print(f"YOLOv8模型載入失敗: {e}")
                self.yolo_model = None

        # 圖像計數器
        self.frame_count = 0
        self.detection_count = 0

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

    def detect_persons(self, cv_image):
        """使用YOLOv8檢測人物"""
        if self.yolo_model is None:
            return []

        try:
            # 執行檢測
            results = self.yolo_model(cv_image, verbose=False)

            detections = []
            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    for box in boxes:
                        # 檢查是否為人（class 0 in COCO dataset）
                        class_id = int(box.cls[0])
                        confidence = float(box.conf[0])

                        if class_id == 0 and confidence > 0.5:  # 人的類別ID是0，信心度閾值0.5
                            # 取得邊界框座標
                            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                            detections.append({
                                'bbox': (int(x1), int(y1), int(x2), int(y2)),
                                'confidence': confidence
                            })

            return detections
        except Exception as e:
            print(f"YOLOv8檢測錯誤: {e}")
            return []

    def draw_detections(self, cv_image, detections):
        """繪製檢測框"""
        for detection in detections:
            x1, y1, x2, y2 = detection['bbox']
            confidence = detection['confidence']

            # 繪製綠色邊界框
            cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # 繪製信心度標籤（可選，如果你想要顯示）
            # label = f'Person: {confidence:.2f}'
            # cv2.putText(cv_image, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    def image_callback(self, msg):
        """處理圖像回調函數"""
        try:
            self.frame_count += 1

            # 轉換圖像
            if USE_CV_BRIDGE:
                try:
                    cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                except Exception as e:
                    print(f"cv_bridge轉換失敗，使用手動轉換: {e}")
                    cv_image = self.manual_image_convert(msg)
            else:
                cv_image = self.manual_image_convert(msg)

            if cv_image is None:
                return

            # 執行人物檢測
            detections = self.detect_persons(cv_image)

            if detections:
                self.detection_count += 1
                # 繪製檢測框
                self.draw_detections(cv_image, detections)

            # 顯示圖像（不再新增文字覆蓋層）
            cv2.imshow('HP60C Camera - Person Detection', cv_image)

            # 自動儲存第一幀用於驗證
            if self.frame_count == 1:
                cv2.imwrite('/tmp/pycharm_hp60c_first_frame.jpg', cv_image)
                print("已儲存第一幀到 /tmp/pycharm_hp60c_first_frame.jpg")

            # 處理按鍵輸入
            self.handle_keyboard(cv_image)

        except Exception as e:
            print(f"圖像處理錯誤: {e}")
            import traceback
            traceback.print_exc()

    def handle_keyboard(self, cv_image):
        """處理鍵盤輸入"""
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q') or key == 27:  # 'q' 或 ESC
            print("退出程式...")
            print(f"總共處理了 {self.frame_count} 幀")
            print(f"檢測到人物的幀數: {self.detection_count}")
            self.shutdown()
        elif key == ord('s'):  # 儲存截圖
            self.save_screenshot(cv_image)

    def save_screenshot(self, cv_image):
        """儲存截圖"""
        timestamp = rospy.Time.now().to_sec()
        filename = f'/tmp/pycharm_hp60c_detection_{timestamp:.0f}.jpg'

        if cv2.imwrite(filename, cv_image):
            print(f"檢測截圖已儲存: {filename}")
        else:
            print("截圖儲存失敗!")

    def shutdown(self):
        """安全關閉程式"""
        cv2.destroyAllWindows()
        rospy.signal_shutdown("使用者關閉")


# ========== 主程式 ==========
def main():
    """主程式進入點"""
    try:
        print("啟動PyCharm HP60C相機人物檢測程式...")

        # 檢查YOLOv8是否可用
        if not YOLO_AVAILABLE:
            print("警告: YOLOv8不可用，程式將僅顯示原始圖像")

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
        print("在PyCharm中成功執行HP60C相機人物檢測!")

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