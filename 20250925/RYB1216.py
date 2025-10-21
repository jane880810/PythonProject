#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
20250924
繁體中文
HP60C 深度相機顯示 - PyCharm相容版本 + YOLOv8人物辨識 + 深度測距 + 俯視圖
解決cv_bridge庫載入問題，加入YOLOv8人物檢測功能、深度測距和俯視圖顯示

"""

import sys
import os
import subprocess
import ctypes
import numpy as np
import cv2
import threading

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

        # 深度圖像相關變數
        self.depth_image = None
        self.depth_lock = threading.Lock()

        # 圖像計數器
        self.frame_count = 0
        self.detection_count = 0

        # 新增俯視圖相關變數
        self.overhead_width = 600
        self.overhead_height = 400
        self.max_display_distance = 300  # 最大顯示距離 (cm)
        self.camera_fov = 60  # 相機視野角度 (度)

        # 初始化俯視圖
        self.init_overhead_view()

        # 訂閱RGB圖像話題
        self.image_sub = rospy.Subscriber(
            '/ascamera_hp60c/rgb0/image',
            Image,
            self.image_callback,
            queue_size=1
        )

        # 深度回調計數器
        self.depth_callback_count = 0

        # 訂閱深度圖像話題 - 嘗試多個可能的話題名稱
        possible_depth_topics = [
            '/ascamera_hp60c/depth/image',
            '/ascamera_hp60c/depth0/image',
            '/ascamera_hp60c/depth/image_raw',
            '/ascamera_hp60c/depth0/image_raw',
            '/camera/depth/image_raw',
            '/camera/aligned_depth_to_color/image_raw'
        ]

        self.depth_sub = None
        self.depth_topic_name = None
        for topic in possible_depth_topics:
            try:
                print(f"嘗試訂閱深度話題: {topic}")
                self.depth_sub = rospy.Subscriber(
                    topic,
                    Image,
                    self.depth_callback,
                    queue_size=1
                )
                self.depth_topic_name = topic
                print(f"成功訂閱深度話題: {topic}")
                break
            except Exception as e:
                print(f"無法訂閱 {topic}: {e}")
                continue

        print("PyCharm HP60C顯示器初始化完成")

        # 設定定時器檢查深度話題狀態
        self.check_timer = rospy.Timer(rospy.Duration(5.0), self.check_depth_status)

        # 列出可用的話題以便調試（使用Python API而不是命令行）
        print("\n正在檢查可用的ROS話題...")
        try:
            # 使用rospy API獲取話題列表
            topics_and_types = rospy.get_published_topics()
            print("找到的相機和深度相關話題:")
            depth_topics = []
            image_topics = []
            for topic, msg_type in topics_and_types:
                if any(keyword in topic.lower() for keyword in ['ascamera', 'depth', 'image', 'camera']):
                    print(f"  - {topic} ({msg_type})")
                    if 'depth' in topic.lower():
                        depth_topics.append(topic)
                    if 'image' in topic.lower():
                        image_topics.append(topic)

            if not depth_topics:
                print("\n警告: 沒有找到深度相關話題!")
                print("可能的解決方案:")
                print("1. 檢查深度相機驅動是否正在運行")
                print("2. 檢查相機是否支援深度功能")
                print("3. 確認相機話題設定正確")
            else:
                print(f"\n找到 {len(depth_topics)} 個深度話題")

        except Exception as e:
            print(f"檢查話題時出錯: {e}")

        print("\n操作說明:")
        print("- 按 'q' 或 ESC 退出")
        print("- 按 's' 儲存截圖")
        print("- 紅色框：距離≤1.2m（Warning）")
        print("- 黃色框：距離1.2m-1.6m（Caution）")
        print("- 綠色框：距離>1.6m（Safe）")
        print("- 俯視圖會顯示人物即時位置")

    def init_overhead_view(self):
        """初始化俯視圖視窗"""
        # 創建俯視圖背景
        self.overhead_bg = self.create_overhead_background()

    def create_overhead_background(self):
        """創建俯視圖背景"""
        # 創建黑色背景
        overhead = np.zeros((self.overhead_height, self.overhead_width, 3), dtype=np.uint8)

        # 相機位置（視窗底部中央）
        camera_x = self.overhead_width // 2
        camera_y = self.overhead_height - 30

        # 距離比例尺：像素/cm
        scale = (self.overhead_height - 60) / self.max_display_distance

        # 繪製距離網格線（每50cm一條）
        for distance in range(50, self.max_display_distance + 1, 50):
            y = camera_y - int(distance * scale)
            if y > 10:
                cv2.line(overhead, (50, y), (self.overhead_width - 50, y),
                         (40, 40, 40), 1)
                # 標示距離
                cv2.putText(overhead, f"{distance}cm", (10, y + 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 100, 100), 1)

        # 繪製警戒線
        # 紅色警戒線 (120cm)
        red_y = camera_y - int(120 * scale)
        if red_y > 10:
            cv2.line(overhead, (50, red_y), (self.overhead_width - 50, red_y),
                     (0, 0, 255), 3)
            cv2.putText(overhead, "Danger Zone (120cm)", (self.overhead_width - 200, red_y - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        # 黃色警戒線 (160cm)
        yellow_y = camera_y - int(160 * scale)
        if yellow_y > 10:
            cv2.line(overhead, (50, yellow_y), (self.overhead_width - 50, yellow_y),
                     (0, 255, 255), 3)
            cv2.putText(overhead, "Caution Zone (160cm)", (self.overhead_width - 200, yellow_y - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        # 繪製相機視野扇形
        # 計算視野邊界
        fov_rad = np.radians(self.camera_fov / 2)
        max_y = 10

        # 左邊界
        left_x = camera_x - int((camera_y - max_y) * np.tan(fov_rad))
        # 右邊界
        right_x = camera_x + int((camera_y - max_y) * np.tan(fov_rad))

        # 繪製視野邊界線
        cv2.line(overhead, (camera_x, camera_y), (left_x, max_y), (0, 100, 0), 2)
        cv2.line(overhead, (camera_x, camera_y), (right_x, max_y), (0, 100, 0), 2)

        # 繪製相機圖標
        cv2.circle(overhead, (camera_x, camera_y), 8, (255, 255, 255), -1)
        cv2.putText(overhead, "Camera", (camera_x - 25, camera_y + 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # 添加標題
        cv2.putText(overhead, "Person Position Overview", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        return overhead

    def calculate_person_position(self, bbox, distance, image_width):
        """計算人物在俯視圖中的位置"""
        if distance is None:
            return None

        x1, y1, x2, y2 = bbox
        person_center_x = (x1 + x2) // 2

        # 計算相對於圖像中心的角度
        image_center_x = image_width // 2
        pixel_offset = person_center_x - image_center_x

        # 假設相機水平視野為60度
        pixels_per_degree = image_width / self.camera_fov
        angle_offset = pixel_offset / pixels_per_degree
        angle_rad = np.radians(angle_offset)

        # 計算在俯視圖中的位置
        camera_x = self.overhead_width // 2
        camera_y = self.overhead_height - 30
        scale = (self.overhead_height - 60) / self.max_display_distance

        # 計算人物位置
        pos_x = camera_x + int(distance * np.sin(angle_rad) * scale)
        pos_y = camera_y - int(distance * np.cos(angle_rad) * scale)

        # 確保位置在視窗範圍內
        pos_x = max(10, min(self.overhead_width - 10, pos_x))
        pos_y = max(10, min(self.overhead_height - 10, pos_y))

        return (pos_x, pos_y)

    def update_overhead_view(self, detections, image_width):
        """更新俯視圖顯示"""
        # 從背景開始
        overhead = self.overhead_bg.copy()

        # 繪製每個檢測到的人物
        for i, detection in enumerate(detections):
            bbox = detection['bbox']

            # 獲取距離資訊
            with self.depth_lock:
                current_depth = self.depth_image.copy() if self.depth_image is not None else None

            distance = self.get_distance_at_bbox(bbox, current_depth)

            if distance is not None and distance <= self.max_display_distance:
                # 計算人物位置
                pos = self.calculate_person_position(bbox, distance, image_width)

                if pos is not None:
                    pos_x, pos_y = pos

                    # 根據距離決定顏色
                    if distance <= 120:
                        color = (0, 0, 255)  # 紅色
                        status = "Danger"
                    elif distance <= 160:
                        color = (0, 255, 255)  # 黃色
                        status = "Caution"
                    else:
                        color = (0, 255, 0)  # 綠色
                        status = "Safe"

                    # 繪製人物位置點
                    cv2.circle(overhead, (pos_x, pos_y), 8, (255, 255, 255), -1)
                    cv2.circle(overhead, (pos_x, pos_y), 10, color, 2)

                    # 顯示距離資訊
                    text = f"P{i + 1}: {distance:.0f}cm"
                    cv2.putText(overhead, text, (pos_x + 15, pos_y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

                    # 繪製從相機到人物的連線
                    camera_pos = (self.overhead_width // 2, self.overhead_height - 30)
                    cv2.line(overhead, camera_pos, (pos_x, pos_y), color, 1)

        # 顯示俯視圖
        cv2.imshow('Person Position Overview', overhead)

    def check_depth_status(self, event):
        """定期檢查深度話題狀態"""
        if self.depth_callback_count == 0:
            print(
                f"\n警告: 已等待 {int(event.current_real.to_sec() - rospy.Time.now().to_sec() + 5)} 秒，仍未收到深度資料")
            print(f"當前訂閱的深度話題: {self.depth_topic_name}")

            # 檢查話題是否真的存在且有發布者
            try:
                topics_and_types = rospy.get_published_topics()
                topic_found = False
                for topic, msg_type in topics_and_types:
                    if topic == self.depth_topic_name:
                        topic_found = True
                        print(f"話題存在，訊息類型: {msg_type}")
                        break

                if not topic_found:
                    print("話題不存在或沒有發布者!")
                    # 嘗試其他深度話題
                    self.try_alternative_depth_topics()
                else:
                    # 話題存在但沒有收到資料，可能是訊息頻率問題
                    print("話題存在但沒有收到資料，請檢查相機是否正常工作")

            except Exception as e:
                print(f"檢查話題狀態時出錯: {e}")
        else:
            # 如果收到深度資料，取消定時器
            self.check_timer.shutdown()
            print(f"深度資料正常，已收到 {self.depth_callback_count} 個深度幀")

    def try_alternative_depth_topics(self):
        """嘗試其他可能的深度話題"""
        try:
            topics_and_types = rospy.get_published_topics()
            alternative_topics = []

            for topic, msg_type in topics_and_types:
                if ('depth' in topic.lower() or 'distance' in topic.lower()) and 'Image' in msg_type:
                    alternative_topics.append(topic)

            if alternative_topics:
                print(f"找到其他可能的深度話題: {alternative_topics}")
                print("嘗試重新訂閱...")

                # 取消當前訂閱
                if self.depth_sub:
                    self.depth_sub.unregister()

                # 嘗試第一個替代話題
                new_topic = alternative_topics[0]
                try:
                    self.depth_sub = rospy.Subscriber(
                        new_topic,
                        Image,
                        self.depth_callback,
                        queue_size=1
                    )
                    self.depth_topic_name = new_topic
                    print(f"重新訂閱深度話題: {new_topic}")
                except Exception as e:
                    print(f"重新訂閱失敗: {e}")
            else:
                print("沒有找到其他深度話題")

        except Exception as e:
            print(f"查找替代話題時出錯: {e}")

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

    def manual_depth_convert(self, msg):
        """手動轉換深度圖像"""
        print(f"手動轉換深度圖像，編碼: {msg.encoding}")

        if msg.encoding == "16UC1":
            # 16位無符號整數，通常單位為mm
            np_arr = np.frombuffer(msg.data, dtype=np.uint16)
            depth_image = np_arr.reshape((msg.height, msg.width))
            return depth_image
        elif msg.encoding == "32FC1":
            # 32位浮點數，通常單位為m
            np_arr = np.frombuffer(msg.data, dtype=np.float32)
            depth_image = np_arr.reshape((msg.height, msg.width))
            return depth_image
        elif msg.encoding == "mono16":
            # 16位單色，通常為深度
            np_arr = np.frombuffer(msg.data, dtype=np.uint16)
            depth_image = np_arr.reshape((msg.height, msg.width))
            return depth_image
        elif msg.encoding == "8UC1":
            # 8位無符號整數
            np_arr = np.frombuffer(msg.data, dtype=np.uint8)
            depth_image = np_arr.reshape((msg.height, msg.width))
            return depth_image.astype(np.uint16)  # 轉換為16位便於處理
        else:
            print(f"不支援的深度圖像編碼: {msg.encoding}")
            # 嘗試用16位解析
            try:
                np_arr = np.frombuffer(msg.data, dtype=np.uint16)
                depth_image = np_arr.reshape((msg.height, msg.width))
                print("嘗試用16位格式解析成功")
                return depth_image
            except:
                print("無法用16位格式解析")
                return None

    def depth_callback(self, msg):
        """處理深度圖像回調函數"""
        try:
            self.depth_callback_count += 1

            # 第一次接收到深度圖像時顯示詳細資訊
            if self.depth_callback_count == 1:
                print(f"\n收到第一個深度圖像:")
                print(f"  - 話題: {self.depth_topic_name}")
                print(f"  - 尺寸: {msg.width}x{msg.height}")
                print(f"  - 編碼: {msg.encoding}")
                print(f"  - 資料長度: {len(msg.data)}")
                print(f"  - 時間戳: {msg.header.stamp}")

            # 每100個回調顯示一次統計
            if self.depth_callback_count % 100 == 0:
                print(f"深度回調計數: {self.depth_callback_count}")

            if USE_CV_BRIDGE:
                try:
                    if msg.encoding == "16UC1":
                        depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
                    elif msg.encoding == "32FC1":
                        depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
                    else:
                        depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
                        if self.depth_callback_count == 1:
                            print(f"使用passthrough模式，編碼: {msg.encoding}")
                except Exception as e:
                    print(f"深度圖像cv_bridge轉換失敗: {e}")
                    depth_image = self.manual_depth_convert(msg)
            else:
                depth_image = self.manual_depth_convert(msg)

            if depth_image is not None:
                # 第一次成功轉換時顯示深度統計
                if self.depth_callback_count == 1:
                    valid_depths = depth_image[depth_image > 0]
                    if len(valid_depths) > 0:
                        print(
                            f"  - 深度統計: min={valid_depths.min()}, max={valid_depths.max()}, mean={valid_depths.mean():.2f}")
                        print(f"  - 資料型別: {depth_image.dtype}")
                        print(
                            f"  - 有效像素比例: {len(valid_depths)}/{depth_image.size} ({100 * len(valid_depths) / depth_image.size:.1f}%)")
                    else:
                        print("  - 警告: 沒有有效的深度值!")

                with self.depth_lock:
                    self.depth_image = depth_image.copy()
            else:
                print("深度圖像轉換失敗!")

        except Exception as e:
            print(f"深度圖像處理錯誤: {e}")
            import traceback
            traceback.print_exc()

    def get_distance_at_bbox(self, bbox, depth_image):
        """計算邊界框中心區域的平均距離"""
        if depth_image is None:
            if self.frame_count % 60 == 0:  # 每60幀提醒一次
                print(f"深度圖像為空 (已處理 {self.frame_count} 幀，深度回調 {self.depth_callback_count} 次)")
            return None

        x1, y1, x2, y2 = bbox

        # 計算中心區域（取中心1/4區域以獲得更穩定的深度值）
        center_x = (x1 + x2) // 2
        center_y = (y1 + y2) // 2

        # 定義中心區域範圍
        region_size = min((x2 - x1) // 4, (y2 - y1) // 4, 20)  # 最大20像素

        x_start = max(0, center_x - region_size)
        x_end = min(depth_image.shape[1], center_x + region_size)
        y_start = max(0, center_y - region_size)
        y_end = min(depth_image.shape[0], center_y + region_size)

        # 取得該區域的深度值
        depth_region = depth_image[y_start:y_end, x_start:x_end]

        # 過濾無效值（0值通常表示無效深度）
        valid_depths = depth_region[depth_region > 0]

        # 調試訊息（僅對第一個檢測框顯示）
        if self.frame_count % 30 == 0:  # 每30幀顯示一次調試訊息
            print(f"深度調試 - 區域: ({x_start},{y_start}) 到 ({x_end},{y_end})")
            print(f"深度值範圍: {depth_region.min()} - {depth_region.max()}")
            print(f"有效深度點數: {len(valid_depths)}/{depth_region.size}")
            if len(valid_depths) > 0:
                print(f"有效深度平均: {np.mean(valid_depths):.2f}")

        if len(valid_depths) == 0:
            # 如果中心區域沒有有效值，擴大搜索範圍
            region_size = min((x2 - x1) // 2, (y2 - y1) // 2, 50)
            x_start = max(0, center_x - region_size)
            x_end = min(depth_image.shape[1], center_x + region_size)
            y_start = max(0, center_y - region_size)
            y_end = min(depth_image.shape[0], center_y + region_size)

            depth_region = depth_image[y_start:y_end, x_start:x_end]
            valid_depths = depth_region[depth_region > 0]

            if len(valid_depths) == 0:
                if self.frame_count % 30 == 0:
                    print("擴大搜索後仍無有效深度值")
                return None

        # 計算平均深度
        avg_depth = np.mean(valid_depths)

        # 根據深度圖像編碼轉換單位到cm
        # 如果是16UC1通常是mm，如果是32FC1通常是m
        if depth_image.dtype == np.uint16:
            # 假設單位為mm，轉換為cm
            distance_cm = avg_depth / 10.0
        elif depth_image.dtype == np.float32:
            # 假設單位為m，轉換為cm
            distance_cm = avg_depth * 100.0
        else:
            # 預設假設為mm
            distance_cm = avg_depth / 10.0

        # 合理性檢查（距離應該在10cm到1000cm之間）
        if distance_cm < 1 or distance_cm > 2000:
            if self.frame_count % 30 == 0:
                print(f"距離值異常: {distance_cm:.1f}cm，原始深度: {avg_depth}")
            # 嘗試其他單位轉換
            if distance_cm < 1:
                distance_cm = avg_depth  # 可能已經是cm
            elif distance_cm > 2000:
                distance_cm = avg_depth / 1000.0  # 可能是μm，轉換為cm

        return distance_cm

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
        """繪製檢測框和距離資訊（根據距離改變框色）"""
        with self.depth_lock:
            current_depth = self.depth_image.copy() if self.depth_image is not None else None

        for detection in detections:
            x1, y1, x2, y2 = detection['bbox']
            confidence = detection['confidence']

            # 計算距離
            distance = self.get_distance_at_bbox((x1, y1, x2, y2), current_depth)

            # 根據距離設定邊界框顏色和狀態文字 (BGR格式)
            if distance is not None:
                if distance <= 120:  # 1.2m以內 - 紅色
                    box_color = (0, 0, 255)  # 紅色
                    status_text = "Warning"
                    text_color = (0, 0, 255)
                elif distance <= 160:  # 1.2m到1.6m之間 - 黃色
                    box_color = (0, 255, 255)  # 黃色
                    status_text = "Caution"
                    text_color = (0, 255, 255)
                else:  # 1.6m以外 - 綠色
                    box_color = (0, 255, 0)  # 綠色
                    status_text = "Safe"
                    text_color = (0, 255, 0)

                distance_text = f"{distance:.0f}cm ({status_text})"
            else:
                # 無法測量距離時使用預設綠色
                box_color = (0, 255, 0)
                distance_text = "N/A"
                text_color = (0, 255, 0)

            # 繪製彩色邊界框（線條粗細為3讓顏色更明顯）
            cv2.rectangle(cv_image, (x1, y1), (x2, y2), box_color, 3)

            # 在邊界框上方顯示距離和狀態
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.7
            font_thickness = 2

            # 計算文字大小以繪製背景
            (text_width, text_height), baseline = cv2.getTextSize(
                distance_text, font, font_scale, font_thickness
            )

            # 繪製文字背景
            text_x = x1
            text_y = y1 - 10
            if text_y < text_height:
                text_y = y2 + text_height + 10

            cv2.rectangle(cv_image,
                          (text_x, text_y - text_height - baseline),
                          (text_x + text_width, text_y + baseline),
                          (0, 0, 0), -1)

            # 繪製距離文字
            cv2.putText(cv_image, distance_text, (text_x, text_y),
                        font, font_scale, text_color, font_thickness)

            # 如果距離太近，在畫面上方顯示額外警告
            if distance is not None and distance <= 120:
                warning_text = "TOO CLOSE!"
                warning_font_scale = 1.2
                warning_thickness = 3

                # 計算警告文字位置（畫面上方中央）
                (warn_width, warn_height), warn_baseline = cv2.getTextSize(
                    warning_text, font, warning_font_scale, warning_thickness
                )

                warn_x = (cv_image.shape[1] - warn_width) // 2
                warn_y = 60

                # 繪製警告文字背景
                cv2.rectangle(cv_image,
                              (warn_x - 10, warn_y - warn_height - 10),
                              (warn_x + warn_width + 10, warn_y + 10),
                              (0, 0, 0), -1)

                # 繪製警告文字
                cv2.putText(cv_image, warning_text, (warn_x, warn_y),
                            font, warning_font_scale, (0, 0, 255), warning_thickness)

    def image_callback(self, msg):
        """處理圖像回調函數（修改版本）"""
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
                # 繪製檢測框和距離資訊
                self.draw_detections(cv_image, detections)

                # 更新俯視圖
                self.update_overhead_view(detections, cv_image.shape[1])

            # 顯示主圖像
            cv2.imshow('HP60C Camera - Person Detection with Distance', cv_image)

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
        print(f"程式關閉統計:")
        print(f"  - 處理的RGB幀數: {self.frame_count}")
        print(f"  - 收到的深度幀數: {self.depth_callback_count}")
        print(f"  - 檢測到人物的幀數: {self.detection_count}")

        # 關閉定時器
        if hasattr(self, 'check_timer'):
            self.check_timer.shutdown()

        cv2.destroyAllWindows()
        rospy.signal_shutdown("使用者關閉")


# ========== 主程式 ==========
def main():
    """主程式進入點"""
    try:
        print("啟動PyCharm HP60C相機人物檢測+深度測距+俯視圖程式...")
        print("Starting HP60C Camera Person Detection + Distance + Overhead View...")

        # 檢查YOLOv8是否可用
        if not YOLO_AVAILABLE:
            print("Warning: YOLOv8 not available, showing camera feed only")

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
        print("HP60C Camera Person Detection + Distance Measurement + Overhead View Running!")
        print("Controls: 'q'=Quit, 's'=Save Screenshot")
        print("Status Colors: Red=Warning(≤1.2m), Yellow=Caution(1.2-1.6m), Green=Safe(>1.6m)")

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