#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HP60C 深度相機顯示 - YOLOv8人物辨識 + XYZ座標顯示
座標系原點 = 畫面中心方向、深度 205cm 處 (0,0,0)
  X = 深度方向（公分，正=遠於205cm，負=近於205cm）
  Y = 水平方向（公分，左負右正）
  Z = 垂直方向（公分，上正下負）
"""

import sys
import os
import ctypes
import numpy as np
import cv2
import threading
import math

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
    print("YOLOv8庫載入成功")
except ImportError as e:
    print(f"YOLOv8庫載入失敗: {e}\n請執行: pip install ultralytics")
    YOLO_AVAILABLE = False

# ========== 環境設定 ==========

def setup_ros_environment():
    os.environ['ROS_MASTER_URI'] = 'http://192.168.40.128:11311'
    os.environ['ROS_IP'] = '192.168.40.128'
    os.environ['ROSCONSOLE_CONFIG_FILE'] = '/opt/ros/noetic/etc/ros/rosconsole.config'
    ros_paths = [
        "/opt/ros/noetic/lib/python3/dist-packages",
        "/home/yahboom/ascam_ws/devel/lib/python3/dist-packages",
        "/usr/lib/python3/dist-packages",
        "/usr/local/lib/python3.8/dist-packages",
    ]
    for path in ros_paths:
        if os.path.exists(path) and path not in sys.path:
            sys.path.insert(0, path)
    lib_paths = ["/opt/ros/noetic/lib", "/usr/lib/x86_64-linux-gnu"]
    cur = os.environ.get('LD_LIBRARY_PATH', '')
    os.environ['LD_LIBRARY_PATH'] = ':'.join(lib_paths) + (':' + cur if cur else '')
    print("ROS環境設定完成")

def load_shared_libraries():
    for lib_path in ['/opt/ros/noetic/lib/libcv_bridge.so',
                     '/opt/ros/noetic/lib/libopencv_core.so']:
        if os.path.exists(lib_path):
            try:
                ctypes.CDLL(lib_path)
            except Exception:
                pass

setup_ros_environment()
load_shared_libraries()

try:
    import rospy
    from sensor_msgs.msg import Image
    print("ROS模組匯入成功")
    try:
        from cv_bridge import CvBridge
        USE_CV_BRIDGE = True
        print("cv_bridge匯入成功")
    except Exception as e:
        print(f"cv_bridge匯入失敗: {e}，使用手動轉換")
        USE_CV_BRIDGE = False
except ImportError as e:
    print(f"ROS模組匯入失敗: {e}")
    sys.exit(1)

# ========== 相機參數（依實際相機調整）==========
# HP60C 預設 FOV，若有 /camera_info 話題可取得更精確值
CAMERA_HFOV_DEG = 69.0   # 水平視角（度）
CAMERA_VFOV_DEG = 42.0   # 垂直視角（度）
# 若已知相機內參可直接設定（優先使用，設 None 則由 FOV 估算）
CAMERA_FX = None   # 焦距 fx（像素）
CAMERA_FY = None   # 焦距 fy（像素）
CAMERA_CX = None   # 光心 cx（像素），None = 影像寬/2
CAMERA_CY = None   # 光心 cy（像素），None = 影像高/2

# ========== 座標原點設定 ==========
# 原點定義：畫面中心方向、距相機 ORIGIN_DEPTH_CM 公分處為 (0,0,0)
# 所有 XYZ 數值皆為相對此原點的偏移量
ORIGIN_DEPTH_CM = 205.0   # 原點深度（公分）

# ========== 主類別 ==========

class PyCharmHP60CViewer:

    def __init__(self):
        print("初始化 HP60C XYZ 座標顯示器...")
        rospy.init_node('pycharm_hp60c_viewer', anonymous=True)

        if USE_CV_BRIDGE:
            self.bridge = CvBridge()

        # 相機內參（首次收圖後初始化）
        self.image_width = self.image_height = None
        self.fx = self.fy = self.cx = self.cy = None

        # YOLOv8
        self.yolo_model = None
        if YOLO_AVAILABLE:
            try:
                print("載入YOLOv8n模型...")
                self.yolo_model = YOLO('yolov8n.pt')
                print("YOLOv8模型載入成功")
            except Exception as e:
                print(f"YOLOv8模型載入失敗: {e}")

        # 深度圖
        self.depth_image = None
        self.depth_lock = threading.Lock()

        # 計數
        self.frame_count = 0
        self.detection_count = 0
        self.depth_callback_count = 0

        # 訂閱
        self.image_sub = rospy.Subscriber(
            '/ascamera_hp60c/rgb0/image', Image, self.image_callback, queue_size=1)

        self.depth_topic_name = None
        self.depth_sub = None
        for topic in ['/ascamera_hp60c/depth/image',
                      '/ascamera_hp60c/depth0/image',
                      '/ascamera_hp60c/depth/image_raw',
                      '/ascamera_hp60c/depth0/image_raw',
                      '/camera/depth/image_raw',
                      '/camera/aligned_depth_to_color/image_raw']:
            try:
                self.depth_sub = rospy.Subscriber(topic, Image, self.depth_callback, queue_size=1)
                self.depth_topic_name = topic
                print(f"訂閱深度話題: {topic}")
                break
            except Exception:
                continue

        self.check_timer = rospy.Timer(rospy.Duration(5.0), self.check_depth_status)
        print("\n操作說明: q/ESC=退出  s=儲存截圖")
        print(f"畫面中心紅色十字 = 座標原點(0,0,0)，深度基準={ORIGIN_DEPTH_CM:.0f}cm")
        print("綠點 = 人物中心，標籤格式: X(深度偏移) Y(水平) Z(垂直) 單位:公分")

    # ---------- 相機內參 ----------

    def init_camera_intrinsics(self, width, height):
        self.image_width, self.image_height = width, height
        if CAMERA_FX is not None:
            self.fx, self.fy = CAMERA_FX, CAMERA_FY
            self.cx = CAMERA_CX or width / 2.0
            self.cy = CAMERA_CY or height / 2.0
            print(f"相機內參(設定值): fx={self.fx:.1f} fy={self.fy:.1f} cx={self.cx:.1f} cy={self.cy:.1f}")
        else:
            self.fx = (width / 2.0) / math.tan(math.radians(CAMERA_HFOV_DEG / 2.0))
            self.fy = (height / 2.0) / math.tan(math.radians(CAMERA_VFOV_DEG / 2.0))
            self.cx, self.cy = width / 2.0, height / 2.0
            print(f"相機內參(FOV估算): fx={self.fx:.1f} fy={self.fy:.1f} cx={self.cx:.1f} cy={self.cy:.1f}")

    # ---------- 座標轉換 ----------

    def pixel_to_xyz(self, px, py, depth_cm):
        """
        針孔相機反投影，以畫面中心、深度 ORIGIN_DEPTH_CM 處為原點 (0,0,0)
        X = 深度偏移（正=遠於原點），Y = 水平偏移（右正），Z = 垂直偏移（上正）
        """
        if self.fx is None or depth_cm is None:
            return None

        # 將人物投影到原點深度平面上求水平/垂直位移
        # 先求人物在相機座標系下的 3D 位置
        person_X = depth_cm
        person_Y = (px - self.cx) / self.fx * depth_cm
        person_Z = -((py - self.cy) / self.fy * depth_cm)

        # 原點在相機座標系下的位置（畫面正中心、深度205cm）
        origin_X = ORIGIN_DEPTH_CM
        origin_Y = 0.0   # 畫面中心 → px=cx → Y=0
        origin_Z = 0.0   # 畫面中心 → py=cy → Z=0

        # 相對偏移
        X = person_X - origin_X
        Y = person_Y - origin_Y
        Z = person_Z - origin_Z

        return (X, Y, Z)

    # ---------- 深度讀取 ----------

    def raw_depth_to_cm(self, raw, dtype):
        if dtype == np.float32:
            cm = float(raw) * 100.0   # m → cm
        else:
            cm = float(raw) / 10.0    # mm → cm (uint16)
        # 合理性修正
        if cm < 5:
            alt = float(raw)  # 可能已是 cm
            if 5 <= alt <= 1500:
                return alt
        if cm > 1500:
            alt = float(raw) / 1000.0
            if 5 <= alt <= 1500:
                return alt
        return cm

    def get_depth_at_pixel(self, px, py, depth_image):
        """取得像素位置深度（公分），使用中位數以提高穩定性"""
        if depth_image is None:
            return None
        h, w = depth_image.shape[:2]
        px, py = max(0, min(w-1, int(px))), max(0, min(h-1, int(py)))
        for r in [10, 20, 35]:
            region = depth_image[max(0,py-r):min(h,py+r), max(0,px-r):min(w,px+r)]
            valid = region[region > 0]
            if len(valid) >= 3:
                return self.raw_depth_to_cm(np.median(valid), depth_image.dtype)
        return None

    # ---------- 深度話題回調 ----------

    def depth_callback(self, msg):
        try:
            self.depth_callback_count += 1
            if self.depth_callback_count == 1:
                print(f"首個深度幀: {msg.width}x{msg.height} 編碼:{msg.encoding}")

            if USE_CV_BRIDGE:
                try:
                    enc = msg.encoding if msg.encoding in ("16UC1","32FC1") else "passthrough"
                    depth_image = self.bridge.imgmsg_to_cv2(msg, enc)
                except Exception:
                    depth_image = self._manual_depth(msg)
            else:
                depth_image = self._manual_depth(msg)

            if depth_image is not None:
                with self.depth_lock:
                    self.depth_image = depth_image.copy()
        except Exception as e:
            print(f"深度回調錯誤: {e}")

    def _manual_depth(self, msg):
        dtype_map = {"16UC1": np.uint16, "mono16": np.uint16,
                     "32FC1": np.float32, "8UC1": np.uint8}
        dtype = dtype_map.get(msg.encoding, np.uint16)
        try:
            arr = np.frombuffer(msg.data, dtype=dtype).reshape((msg.height, msg.width))
            return arr.astype(np.uint16) if dtype == np.uint8 else arr
        except Exception:
            return None

    # ---------- 深度狀態檢查 ----------

    def check_depth_status(self, event):
        if self.depth_callback_count == 0:
            print(f"警告: 尚未收到深度資料 (話題: {self.depth_topic_name})")
            try:
                tts = rospy.get_published_topics()
                if not any(t == self.depth_topic_name for t, _ in tts):
                    alts = [t for t, mt in tts if 'depth' in t.lower() and 'Image' in mt]
                    if alts:
                        print(f"找到替代深度話題: {alts[0]}，重新訂閱...")
                        if self.depth_sub:
                            self.depth_sub.unregister()
                        self.depth_sub = rospy.Subscriber(alts[0], Image, self.depth_callback, queue_size=1)
                        self.depth_topic_name = alts[0]
            except Exception as e:
                print(f"話題檢查錯誤: {e}")
        else:
            self.check_timer.shutdown()

    # ---------- YOLO檢測 ----------

    def detect_persons(self, cv_image):
        if self.yolo_model is None:
            return []
        try:
            results = self.yolo_model(cv_image, verbose=False)
            detections = []
            for result in results:
                if result.boxes is None:
                    continue
                for box in result.boxes:
                    if int(box.cls[0]) == 0 and float(box.conf[0]) > 0.5:
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        detections.append({
                            'bbox': (int(x1), int(y1), int(x2), int(y2)),
                            'confidence': float(box.conf[0])
                        })
            return detections
        except Exception as e:
            print(f"YOLOv8錯誤: {e}")
            return []

    # ---------- 繪製 ----------

    def draw_detections(self, cv_image, detections):
        with self.depth_lock:
            cur_depth = self.depth_image.copy() if self.depth_image is not None else None

        for det in detections:
            x1, y1, x2, y2 = det['bbox']

            # 綠色邊界框
            cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # 中心點座標（像素）
            cpx = (x1 + x2) // 2
            cpy = (y1 + y2) // 2

            # 綠色實心中心點
            cv2.circle(cv_image, (cpx, cpy), 5, (0, 255, 0), -1)

            # 計算 XYZ
            label = "X:-- Y:-- Z:-- cm"
            if self.fx is not None:
                depth_cm = self.get_depth_at_pixel(cpx, cpy, cur_depth)
                if depth_cm is not None:
                    xyz = self.pixel_to_xyz(cpx, cpy, depth_cm)
                    if xyz:
                        X, Y, Z = xyz
                        label = f"X:{X:.0f} Y:{Y:+.0f} Z:{Z:+.0f} cm"
                elif cur_depth is None:
                    label = "等待深度資料"

            # 文字標籤（帶黑底）
            self._put_label(cv_image, label, cpx, cpy)

    def _put_label(self, img, text, cx, cy):
        font, fs, ft = cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1
        color_text, color_bg = (0, 255, 0), (0, 0, 0)
        (tw, th), bl = cv2.getTextSize(text, font, fs, ft)
        h, w = img.shape[:2]
        tx = cx + 10 if cx + 10 + tw < w else cx - tw - 10
        ty = cy + th + 10 if cy + th + 10 < h else cy - 10
        # 黑底
        cv2.rectangle(img, (tx-2, ty-th-2), (tx+tw+2, ty+bl+2), color_bg, -1)
        cv2.putText(img, text, (tx, ty), font, fs, color_text, ft, cv2.LINE_AA)

    def draw_origin_crosshair(self, img):
        """畫面中心紅色十字，代表深度 ORIGIN_DEPTH_CM 處的座標原點"""
        if self.cx is None:
            return
        cx, cy = int(self.cx), int(self.cy)
        c, s = (0, 0, 255), 15   # 紅色
        cv2.line(img, (cx-s, cy), (cx+s, cy), c, 1)
        cv2.line(img, (cx, cy-s), (cx, cy+s), c, 1)
        cv2.circle(img, (cx, cy), 3, c, -1)
        cv2.putText(img, f"(0,0,0) @{ORIGIN_DEPTH_CM:.0f}cm", (cx+8, cy-6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, c, 1, cv2.LINE_AA)

    # ---------- 圖像回調 ----------

    def image_callback(self, msg):
        try:
            self.frame_count += 1
            if self.fx is None:
                self.init_camera_intrinsics(msg.width, msg.height)

            if USE_CV_BRIDGE:
                try:
                    cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                except Exception:
                    cv_image = self._manual_rgb(msg)
            else:
                cv_image = self._manual_rgb(msg)

            if cv_image is None:
                return

            # 畫原點十字
            self.draw_origin_crosshair(cv_image)

            # 人物偵測與XYZ繪製
            detections = self.detect_persons(cv_image)
            if detections:
                self.detection_count += 1
                self.draw_detections(cv_image, detections)

            cv2.imshow('HP60C - XYZ Person Tracking', cv_image)

            if self.frame_count == 1:
                cv2.imwrite('/tmp/hp60c_xyz_first_frame.jpg', cv_image)

            self._handle_key(cv_image)

        except Exception as e:
            import traceback
            print(f"圖像處理錯誤: {e}")
            traceback.print_exc()

    def _manual_rgb(self, msg):
        if msg.encoding == "bgr8":
            return np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
        elif msg.encoding == "rgb8":
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
            return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        return None

    def _handle_key(self, cv_image):
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            print(f"\n退出 - RGB:{self.frame_count}幀 深度:{self.depth_callback_count}幀 偵測:{self.detection_count}次")
            self.shutdown()
        elif key == ord('s'):
            ts = rospy.Time.now().to_sec()
            fn = f'/tmp/hp60c_xyz_{ts:.0f}.jpg'
            cv2.imwrite(fn, cv_image)
            print(f"截圖儲存: {fn}")

    def shutdown(self):
        if hasattr(self, 'check_timer'):
            self.check_timer.shutdown()
        cv2.destroyAllWindows()
        rospy.signal_shutdown("使用者關閉")


# ========== 主程式 ==========

def main():
    try:
        print("=" * 55)
        print("HP60C 人物 XYZ 座標追蹤程式")
        print(f"座標原點: 畫面中心方向、深度 {ORIGIN_DEPTH_CM:.0f}cm 處 = (0,0,0)")
        print("  X = 深度偏移 (正=遠於原點，負=近於原點，公分)")
        print("  Y = 水平偏移 (正=右，負=左，公分)")
        print("  Z = 垂直偏移 (正=上，負=下，公分)")
        print(f"  FOV設定: H={CAMERA_HFOV_DEG}°  V={CAMERA_VFOV_DEG}°")
        print("  提示: 修改 ORIGIN_DEPTH_CM 可調整原點深度")
        print("=" * 55)

        if not YOLO_AVAILABLE:
            print("警告: YOLOv8不可用")

        try:
            rospy.get_master().getPid()
            print("ROS Master 連線正常")
        except:
            print("無法連線到 ROS Master！請確認 roscore 已啟動")
            return

        viewer = PyCharmHP60CViewer()
        rospy.spin()

    except rospy.ROSInterruptException:
        print("ROS中斷")
    except KeyboardInterrupt:
        print("Ctrl+C")
    except Exception as e:
        import traceback
        print(f"程式錯誤: {e}")
        traceback.print_exc()
    finally:
        cv2.destroyAllWindows()
        print("程式關閉")

if __name__ == '__main__':
    main()