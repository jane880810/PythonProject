#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HP60C 深度相機顯示 - YOLOv8人物辨識 + 機械手臂座標系
原點 (0,0,0) = 機械手臂基座中心（相機前方 205cm）
每人顯示距手臂原點最近的人體像素點 XYZ 座標
偵測與顯示更新頻率：0.5 秒一次
"""

import sys
import os
import ctypes
import time
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

# ========== 相機參數 ==========
CAMERA_HFOV_DEG = 69.0
CAMERA_VFOV_DEG = 42.0
CAMERA_FX = None
CAMERA_FY = None
CAMERA_CX = None
CAMERA_CY = None

# ========== 原點設定 ==========
# 機械手臂基座 = 相機前方 205cm 處
ORIGIN_DEPTH_CM = 205.0

# ========== 偵測更新間隔 ==========
DETECTION_INTERVAL = 0.5   # 秒

# ========== 主類別 ==========

class PyCharmHP60CViewer:

    def __init__(self):
        print("初始化 HP60C 機械手臂安全監控...")
        rospy.init_node('pycharm_hp60c_viewer', anonymous=True)

        if USE_CV_BRIDGE:
            self.bridge = CvBridge()

        # 相機內參
        self.image_width = self.image_height = None
        self.fx = self.fy = self.cx = self.cy = None

        # YOLOv8（使用 seg 模型取得人體遮罩）
        self.yolo_model = None
        if YOLO_AVAILABLE:
            try:
                print("載入YOLOv8n-seg模型...")
                self.yolo_model = YOLO('yolov8n-seg.pt')
                print("YOLOv8n-seg模型載入成功")
            except Exception as e:
                print(f"seg模型載入失敗，改用detection模型: {e}")
                try:
                    self.yolo_model = YOLO('yolov8n.pt')
                    print("YOLOv8n模型載入成功（無遮罩功能）")
                except Exception as e2:
                    print(f"模型載入失敗: {e2}")

        # 深度圖
        self.depth_image = None
        self.depth_lock = threading.Lock()

        # 計數
        self.frame_count = 0
        self.depth_callback_count = 0

        # ── 0.5秒更新控制 ──
        self.last_detect_time = 0.0
        # 快取上一次的偵測結果，畫面每幀都用這份資料繪製
        self.cached_draw_data = []   # list of dict，每人一筆

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
                self.depth_sub = rospy.Subscriber(
                    topic, Image, self.depth_callback, queue_size=1)
                self.depth_topic_name = topic
                print(f"訂閱深度話題: {topic}")
                break
            except Exception:
                continue

        self.check_timer = rospy.Timer(rospy.Duration(5.0), self.check_depth_status)
        print("\n操作說明: q/ESC=退出  s=儲存截圖")
        print(f"原點(0,0,0) = 機械手臂基座 (相機前方 {ORIGIN_DEPTH_CM:.0f}cm)")
        print(f"偵測更新頻率: 每 {DETECTION_INTERVAL} 秒一次")

    # ---------- 相機內參 ----------

    def init_camera_intrinsics(self, width, height):
        self.image_width, self.image_height = width, height
        if CAMERA_FX is not None:
            self.fx, self.fy = CAMERA_FX, CAMERA_FY
            self.cx = CAMERA_CX or width / 2.0
            self.cy = CAMERA_CY or height / 2.0
        else:
            self.fx = (width / 2.0) / math.tan(math.radians(CAMERA_HFOV_DEG / 2.0))
            self.fy = (height / 2.0) / math.tan(math.radians(CAMERA_VFOV_DEG / 2.0))
            self.cx, self.cy = width / 2.0, height / 2.0
        print(f"相機內參: fx={self.fx:.1f} fy={self.fy:.1f} cx={self.cx:.1f} cy={self.cy:.1f}")

    # ---------- 深度轉換 ----------

    def depth_array_to_cm(self, raw_array, dtype):
        """將整個深度陣列批次轉換為公分（float32）"""
        if dtype == np.float32:
            cm = raw_array.astype(np.float32) * 100.0   # m → cm
        else:
            cm = raw_array.astype(np.float32) / 10.0    # mm → cm
        return cm

    def raw_depth_to_cm(self, raw, dtype):
        if dtype == np.float32:
            cm = float(raw) * 100.0
        else:
            cm = float(raw) / 10.0
        if cm < 5:
            alt = float(raw)
            if 5 <= alt <= 1500:
                return alt
        if cm > 1500:
            alt = float(raw) / 1000.0
            if 5 <= alt <= 1500:
                return alt
        return cm

    def get_depth_at_pixel(self, px, py, depth_image):
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

    # ---------- 核心：矩陣計算人體最近點 ----------

    def find_closest_point(self, mask_bool, depth_image):
        """
        輸入人體遮罩（bool H×W）與深度圖，
        用矩陣運算一次算出所有人體像素的 XYZ 及到原點距離，
        回傳距離手臂原點最近的點資訊。
        回傳 dict: {X, Y, Z, dist, px, py} 或 None
        """
        if depth_image is None or self.fx is None:
            return None

        # 取出所有人體像素座標
        ys, xs = np.where(mask_bool)
        if len(xs) == 0:
            return None

        # 批次取深度並轉換為公分
        raw_depths = depth_image[ys, xs]
        depths_cm = self.depth_array_to_cm(raw_depths, depth_image.dtype)

        # 過濾無效深度（0 或超出合理範圍）
        valid = (depths_cm > 5) & (depths_cm < 1500)
        if not np.any(valid):
            return None
        xs, ys, depths_cm = xs[valid], ys[valid], depths_cm[valid]

        # 矩陣計算所有像素的 XYZ（相對手臂原點）
        X_all = depths_cm - ORIGIN_DEPTH_CM
        Y_all = (xs - self.cx) / self.fx * depths_cm
        Z_all = -((ys - self.cy) / self.fy * depths_cm)

        # 計算每個像素到原點 (0,0,0) 的歐式距離
        dist_all = np.sqrt(X_all**2 + Y_all**2 + Z_all**2)

        # 找最近點
        min_idx = np.argmin(dist_all)

        return {
            'X':    float(X_all[min_idx]),
            'Y':    float(Y_all[min_idx]),
            'Z':    float(Z_all[min_idx]),
            'dist': float(dist_all[min_idx]),
            'px':   int(xs[min_idx]),
            'py':   int(ys[min_idx]),
        }

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
                        if self.depth_sub:
                            self.depth_sub.unregister()
                        self.depth_sub = rospy.Subscriber(
                            alts[0], Image, self.depth_callback, queue_size=1)
                        self.depth_topic_name = alts[0]
                        print(f"改訂深度話題: {alts[0]}")
            except Exception as e:
                print(f"話題檢查錯誤: {e}")
        else:
            self.check_timer.shutdown()

    # ---------- YOLO 偵測（含 seg 遮罩）----------

    def detect_persons(self, cv_image):
        """
        回傳 list of dict，每人包含:
          bbox: (x1,y1,x2,y2)
          mask: bool ndarray H×W（若模型無 seg 則為 None，改用 bbox 區域）
          confidence: float
        """
        if self.yolo_model is None:
            return []
        try:
            results = self.yolo_model(cv_image, classes=[0], imgsz=320, verbose=False)
            detections = []
            h_img, w_img = cv_image.shape[:2]
            for result in results:
                if result.boxes is None:
                    continue
                has_masks = (result.masks is not None)
                for i, box in enumerate(result.boxes):
                    if int(box.cls[0]) != 0 or float(box.conf[0]) < 0.5:
                        continue
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    bbox = (int(x1), int(y1), int(x2), int(y2))

                    # 取 seg 遮罩
                    mask_bool = None
                    if has_masks:
                        try:
                            raw_mask = result.masks.data[i].cpu().numpy()
                            resized = cv2.resize(
                                raw_mask, (w_img, h_img),
                                interpolation=cv2.INTER_LINEAR)
                            mask_bool = resized > 0.5
                        except Exception:
                            mask_bool = None

                    # 若無遮罩，用 bbox 區域代替
                    if mask_bool is None:
                        mask_bool = np.zeros((h_img, w_img), dtype=bool)
                        mask_bool[int(y1):int(y2), int(x1):int(x2)] = True

                    detections.append({
                        'bbox': bbox,
                        'mask': mask_bool,
                        'confidence': float(box.conf[0])
                    })
            return detections
        except Exception as e:
            print(f"YOLOv8錯誤: {e}")
            return []

    # ---------- 0.5秒偵測排程 ----------

    def run_detection_if_due(self, cv_image, cur_depth):
        """
        每 DETECTION_INTERVAL 秒執行一次偵測與最近點計算，
        結果存入 self.cached_draw_data。
        """
        now = time.time()
        if now - self.last_detect_time < DETECTION_INTERVAL:
            return
        self.last_detect_time = now

        detections = self.detect_persons(cv_image)
        new_cache = []

        for det in detections:
            closest = self.find_closest_point(det['mask'], cur_depth)
            new_cache.append({
                'bbox':     det['bbox'],
                'mask':     det['mask'],
                'closest':  closest,   # dict 或 None
            })

        self.cached_draw_data = new_cache

    # ---------- 繪製 ----------

    def draw_cached_results(self, cv_image):
        """用快取的偵測結果繪製到當前幀，不重新計算"""
        for data in self.cached_draw_data:
            x1, y1, x2, y2 = data['bbox']

            # 綠色邊界框
            cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

            closest = data['closest']
            if closest is not None:
                px, py = closest['px'], closest['py']
                X, Y, Z = closest['X'], closest['Y'], closest['Z']
                dist = closest['dist']

                # 最近點綠點
                cv2.circle(cv_image, (px, py), 5, (0, 255, 0), -1)

                # 標籤：XYZ + 到原點距離
                label = f"X:{X:+.0f} Y:{Y:+.0f} Z:{Z:+.0f}  D:{dist:.0f}cm"
                self._put_label(cv_image, label, px, py)
            else:
                # 無深度資料時只顯示框
                cpx = (x1 + x2) // 2
                cpy = (y1 + y2) // 2
                self._put_label(cv_image, "等待深度資料", cpx, cpy)

    def _put_label(self, img, text, cx, cy):
        font, fs, ft = cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1
        color_text, color_bg = (0, 255, 0), (0, 0, 0)
        (tw, th), bl = cv2.getTextSize(text, font, fs, ft)
        h, w = img.shape[:2]
        tx = cx + 10 if cx + 10 + tw < w else cx - tw - 10
        ty = cy + th + 10 if cy + th + 10 < h else cy - 10
        cv2.rectangle(img, (tx-2, ty-th-2), (tx+tw+2, ty+bl+2), color_bg, -1)
        cv2.putText(img, text, (tx, ty), font, fs, color_text, ft, cv2.LINE_AA)

    def draw_origin_crosshair(self, img):
        if self.cx is None:
            return
        cx, cy = int(self.cx), int(self.cy)
        c, s = (0, 0, 255), 15
        cv2.line(img, (cx-s, cy), (cx+s, cy), c, 1)
        cv2.line(img, (cx, cy-s), (cx, cy+s), c, 1)
        cv2.circle(img, (cx, cy), 3, c, -1)
        cv2.putText(img, f"(0,0,0) @{ORIGIN_DEPTH_CM:.0f}cm",
                    (cx+8, cy-6), cv2.FONT_HERSHEY_SIMPLEX, 0.38, c, 1, cv2.LINE_AA)

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

            # 取目前深度圖快照
            with self.depth_lock:
                cur_depth = self.depth_image.copy() if self.depth_image is not None else None

            # 每 0.5 秒執行一次偵測
            self.run_detection_if_due(cv_image, cur_depth)

            # 每幀都繪製（用快取資料，不卡頓）
            self.draw_origin_crosshair(cv_image)
            self.draw_cached_results(cv_image)

            cv2.imshow('HP60C - Robot Arm Safety Monitor', cv_image)

            if self.frame_count == 1:
                cv2.imwrite('/tmp/hp60c_first_frame.jpg', cv_image)

            self._handle_key(cv_image)

        except Exception as e:
            import traceback
            print(f"圖像處理錯誤: {e}")
            traceback.print_exc()

    def _manual_rgb(self, msg):
        if msg.encoding == "bgr8":
            return np.frombuffer(msg.data, dtype=np.uint8).reshape(
                (msg.height, msg.width, 3))
        elif msg.encoding == "rgb8":
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                (msg.height, msg.width, 3))
            return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        return None

    def _handle_key(self, cv_image):
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            print(f"\n退出 - RGB:{self.frame_count}幀  深度:{self.depth_callback_count}幀")
            self.shutdown()
        elif key == ord('s'):
            ts = rospy.Time.now().to_sec()
            fn = f'/tmp/hp60c_{ts:.0f}.jpg'
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
        print("HP60C 機械手臂安全監控")
        print(f"原點(0,0,0) = 機械手臂基座 (前方 {ORIGIN_DEPTH_CM:.0f}cm)")
        print(f"偵測更新: 每 {DETECTION_INTERVAL} 秒")
        print(f"FOV: H={CAMERA_HFOV_DEG}°  V={CAMERA_VFOV_DEG}°")
        print("顯示格式: X(深度) Y(水平) Z(垂直)  D=距原點距離")
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