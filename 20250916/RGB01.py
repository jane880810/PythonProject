#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HP60C 深度相机显示 - PyCharm兼容版本
解决cv_bridge库加载问题
"""

import sys
import os
import subprocess
import ctypes
import numpy as np
import cv2


# ========== 环境设置 ==========
def setup_ros_environment():
    """设置ROS环境"""
    print("正在配置ROS环境...")

    # 设置环境变量
    os.environ['ROS_MASTER_URI'] = 'http://192.168.40.128:11311'
    os.environ['ROS_IP'] = '192.168.40.128'

    # 添加ROS路径到Python path
    ros_paths = [
        "/opt/ros/noetic/lib/python3/dist-packages",
        "/home/yahboom/ascam_ws/devel/lib/python3/dist-packages"
    ]

    for path in ros_paths:
        if os.path.exists(path) and path not in sys.path:
            sys.path.insert(0, path)
            print(f"已添加路径: {path}")

    # 设置库路径
    lib_paths = [
        "/opt/ros/noetic/lib",
        "/usr/lib/x86_64-linux-gnu"
    ]

    current_ld_path = os.environ.get('LD_LIBRARY_PATH', '')
    new_ld_path = ':'.join(lib_paths)
    if current_ld_path:
        new_ld_path = new_ld_path + ':' + current_ld_path
    os.environ['LD_LIBRARY_PATH'] = new_ld_path

    print("ROS环境配置完成")


def load_shared_libraries():
    """预加载必要的共享库"""
    try:
        # 尝试加载cv_bridge相关库
        lib_paths = [
            '/opt/ros/noetic/lib/libcv_bridge.so',
            '/opt/ros/noetic/lib/libopencv_core.so',
            '/opt/ros/noetic/lib/libopencv_imgproc.so'
        ]

        for lib_path in lib_paths:
            if os.path.exists(lib_path):
                try:
                    ctypes.CDLL(lib_path)
                    print(f"成功加载库: {lib_path}")
                except Exception as e:
                    print(f"库加载警告 {lib_path}: {e}")
    except Exception as e:
        print(f"库加载过程出现问题: {e}")


# 配置环境
setup_ros_environment()
load_shared_libraries()

# ========== 导入ROS模块 ==========
try:
    import rospy
    from sensor_msgs.msg import Image

    print("ROS模块导入成功")

    # 尝试导入cv_bridge，如果失败则使用手动转换
    try:
        from cv_bridge import CvBridge

        USE_CV_BRIDGE = True
        print("cv_bridge导入成功")
    except Exception as e:
        print(f"cv_bridge导入失败: {e}")
        print("将使用手动图像转换")
        USE_CV_BRIDGE = False

except ImportError as e:
    print(f"ROS模块导入失败: {e}")
    sys.exit(1)


# ========== 相机显示类 ==========
class PyCharmHP60CViewer:
    def __init__(self):
        print("初始化PyCharm HP60C相机显示器...")

        # 初始化ROS节点
        rospy.init_node('pycharm_hp60c_viewer', anonymous=True)

        # 初始化cv_bridge（如果可用）
        if USE_CV_BRIDGE:
            self.bridge = CvBridge()

        # 图像计数器
        self.frame_count = 0
        self.save_next_frame = False

        # 订阅RGB图像话题
        self.image_sub = rospy.Subscriber(
            '/ascamera_hp60c/rgb0/image',
            Image,
            self.image_callback,
            queue_size=1
        )

        print("PyCharm HP60C显示器初始化完成")
        print("操作说明:")
        print("- 按 'q' 或 ESC 退出")
        print("- 按 's' 保存截图")
        print("- 按 'i' 显示图像信息")

    def manual_image_convert(self, msg):
        """手动转换ROS Image到OpenCV格式"""
        if msg.encoding == "bgr8":
            # BGR8格式：每像素3字节
            np_arr = np.frombuffer(msg.data, dtype=np.uint8)
            cv_image = np_arr.reshape((msg.height, msg.width, 3))
            return cv_image
        elif msg.encoding == "rgb8":
            # RGB8格式：转换为BGR
            np_arr = np.frombuffer(msg.data, dtype=np.uint8)
            rgb_image = np_arr.reshape((msg.height, msg.width, 3))
            cv_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
            return cv_image
        else:
            print(f"不支持的图像编码: {msg.encoding}")
            return None

    def image_callback(self, msg):
        """处理图像回调"""
        try:
            self.frame_count += 1

            # 转换图像
            if USE_CV_BRIDGE:
                try:
                    cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                    conversion_method = "cv_bridge"
                except Exception as e:
                    print(f"cv_bridge转换失败，使用手动转换: {e}")
                    cv_image = self.manual_image_convert(msg)
                    conversion_method = "manual"
            else:
                cv_image = self.manual_image_convert(msg)
                conversion_method = "manual"

            if cv_image is None:
                return

            # 添加信息覆盖
            self.add_info_overlay(cv_image, msg, conversion_method)

            # 显示图像
            cv2.imshow('PyCharm HP60C Camera', cv_image)

            # 自动保存第一帧用于验证
            if self.frame_count == 1:
                cv2.imwrite('/tmp/pycharm_hp60c_first_frame.jpg', cv_image)
                print("已保存第一帧到 /tmp/pycharm_hp60c_first_frame.jpg")

            # 处理按键
            self.handle_keyboard(cv_image, msg)

        except Exception as e:
            print(f"图像处理错误: {e}")
            import traceback
            traceback.print_exc()

    def add_info_overlay(self, image, msg, conversion_method):
        """添加信息覆盖层"""
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        color = (0, 255, 0)
        thickness = 2

        # 信息文本
        info_lines = [
            f"PyCharm HP60C Camera",
            f"Size: {msg.width}x{msg.height}",
            f"Encoding: {msg.encoding}",
            f"Method: {conversion_method}",
            f"Frame: {self.frame_count}",
            f"Press 'q' to exit, 's' to save"
        ]

        # 绘制半透明背景
        overlay = image.copy()
        cv2.rectangle(overlay, (10, 10), (350, 160), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, image, 0.3, 0, image)

        # 绘制文本
        for i, line in enumerate(info_lines):
            y_pos = 30 + i * 20
            cv2.putText(image, line, (20, y_pos), font, font_scale, color, thickness)

    def handle_keyboard(self, cv_image, msg):
        """处理键盘输入"""
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q') or key == 27:  # 'q' 或 ESC
            print("退出程序...")
            self.shutdown()
        elif key == ord('s'):  # 保存截图
            self.save_screenshot(cv_image)
        elif key == ord('i'):  # 显示图像信息
            self.show_image_info(cv_image, msg)

    def save_screenshot(self, cv_image):
        """保存截图"""
        timestamp = rospy.Time.now().to_sec()
        filename = f'/tmp/pycharm_hp60c_screenshot_{timestamp:.0f}.jpg'

        if cv2.imwrite(filename, cv_image):
            print(f"截图已保存: {filename}")
        else:
            print("截图保存失败!")

    def show_image_info(self, cv_image, msg):
        """显示详细图像信息"""
        print("\n" + "=" * 50)
        print("图像详细信息:")
        print(f"  ROS消息尺寸: {msg.width} x {msg.height}")
        print(f"  OpenCV形状: {cv_image.shape}")
        print(f"  编码格式: {msg.encoding}")
        print(f"  数据类型: {cv_image.dtype}")
        print(f"  像素值范围: {cv_image.min()} - {cv_image.max()}")
        print(f"  内存大小: {cv_image.nbytes} bytes")
        print(f"  当前帧数: {self.frame_count}")
        print("=" * 50 + "\n")

    def shutdown(self):
        """安全关闭"""
        cv2.destroyAllWindows()
        rospy.signal_shutdown("用户关闭")


# ========== 主程序 ==========
def main():
    """主程序入口"""
    try:
        print("启动PyCharm HP60C相机显示程序...")

        # 检查ROS连接
        try:
            rospy.get_master().getPid()
            print("ROS Master连接正常")
        except:
            print("无法连接到ROS Master!")
            print("请确保:")
            print("1. roscore正在运行")
            print("2. 相机驱动已启动")
            return

        # 创建并运行显示器
        viewer = PyCharmHP60CViewer()

        print("程序运行中，等待图像数据...")
        print("在PyCharm中成功运行HP60C相机显示!")

        # 保持程序运行
        rospy.spin()

    except rospy.ROSInterruptException:
        print("ROS连接中断")
    except KeyboardInterrupt:
        print("检测到Ctrl+C")
    except Exception as e:
        print(f"程序错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        cv2.destroyAllWindows()
        print("程序安全关闭")


if __name__ == '__main__':
    main()