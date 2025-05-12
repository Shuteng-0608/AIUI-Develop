#!/usr/bin/env python3
# export LD_PRELOAD=/lib/x86_64-linux-gnu/libtiff.so.5
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
from fastapi import FastAPI
import threading
import base64
import cv2

# ROS节点初始化（必须在主线程）
if not rospy.core.is_initialized():
    rospy.init_node('camera_api_node', anonymous=True)

app = FastAPI()
bridge = CvBridge()

# 共享数据存储结构
camera_data = {
    "color_image": None,  # 彩色图数据
    "depth_image": None,  # 深度图数据
    "color_info": None,   # 彩色相机参数
    "depth_info": None    # 深度相机参数
}

# 同步事件（用于等待话题数据）
color_image_event = threading.Event()
depth_image_event = threading.Event()
color_info_event = threading.Event()
depth_info_event = threading.Event()

# 线程锁（保护camera_data）
data_lock = threading.Lock()

def numpy_to_base64(image: np.ndarray) -> str:
    """将NumPy数组转为Base64编码的JPEG图像"""
    _, buffer = cv2.imencode('.jpg', image)
    return base64.b64encode(buffer).decode('utf-8')

def color_image_callback(msg):
    """彩色图像回调函数"""
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")  # ROS转OpenCV格式
        with data_lock:
            camera_data["color_image"] = {
                "shape": cv_image.shape,
                "dtype": str(cv_image.dtype),
                "data": numpy_to_base64(cv_image)  # Base64编码
            }
        color_image_event.set()  # 通知数据已就绪
    except Exception as e:
        rospy.logerr(f"彩色图像处理失败: {str(e)}")

def depth_image_callback(msg):
    """深度图像回调函数"""
    try:
        # 使用passthrough保留原始深度数据格式（如16UC1）
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        with data_lock:
            camera_data["depth_image"] = {
                "shape": cv_image.shape,
                "dtype": str(cv_image.dtype),
                "data": numpy_to_base64(cv_image)
            }
        depth_image_event.set()
    except Exception as e:
        rospy.logerr(f"深度图像处理失败: {str(e)}")

def color_info_callback(msg):
    """彩色相机参数回调"""
    with data_lock:
        camera_data["color_info"] = {
            "header": {
                "seq": msg.header.seq,
                "stamp": str(msg.header.stamp),
                "frame_id": msg.header.frame_id
            },
            "height": msg.height,
            "width": msg.width,
            "distortion_model": msg.distortion_model,
            "D": msg.D,  # 畸变系数
            "K": msg.K,  # 内参矩阵
            "R": msg.R,  # 旋转矩阵
            "P": msg.P   # 投影矩阵
        }
    color_info_event.set()

def depth_info_callback(msg):
    """深度相机参数回调"""
    with data_lock:
        camera_data["depth_info"] = {
            "header": {
                "seq": msg.header.seq,
                "stamp": str(msg.header.stamp),
                "frame_id": msg.header.frame_id
            },
            # 其余字段与color_info相同
            "height": msg.height,
            "width": msg.width,
            "distortion_model": msg.distortion_model,
            "D": msg.D,
            "K": msg.K,
            "R": msg.R,
            "P": msg.P
        }
    depth_info_event.set()

@app.get("/camera_data")
def get_camera_data():
    """FastAPI接口：获取最新相机数据"""
    global camera_data
    # 重置数据和事件标志
    with data_lock:
        camera_data = {k: None for k in camera_data}
    color_image_event.clear()
    depth_image_event.clear()
    color_info_event.clear()
    depth_info_event.clear()

    # 临时订阅话题（非阻塞）
    subs = [
        rospy.Subscriber('/camera/color/image_raw', Image, color_image_callback),
        rospy.Subscriber('/camera/depth/image_raw', Image, depth_image_callback),
        rospy.Subscriber('/camera/color/camera_info', CameraInfo, color_info_callback),
        rospy.Subscriber('/camera/depth/camera_info', CameraInfo, depth_info_callback)
    ]

    # 等待所有数据到达（超时5秒）
    received = all([
        color_image_event.wait(5.0),
        depth_image_event.wait(5.0),
        color_info_event.wait(5.0),
        depth_info_event.wait(5.0)
    ])

    # 清理订阅器
    for sub in subs:
        sub.unregister()

    if not received:
        return {"error": "等待相机数据超时"}
    
    with data_lock:
        return camera_data

if __name__ == "__main__":
    import uvicorn
    # 在子线程启动FastAPI
    fastapi_thread = threading.Thread(
        target=lambda: uvicorn.run(app, host="0.0.0.0", port=7779),
        daemon=True
    )
    fastapi_thread.start()
    
    # 主线程运行ROS事件循环
    rospy.spin()