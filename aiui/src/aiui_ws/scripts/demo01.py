#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

# 根据实际服务类型导入对应的srv定义
# 假设服务类型为 my_pkg/VLMService，实际请替换为你的服务定义
from aiui.srv import VLMProcess, VLMProcessRequest

def call_vlm_service():
    # 初始化ROS节点（名称唯一即可）
    rospy.init_node('vlm_service_client', anonymous=True)
    
    # 等待服务可用
    rospy.loginfo("等待 /vlm_service 服务...")
    rospy.wait_for_service('/vlm_service')
    
    try:
        # 创建服务代理
        vlm_proxy = rospy.ServiceProxy('/vlm_service', VLMProcess)
        
        # 构建请求对象
        req = VLMProcessRequest()
        req.prompt = "我手上拿得是啥"  # 设置请求参数
        
        # 发送请求并获取响应
        rospy.loginfo("发送请求：%s", req.prompt)
        resp = vlm_proxy(req)
        
        # 处理响应（假设响应包含result字段）
        rospy.loginfo("收到响应：%s", resp.vlm_result)
        
        
    except rospy.ServiceException as e:
        rospy.logerr("服务调用失败: %s", str(e))

if __name__ == "__main__":
    call_vlm_service()

