#!/usr/bin/env python
# -*- coding:utf-8 -*-
import json
import rospy
from aiui.srv import VLMProcess, VLMProcessResponse
import requests
import base64
import os
import ast


class VLMServiceServer:
    def __init__(self):
        # 从ROS参数服务器获取配置参数
        self.image_path = rospy.get_param('~image_path', '/home/whc/aiui_ws/src/aiui/img/image01.jpg')
        self.api_url = rospy.get_param('~api_url', 'http://172.18.35.200:8000/uploads/vlm_queries')
        
        # 初始化服务
        self.service = rospy.Service('vlm_service', VLMProcess, self.handle_vlm_request)
        rospy.loginfo("VLM Service Server is ready")

    def handle_vlm_request(self, req):
        """处理服务请求的回调函数"""
        try:
            # 读取并编码图片
            with open(self.image_path, "rb") as f:
                encoded_image = base64.b64encode(f.read()).decode('utf-8')
        except Exception as e:
            error_msg = f"Image processing error: {str(e)}"
            rospy.logerr(error_msg)
            return VLMProcessResponse(error_msg)

        # 准备请求数据
        payload = {
            "image": encoded_image,
            "prompt": req.prompt
            
        }

        # 发送API请求
        try:
            response = requests.post(self.api_url, json=payload, timeout=10)
            if response.status_code == 200:
                result = response.json().get("read_message", "No valid result returned")
                
                resp = VLMProcessResponse()
                resp.vlm_result = result[0] if isinstance(result, list) and len(result) > 0 else result
                resp.vlm_result = ast.literal_eval(f'"{resp.vlm_result}"')
                print(result[0])
                
                # resp.vlm_result = result[0]

               
                return resp
                
            else:
                error_msg = f"API request failed with code {response.status_code}: {response.text}"
                rospy.logerr(error_msg)
                return VLMProcessResponse(error_msg)
        except Exception as e:
            error_msg = f"Communication error: {str(e)}"
            rospy.logerr(error_msg)
            return VLMProcessResponse(error_msg)

if __name__ == "__main__":
    rospy.init_node('vlm_service_server')
    server = VLMServiceServer()
    rospy.spin()