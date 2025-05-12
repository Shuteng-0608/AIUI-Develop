#! /usr/bin/env python3
import struct
import logging
import time
import json
from socket import *
import signal
from processStrategy import AiuiMessageProcess, ConfirmProcess
from threading import Thread, Event
import subprocess
from sr_modbus_sdk import SRModbusSdk
import rospy
from aiui.srv import TTS, TTSRequest, TTSResponse
from aiui.srv import VLMProcess, VLMProcessRequest, VLMProcessResponse

def stop_handle(sig, frame):
    global run
    run = False


signal.signal(signal.SIGINT, stop_handle)
run = True

# 配置日志记录
logger = logging.getLogger()
logger.setLevel(logging.INFO)

# 文件日志处理器
file_handler = logging.FileHandler('socket_demo.log')
file_handler.setLevel(logging.INFO)
file_formatter = logging.Formatter(
    '%(asctime)s %(levelname)s:%(message)s', datefmt='%Y-%m-%d %H:%M:%S')
file_handler.setFormatter(file_formatter)

# 终端日志处理器
console_handler = logging.StreamHandler()
console_handler.setLevel(logging.INFO)
console_formatter = logging.Formatter(
    '%(asctime)s %(levelname)s:%(message)s', datefmt='%Y-%m-%d %H:%M:%S')
console_handler.setFormatter(console_formatter)

# 添加处理器到记录器
# logger.addHandler(file_handler)
logger.addHandler(console_handler)

class SocketDemo(Thread):
    def __init__(self):
        super().__init__()
        self.client_socket = None
        self.server_ip_port = ('192.168.10.141', 19199)
        self.server_ip = self.server_ip_port[0]
        self.connected_event = Event()
        self.stop_event = Event()
        self.connect()
        self.start_ping_check()
        self.detected_intent = None
        self.tts_text = ""

    def connect(self):
        while not self.stop_event.is_set():
            try:
                if self.client_socket:
                    self.client_socket.close()
                self.client_socket = socket(AF_INET, SOCK_STREAM)
                self.client_socket.settimeout(5)  # 设置连接超时
                self.client_socket.connect(self.server_ip_port)
                print(f"Connected to {self.server_ip_port}")
                self.client_socket.settimeout(None)  # 连接成功后取消超时
                self.connected_event.set()
                break
            except (ConnectionError, OSError, timeout) as e:
                print(f"Connection error: {e}. Retrying in 5 seconds...")
                self.connected_event.clear()
                time.sleep(5)

    def receive_full_data(self, expected_length):
        received_data = bytearray()
        while len(received_data) < expected_length:
            try:
                chunk = self.client_socket.recv(
                    expected_length - len(received_data))
                if not chunk:
                    # print("Connection closed by the server.")
                    return None
                received_data.extend(chunk)
            except timeout:
                # print(f"Socket timeout")
                return None
        return bytes(received_data)

    def start_ping_check(self):
        Thread(target=self.ping_check, daemon=True).start()

    def ping_check(self):
        while not self.stop_event.is_set():
            try:
                response = subprocess.run(
                    ["ping", "-c", "1", self.server_ip],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE
                )
                if response.returncode != 0:
                    print(
                        f"Ping to {self.server_ip} failed. Reconnecting...")
                    self.connected_event.clear()
                    self.connect()
            except Exception as e:
                print(f"Error during ping: {e}")
            time.sleep(10)

    def close(self):
        self.stop_event.set()
        if self.client_socket:
            self.client_socket.close()
        print("Socket closed")

    def get_aiui_type(self, data):
        if 'content' in data:
            content = data['content']
            if 'info' in content:
                info = content['info']
                if 'data' in info and isinstance(info['data'], list) and len(info['data']) > 0:
                    data_item = info['data'][0]
                    if 'params' in data_item:
                        params = data_item['params']
                        sub_value = params.get('sub')
                        if sub_value is not None:
                            self.aiui_type = sub_value

    def get_iat_result(self, data):
        # 提取并拼接 w 字段
        words = []
        ws_list = data.get('content', {}).get(
            'result', {}).get('text', {}).get('ws', [])
        for item in ws_list:
            cw_list = item.get('cw', [])
            for cw in cw_list:
                words.append(cw.get('w', ''))
        # 拼接成字符串
        sn_value = data.get('content', {}).get(
            'result', {}).get('text', {}).get('sn')
        ls_value = data.get('content', {}).get(
            'result', {}).get('text', {}).get('ls')
        status_value = -1
        if (sn_value == 1):
            status_value = 0
        elif (ls_value == True):
            status_value = 2
        else:
            status_value = 1
        result_string = ''.join(words)
        if (result_string != "" or status_value == 2):
            print(f"识别结果是: {result_string} {status_value}")


    def labTour(self, start, end):
        mb_server = SRModbusSdk()
        mb_server.connect_tcp('192.168.10.141')
        for i in range(start, end+1):
            mb_server.move_to_station_no(i, 1)
            mb_server.wait_movement_task_finish(1) 


    def handle_detected_intent(self, intent):
        if intent == "SayHi":
            print(f"检测到 [{intent}] 意图, 执行打招呼动作")
            # client = rospy.ServiceProxy("Srv_name",SrvType)
            # req = SrvTypeRequest()
            # req.request = # TODO: 填写请求参数
            # client.wait_for_service()
            # client.call(req)
            # client.close()
        elif intent == "handshake":
            print(f"检测到 [{intent}] 意图, 执行握手动作")
        elif intent == "LabTour":
            print(f"检测到 [{intent}] 意图, 执行实验室游览动作")
            # self.labTour()
        elif intent == "Bow":
            print(f"检测到 [{intent}] 意图, 执行鞠躬欢送动作")
        elif intent == "Nod":
            print(f"检测到 [{intent}] 意图, 执行点头动作")
        elif intent == "VLM":
            print(f"检测到 [{intent}] 意图, 执行描述动作")
            client = rospy.ServiceProxy("vlm_service",VLMProcess)
            req = VLMProcessRequest()
            req.request = "我手上拿得是啥" 
            client.wait_for_service()
            resp = client.call(req)
            vlm_result = resp.vlm_result
            print(f"VLM 结果: {vlm_result}")
            client.close()
            client = rospy.ServiceProxy("tts_service",TTS)
            req = TTSRequest()
            req.request = vlm_result
            client.wait_for_service()
            client.call(req)
            client.close()



    def get_nlp_result(self, data):
        # 提取 text 字段
        text_value = data.get('content', {}).get(
            'result', {}).get('nlp', {}).get('text')

        status_value = data.get('content', {}).get(
            'result', {}).get('nlp', {}).get('status')

        if text_value is not None and status_value is not None:
            print(f"大模型回答结果是: {text_value}  {status_value}")
        
            parsed_data = json.loads(text_value)
            if not isinstance(parsed_data, dict):
                logging.warning("解析后的数据不是字典格式")
                return
            # self.detected_intent = parsed_data.get('intent', {}).get('semantic', {})[0].get('intent', {})
            # self.handle_detected_intent(self.detected_intent)
            try:
                # if parsed_data.get('intent', {}).get('semantic', [])
                self.detected_intent = parsed_data.get('intent', {}).get('semantic', {})[0].get('intent', {})
                self.tts_text = parsed_data.get('intent', {}).get('answer',{}).get('text')
            except (IndexError, AttributeError, TypeError, KeyError) as e:
                self.detected_intent = None
                self.tts_text = ""
                logging.debug(f"语义解析小异常: {str(e)}")

            if self.tts_text:
                logging.info(f"成功提取回答: {self.tts_text}")
                client = rospy.ServiceProxy("tts_service",TTS)
                req = TTSRequest()
                req.request = self.tts_text
                client.wait_for_service()
                client.call(req)
                client.close()

            else:
                logging.info(f"未成功提取回答")

            if self.detected_intent:
                print(f"成功提取意图: {self.detected_intent}")
                self.handle_detected_intent(self.detected_intent)
            else:
                print("未检测到预设动作指令意图")

    def get_intent_result(self, data):
        # intent_data = data.get('content', {}).get('result', {}).get()
        text_value = data.get('content', {}).get(
            'result', {}).get('cbm_semantic', {}).get('text')
        intent = json.loads(text_value)
        rc = intent['rc']
        if (rc == 0):
            category = intent.get('category', "")
            print(f"技能结果: {category} ")

    def run(self):
        try:
            print("Program started")
            while run:
                demo.process()
        finally:
            demo.close()
            print("Program terminated")

    def process(self):
        try:
            self.client_socket.settimeout(3)  # 设置接收超时
            recv_data = self.receive_full_data(7)
            if not recv_data:
                print("No data received. Reconnecting...")
                self.connected_event.clear()
                self.connect()
                return

            if len(recv_data) < 7:
                print(f"Incomplete data received: {recv_data}")
                return

            sync_head, user_id, msg_type, msg_length, msg_id = struct.unpack(
                '<BBBHH', recv_data)

            # 校验接收的数据长度
            msg_data = self.receive_full_data(msg_length + 1)

            if len(msg_data) < msg_length + 1:
                print(f"Incomplete data received: {msg_data}")
                return

            # 解析消息数据
            msg = msg_data[: msg_length]
            # 校检码（最后一个字节）
            check_code = msg_data[-1]

            if sync_head == 0xa5 and user_id == 0x01:
                # success, result = AiuiMessageProcess().process(self.client_socket, msg)
                # print(f"msg_type: {msg_type} ")
                # print(f"收到数据: {result} ")

                if msg_type == 0x01:
                    ConfirmProcess().process(self.client_socket, msg_id)
                elif msg_type == 0x04:
                    ConfirmProcess().process(self.client_socket, msg_id)
                    success, result = AiuiMessageProcess().process(self.client_socket, msg)
                    if success:
                        self.aiui_type = ""
                        data = json.loads(result)
                        self.get_aiui_type(data)
                        # print(f"AIUI message processed successfully: {result.decode('utf-8')}")

                        if data.get('content', {}).get('eventType', {}) == 4:
                            print(f"唤醒成功：==== 我在 ==== ")
                            client = rospy.ServiceProxy("tts_service",TTS)
                            req = TTSRequest()
                            req.request = "我在！"
                            client.wait_for_service()
                            client.call(req)
                            client.close()
                            # AiuiTTS().tts(self.client_socket, msg_id, "我在")
                        if (self.aiui_type == "iat"):
                            self.get_iat_result(data)

                        elif (self.aiui_type == "nlp"):
                            self.get_nlp_result(data)

                        elif (self.aiui_type == "cbm_semantic"):
                            # pass
                            self.get_intent_result(data)

                        # print(f"AIUI message processed successfully: {result.decode('utf-8')}")
                    else:
                        logging.warning("AIUI message processing failed")
            else:
                return
        except timeout:
            return
        except (ConnectionError, OSError) as e:
            print(
                f"Connection error during process: {e}. Reconnecting...")
            self.connected_event.clear()
            self.connect()


if __name__ == '__main__':
    rospy.init_node("AIUI_node")
    
    demo = SocketDemo()
    demo.start()

    rospy.spin()
    