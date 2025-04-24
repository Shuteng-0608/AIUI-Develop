import struct
import logging
import time
import json
from socket import *
import signal
from processStrategy import *
from threading import Thread, Event
import subprocess
from enum import Enum

class AIUIMessage(Enum):
    CMD_GET_STATE = 1
    CMD_WRITE = 2
    CMD_STOP_WRITE = 3
    CMD_START = 5
    CMD_STOP = 6
    CMD_WAKEUP = 7
    CMD_RESET_WAKEUP = 8
    CMD_SET_PARAMS = 10
    CMD_SYNC = 13
    CMD_RESULT_VALIDATION_ACK = 20
    CMD_CLEAN_DIALOG_HISTORY = 21
    CMD_START_RECORD = 22
    CMD_STOP_RECORD = 23
    CMD_QUERY_SYNC_STATUS = 24
    CMD_TTS = 27

class AIUIEvent(Enum):
    EVENT_RESULT = 1
    EVENT_ERROR = 2
    EVENT_STATE = 3
    EVENT_WAKEUP = 4
    EVENT_SLEEP = 5
    EVENT_VAD = 6
    EVENT_CMD_RETURN = 8
    EVENT_PRE_SLEEP = 10
    EVENT_START_RECORD = 11
    EVENT_STOP_RECORD = 12
    EVENT_CONNECTED_TO_SERVER = 13
    EVENT_SERVER_DISCONNECTED = 14
    EVENT_TTS = 15


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


class SocketDemo:
    def __init__(self):
        self.client_socket = None
        self.server_ip_port = ('192.168.8.158', 19199)
        self.server_ip = self.server_ip_port[0]
        self.connected_event = Event()
        self.stop_event = Event()
        self.wakeup_event = Event()
        self.connect()
        self.start_ping_check()
        self.detected_intent = None

    def connect(self):
        while not self.stop_event.is_set():
            try:
                if self.client_socket:
                    self.client_socket.close()
                self.client_socket = socket(AF_INET, SOCK_STREAM) # IPv4, TCP
                self.client_socket.settimeout(5)  # 设置连接超时
                self.client_socket.connect(self.server_ip_port)
                logging.info(f"Connected to {self.server_ip_port}")
                self.client_socket.settimeout(None)  # 连接成功后取消超时
                self.connected_event.set()
                break
            except (ConnectionError, OSError, timeout) as e:
                logging.error(
                    f"Connection error: {e}. Retrying in 5 seconds...")
                self.connected_event.clear()
                time.sleep(5)

    def receive_full_data(self, expected_length):
        received_data = bytearray()
        while len(received_data) < expected_length:
            try:
                chunk = self.client_socket.recv(
                    expected_length - len(received_data))
                if not chunk:
                    # logging.error("Connection closed by the server.")
                    return None
                received_data.extend(chunk)
            except timeout:
                # logging.error(f"Socket timeout")
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
                    logging.error(
                        f"Ping to {self.server_ip} failed. Reconnecting...")
                    self.connected_event.clear()
                    self.connect()
            except Exception as e:
                logging.error(f"Error during ping: {e}")
            time.sleep(10)

    def close(self):
        self.stop_event.set()
        if self.client_socket:
            self.client_socket.close()
        logging.info("Socket closed")

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
            logging.info(f"识别结果是: {result_string} {status_value}")

    def handle_detected_intent(self, intent):
        if intent == "SayHi":
            logging.info(f"检测到 [{intent}] 意图, 执行打招呼动作")
        elif intent == "handshake":
            logging.info(f"检测到 [{intent}] 意图, 执行握手动作")
        elif intent == "LabTour":
            logging.info(f"检测到 [{intent}] 意图, 执行实验室游览动作")


    def get_nlp_result(self, data):
        # 提取 text 字段
        text_value = data.get('content', {}).get(
            'result', {}).get('nlp', {}).get('text')

        status_value = data.get('content', {}).get(
            'result', {}).get('nlp', {}).get('status')

        if text_value is not None and status_value is not None:
            logging.info(f"大模型回答结果是: {text_value}  {status_value}")
        
            parsed_data = json.loads(text_value)
            if not isinstance(parsed_data, dict):
                logging.warning("解析后的数据不是字典格式")
                return
            # self.detected_intent = parsed_data.get('intent', {}).get('semantic', {})[0].get('intent', {})
            # self.handle_detected_intent(self.detected_intent)
            try:
                # if parsed_data.get('intent', {}).get('semantic', [])
                self.detected_intent = parsed_data.get('intent', {}).get('semantic', {})[0].get('intent', {})
            except (IndexError, AttributeError, TypeError, KeyError) as e:
                self.detected_intent = None
                logging.debug(f"语义解析小异常: {str(e)}")

            if self.detected_intent:
                logging.info(f"成功提取意图: {self.detected_intent}")
                self.handle_detected_intent(self.detected_intent)
            else:
                logging.info("未检测到预设动作指令意图")

    
        
        
    def get_intent_result(self, data):
        # intent_data = data.get('content', {}).get('result', {}).get()
        text_value = data.get('content', {}).get(
            'result', {}).get('cbm_semantic', {}).get('text')
        intent = json.loads(text_value)
        rc = intent['rc']
        if (rc == 0):
            category = intent.get('category', "")
            logging.info(f"技能结果: {category} ")

    def process(self):
        try:
            self.client_socket.settimeout(3)  # 设置接收超时
            recv_data = self.receive_full_data(7)
            if not recv_data:
                logging.error("No data received. Reconnecting...")
                self.connected_event.clear()
                self.connect()
                return

            if len(recv_data) < 7:
                logging.error(f"Incomplete data received: {recv_data}")
                return

            sync_head, user_id, msg_type, msg_length, msg_id = struct.unpack(
                '<BBBHH', recv_data)

            # 校验接收的数据长度
            msg_data = self.receive_full_data(msg_length + 1)

            if len(msg_data) < msg_length + 1:
                logging.error(f"Incomplete data received: {msg_data}")
                return

            # 解析消息数据
            msg = msg_data[: msg_length]
            # 校检码（最后一个字节）
            check_code = msg_data[-1]

            if sync_head == 0xa5 and user_id == 0x01:
                if msg_type == 0x01:
                    ConfirmProcess().process(self.client_socket, msg_id)
                elif msg_type == 0x04:
                    ConfirmProcess().process(self.client_socket, msg_id)
                    success, result = AiuiMessageProcess().process(self.client_socket, msg)
                    if success:
                        self.aiui_type = ""
                        data = json.loads(result)
                        self.get_aiui_type(data)
                        # logging.info(f"AIUI message processed successfully: {result.decode('utf-8')}")
                        if (self.aiui_type == "iat"):
                            self.get_iat_result(data)

                        elif (self.aiui_type == "nlp"):
                            self.get_nlp_result(data)

                        elif (self.aiui_type == "cbm_semantic"):
                            # pass
                            self.get_intent_result(data)

                        # logging.info(f"AIUI message processed successfully: {result.decode('utf-8')}")
                    else:
                        logging.warning("AIUI message processing failed")
            else:
                return
        except timeout:
            return
        except (ConnectionError, OSError) as e:
            logging.error(
                f"Connection error during process: {e}. Reconnecting...")
            self.connected_event.clear()
            self.connect()


if __name__ == '__main__':
    demo = SocketDemo()
    try:
        while run:
            demo.process()
    finally:
        demo.close()
        logging.info("Program terminated")
