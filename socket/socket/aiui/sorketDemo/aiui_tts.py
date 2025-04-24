import _thread as thread
import base64
import datetime
import hashlib
import hmac
import json
from urllib.parse import urlparse
import time
from datetime import datetime
from time import mktime
from urllib.parse import urlencode
from wsgiref.handlers import format_date_time

import websocket

## 修改应用应用配置后直接执行即可

# 请求地址
url = "wss://aiui.xf-yun.com/v2/aiint/ws"

appid = "e1ac1785"
api_key = "ee8edf3ac8a559787df7a819182e81aa"
api_secret = "NWZiZTg5Y2U5ZDZiNGU0M2YyZTVhMDI5"

# 场景
# scene = "IFLYTEK.hts"
scene = "main_box"

# 请求类型用来设置文本请求还是音频请求，text/audio
data_type = 'text'

# 请求文本
question = "你好呀"

# 每帧音频数据大小，单位字节
chuncked_size = 1024

audio_save_fp = open('/home/rosnoetic/socket/socket/aiui/socketDemo/save_hts_audio.pcm', mode='wb+')

class AIUIV2WsClient(object):
    # 初始化
    def __init__(self):
        self.handshake = self.assemble_auth_url(url)

    # 生成握手url
    def assemble_auth_url(self, base_url):
        host = urlparse(base_url).netloc
        path = urlparse(base_url).path
        # 生成RFC1123格式的时间戳
        now = datetime.now()
        date = format_date_time(mktime(now.timetuple()))

        # 拼接字符串
        signature_origin = "host: " + host + "\n"
        signature_origin += "date: " + date + "\n"
        signature_origin += "GET " + path + " HTTP/1.1"

        # 进行hmac-sha256进行加密
        print(signature_origin)
        signature_sha = hmac.new(api_secret.encode('utf-8'), signature_origin.encode('utf-8'),
                                 digestmod=hashlib.sha256).digest()

        signature_sha_base64 = base64.b64encode(signature_sha).decode(encoding='utf-8')

        authorization_origin = f'api_key="{api_key}", algorithm="hmac-sha256", headers="host date request-line", signature="{signature_sha_base64}"'

        print('get authorization_origin:', authorization_origin)
        authorization = base64.b64encode(authorization_origin.encode('utf-8')).decode(encoding='utf-8')

        # 将请求的鉴权参数组合为字典
        v = {
            "host": host,
            "date": date,
            "authorization": authorization,
        }
        # 拼接鉴权参数，生成url
        url = base_url + '?' + urlencode(v)
        # 此处打印出建立连接时候的url,参考本demo的时候可取消上方打印的注释，比对相同参数时生成的url与自己代码生成的url是否一致
        return url

    def on_open(self, ws):
        # 连接建立成功后开始发送数据
        print("### ws connect open")
        thread.start_new_thread(self.run, ())

    def run(self):
        if data_type == "text":
            self.text_req()

    def text_req(self):
        aiui_data = {
            "header": {
                "sn":"1234567891",
                "appid": appid,
                "stmid": "text-1",
                "status": 3,
                "scene": scene,
                "pers_param":"{\"appid\":\"\",\"uid\":\"iflytek-test\"}"
            },
            "parameter": {
                "nlp": {
                    "nlp": {
                        "compress": "raw",
                        "format": "json",
                        "encoding": "utf8"
                    },
                    # 动态实体
                    "sub_scene": "cbm_v45",
                    "new_session": True
                },
                "tts": {
                    "vcn": "x4_lingxiaoxuan_oral",
                    "oral_level": "mid",
                    "emotion_scale": 0,
                    "emotion": 5,
                    "tts": {
                        "channels": 1,
                        "sample_rate": 16000,
                        "bit_depth": 16,
                        "encoding": "raw"
                    },
                    "tts_res_type": "url"
                }
            },
            "payload": {
                "text": {
                    "compress": "raw",
                    "format": "plain",
                    "text": base64.b64encode(question.encode('utf-8')).decode('utf-8'),
                    "encoding": "utf8",
                    "status": 3
                }
            }
        }
        data = json.dumps(aiui_data, indent=2)
        print('--------text request data--------\n', data,'\n--------text request data--------\n')
        self.ws.send(data)


    # 收到websocket消息的处理
    def on_message(self, ws, message):
        data = json.loads(message)

        # print('原始结果:', message)
        header = data['header']
        code = header['code']
        # 结果解析
        if code != 0:
            print('请求错误:', code, json.dumps(data, ensure_ascii=False))
            ws.close()
        if 'nlp' in message:
            nlp_json = data['payload']['nlp']
            nlp_text_bs64 = nlp_json['text']
            nlp_text = base64.b64decode(nlp_text_bs64).decode('utf-8')
            nlp_all = json.dumps(data,indent=2)
            # print("--------nlp--------\n" , nlp_all)
            # print("语义结果：",nlp_text,"--------nlp--------\n")
        if 'tts' in message:
            tts_all = json.dumps(data,indent=2)
            print("--------hts_all--------\n",tts_all,"\n--------hts_all_all--------\n")
            tts_audio =  data['payload']['tts']['audio']
            print(tts_audio)
            tts_content = base64.b64decode(tts_audio)
            print(tts_content)
            audio_save_fp.write(tts_content)

        if 'status' in header and header['status'] == 2:
            # 接受最后一帧结果，关闭连接
            ws.close()

    def on_error(self, ws, error):
        print("### connection error: " + error)
        ws.close()

    def on_close(self, ws, close_status_code, close_msg):
        print("### connection is closed ###, cloce code:" + str(close_status_code))

    def start(self):
        self.ws = websocket.WebSocketApp(
            self.handshake,
            on_open=self.on_open,
            on_message=self.on_message,
            on_error=self.on_error,
            on_close=self.on_close,
        )
        self.ws.run_forever()

if __name__ == "__main__":

    client = AIUIV2WsClient()
    client.start()
