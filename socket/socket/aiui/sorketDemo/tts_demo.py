from ws4py.client.threadedclient import WebSocketClient
import base64
import hashlib
import json
import time
import uuid
import wave  # 用于保存 WAV 文件
import subprocess  # 用于调用系统播放器
import requests

base_url = "ws://wsapi.xfyun.cn/v1/aiui"
app_id = "e1ac1785"
api_key = "ee8edf3ac8a559787df7a819182e81aa"
data_type = "text"
scene = "IFLYTEK.tts"
text_msg = "床前明月光，疑是地上霜。"
file_path = r"weather.pcm"  # 用户自己收集
end_tag = "--end--"

# 保存 WAV 文件的路径
output_wav = "output.wav"

class WsapiClient(WebSocketClient):
    def opened(self):
        pass

    def closed(self, code, reason=None):
        if code == 1000:
            print("连接正常关闭")
        else:
            print("连接异常关闭，code：" + str(code) + " ，reason：" + str(reason))

    def received_message(self, m):
        s = json.loads(str(m))

        if s['action'] == "started":
            if scene == "IFLYTEK.tts":
                self.send(text_msg.encode("utf-8"))
            else:
                if data_type == "text":
                    self.send(text_msg.encode("utf-8"))
                if data_type == "audio":
                    file_object = open(file_path, 'rb')
                    try:
                        while True:
                            chunk = file_object.read(1280)
                            if not chunk:
                                break
                            self.send(chunk)
                            time.sleep(0.04)
                    finally:
                        file_object.close()
            self.send(bytes(end_tag.encode("utf-8")))
            print("发送结束标识")

        elif s['action'] == "result":
            data = s['data']
            if data['sub'] == "iat":
                print("user: ", data["text"])
            elif data['sub'] == "nlp":
                intent = data['intent']
                if intent['rc'] == 0:
                    
                    print("server: ", intent['answer']['text'])
                else:
                    print("我没有理解你说的话啊")
            elif data['sub'] == "tts":
                if 'content' in data:
                # 如果是 URL（MP3 格式）
                    if data['content'].startswith("http"):
                        mp3_url = data['content']
                        print(f"获取到 MP3 音频 URL: {mp3_url}")
                        
                        # 下载 MP3 文件
                        try:
                            response = requests.get(mp3_url)
                            response.raise_for_status()  # 检查请求是否成功
                            
                            # 保存为 MP3
                            with open("output.mp3", "wb") as f:
                                f.write(response.content)
                            print("MP3 文件已保存为 output.mp3")
                            
                            # 用系统默认播放器播放
                            subprocess.call(["xdg-open", "output.mp3"])  # Linux
                            # subprocess.call(["start", "output.mp3"], shell=True)  # Windows
                            # subprocess.call(["open", "output.mp3"])  # Mac
                        except Exception as e:
                            print(f"下载或播放 MP3 失败: {e}")
                    else:
                        # 如果是 base64 编码的 PCM 数据（旧版 API）
                        audio_data = base64.b64decode(data['content'])
                        print(f"音频数据长度: {len(audio_data)} 字节")
                        
                        # 保存为 WAV
                        with wave.open("output.wav", "wb") as wav_file:
                            wav_file.setnchannels(1)
                            wav_file.setsampwidth(2)
                            wav_file.setframerate(16000)
                            wav_file.writeframes(audio_data)
                        print("WAV 文件已保存为 output.wav")
                        
                        # 播放 WAV
                        subprocess.call(["xdg-open", "output.wav"])  # Linux
                else:
                    print("错误: TTS 未返回音频数据或 URL")
        else:
            print(s)

def get_auth_id():
    mac = uuid.UUID(int=uuid.getnode()).hex[-12:]
    return hashlib.md5(":".join([mac[e:e + 2] for e in range(0, 11, 2)]).encode("utf-8")).hexdigest()

if __name__ == '__main__':
    try:
        curTime = int(time.time())
        auth_id = get_auth_id()
        
        param_iat = """{{
            "auth_id": "{0}",
            "result_level": "plain",
            "data_type": "audio",
            "aue": "raw",
            "scene": "main_box",
            "sample_rate": "16000",
            "ver_type": "monitor",
            "close_delay": "200",
            "tts_res_type": "url",
            "context": "{{\\\"sdk_support\\\":[\\\"iat\\\",\\\"nlp\\\",\\\"tts\\\"]}}"
        }}"""
        
        param_text = """{{
            "auth_id": "{0}",
            "result_level": "plain",
            "data_type": "text",
            "scene": "main_box",
            "ver_type": "monitor",
            "close_delay": "200",
            "tts_res_type": "url",
            "context": "{{\\\"sdk_support\\\":[\\\"iat\\\",\\\"nlp\\\",\\\"tts\\\"]}}"
        }}"""
        
        param_tts = """{{
            "auth_id": "{0}",
            "data_type": "text",
            "vcn": "x4_lingxiaoxuan_oral",
            "scene": "IFLYTEK.tts",
            "tts_res_type": "url",
            "context": "{{\\\"sdk_support\\\":[\\\"tts\\\"]}}"
        }}"""
        
        param = ""
        if scene == 'IFLYTEK.tts':
            param = param_tts.format(auth_id).encode(encoding="utf-8")
        else:
            if data_type == 'text':
                param = param_text.format(auth_id).encode(encoding="utf-8")
            if data_type == 'audio':
                param = param_iat.format(auth_id).encode(encoding="utf-8")
        
        paramBase64 = base64.b64encode(param).decode()
        checkSumPre = api_key + str(curTime) + paramBase64
        checksum = hashlib.md5(checkSumPre.encode("utf-8")).hexdigest()
        connParam = "?appid=" + app_id + "&checksum=" + checksum + "&param=" + paramBase64 + "&curtime=" + str(
            curTime) + "&signtype=md5"

        ws = WsapiClient(base_url + connParam, protocols=['chat'], headers=[("Origin", "https://wsapi.xfyun.cn")])
        ws.connect()
        ws.run_forever()
    except KeyboardInterrupt:
        pass