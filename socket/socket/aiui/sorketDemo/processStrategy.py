import logging
import zlib
import json
from abc import abstractmethod, ABC


class ProcessStrategy(ABC):
    # 构建Msg
    def makepacket(self, sid, type, content):
        size = len(content)
        temp = bytearray()
        temp.append(0xa5)
        temp.append(0x01)
        temp.append(type)
        temp.append(size & 0xff)
        temp.append((size >> 8) & 0xff)
        temp.append(sid & 0xff)
        temp.append((sid >> 8) & 0xff)
        temp.extend(content)
        temp.append(self.checkcode(temp))
        # print(temp)
        return temp

    # 校验码计算
    def checkcode(self, data):
        total = sum(data)
        checkcode = (~total + 1) & 0xFF
        return checkcode

    @abstractmethod
    def process(self, client_socket, data):
        pass


# 确认消息
class ConfirmProcess(ProcessStrategy):
    def process(self, client_socket, msg_id):
        temp = bytearray()
        temp.append(0xa5)
        temp.append(0x00)
        temp.append(0x00)
        temp.append(0x00)
        send_data = self.makepacket(msg_id, 0xff, temp)
        client_socket.send(send_data)


# AIUI消息
class AiuiMessageProcess(ProcessStrategy):
    def process(self, client_socket, data):
        if not data:
            return False, bytearray()
        try:
            decompressor = zlib.decompressobj(16 + zlib.MAX_WBITS)
            output = decompressor.decompress(data)
            output += decompressor.flush()
            return True, output
        except zlib.error as e:
            return False, bytearray()



# 语音合成播报
class AiuiTTS(ProcessStrategy):
    def process(self, client_socket, msg_id, tts_text):
        command = {
            "type": "tts",  
            "content": {
                "action": "start",
                "text": tts_text,  
                "parameters": {
                    # "emot": "neutral",
                    "vcn": "x4_lingxiaoxuan_oral",
                    "speed": 50,
                    "pitch": 50,
                    "volume": 50
                }
            }
        }
        # aiui_data = {
                
        #     "header": {
        #         "sn": "1234567890",
        #         "appid": "a44e0f36",
        #         "stmid": "text-1",
        #         "status": 3,
        #         "scene": "IFLYTEK.hts"
        #     },
        #     "parameter": {
        #         "tts": {
        #             "vcn": "x4_lingxiaoxuan_oral",
        #             "oral_level": "mid",
        #             "emotion_scale": 0,
        #             "emotion": 5,
        #             "tts": {
        #                 "channels": 1,
        #                 "sample_rate": 16000,
        #                 "bit_depth": 16,
        #                 "encoding": "raw"
        #             }
        #         }
        #     },
        #     "payload": {
        #         "text": {
        #             "compress": "raw",
        #             "format": "plain",
        #             "text": "5omT55S16K+d57uZ5byg5LiJ",
        #             "encoding": "utf8",
        #             "status": 3
        #         }
        #     }
        # }
        # data = json.dumps(aiui_data, indent=2)
        json_bytes = json.dumps(command, ensure_ascii=False).encode('utf-8')
        send_data = self.makepacket(1, 0x1B, json_bytes)
        client_socket.send(send_data)
