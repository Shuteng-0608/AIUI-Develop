


import requests
from PIL import Image
import base64

url = "http://172.18.35.200:8000/uploads/vlm_queries"

try:
    with open("/home/rosnoetic/socket/socket/aiui/socketDemo/vlm_images/image05.jpg", "rb") as image_file:
        # Read the binary data and encode it as base64
        encoded_image = base64.b64encode(image_file.read()).decode('utf-8')

except FileNotFoundError:
    print("Error: The image file was not found at the specified path")
    exit()
except IOError:
    print("Error: The file is not a valid image or is corrupted")
    exit()

# Prepare JSON payload
data = {
    "image": encoded_image,  # Send as base64 string
    "prompt": "算一下结果"
}

try:
    response = requests.post(url, json=data)
    
    if response.status_code == 200:
        result = response.json().get("read_message", "未返回有效结果")
        print("模型输出:", result)
    else:
        print("请求失败，状态码:", response.status_code)
        print("错误详情:", response.text)

except requests.exceptions.RequestException as e:
    print("请求发生错误:", str(e))