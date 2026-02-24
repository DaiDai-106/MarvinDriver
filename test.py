import requests
import json

url = "http://192.168.2.172:8010/set_tool?robot_id=B"  # robot_id在URL中

data = {
    "kine_params": [0,0,0,0,0,0],
    "dynamic_params": [1.101471, -23.319462, 80.805658, 68.266619, 0.041829, 0.0, 0.0, 0.001, 0.0, 0.023423]
}

response = requests.post(url, json=data)
print(f"状态码: {response.status_code}")
print(f"响应: {response.text}")