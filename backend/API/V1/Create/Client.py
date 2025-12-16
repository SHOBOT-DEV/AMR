import requests

url = "http://192.168.1.50:5000/api/v1/maps"
payload = {"name": "new_lab_map"}

response = requests.post(url, json=payload)
print(response.json())
