import requests
import json
import serial
import openpyxl as op
import time
import threading
from datetime import datetime

ser = serial.Serial(port = 'COM8', baudrate = 115200)

wb = op.load_workbook(r"Weather_excel.xlsx")
ws = wb.active
excel_result = 0

city = "Seoul" #도시
apiKey = "a50d7fd264516c246ccc2013bf70c4d4"
lang = "kr" #언어
units = "metric" #화씨 온도를 섭씨 온도로 변경
api = f"https://api.openweathermap.org/data/2.5/weather?q={city}&appid={apiKey}&units={units}"

desired_temp, desired_hum = map(int, input("temp and hum").split())

while True:
    current_month = datetime.today().month
    result = requests.get(api)
    result = json.loads(result.text)

    weather = result['weather'][0]['main']
    temperature = result['main']['temp']
    humidity = result['main']['humidity']
    wind_speed = result['wind']['speed']
    weather_des = result['weather'][0]['description']

    commend = f"*{1},{temperature},{humidity},{wind_speed},{desired_temp},{desired_hum},{current_month}\n"
    ser.write(commend.encode())

    time.sleep(0.1)

    print(weather_des, temperature, humidity, wind_speed, desired_temp, desired_hum)

    if ser.readable():
        response = ser.readline()

        print(response[:len(response)-1].decode())
