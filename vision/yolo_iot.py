import cv2
import time
import socket
from datetime import datetime
from ultralytics import YOLO
from gtts import gTTS
import os

model = YOLO("yolov8n.pt")
cap = cv2.VideoCapture(0)

person_detected_time = None
sent_human = False
sent_dog = False
servo_on_time = None
servo_on = False

SERVER_IP = "10.10.141.77"
PORT = 5000
ID = "JDH_ARD"
SEND_ID = "1"

LOG_FILE = f"detection_log_{datetime.now().strftime('%Y%m%d')}.txt"

def speak(text, lang='ko'):
    try:
        tts = gTTS(text=text, lang=lang)
        tts.save("tts_output.mp3")
        os.system("mpg123 -q tts_output.mp3")
    except Exception as e:
        print("gTTS error:", e)

def send_message(message):
    try:
        full_message = f"[{SEND_ID}]{message}"
        print(f"message send: {full_message}")
        
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client:
            client.connect((SERVER_IP, PORT))
            client.sendall(f"[{ID}:PASSWD]".encode())
            time.sleep(0.1)
            client.sendall((full_message + "\n").encode())
        
        now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        with open(LOG_FILE, "a") as f:
            f.write(f"[{now}] {full_message}\n")
         
    except Exception as e:
        print("send error:", e)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    results = model(frame, verbose=False)
    labels = [model.names[int(cls)] for cls in results[0].boxes.cls]

    now = time.time()

    if "person" in labels:
        if person_detected_time is None:
            person_detected_time = now
        elif (now - person_detected_time >= 2.0) and not sent_human:
            send_message("senser@human")
            sent_human = True
            speak("사람이 감지되었습니다. 문이 열립니다")

            send_message("DOOR@OPEN")
            servo_on_time = time.time()
            servo_on = True

    else:
        person_detected_time = None
        sent_human = False

        if servo_on and time.time() - servo_on_time >= 10:
            send_message("DOOR@CLOSE")
            servo_on = False

    for label in labels:
        if label in ["dog", "cat", "bird"] and not sent_dog:
            send_message(f"senser@{label}")
            sent_dog = True
            speak("동물이 감지되었습니다. 관리실에 문의해주세요")
    else:
        sent_dog = False

    annotated = results[0].plot()
    cv2.imshow("YOLOv8 Detection", annotated)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()