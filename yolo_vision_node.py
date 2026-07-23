import cv2
import time
import json
import socket
import threading
from ultralytics import YOLO

# --- НАСТРОЙКИ ---
STREAM_URL = "http://172.24.192.171:8080" #
MODEL_NAME = "best_detect.pt" #
CONFIDENCE = 0.20 #
TARGET_CLASSES = [0, 1] #
BBOX_HEIGHT_AT_MAX_DISTANCE = 0.04
BBOX_HEIGHT_AT_CAPTURE_DISTANCE = 0.45

UDP_IP = "127.0.0.1" 
UDP_PORT = 5005 

# Настройка UDP-клиента
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) 

# Многопоточный захват кадров (Борьба с сетевым лагом)
latest_frame = None 
frame_lock = threading.Lock() 

def capture_thread(cap): #
    global latest_frame #
    while True: #[cite: 2]
        ret, frame = cap.read() #[cite: 2]
        if ret: #[cite: 2]
            with frame_lock: #[cite: 2]
                latest_frame = frame #[cite: 2]
        else: #[cite: 2]
            time.sleep(0.01) #[cite: 2]

# Инициализация YOLO и захвата[cite: 2]
model = YOLO(MODEL_NAME) #[cite: 2]
cap = cv2.VideoCapture(STREAM_URL) 
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1) #[cite: 2]

# Запуск фонового потока чтения камеры[cite: 2]
thread = threading.Thread(target=capture_thread, args=(cap,), daemon=True) 
thread.start() 

while True: 
    with frame_lock: 
        frame = latest_frame.copy() if latest_frame is not None else None 
        
    if frame is None:
        print('lol') 
        time.sleep(0.01) 
        continue 
        
    h, w = frame.shape[:2] 
    
    # Использование Трекера вместо Детектора[cite: 2]
    results = model.track(frame, persist=True, conf=CONFIDENCE, classes=TARGET_CLASSES) #[cite: 2]
    
    # Фильтрация ложных объектов (Выбор по площади)[cite: 2]
    best_box = None #[cite: 2]
    max_area = 0 #[cite: 2]
    
    if len(results) > 0 and results[0].boxes is not None: 
        for box in results[0].boxes: #[cite: 2]
            x1_, y1_, x2_, y2_ = box.xyxy[0].cpu().numpy() #[cite: 2]
            area = (x2_ - x1_) * (y2_ - y1_) #[cite: 2]
            if area > max_area: #[cite: 2]
                max_area = area #[cite: 2]
                best_box = box #[cite: 2]
                
    sees = 0.0 
    x_norm = 0.0 
    y_norm = 0.0 
    conf_val = 0.0 
    ball_w = 0.0 
    ball_h = 0.0 
    
    if best_box is not None: 
        sees = 1.0 
        x1_, y1_, x2_, y2_ = best_box.xyxy[0].cpu().numpy() 
        center_x = (x1_ + x2_) / 2.0 
        
        ball_w = x2_ - x1_ 
        ball_h = y2_ - y1_ 
        conf_val = float(best_box.conf[0].cpu().numpy()) 
        
        # Математика нормализации координат[cite: 2]
        x_norm = (center_x - w / 2.0) / (w / 2.0) #[cite: 2]
        bbox_height_norm = max(0.0, min(1.0, ball_h / h)) #[cite: 2]
        closeness = (bbox_height_norm - BBOX_HEIGHT_AT_MAX_DISTANCE) / (
            BBOX_HEIGHT_AT_CAPTURE_DISTANCE - BBOX_HEIGHT_AT_MAX_DISTANCE
        )
        closeness = max(0.0, min(1.0, closeness))
        y_norm = 1.0 - closeness #[cite: 2]
        
    # Формирование JSON для UDP[cite: 2]
    data = { #[cite: 2]
        "angle": float(x_norm), #[cite: 2]
        "distance": float(y_norm), #[cite: 2]
        "sees": float(sees), #[cite: 2]
        "conf": float(conf_val), #[cite: 2]
        "w": float(ball_w), #[cite: 2]
        "h": float(ball_h) #[cite: 2]
    } #[cite: 2]
    
    sock.sendto(json.dumps(data).encode(), (UDP_IP, UDP_PORT)) #[cite: 2]
