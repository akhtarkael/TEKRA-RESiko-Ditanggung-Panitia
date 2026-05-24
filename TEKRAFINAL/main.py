from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse
import paho.mqtt.client as mqtt
import json
import time
import threading
import cv2
import os
from ultralytics import YOLO
from collections import deque
from datetime import datetime
import uvicorn

# =============================================
# Setup FastAPI
# =============================================
app = FastAPI(title="SmartRoom Backend - TEKRA 2026")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# =============================================
# Konfigurasi MQTT
# =============================================
BROKER = "broker.hivemq.com"
PORT   = 1883
PREFIX = "tekra2026/RESikoDitanggungPanitia"
TOPIC_SENSOR = f"{PREFIX}/esp32/sensor"
TOPIC_LAMPU  = f"{PREFIX}/esp32/cmd/lampu"
TOPIC_MOTOR  = f"{PREFIX}/esp32/cmd/motor"
TOPIC_SERVO  = f"{PREFIX}/esp32/cmd/servo"
ESP_IP_URL   = "http://ganti.ip.address.streaming/" #ganti ip address streaming espcam

# =============================================
# Module-Level State
# =============================================
_shared = {
    "sensor": {"kelembapan":0,"gas":0,"kecerahan":0,"gerakan":0,"toggleMotor":0,"saklar":1},
    "aktuator": {"lampu":0,"motor":0,"servo":0},
    "status": {"mqtt_ok":False},
    "_servo_active":False, "_servo_timer":0,
    "_motor_locked":False, "_motor_timer":0,
    "_waktu_ada_orang":time.time(),
    "_status_ada_orang":False,
    "_yolo_detect":False,
    "_camera_open":False,
    "_mqtt_client":None,
}
_history = {"gas":deque(maxlen=80),"hum":deque(maxlen=80),"ldr":deque(maxlen=80)}
_activity_log = deque(maxlen=50)
_lock = threading.Lock()
TIMEOUT_KOSONG = 30

_latest_jpeg = None # Menyimpan frame kamera terakhir untuk dikirim ke frontend

def _log(msg, level="info"):
    _activity_log.appendleft({"ts":datetime.now().strftime("%H:%M:%S"),"msg":msg,"level":level})

# =============================================
# MQTT Callback
# =============================================
def _on_connect(client, userdata, flags, rc):
    if rc == 0:
        _shared["status"]["mqtt_ok"] = True
        client.subscribe(TOPIC_SENSOR)
        _log("Connected to HiveMQ broker", "ok")
    else:
        _shared["status"]["mqtt_ok"] = False

def _on_disconnect(client, userdata, rc):
    _shared["status"]["mqtt_ok"] = False
    _log("MQTT connection lost", "warn")

def _on_message(client, userdata, msg):
    try:
        p = json.loads(msg.payload.decode())
        with _lock:
            s = _shared["sensor"]
            s["kelembapan"]  = p.get("kelembapan",  0)
            s["gas"]         = p.get("gas",         0)
            s["kecerahan"]   = p.get("kecerahan",   0)
            s["gerakan"]     = p.get("gerakan",       0)
            s["toggleMotor"] = p.get("toggleMotor",  0)
            s["saklar"]      = p.get("saklar",       1)
            
            _history["gas"].append(s["gas"])
            _history["hum"].append(s["kelembapan"])
            _history["ldr"].append(s["kecerahan"])
    except Exception:
        pass

# Inisialisasi Model & Kamera Global
model = YOLO("yolov8n.pt")
cap = cv2.VideoCapture()

# =============================================
# Automation Logic
# =============================================
def _loop_otomasi():
    while True:
        with _lock:
            d  = _shared["sensor"].copy()
            ak = _shared["aktuator"]
            ada_orang = _shared["_status_ada_orang"]
        mc  = _shared.get("_mqtt_client")
        now = time.time()
        
        if not mc:
            time.sleep(0.2); continue

        # Layer 1 — Emergency
        if d["gas"] > 500 or d["kelembapan"] > 75:
            mc.publish(TOPIC_SERVO, json.dumps({"angle": 90}))
            mc.publish(TOPIC_MOTOR, json.dumps({"status": 1}))
            with _lock:
                ak["servo"]=90; ak["motor"]=1
                _shared["_servo_active"]=True; _shared["_servo_timer"]=now
                _shared["_motor_locked"]=True; _shared["_motor_timer"]=now
            if d["gas"] > 500: _log(f"EMERGENCY — Gas level critical: {d['gas']}", "alert")
            if d["kelembapan"] > 75: _log(f"EMERGENCY — Humidity critical: {d['kelembapan']}%", "alert")
        else:
            with _lock:
                sa=_shared["_servo_active"]; st_=_shared["_servo_timer"]
                ml=_shared["_motor_locked"]; mt=_shared["_motor_timer"]
            if sa and (now-st_ >= 10):
                mc.publish(TOPIC_SERVO, json.dumps({"angle":0}))
                with _lock: ak["servo"]=0; _shared["_servo_active"]=False
            if ml and (now-mt >= 10):
                with _lock: _shared["_motor_locked"]=False

        with _lock: motor_locked=_shared["_motor_locked"]

        if not motor_locked:
            if d["saklar"] == 0:
                mc.publish(TOPIC_LAMPU, json.dumps({"status":0}))
                mc.publish(TOPIC_MOTOR, json.dumps({"status":0}))
                with _lock: ak["lampu"]=0; ak["motor"]=0
            else:
                if ada_orang:
                    lmp = 1 if d["kecerahan"] >= 900 else 0
                    mtr = 1 if d["toggleMotor"] else 0
                    mc.publish(TOPIC_LAMPU, json.dumps({"status":lmp}))
                    mc.publish(TOPIC_MOTOR, json.dumps({"status":mtr}))
                    with _lock: ak["lampu"]=lmp; ak["motor"]=mtr
                else:
                    mc.publish(TOPIC_LAMPU, json.dumps({"status":0}))
                    mc.publish(TOPIC_MOTOR, json.dumps({"status":0}))
                    with _lock: ak["lampu"]=0; ak["motor"]=0
        time.sleep(0.15)

# =============================================
# Camera & YOLO
# =============================================
def _loop_camera():
    global _latest_jpeg
    while True:
        with _lock:
            s = _shared["sensor"].copy()
            saklar_on = s["saklar"] == 1
            cam_open = _shared["_camera_open"]

        if saklar_on and (s["gerakan"] or cam_open):
            if not cap.isOpened(): cap.open(ESP_IP_URL)
            if cap.isOpened():
                ret, frame = cap.read()
                if ret and frame is not None:
                    results  = model(frame, classes=[0], conf=0.5, verbose=False)
                    yolo_now = len(results[0].boxes) > 0
                    ann      = results[0].plot()
                    
                    # Encode frame untuk stream UI (HTML)
                    ret_jpg, jpeg = cv2.imencode('.jpg', ann)
                    if ret_jpg:
                        _latest_jpeg = jpeg.tobytes()

                    with _lock:
                        _shared["_yolo_detect"] = yolo_now
                        _shared["_camera_open"] = True
                        if yolo_now:
                            _shared["_status_ada_orang"] = True
                            _shared["_waktu_ada_orang"]  = time.time()
                        else:
                            if time.time() - _shared["_waktu_ada_orang"] > TIMEOUT_KOSONG:
                                _shared["_status_ada_orang"] = False
                                cap.release()
                                _shared["_camera_open"] = False
                else:
                    with _lock: _shared["_camera_open"] = False
        else:
            if cap.isOpened(): cap.release()
            with _lock:
                _shared["_camera_open"] = False
                _shared["_yolo_detect"] = False
                if not saklar_on: _shared["_status_ada_orang"] = False
        
        time.sleep(0.1) # Hindari CPU overheat

# =============================================
# API Endpoints
# =============================================

@app.on_event("startup")
def startup_event():
    # Mulai MQTT
    c = mqtt.Client(client_id="smartroom-backend-tekra2026", clean_session=True)
    c.on_connect = _on_connect
    c.on_disconnect = _on_disconnect
    c.on_message = _on_message
    try:
        c.connect(BROKER, PORT, 60)
        c.loop_start()
    except Exception as e:
        print(f"Gagal konek MQTT: {e}")
    _shared["_mqtt_client"] = c

    threading.Thread(target=_loop_otomasi, daemon=True).start()
    threading.Thread(target=_loop_camera, daemon=True).start()

@app.get("/api/data")
def get_data():
    """Mengirim semua data state terkini ke Frontend"""
    with _lock:
        return {
            "sensor": _shared["sensor"],
            "aktuator": _shared["aktuator"],
            "status": _shared["status"],
            "system": {
                "ada_orang": _shared["_status_ada_orang"],
                "yolo_detect": _shared["_yolo_detect"],
                "camera_open": _shared["_camera_open"]
            },
            "logs": list(_activity_log)
        }

def generate_video_frames():
    """Generator untuk stream MJPEG"""
    while True:
        if _latest_jpeg is not None:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + _latest_jpeg + b'\r\n')
        time.sleep(0.1)

@app.get("/api/video_feed")
def video_feed():
    """Endpoint untuk stream video (masukkan ke dalam src tag <img> di HTML)"""
    return StreamingResponse(generate_video_frames(), media_type="multipart/x-mixed-replace; boundary=frame")


# =============================================
# Run Server
# =============================================
if __name__ == "__main__":
    uvicorn.run("main:app", host="127.0.0.1", port=8000, reload=True)