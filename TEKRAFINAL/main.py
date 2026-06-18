from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse, FileResponse
import paho.mqtt.client as mqtt
from dotenv import load_dotenv
import json
import time
import threading
import cv2
import os
from ultralytics import YOLO
from collections import deque
from datetime import datetime
import uvicorn
import math

load_dotenv()

# Setup FastAPI
app = FastAPI(title="SmartRoom Backend - TEKRA 2026")
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Konfigurasi MQTT
BROKER     = os.getenv("MQTT_BROKER")
PORT       = int(os.getenv("MQTT_PORT"))
MQTT_USER  = os.getenv("MQTT_USER")
MQTT_PASS  = os.getenv("MQTT_PASS")
RELAY_TIMEOUT = 30  # ganti ke 600 saat pameran (10 menit)

LDR_UTP = 950
LDR_LTP = 750

PREFIX            = "tekra2026/RESikoDitanggungPanitia"
TOPIC_SENSOR      = f"{PREFIX}/esp32/sensor"
TOPIC_LAMPU       = f"{PREFIX}/esp32/cmd/lampu"
TOPIC_MOTOR       = f"{PREFIX}/esp32/cmd/motor"
TOPIC_SERVO       = f"{PREFIX}/esp32/cmd/servo"
TOPIC_WINDOW      = f"{PREFIX}/esp32/cmd/window"
TOPIC_RESET_PZEM  = f"{PREFIX}/esp32/cmd/reset_pzem"

HUM_OPEN      = 65;  GAS_OPEN_PPM  = 1000  # ppm CO2 equiv
HUM_CLOSE     = 50;  GAS_CLOSE_PPM = 600
GAS_EMERG_PPM = 2500

ESP_IP_URL = "http://YOUR_ESP_CAM_IP/"  # ganti IP ESP-CAM sesuai jaringan

# State sistem
_shared = {
    "sensor": {
        "kelembapan": 0, "suhu": 0, "gas": 0, "kecerahan": 0,
        "gas_ppm": 400.0, "kecerahan_lux": 200.0,
        "gerakan": 0, "toggleMotor": 0, "saklar": 1,
        "tegangan": 0.0, "arus": 0.0, "daya": 0.0,
        "energi": 0.0, "frekuensi": 0.0, "powerFactor": 0.0,
    },
    "aktuator": {"lampu": 0, "motor": 0, "servo": 0, "window": 0},
    "status": {"mqtt_ok": False},
    "_motor_locked": False, "_motor_timer": 0,
    "_emergency": False,
    "_relay_on": False, "_relay_empty_since": None,
    "_waktu_ada_orang": time.time(),
    "_status_ada_orang": False,
    "_yolo_detect": False,
    "_camera_open": False,
    "_mqtt_client": None,
}
_history      = {"gas": deque(maxlen=80), "hum": deque(maxlen=80), "ldr": deque(maxlen=80)}
_activity_log = deque(maxlen=50)
_lock         = threading.Lock()
TIMEOUT_KOSONG = 30

# Sensor smoothing & konversi
EMA_ALPHA = 0.10
_ema_gas  = None
_ema_ldr  = None

# MQ-135 CO2 — dikalibrasi pada ADC=379 (udara bersih ≈ 400 ppm)
_MQ135_RLOAD = 10.0
_MQ135_R0    = 154.0  # kΩ, ubah jika resistor beban berbeda
_MQ135_A     = 110.47
_MQ135_B     = -2.862

def _adc_to_co2_ppm(adc: float) -> float:
    v_rl = adc / 4095.0 * 3.3
    if v_rl < 0.02: return 400.0
    rs    = _MQ135_RLOAD * (3.3 - v_rl) / v_rl
    ratio = rs / _MQ135_R0
    if ratio <= 0: return 400.0
    ppm = _MQ135_A * (ratio ** _MQ135_B)
    return round(max(350.0, min(10000.0, ppm)), 1)

# LDR → Lux, GL5528 power-law (R_fixed pull-up, LDR pull-down ke GND)
_LDR_RFIXED = 10.0  # kΩ — ubah sesuai resistor di skema
_LDR_R0_LUX = 10.0
_LDR_GAMMA  = 0.7

def _adc_to_lux(adc: float) -> float:
    v = adc / 4095.0 * 3.3
    if v < 0.01: return 10000.0
    if v > 3.28: return 0.1
    r_ldr = _LDR_RFIXED * v / (3.3 - v)
    if r_ldr <= 0: return 10000.0
    lux = 10.0 * (_LDR_R0_LUX / r_ldr) ** (1.0 / _LDR_GAMMA)
    return round(max(0.1, min(100000.0, lux)), 1)

_latest_jpeg      = None
_last_ann         = None
_latest_raw_frame = None
_raw_lock         = threading.Lock()
_fps_value        = 0.0

def _log(msg, level="info"):
    _activity_log.appendleft({"ts": datetime.now().strftime("%H:%M:%S"), "msg": msg, "level": level})


# MQTT callbacks
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
    global _ema_gas, _ema_ldr
    try:
        p       = json.loads(msg.payload.decode())
        raw_gas = float(p.get("gas",       0))
        raw_ldr = float(p.get("kecerahan", 0))

        _ema_gas = raw_gas if _ema_gas is None else EMA_ALPHA * raw_gas + (1 - EMA_ALPHA) * _ema_gas
        _ema_ldr = raw_ldr if _ema_ldr is None else EMA_ALPHA * raw_ldr + (1 - EMA_ALPHA) * _ema_ldr

        with _lock:
            s = _shared["sensor"]
            s["kelembapan"]    = p.get("kelembapan",  0)
            s["suhu"]          = p.get("suhu",        0)
            s["gas"]           = round(_ema_gas, 1)
            s["kecerahan"]     = round(_ema_ldr, 1)
            s["gas_ppm"]       = _adc_to_co2_ppm(_ema_gas)
            s["kecerahan_lux"] = _adc_to_lux(_ema_ldr)
            s["gerakan"]       = p.get("gerakan",     0)
            s["toggleMotor"]   = p.get("toggleMotor", 0)
            s["saklar"]        = p.get("saklar",      1)
            s["tegangan"]      = p.get("tegangan",    0.0)
            s["arus"]          = p.get("arus",        0.0)
            s["daya"]          = p.get("daya",        0.0)
            s["energi"]        = p.get("energi",      0.0)
            s["frekuensi"]     = p.get("frekuensi",   0.0)
            s["powerFactor"]   = p.get("powerFactor", 0.0)

            _history["gas"].append(s["gas_ppm"])
            _history["hum"].append(s["kelembapan"])
            _history["ldr"].append(s["kecerahan_lux"])
    except Exception:
        pass


model = YOLO("yolov8n.pt")
cap   = cv2.VideoCapture()


# Otomasi relay, lampu, motor, window servo
def _loop_otomasi():
    _last_relay = None
    _last_win   = None  # None agar force-publish pertama override stale retained message
    while True:
        with _lock:
            d                 = _shared["sensor"].copy()
            ak                = _shared["aktuator"]
            ada_orang         = _shared["_status_ada_orang"]
            relay_on          = _shared["_relay_on"]
            relay_empty_since = _shared["_relay_empty_since"]
        mc  = _shared.get("_mqtt_client")
        now = time.time()

        if not mc:
            time.sleep(0.2); continue

        # Emergency fan
        if d["gas_ppm"] > GAS_EMERG_PPM or d["kelembapan"] > 75:
            mc.publish(TOPIC_MOTOR, json.dumps({"status": 1}))
            with _lock:
                ak["motor"] = 1
                _shared["_motor_locked"] = True
                _shared["_motor_timer"]  = now
            if d["gas_ppm"] > GAS_EMERG_PPM:
                _log(f"EMERGENCY — Gas critical: {d['gas_ppm']} ppm", "alert")
            if d["kelembapan"] > 75:
                _log(f"EMERGENCY — Humidity critical: {d['kelembapan']}%", "alert")
        else:
            with _lock:
                ml = _shared["_motor_locked"]
                mt = _shared["_motor_timer"]
            if ml and (now - mt >= 10):
                with _lock: _shared["_motor_locked"] = False

        with _lock:
            motor_locked = _shared["_motor_locked"]
            emergency    = _shared["_emergency"]

        # Main relay
        if emergency:
            relay_on = False
            relay_empty_since = None
        else:
            if ada_orang:
                relay_on = True
                relay_empty_since = None
            else:
                if relay_on:
                    if relay_empty_since is None:
                        relay_empty_since = now
                    elif now - relay_empty_since >= RELAY_TIMEOUT:
                        relay_on = False
                        relay_empty_since = None
        with _lock:
            if _shared["_emergency"]:  # re-check: emergency bisa aktif antara read dan write
                relay_on = False
                relay_empty_since = None
            _shared["_relay_on"]          = relay_on
            _shared["_relay_empty_since"] = relay_empty_since

        # Window servo
        with _lock: curr_win = ak["window"]
        if d["kelembapan"] > HUM_OPEN or d["gas_ppm"] > GAS_OPEN_PPM:
            win = 1
        elif d["kelembapan"] < HUM_CLOSE or d["gas_ppm"] < GAS_CLOSE_PPM:
            win = 0
        else:
            win = curr_win
        if win != _last_win:
            mc.publish(TOPIC_WINDOW, json.dumps({"status": win}), retain=True)
            with _lock: ak["window"] = win
            if _last_win is not None:
                _log(f"Window servo {'BUKA' if win else 'TUTUP'} — Hum:{d['kelembapan']}% Gas:{d['gas_ppm']} ppm", "info")
            _last_win = win

        relay = 1 if (relay_on and d["saklar"] == 1) else 0
        if relay != _last_relay:
            mc.publish(TOPIC_SERVO, json.dumps({"status": relay}), retain=True)
            with _lock: ak["servo"] = relay
            _last_relay = relay

        if not motor_locked:
            if d["saklar"] == 0:
                mc.publish(TOPIC_LAMPU, json.dumps({"status": 0}))
                mc.publish(TOPIC_MOTOR, json.dumps({"status": 0}))
                with _lock: ak["lampu"] = 0; ak["motor"] = 0
            else:
                if relay_on:
                    lmp = 1
                    mtr = 1 if (ada_orang and d["toggleMotor"]) else 0
                    mc.publish(TOPIC_LAMPU, json.dumps({"status": lmp}))
                    mc.publish(TOPIC_MOTOR, json.dumps({"status": mtr}))
                    with _lock: ak["lampu"] = lmp; ak["motor"] = mtr
                else:
                    mc.publish(TOPIC_LAMPU, json.dumps({"status": 0}))
                    mc.publish(TOPIC_MOTOR, json.dumps({"status": 0}))
                    with _lock: ak["lampu"] = 0; ak["motor"] = 0
        time.sleep(0.15)


# Frame grabber — thread terpisah agar _loop_camera selalu dapat frame terbaru
def _frame_grabber():
    global _latest_raw_frame, _fps_value
    fps_count   = 0
    fps_ts      = time.time()
    fail_streak = 0

    while True:
        with _lock:
            s         = _shared["sensor"]
            saklar_on = s["saklar"] == 1
            gerakan   = s["gerakan"]
            cam_open  = _shared["_camera_open"]

        should_grab = saklar_on and (gerakan or cam_open)

        if should_grab:
            if not cap.isOpened():
                cap.open(ESP_IP_URL)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                time.sleep(0.15)
                continue
            ret, frame = cap.read()
            if ret and frame is not None:
                with _raw_lock: _latest_raw_frame = frame
                fail_streak = 0
                fps_count  += 1
                now = time.time()
                if now - fps_ts >= 1.0:
                    _fps_value = round(fps_count / (now - fps_ts), 1)
                    fps_count  = 0
                    fps_ts     = now
            else:
                fail_streak += 1
                if fail_streak > 15:
                    if cap.isOpened(): cap.release()
                    with _raw_lock: _latest_raw_frame = None
                    fail_streak = 0
        else:
            if cap.isOpened(): cap.release()
            with _raw_lock: _latest_raw_frame = None
            _fps_value = 0.0
            time.sleep(0.02)


# YOLO deteksi + encode MJPEG
def _loop_camera():
    global _latest_jpeg, _last_ann
    yolo_counter = 0

    while True:
        with _lock:
            saklar_on = _shared["sensor"]["saklar"] == 1
            gerakan   = _shared["sensor"]["gerakan"]
            ada_orang = _shared["_status_ada_orang"]

        with _raw_lock:
            frame = _latest_raw_frame

        # Gunakan ada_orang bukan cam_open agar tidak ada dependency cycle
        active = saklar_on and (gerakan or ada_orang)

        if active:
            with _lock: _shared["_camera_open"] = True  # set sebelum frame tiba agar overlay tidak flash

            if frame is not None:
                frame_hd = cv2.resize(frame, (640, 480)) if frame.shape[1] < 640 else frame

                yolo_counter += 1
                run_yolo = (yolo_counter % 2 == 0)

                if run_yolo:
                    results  = model(frame_hd, classes=[0], conf=0.5, verbose=False)
                    yolo_now = len(results[0].boxes) > 0
                    if yolo_now:
                        ann = results[0].plot()
                        _last_ann = ann
                    else:
                        ann = frame_hd
                        _last_ann = None
                else:
                    yolo_now = None
                    ann = _last_ann if _last_ann is not None else frame_hd

                ret_jpg, jpeg = cv2.imencode('.jpg', ann, [cv2.IMWRITE_JPEG_QUALITY, 70])
                if ret_jpg:
                    _latest_jpeg = jpeg.tobytes()

                with _lock:
                    if run_yolo:
                        _shared["_yolo_detect"] = yolo_now
                        if yolo_now:
                            _shared["_status_ada_orang"] = True
                            _shared["_waktu_ada_orang"]  = time.time()
                        else:
                            # Timeout hanya matikan ada_orang; camera_open tutup via jalur active=False
                            if time.time() - _shared["_waktu_ada_orang"] > TIMEOUT_KOSONG:
                                _shared["_status_ada_orang"] = False
        else:
            with _lock:
                _shared["_camera_open"] = False
                _shared["_yolo_detect"] = False
                if not saklar_on:
                    _shared["_status_ada_orang"] = False

        time.sleep(0.01)


# API Endpoints
@app.get("/")
def serve_dashboard():
    return FileResponse(os.path.join(os.path.dirname(os.path.abspath(__file__)), "index.html"))

@app.on_event("startup")
def startup_event():
    c = mqtt.Client(client_id="smartroom-backend-tekra2026", clean_session=True)
    c.on_connect    = _on_connect
    c.on_disconnect = _on_disconnect
    c.on_message    = _on_message
    try:
        c.connect(BROKER, PORT, 60)
        c.loop_start()
    except Exception as e:
        print(f"Gagal konek MQTT: {e}")
    _shared["_mqtt_client"] = c

    threading.Thread(target=_loop_otomasi,  daemon=True).start()
    threading.Thread(target=_frame_grabber, daemon=True).start()
    threading.Thread(target=_loop_camera,   daemon=True).start()

@app.post("/api/emergency_shutdown")
def emergency_shutdown():
    mc = _shared.get("_mqtt_client")
    if not mc:
        return {"ok": False, "reason": "MQTT not connected"}
    with _lock:
        activate = not _shared["_emergency"]
        _shared["_emergency"] = activate
        if activate:
            _shared["_relay_on"] = False
            _shared["_relay_empty_since"] = None
            _shared["aktuator"]["servo"] = 0
    if activate:
        mc.publish(TOPIC_SERVO, json.dumps({"status": 0}), retain=True)
        _log("EMERGENCY LOCK AKTIF — relay diblokir, sistem terkunci", "alert")
    else:
        _log("EMERGENCY LOCK DILEPAS — sistem kembali normal", "ok")
    return {"ok": True, "emergency": activate}

@app.get("/api/data")
def get_data():
    now = time.time()
    with _lock:
        empty_since    = _shared["_relay_empty_since"]
        time_until_off = round(max(0, RELAY_TIMEOUT - (now - empty_since))) if empty_since else None
        return {
            "sensor":   _shared["sensor"].copy(),
            "aktuator": _shared["aktuator"].copy(),
            "status":   _shared["status"],
            "system": {
                "ada_orang":    _shared["_status_ada_orang"],
                "yolo_detect":  _shared["_yolo_detect"],
                "camera_open":  _shared["_camera_open"],
                "fps":          _fps_value,
            },
            "emergency": _shared["_emergency"],
            "relay_info": {
                "active":         _shared["_relay_on"],
                "time_until_off": time_until_off,
            },
            "logs": list(_activity_log),
        }

def generate_video_frames():
    while True:
        if _latest_jpeg is not None:
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + _latest_jpeg + b'\r\n')
        time.sleep(0.04)

@app.post("/api/reset_pzem")
def reset_pzem():
    mc = _shared.get("_mqtt_client")
    if not mc:
        return {"ok": False, "reason": "MQTT not connected"}
    mc.publish(TOPIC_RESET_PZEM, json.dumps({"reset": 1}))
    with _lock: _shared["sensor"]["energi"] = 0.0
    _log("PZEM energy counter di-reset", "info")
    return {"ok": True}

@app.get("/api/video_feed")
def video_feed():
    return StreamingResponse(generate_video_frames(), media_type="multipart/x-mixed-replace; boundary=frame")

if __name__ == "__main__":
    uvicorn.run("main:app", host="0.0.0.0", port=8000, reload=False)
