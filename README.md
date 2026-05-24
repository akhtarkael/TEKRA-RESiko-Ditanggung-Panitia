# HYDRA — Hybrid Dynamic Room Automation
### Sistem Monitoring dan Otomasi Energi Ruang Kelas Berbasis IoT & AI
**TEKRA 2026 | IoT & Embedded System Concept Challenge**
**Tim: RESiko Ditanggung Panitia | FILKOM Universitas Brawijaya**

---

## Tentang HYDRA

**HYDRA** (*Hybrid Dynamic Room Automation*) adalah sistem otomasi dan monitoring energi ruang kelas berbasis IoT yang dirancang untuk menjawab permasalahan pemborosan energi di Universitas Brawijaya. Nama HYDRA terinspirasi dari mitologi Yunani, makhluk berkepala tiga, yang merepresentasikan **tiga lapis pertahanan sistem** dalam mengelola energi ruangan secara cerdas, efisien, dan andal.

Sistem ini dikembangkan dalam konteks tema **"Smart Campus Era: Where Technology Meets Creativity"** sebagai solusi nyata terhadap tantangan pengelolaan energi kampus yang selama ini dilakukan secara manual dan tidak efisien.

---

## Mengapa HYDRA Dibutuhkan?

Universitas Brawijaya dengan lebih dari 75.000 mahasiswa aktif mengoperasikan ribuan ruang kelas setiap harinya. Permasalahan utama yang dihadapi:

- **Tidak ada sistem monitoring terpusat**: petugas harus berkeliling manual untuk memastikan lampu dan AC dimatikan setelah jam kuliah
- **Pemborosan energi yang signifikan**: ruang kelas dibiarkan menyala berjam-jam meski sudah kosong, dengan estimasi kerugian puluhan hingga ratusan juta rupiah per tahun
- **Tidak ada data granular per ruangan**: laporan konsumsi listrik hanya mencerminkan total gedung, sehingga titik boros energi tidak bisa diidentifikasi
- **Kualitas lingkungan tidak terpantau**: suhu, kelembaban, dan kualitas udara yang memengaruhi konsentrasi belajar tidak dimonitor secara sistematis
- **Hambatan target Green Campus**: tanpa data efisiensi yang terukur, UB kesulitan membuktikan komitmen Green Campus 2023–2028

---

## Bagaimana HYDRA Bekerja — Tiga Lapis Sistem

HYDRA menggunakan pendekatan berlapis (*layered defense*) dalam mengelola energi ruangan. Setiap lapisan saling melengkapi dan memperkuat satu sama lain.

---

### Layer 1: Smart Environment Monitoring
**"Otomasi berbasis kondisi lingkungan"**

Layer pertama bertugas memantau kondisi lingkungan ruangan secara real-time menggunakan sensor:

| Sensor      | Fungsi                                                                                                                |
|-------------|-----------------------------------------------------------------------------------------------------------------------|
| **LDR**     | Mengukur intensitas cahaya alami. Jika cahaya alami sudah cukup, lampu tidak dinyalakan meskipun ada orang di ruangan |
| **DHT11**   | Mengukur suhu dan kelembaban ruangan. Jika kelembaban > 75%, sistem membuka ventilasi otomatis                        |
| **MHQ-135** | Monitoring kualitas udara pada ruangan. Jika kandungan gas berbahaya terlalu besar maka sistem membuka ventilasi      |

**Logika Layer 1:**
- Lampu hanya menyala jika: cahaya alami **kurang** (LDR ≥ 900) **DAN** ada orang terdeteksi
- Ventilasi (servo) terbuka jika: gas berbahaya terdeteksi (> 500) **ATAU** kelembaban terlalu tinggi (> 75%)
- Ventilasi tetap terbuka selama 10 detik setelah kondisi kembali normal (*safety timer*)

---

### Layer 2: AI-Powered Occupancy Detection
**"Otomasi berbasis kecerdasan buatan"**

Layer kedua menggunakan **computer vision berbasis YOLOv8** untuk mendeteksi keberadaan orang di ruangan secara akurat melalui kamera.

**Komponen:**
- **ESP32-CAM**: menangkap video stream ruangan secara real-time dan mengirimkannya via HTTP ke laptop/server
- **YOLOv8n (YOLO versi nano)**: model AI yang berjalan di laptop untuk mendeteksi objek kelas "person" dengan confidence threshold 0.5
- **PIR (Passive Infrared)**: Sebagai sinyal untuk memulai image processing
- **Laptop/Server**: memproses video stream, menjalankan inferensi YOLO, dan mengambil keputusan berdasarkan hasil deteksi

**Logika Layer 2:**
- PIR akan memonitor ruangan untuk pergerakan, jika **terdapat** pergerakan maka PIR akan memberi sinyal kepada laptop/server untuk memulai image processing
- Jika YOLO mendeteksi ≥ 1 orang → sinyal "ada orang" dikirim ke sistem
- Pencahayaan akan otomatis nyala jika terdapat orang dan pencahayaan alami tidak memadai
- Motor DC (simulasi Air Conditioner) hanya menyala jika: ada orang terdeteksi **DAN** cahaya kurang **DAN** toggle ON

**Mengapa arsitektur ini?**
ESP32-CAM memiliki keterbatasan processing power untuk menjalankan model AI. Oleh karena itu, ESP32-CAM hanya bertugas sebagai "mata" (capture & transmit), sedangkan pemrosesan berat dilakukan di laptop.

---

### Layer 3: Failsafe & Manual Override
**"Kontrol manual terpusat sebagai jaring pengaman"**

Layer ketiga adalah sistem failsafe yang memastikan seluruh perangkat listrik dapat dimatikan secara terpusat meskipun sistem otomatis mengalami gangguan.

**Komponen:**
- **Saklar/RFID Card**: toggle manual untuk mengaktifkan/menonaktifkan motor DC (simulasi Air Conditioner). Berguna ketika pengguna ingin override keputusan otomatis sistem

**Logika Layer 3:**
- Saklar atau RFID Card akan mematikan seluruh listrik pada ruangan yang terhubung jika berada pada posisi OFF atau kartu diangkat

---

## Arsitektur Sistem

```
┌─────────────────────────────────────────────────────────┐
│                     HYDRA ARCHITECTURE                  │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  [Sensor Layer]          [Processing Layer]             │
│                                                         │
│  PIR ──────────┐                                        │
│  LDR ──────────┤                                        │
│  DHT11 ────────┼──► ESP32 Utama ──► MQTT (HiveMQ) ──►   │
│  MHQ-135 ──────┤        ▲                   │           │
│  PZEM-004T-────┘        │                   ▼           │
│                         │           Laptop/Server       │
│  ESP32-CAM ──► HTTP Stream ──►    (Python Processor)    │
│                                   - Logika Rule-Based   │
│                                   - YOLO Inference      │
│                                   - Dashboard           │
│                                         │               │
│  [Actuator Layer]                       │               │
│                                         ▼               │
│  LED (Lampu) ◄──────────────── MQTT Commands            │
│  Motor DC (Air Conditioner) ◄──────────┤                │
│  Servo (Ventilasi) ◄───────────────────┘                │
│                                                         │
│  [Dashboard]                                            │
│  HTML      ── Status sensor real-time                   │
│            ── Status aktuator                           │
│            ── Live camera feed + YOLO annotation        │
│            ── Monitoring konsumsi energi                │
└─────────────────────────────────────────────────────────┘
```

---

## Protokol Komunikasi

| Komunikasi           | Protokol           | Keterangan                                      |
|----------------------|--------------------|-------------------------------------------------|
| ESP32 Utama → Laptop | **MQTT over WiFi** | Publish data sensor ke HiveMQ public broker     |
| Laptop → ESP32 Utama | **MQTT over WiFi** | Publish perintah aktuator (lampu, motor, servo) |
| ESP32-CAM → Laptop   | **HTTP Stream**    | Video stream MJPEG real-time                    |
| Internal ESP32       | **FreeRTOS Tasks** | Multitasking sensor dan aktuator                |

**MQTT Broker:** HiveMQ Public (`broker.hivemq.com:1883`)
**Topic prefix:** `tekra2026/RESikoDitanggungPanitia/esp32/`

---

## Komponen Hardware

### ESP32 Utama
| Komponen         | Pin     | Fungsi                 |
|------------------|---------|------------------------|
| PIR HC-SR501 (1) | GPIO 34 | Deteksi gerakan zona 1 |
| PIR HC-SR501 (2) | GPIO 18 | Deteksi gerakan zona 2 |
| DHT11            | GPIO 32 | Suhu & kelembaban      |
| MHQ-135          | GPIO 33 | Kualitas udara         |
| LDR              | GPIO 35 | Intensitas cahaya      |
| LED (Lampu)      | GPIO 21 | Simulasi lampu ruangan |
| Motor DC         | GPIO 25 | Simulasi AC            |
| Servo            | GPIO 27 | Simulasi ventilasi     |
| Push Button      | GPIO 15 | Toggle manual motor    |
| Saklar           | GPIO 17 |

### ESP32-CAM
- Model: AI Thinker ESP32-CAM
- Format: YUV422 → JPEG (konversi real-time)
- Resolusi: QVGA (320x240)
- Fungsi: HTTP video streaming ke laptop

---

## 💻 Software Stack

| Komponen            | Teknologi                              |
|---------------------|----------------------------------------|
| Firmware ESP32      | Arduino IDE (C++) + FreeRTOS           | 
| Processing & Logika | Python 3                               |
| Computer Vision     | YOLOv8n (Ultralytics) + OpenCV         |
| Dashboard           | HTML5 + FastAPI + Uvicorn Async Server |
| IoT Communication   | paho-mqtt                              |
| MQTT Broker         | HiveMQ Public                          |

---

## Cara Menjalankan

### Prasyarat
```bash
pip install paho-mqtt ultralytics streamlit opencv-python
```

### ESP32 Utama
1. Install library: `PubSubClient`, `ArduinoJson`, `ESP32Servo`, `DHT`
2. Ganti `NamaWiFi` dan `PasswordWiFi` di kode
3. Upload `esp32_utama_final.ino` via Arduino IDE

### ESP32-CAM
1. Ganti `NamaWiFi` dan `PasswordWiFi` di kode
2. Upload kode ESP32-CAM via Arduino IDE (gunakan FTDI programmer, IO0 di-ground saat upload)
3. Catat IP yang muncul di Serial Monitor

### Laptop
1. Ganti `ESP_IP_URL` dengan IP ESP32-CAM
2. Jalankan kode python
3. Buka file index.html

---

## Limitasi & Pengembangan ke Depan

| Limitasi Saat Ini                              | Solusi ke Depan                                      |
|------------------------------------------------|------------------------------------------------------|
| Prototipe skala diorama                        | Implementasi di ruang kelas nyata                    |
| Broker publik HiveMQ                           | Self-hosted MQTT broker (Mosquitto) di server kampus |
| Satu ruangan per ESP32                         | Jaringan multi-node per gedung                       |
| Pembacaan energi yang belum diimplementasikan  | Memakai PZEM-004T untuk pembacaan energi             |
| Laptop sebagai server processing               | Raspberry Pi atau edge server dedicated              |
| Dashboard masih lokal dan file statis          | Deployment ke webserver terpusat                     |

---

## Tim

| Nama     | Peran                      |
|----------|----------------------------|
| Kael     | Hardware & Embedded System |
| Jennifer | Computer Vision & AI       |
| Aaisyah  | UI/UX & Dashboard          |

**FILKOM Universitas Brawijaya | TEKRA 2026**
