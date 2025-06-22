# 🔥 Fire Detection System

## 🚀 Overview

This project implements a **real-time fire detection system** using both:
- **Environmental sensors** for physical indicators (gas, temperature, flame)
- **Visual confirmation** using a lightweight ML model deployed on the ESP32-CAM

By combining the two, we create a **robust hybrid system** ideal for indoor environments like **kitchens, labs, and factories**, minimizing false positives.

---

## 🧰 Tools & Hardware

- **Edge Impulse Studio** — Train and deploy the image classification model
- **Arduino IDE** — Program and flash ESP32 & ESP32-CAM
- **ESP32-CAM** — Captures images and runs the ML model
- **ESP32** — Supports extra sensors via I2C/analog
- **Serial Monitor** — For live sensor + inference debugging

---

## 🛠️ Sensors Used

| Sensor       | Purpose                            |
|--------------|-------------------------------------|
| MQ-2         | Smoke detection                     |
| MQ-5         | LPG gas detection                   |
| MQ-7         | CO detection (with heating cycle)   |
| Flame Sensor | Detects IR light from open flames   |
| MLX90614     | Measures object temperature         |
| BME680       | Temperature, humidity, gas resistance |

---

## 📷 Machine Learning

We trained a custom **image classification model** with:
- `fire`
- `non-fire`
- `light`

This model was deployed to the **ESP32-CAM** using Edge Impulse, enabling on-device inference with no need for cloud connectivity.

---

## 🔄 System Flow

1. **Sensors & camera** continuously collect data
2. **Sensor thresholds** are checked
3. **ML model** classifies the scene
4. If **either** sensor triggers or ML classifies fire → fire is flagged
5. Serial output displays full readings and classification

---

## ⚡ What We Demonstrate

- A complete hybrid fire detection system
- Reduced false positives using sensor fusion + ML
- Embedded AI on low-power microcontrollers
- Real-time logging and feedback via Serial

---

## 📚 Libraries Used

- `Wire.h` — I2C communication
- `Adafruit_MLX90614.h` — MLX90614 IR temperature sensor
- `Adafruit_BME680.h` — BME680 environmental sensor
- `Adafruit_Sensor.h` — Sensor base library
- `ArduinoJson.h` — JSON parsing and serial formatting
- `esp_camera.h` — Camera capture on ESP32-CAM
- `Indoor-fire_ML_model_inferencing.h` — Edge Impulse model header

---

## 🧠 Embedded Intelligence

This project showcases how **embedded ML + sensor fusion** can create intelligent safety systems that:
- Don’t rely on internet/cloud
- Run efficiently on microcontrollers
- Are cost-effective and portable

---

## 🧩 Future Improvements

- Add alert system (buzzer, LEDs, wireless messaging)
- Cloud logging via MQTT or Firebase
- Real-time dashboard for monitoring
