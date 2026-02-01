# 🪖 LifeLine Helmet – Smart Safety System

A smart system for motorcycles to prevent ignition when a drunk or non-helmet-wearing rider attempts to start the engine and also sends an SOS message when the rider is involved in an accident.

---

## 🚨 Problem Statement

Accidents caused by drunk riders and non-wearing of helmets are common for two-wheelers in India. The current system is not effective for preventing the said accidents while the ignition of the bike takes place.

---

## 💡 Proposed Solution

LifeLine Helmet is a safety system for the rider of a two-wheeler, which includes safety sensors, ignition control, accident detection, and GPS tracking.

---

## 🏗️ System Architecture

### Helmet Unit

* ESP32 C6
* MQ-3 Alcohol Sensor
* FSR (Helmet Detection)
* MPU-6050
* Neo-6M GPS

### Bike Unit

* ESP32 Dev Module
* Relay Module
* 16x2 LCD Display

---

## ⚙️ Working Logic

* ✅ Helmet Worn & No alcohol detected → Ignition enabled
* ❌ Helmet removed or Alcohol detected → Ignition disabled
* 🚑 Accident detected → SOS message sent

---

## 🔧 Components Used

* ESP32 Dev Module
* ESP32 C6
* MQ-3 Alcohol Sensor
* Force Sensitive Resistor (FSR)
* MPU-6050 Accelerometer & Gyroscope
* Neo-6M GPS Module
* 5V Relay Module
* 16x2 LCD Display
* Li-ion / LiPo Battery

---

## 🌐 Technology Used

* Embedded C / Arduino Framework
* ESP32 WiFi Communication
* REST API Integration
* JSON Data Handling
* Cloud-Based SMS Gateway (TextBee API)

### 📩 SMS Alert System

This project utilizes the **TextBee.dev Cloud SMS API** to send emergency SOS messages via the internet.

By utilizing a cloud-based SMS gateway, this system does not require the usage of a GSM module like SIM800/SIM900. This allows for:

* ✅ Reduced hardware costs
* ✅ Lower power consumption
* ✅ No SIM card dependency
* ✅ Improved scalability

🔗 TextBee ESP32 Integration Repository:

```
https://github.com/hasheerahmed/ESP32-WiFi-SMS-Without-GSM.git
```

---

## 📘 Documentation

The project report can be accessed in the `docs/` directory.

---

## 🚀 Future Enhancements

* AI-based rider behavior analysis
* IoT cloud dashboard
* Mobile application integration
* Solar-powered charging system
* Helmet authentication for anti-theft

---

## 🤝 Acknowledgment & Contribution

This project utilizes the **TextBee.dev SMS Gateway API** for emergency alerts and notifications.

We would like to thank and appreciate the TextBee team for providing a fair-use cloud messaging platform for educational and IoT projects.

The TextBee API support helps eliminate the usage of GSM hardware and improves the overall system efficiency.

---

## 📝 Note on Contribution

This project was initially designed and created for a mini-project in a group assignment.

The overall repository structure, firmware development, system integration, cloud API integration, and documentation are independently organized and handled by the repository owner.

---

## 📄 License

This project is for educational and academic purposes only. Please be sure to read the terms and conditions of third-party APIs (TextBee) before usage in production.
