# LifeLine Helmet – Smart Safety System
A smart helmet system which prevents motorcycles from being ignited by drunk or non-helmet riders, and sends SOS messages automatically in case of an accident.

## Problem Statement
Accidents caused by drunken riders and non-compliance with helmet usage are common for two-wheelers in India. Current methods are ineffective in preventing unsafe riding habits while ignition occurs.

## Proposed Solution
LifeLine Helmet will include an alcohol sensor, helmet sensor, ignition control, accident detection, and SOS messaging.

## System Architecture
- **Helmet Unit**: ESP32 C6, MQ-3, FSR, MPU-6050, GPS
- **Bike Unit**: ESP32 Dev Module, Relay, LCD Display

## Working Logic
- Helmet on, no alcohol: Ignition enabled
- Either helmet off or alcohol detected: Ignition disabled
- Accident detected: SOS message automatically sent with GPS coordinates

## Components Used
- ESP32 Dev Module
- ESP32 C6
- MQ-3 Alcohol Sensor
- Force Sensitive Resistor
- MPU-6050 Accelerometer & Gyroscope
- Neo-6M GPS Module
- 5V Relay Module
- 16x2 LCD Display
- Li-ion/Lipo Battery

## Documentation
A detailed project report is available in the `docs/` directory.

## Future Enhancements
- AI-driven analysis of rider behavior
- IoT integration
- Solar panel integration
- Helmet authentication for anti-theft

## Authors
- Hasheer Ahmed
- M D Zameer Hussain
- Muhammad Furqan
- Vinay B Subramanya
