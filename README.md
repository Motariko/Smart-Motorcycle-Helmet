# Smart-Motorcycle-Helmet# ScalPro: 
Motorcycle Safety and Accident Detection System

is an integrated IoT safety solution designed for motorcycle riders to enhance road awareness and ensure rapid accident response. The system operates through a distributed architecture comprising two primary hardware nodes that communicate using the high-speed ESP-NOW protocol.

## System Architecture

The project is divided into two distinct units that perform specialized tasks to maintain a continuous safety net for the rider.

### Helmet Unit

The Helmet Unit serves as the primary detection node attached to the rider. It utilizes an MPU6050 Inertial Measurement Unit to monitor orientation and movement. To ensure high data integrity, a Kalman Filter is implemented to fuse accelerometer and gyroscope data, effectively removing noise caused by engine vibrations or road irregularities. This allows the system to accurately identify critical lean angles and sudden impacts that indicate a fall or collision.

### Tail Unit

The Tail Unit is mounted on the motorcycle and functions as the communication gateway. It is equipped with ultrasonic sensors to monitor the proximity of vehicles approaching from the rear, providing real-time blind-spot alerts. Additionally, it integrates a GPS module to maintain constant geographical coordinates. When the Helmet Unit detects an emergency, the Tail Unit acts as a bridge, utilizing its WiFi capabilities to transmit accident data and precise locations to a centralized server via HTTP POST requests.

## Communication and Reliability

A core feature of ScalPro is the use of the ESP-NOW protocol for inter-device communication. This choice provides a low-latency connection that is significantly faster than standard WiFi, which is crucial for time-sensitive proximity alerts. Furthermore, it offers superior power efficiency for the battery-operated Helmet Unit and maintains a stable point-to-point link without requiring an external router.

## Technical Specifications

The system is built using ESP32 and ESP8266 microcontrollers programmed in C++ within the Arduino framework. Key sensors include the MPU6050 for motion tracking, ultrasonic sensors for distance measurement, and GPS modules for location tracking. Software implementation involves complex algorithms for signal processing and the use of libraries such as TinyGPSPlus and WiFiManager for robust connectivity management.

## Project Structure and Usage

The repository is organized into specific directories for the Helmet Node and the Tail Node to maintain clear separation of concerns. To deploy the system, users must rename the provided configuration template to a standard header file and input their specific hardware MAC addresses and server endpoints. While the server-side API is not included in this repository, the system is designed to be compatible with any backend capable of receiving standard JSON payloads.

Developed by Isara Phetsila, Computer Engineering Student at Ramkhamhaeng University.
