# STM32 Drone Flight Controller (WIP)  
A real-time, flight controller for STM32F4-based drones, built using FreeRTOS and custom drivers.

---

## Features  
- Modular FreeRTOS-based architecture (manual + autonomous modes)
- Angle Mode
- Madgwick sensor fusion for stable orientation  
- PID control for roll, pitch, yaw, and throttle  
- Altitude hold via barometer  
- Radio: nRF24L01 (override)
- DMA-ready sensor acquisition (for FFT / EMI work)  
- Watchdog-protected system with failsafe reboot  
- Clean, low-latency motor PWM generation via timers  
- Custom lightweight libraries 

---

## Hardware  
- **STM32F411 Blackpill**  
- **MPU6050** (IMU)  
- **QMC5883P** (Magnetometer)  
- **BMP280** (Barometer)  
- GPS (optional)  
- **nRF24L01+ PA/LNA** modules  
- **Brushless motors + ESCs**  

---

## ⚙️ Getting Started  

> Requirements:  
> PlatformIO / VS Code, ST-Link, basic STM32 toolchain



Human input from the controller is Angle in Degrees. Then the flight code will convert it into radians for computations.
