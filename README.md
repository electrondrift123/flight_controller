# STM32 Drone Flight Controller (WIP)  
A real-time, flight controller for STM32F4-based drones, built using FreeRTOS and custom drivers.

**450f Quadrotor**
![alt text](Drone.jpg)

**KiCAD Flight Controller**
![alt text](FC_3d_render.jpg)
![alt text](FC_black_pcb.jpg)
![alt text](FC_real.jpg)

V2.0
![alt text](FC_PCB_v2.jpg)

---

## Features  
- Modular FreeRTOS-based architecture
- ATTi (Attitude) Mode
- Madgwick Sensor Fusion (9DoF) for Attitude Estimation
- Vertical Velocity 3-State Kalman Filter Estimation 
- Vertical Velocity PI Controller  
- Radio: nRF24L01 
- Watchdog-protected system with failsafe reboot  (SENSOR GLITCH)
- Clean, low-latency motor PWM generation via timers  
- Custom lightweight libraries 

---

## Hardware  
- **STM32F411CEU** (MCU)  
- **MPU6050** (IMU)  
- **QMC5883P** (Magnetometer: Optional)  
- **BMP280** (Barometer)  
- **nRF24L01+ PA/LNA** modules  
- **930KV Brushless motors + 30A ESCs**  
- **1.2A 3.3V DC-DC Buck** 
---

## ⚙️ Getting Started  

> Requirements:  
> PlatformIO / VS Code, ST-Link, basic STM32 toolchain

Human input from the controller is Angle in Degrees. Then the flight code will convert it into radians for computations.

Adaptive P-PID per axis: 
P outer loop: angle error (100 Hz)
PID inner loop: angular rates error (500Hz)
1:5 ratio

# Control Input
- Roll and Pitch inputs are angle [-20,20] deg for safety.
- Yaw will be angular rate max cmd: [-360,360] deg/sec.
- Vertical velocity: [-0.8,1.0] m/s
- Internal calculation uses radians.

# Control Scheme
- Cascaded adaptive P-PID architecture.
- Outer loop, P-controller, angle position, running on 50 Hz,
  and output (desired angular rate) is clamped by [-pi,pi].
- Inner loop, PID-controller, angular rate, running on 250 Hz,
  and output is directly clamped to pwm ticks for the mixer
  by their authority: [-150,150] for roll & pitch, [-100,100] for yaw.
- Throttle is clamped: [0,750]. 

# Communication data transforms
- max of 32 kB transmission
- 2 way communication 
  
# ESC calibration:[1000,2000] us pwm ticks

# LPFs
- PT1 & PT2 using EMA or Cascaded EMA

# Controller Block Diagram (Attitude):
**Roll & Pitch Cascaded P-PID Controller**
![alt text](Roll_Pitch_PID.jpg)

**Yaw PI Controller**
![alt text](Yaw_PI.jpg)

# Timing
- read sensor:              1 kHz
- Attiude PID & Madgwick:   500 Hz
- Vertical Velocity PI:     100 Hz
- Radio:                    interrupt driven
- WDT:                      1 Hz
