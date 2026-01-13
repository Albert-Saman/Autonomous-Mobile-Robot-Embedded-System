# Autonomous Mobile Robot (PIC16F877A) 🤖

![Language](https://img.shields.io/badge/Language-Embedded%20C-blue)
![Platform](https://img.shields.io/badge/Platform-PIC16F877A-green)
![Compiler](https://img.shields.io/badge/Compiler-MikroC-orange)

## 📖 Project Overview
This project details the design and implementation of an autonomous mobile robot built for the **Embedded Systems** course at **Princess Sumaya University for Technology**. 

The robot is designed to navigate a complex multi-zone track comprising:
1.  **Line Following Zone:** High-speed tracking.
2.  **Tunnel Zone:** Automatic buzzer activation via LDR.
3.  **Obstacle Zone:** Logic to detect and navigate around physical barriers.
4.  **Parking Zone:** Precision maneuvering and flag raising.

**Key Constraint:** The firmware was written entirely from scratch **without using built-in libraries** (no `Delay_ms`, no Servo library, no Ultrasonic library). Custom drivers were written for Timer Interrupts, PWM, and ADC.

---

## 📺 Video Demo
[[Watch the video](https://www.youtube.com)
*(Click the link above to watch the robot in action)*

---

## 🛠️ Hardware Specifications

### **Microcontroller & Power**
* **MCU:** PIC16F877A (Standalone PCB)
* **Clock:** 8MHz Crystal Oscillator
* **Driver:** L293D H-Bridge
* **Power:** 12V Li-Po (Motors) + 5V Regulated (Logic)

### **Pin Configuration Table**
| Component | Pin | Function |
| :--- | :--- | :--- |
| **LDR Sensor** | `RA0` | Analog Input (Tunnel Detection) |
| **Start Button** | `RB0` | Input (Mission Start) |
| **Status LED** | `RB1` | Output (Active Mode) |
| **Line Sensor (L)**| `RB2` | Digital Input |
| **Line Sensor (R)**| `RB3` | Digital Input |
| **Obstacle IR (L)**| `RB4` | Digital Input (Close Range) |
| **Obstacle IR (R)**| `RB5` | Digital Input (Close Range) |
| **Motor Enable B** | `RC1` | PWM Left Speed Control |
| **Motor Enable A** | `RC2` | PWM Right Speed Control |
| **Buzzer** | `RC6` | Output (Tunnel/Park Alert) |
| **Motor Direction** | `PORTD`, `RD0 --> RD3` | Direction Control (IN1-IN4) |
| **Ultrasonic Trig**| `RE0` | Output (Trigger Pulse) |
| **Ultrasonic Echo**| `RE1` | Input (Timer1 Capture) |
| **Servo Motor** | `RE2` | Output (PWM Flag) |

---

## ⚙️ Software Architecture

The system operates on a **Finite State Machine (FSM)**. 

### **Key Features**
1.  **Custom Ultrasonic Driver:** Uses **Timer1** to measure Echo pulse width with 1µs precision.
2.  **Software PWM:** Uses **Timer0 Interrupts** to generate variable duty cycles for motor speed control on standard digital pins.
3.  **Adaptive Logic:** Distinguishes between a physical wall (Obstacle) and a 30° incline (Bump) by analyzing the change in sensor data.

### **Logic Flowchart**
<img width="2712" height="1826" alt="image" src="https://github.com/user-attachments/assets/c5320106-f605-4e3f-b7d9-4955a4b34d36" />

### **Electrical Connections Design in Proteus Software**
<img width="2648" height="1463" alt="image" src="https://github.com/user-attachments/assets/49b9c059-c1fc-45c3-9d9a-4ac54c2021e6" />

# 📚 References
- Microchip Technology Inc., "PIC16F877A Data Sheet," 2013.
- T. Wilmshurst, Designing Embedded Systems with PIC Microcontrollers, 2nd ed. Newnes, 2010.

## Authors
- Albert Saman
- Omar Khalil
- Ali Bani-Bakkar
