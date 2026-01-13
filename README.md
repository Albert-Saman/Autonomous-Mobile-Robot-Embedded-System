# Autonomous Mobile Robot (PIC16F877A) 🤖

![Language](https://img.shields.io/badge/Language-Embedded%20C-blue?style=flat-square)
![Platform](https://img.shields.io/badge/Platform-PIC16F877A-green?style=flat-square)
![Compiler](https://img.shields.io/badge/Compiler-MikroC%20PRO-orange?style=flat-square)
![Electrical Design Simulation](https://img.shields.io/badge/Simulation-Proteus%209-purple?style=flat-square)
![Status](https://img.shields.io/badge/Status-Completed-success?style=flat-square)

> **Course:** Embedded Systems (22442) | **Institution:** Princess Sumaya University for Technology

---

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
[Watch the video](https://www.youtube.com)
*(Click the link above to watch the robot in action)*

---

## 🛠️ Hardware Specifications

### **Microcontroller & Power**
* **MCU:** Microchip PIC16F877A (40 pins)
* **Clock:** 8MHz Crystal Oscillator
* **Actuators:**
    * 2x DC Gear Motors
    * 1x SG90 Micro Servo
    * 1x Active Piezo Buzzer
    * 1x Yellow LED
* **Drivers:** L293D Dual H-Bridge Motor Driver
* **Sensors:** HC-SR04 (Ultrasonic), LDR (Light), TCRT5000 (IR Line/Obstacle)
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
## 💻 Software Implementation

### The "No-Library" Challenge
To demonstrate low-level understanding, standard libraries were replaced with custom drivers:

1.  **Ultrasonic Driver (Timer1):**
    * Instead of `PulseIn()`, we configured **Timer1** with a 1:2 prescaler.
    * The code gates the timer while the Echo pin is HIGH.
    * **Formula:** `Distance (cm) = Timer_Ticks / 58` (Derived for 8MHz Clock).

2.  **Software PWM (Timer0):**
    * **Timer0** generates an interrupt every 1 ms.
    * A global PWM counter (0-99) increments on every interrupt.
    * Motor pins are toggled based on a comparison between the counter and the target speeds `speed_req_left` and `speed_req_right`, allowing for smooth motion.

### Finite State Machine
The main loop executes a non-blocking switch-case state machine:

### **Key Features**
1.  **Custom Ultrasonic Driver:** Uses **Timer1** to measure Echo pulse width with 1µs precision.
2.  **Software PWM:** Uses **Timer0 Interrupts** to generate variable duty cycles for motor speed control on standard digital pins.
3.  **Adaptive Logic:** Distinguishes between a physical wall (Obstacle) and a 30° incline (Bump) by analyzing the change in sensor data.

## 🏁 Getting Started
Prerequisites
- MikroC PRO for PIC (Compiler)
- EasyPIC development kit board
- Proteus 9 Professional (Simulation - Optional)

### Installation
#### Clone the Repo:

`gh repo clone Albert-Saman/Autonomous-Mobile-Robot-Embedded-System`

#### Open Project:

Launch MikroC and open Code (.mcppi)

#### Build:

Press Build All to compile and upload the hex file to the flash program memory of the PIC. Ensure the clock is set to 8.000000 MHz.

## 📸 Gallery

### **Logic Flowchart**
<p align="center">
<img width="2712" height="1826" alt="image" src="https://github.com/user-attachments/assets/c5320106-f605-4e3f-b7d9-4955a4b34d36" />
</p>

### **Electrical Connections Design in Proteus Software**
<p align="center">
<img width="2648" height="1463" alt="image" src="https://github.com/user-attachments/assets/49b9c059-c1fc-45c3-9d9a-4ac54c2021e6" />
</p>

### **Photos of the Robot**
<p align="center">
<img width="180" height="320" alt="image" src="https://github.com/user-attachments/assets/ee53add0-0ce7-469f-8da5-e001719eb2a7" />
<img width="240" height="320" alt="image" src="https://github.com/user-attachments/assets/143f67c7-f085-40cc-987b-1b293877ed97" />
<img width="240" height="320" alt="image" src="https://github.com/user-attachments/assets/ad2e24a2-42d9-465b-8456-c2428f97df63" />
</p>

<p align="center">
<img width="320" height="240" alt="image" src="https://github.com/user-attachments/assets/d6eec96f-8e2b-4e5a-9ca2-81e94afc8c66" />
<img width="267" height="240" alt="image" src="https://github.com/user-attachments/assets/1fb86510-b50a-4103-8d17-4979c904c440" />
</p>

# 📚 References
- Microchip Technology Inc., "PIC16F877A Data Sheet," 2013.
- T. Wilmshurst, Designing Embedded Systems with PIC Microcontrollers, 2nd ed. Newnes, 2010.

## Authors
- Albert Saman
- Omar Khalil
- Ali Bani-Bakkar
