# DRV8825 Stepper Motor Driver with ESP32 and Limit Switch

This project demonstrates how to control a stepper motor using an **ESP32**, a **DRV8825 motor driver**, and a **limit switch** for homing or safety purposes. 
![DRV8825 Motor Driver](DRV8825.jpeg)
---

## Circuit Description

The **ESP32** sends step and direction signals to the **DRV8825**, which drives a bipolar stepper motor.  
A **limit switch** is added to detect the end position or provide a safe stop.

---

## Connections

### DRV8825 → Stepper Motor
- **A1, A2, B1, B2** → Connect to stepper motor coils  
  - A coil: A1, A2  
  - B coil: B1, B2  

### DRV8825 → ESP32
- **STEP** → GPIO 18 (pulse to step motor)  
- **DIR** → GPIO 19 (sets rotation direction)  
- **EN (Enable)** → GPIO 21 (active LOW, optional)  
- **GND** → ESP32 GND  
- **VMOT** → External motor supply (8.2–45V depending on motor) but for this Project 12V was used
- **GND (Motor)** → Power supply ground  

**Important:** Place a 100µF electrolytic capacitor across VMOT and GND close to the DRV8825 to protect against voltage spikes.  

### Limit Switch → ESP32
- One terminal → GPIO 23  
- Other terminal → GND  
- Use **internal pull-up resistor** in ESP32 code.  

---

## Circuit Diagram

```plaintext
              +-------------------+
   VMOT 8-45V |                   | Stepper Motor
   + ---------| VMOT        A1 ---|---- Coil A
   - ---------| GND         A2 ---|---- Coil A
              |                   |
ESP32 GPIO 18 | STEP        B1 ---|---- Coil B
ESP32 GPIO 19 | DIR         B2 ---|---- Coil B
ESP32 GPIO 21 | EN                |
ESP32   GND - | GND               |
              +-------------------+

Limit Switch:
  GPIO 23 ---/ ---- GND
![PCB Schematic](schematic.png) 
