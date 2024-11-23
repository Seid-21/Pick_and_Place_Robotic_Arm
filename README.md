# Pick_and_Place_Robotic_Arm
# Robotic Arm Control Mechanism

This project involves the design and control of a robotic arm capable of executing tasks such as material handling, light assembly, and controlled movement via an Android application. The robotic arm operates with four degrees of freedom (DoF) and uses servo motors for precise movement.

---
[![BreadcrumbsPick_and_Place_Robotic_Arm
](https://img.youtube.com/vi/HjswL8kaAqY/edit)](https://www.youtube.com/watch?v=HjswL8kaAqY/edit)
---
---

## Table of Contents
1. [Introduction](#introduction)
2. [Objectives](#objectives)
3. [Methodology](#methodology)
4. [System Components](#system-components)
5. [Features](#features)
6. [Implementation Details](#implementation-details)
7. [Results](#results)
8. [References](#references)

---

## Introduction

The robotic arm is a programmable electro-mechanical manipulator designed for industrial and research purposes. It integrates **Arduino Uno**, **servo motors**, and **Bluetooth HC-05 module** for user-friendly control through an Android application.

The robotic arm is designed to:
- Perform pick-and-place operations.
- Simplify tasks in hazardous environments.
- Enhance precision and efficiency in industrial operations.

---

## Objectives

### General Objective:
To design and control a robotic arm using an **HC-05 Bluetooth module** connected to an Android device.

### Specific Objectives:
- Develop the mechanical structure of the robotic arm.
- Implement control using Arduino Uno.
- Design and program the app interface for real-time control.

---

## Methodology

1. **Research and Planning**:
   - Literature review on existing robotic arms.
   - Analysis of industrial applications.

2. **Design and Development**:
   - Mechanical and electrical design using Arduino Uno and servo motors.
   - Integration of a Bluetooth communication module.

3. **Prototyping**:
   - Build a prototype using acrylic materials.
   - Program servo motors for precise motion.

4. **Testing and Validation**:
   - Real-time testing with Android app.
   - Validation of load-bearing capabilities and accuracy.

---

## System Components

1. **Arduino Uno**:
   - ATmega328P microcontroller.
   - Supports 32KB memory, 16MHz clock speed.

2. **Servo Motors (G9)**:
   - Weight: 9g
   - Torque: 1.8 kgf·cm
   - Operating voltage: 4.8V

3. **Bluetooth Module (HC-05)**:
   - Bluetooth v2.0 + EDR.
   - UART interface for communication.

4. **End Effector**:
   - Electric gripper controlled via a servo motor.

5. **Android Application**:
   - Created using MIT App Inventor for wireless control.

---

## Features

- **4 Degrees of Freedom**: Enables multi-directional movement.
- **Android Control**: Real-time operation using a mobile application.
- **Compact and Lightweight Design**: Built with durable acrylic material.
- **Bluetooth Connectivity**: Wireless control within a 10m range.

---

## Implementation Details

1. **Mechanical Design**:
   - Arm joints allow precise rotational motion.
   - Free body diagrams and torque calculations ensure load efficiency.

2. **Control System**:
   - Servo motors controlled via Arduino Uno.
   - Commands sent via Bluetooth to the microcontroller.

3. **Programming**:
   - Arduino code written to interpret Bluetooth signals.
   - MIT App Inventor used to design the Android interface.

4. **System Model**:
   - Armature voltage controls angular position.
   - Servo feedback loop ensures accuracy.

---

## Results

- Successfully designed and implemented a robotic arm with 4 DoF.
- Achieved precise control through Bluetooth and Android application.
- Demonstrated feasibility for industrial applications like material handling and light assembly tasks.

---

## References

1. [Arduino Official Website](https://www.arduino.cc/)
2. M. Gemeinder and M. Gerke, "GA-based Path Planning for Mobile Robot Systems."
3. "Development of a Microcontroller Based Robotic Arm," Computer Science and IT Education Conference.
4. Robert J. Shilling, *Fundamentals of Robotics Analysis and Control*.

---

## Appendix

### Sample Arduino Code
```cpp
#include <SoftwareSerial.h>
#include <Servo.h>

Servo myservo1, myservo2, myservo3, myservo4;
int bluetoothTx = 10;
int bluetoothRx = 11;

SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);

void setup() {
    myservo1.attach(3);
    myservo2.attach(5);
    myservo3.attach(6);
    myservo4.attach(9);
    Serial.begin(9600);
    bluetooth.begin(9600);
}

void loop() {
    if (bluetooth.available() >= 2) {
        unsigned int servopos = bluetooth.read();
        unsigned int servopos1 = bluetooth.read();
        unsigned int realservo = (servopos1 * 256) + servopos;

        if (realservo >= 1000 && realservo < 1180) {
            int servo1 = map(realservo, 1000, 1180, 0, 180);
            myservo1.write(servo1);
        }
    }
}
