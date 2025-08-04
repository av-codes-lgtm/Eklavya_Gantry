# **ScrewTron: Automated Gantry-Based Screwing System**

## Problem Statement

**ScrewTron** is an automated gantry-based system designed to **screw and unscrew \~280 miniature screws (1 mm size)** on a delicate LED panel with precision, speed, and safety.

The LED panel consists of two layers:

*  **Top Layer:** Contains **200 screws**, which must be **removed first**.
*  **Bottom Layer:** Contains **80 screws**, accessible only after the top layer is unscrewed.

The **installation process follows the reverse**: bottom layer first, then top.

Given the **proximity to sensitive circuitry**, the system must:

* Avoid excessive pressure or misalignment
* Ensure screw orientation is accurate
* Maintain precise force control
* Operate with sub-millimeter repeatability
* Complete each screwing/unscrewing task in **under 5 seconds**

---

## Project Description

ScrewTron consists of a **high-precision 3-axis gantry** (X, Y, Z) integrated with:

* A specialized **toolhead** for screwing and unscrewing
* An automated **screw feeding mechanism** ensuring correct orientation and uninterrupted operation

The system is engineered for:

* **Precision-controlled movements**
* **High-speed operation**
* **Safety around fragile components**
* **Reliable task repetition with minimal error margin**

---

## Hardware Overview

* **Microcontroller:** [ESP32](https://www.espressif.com/en/products/socs/esp32)
* **Motor Drivers:** [DRV8825 Stepper Drivers](https://www.pololu.com/product/2133)
* **Actuation:** Stepper motors for all three axes

---

## Repository Contents

* `firmware/`: ESP-IDF based firmware for ESP32
* `PCB/`: KiCad schematics and board layouts for controller and driver circuitry
* `mechanics/`: CAD files and design documents (if applicable)
* `docs/`: Documentation related to system design and operation

---

## Goals

* Complete each screwing/unscrewing task in < **5 seconds**
* Maintain **sub-millimeter positioning accuracy**
* Prevent **any damage** to the LED panel or circuit
* Enable **repeatable and automated** operation with minimal human intervention
