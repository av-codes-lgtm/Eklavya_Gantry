# Gantry CNC Motion Control

This repository contains firmware and test programs for a **3-axis gantry-style CNC machine**, supporting multiple stepper motor drivers (DRV8825 and TMC2208).  
The codebase is organized into modules for motion control, homing, positioning, and testing.

---

## 📂 Repository Structure

### Root Folders
- **.vscode/**  
  VSCode project settings.

- **DRV8825/**  
  Codebase for the gantry using the **DRV8825 stepper driver**.

- **TMC2208/**  
  Codebase for the gantry using the **TMC2208 stepper driver**.

- **Screwdriver/Screw/**  
  Additional implementation for screw-driven actuators.
---
### Inside `DRV8825/`

Each driver folder contains multiple subfolders for different functionalities:

- **Double_axes/**  
  Control programs for two axes working together.

- **SRA/**  
  Plots “SRA” on the plane using the gantry

- **circle/**  
  Code for drawing/controlling circular motion.

- **final/**  
  Final stable build for **3-axis gantry motion control**.

- **go_to_pos/**  
  Precise linear positioning of one axis.

- **homing/**  
  Executes homing in one axis.

- **limit/**  
  Test programs for Limit switch functionality.

- **step_func/**  
  Test programs for verifying Acceleration and Deceleration. 

- **stepper_test/**  
  Test programs for verifying stepper motor functionality.

- **total_positioning/**  
  Precise 2D positioning across the plane.

- **totalhoming/**  
  Executes Homing in all axes of the gantry simultaneously

---
### Inside `TMC2208/`

This folder contains code specific to the **TMC2208 stepper driver**:

- **Go_to_pos/**
  Moves the gantry to a specified position.
    
- **step_func/**
  Basic testing for stepper stepping functions.
   
- **uart/**
  UART codes for TMC2208.  

