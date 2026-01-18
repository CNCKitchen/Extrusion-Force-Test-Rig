# Extrusion Force Test Rig (3D Printing) – Firmware + GUI

A test bench for investigating **extrusion behavior** in FDM 3D printing:  
We measure **temperature**, **feed rate**, **extrusion force** (via load cell), and **filament slip** (via encoder) and visualize the measurement live in a **Python GUI**.

> Status: Prototype / Lab Setup  
> Goal: reproducible measurement series for material and parameter comparisons

---

## Contents / Features

- **Live Measurement** & Plotting (GUI)
- Parameterization (target temperature, feed rate, filament length)
- Logging/Export of measurement data (CSV)
- Modular firmware architecture (Hotend / LoadCell / Stepper / Encoder / Communication)

---

## Repository Structure

This repo is organized as a PlatformIO project + Python GUI:

- `src/` – Firmware (Entry/Tasks/Main)
- `include/` – Headers / Interfaces (Classes for HotEnd, LoadCell, ExtruderStepper, Encoder, GUI Communication)
- `lib/` – External/additional libraries 
- `Python/` – PC software (GUI, Plotting, Logging, Tools/Notebooks)
- `Manual/` – Documentation/Instructions/Build Info
- `GUI/` - .exe for GUI
- `platformio.ini` – PlatformIO build configuration

---

## Hardware 

- Microcontroller: **ESP32-S3 DevKitC-1** 
- Extruder drive: Stepper + driver (**TMC2209**)
- Hotend/Heater: Heater cartridge + fan + temperature measurement (**104NT thermistor**)
- Force measurement: Load cell + **HX711**
- Filament path/slip: Rotary encoder on filament path
- Power electronics: MOSFET driver + MOSFET (for heater and fan)
- Power supply suitable for heater/stepper (note power consumption --> test bench approx. 62W)

---

## Software Requirements

### Firmware (PlatformIO)
- VS Code + **PlatformIO** Extension
- HX711 Library by bodge
- AccelStepper Library by Mike McCauley

### PC GUI (Python)
- Python **3.10+** recommended
- Packages: `pyserial`, `matplotlib`, `numpy`, `tkinter`

---

## Quickstart

### 1) Flash firmware (PlatformIO)

### 2) Connect ESP32 via USB cable

### 3) Run GUI .exe file

## Graphical User Interface

### GUI Layout
- 6 frames: 3 left, 3 right
- 3 input fields for parameters, 1 input field for CSV filename and 1 checkbox
- 2 buttons: tare and start test
- optional: Monitor for debugging: all serial prints from the ESP are displayed

### Python Code
- baud rate identical to ESP's
- if using a different board, vendor_id and product_id may need to be changed
- Port is automatically detected if PC is connected to ESP
- Filament diameter is adjustable (necessary for calculating flow rate)

### exe File
- if Python code is changed, a new exe file must be created: 
   1. install pyinstaller (only necessary once)
   2. open PowerShell in the folder containing the Python code
   3. execute the following command:
      pyinstaller --onefile -w 'filename.py'
   4. exe file will be created in a folder named dist
