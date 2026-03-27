# Automated Acid Sampling System (Capstone Project)

## Overview
This project is a bench-scale prototype of an automated acid sampling system designed to improve safety and consistency in industrial environments such as fertilizer plants. 

The system replaces manual sampling by automating the process of lowering a sampling mechanism into a reactor, collecting a sample, and returning it for analysis.

---
## Industry Collaboration

This project was developed in collaboration with engineers from Mosaic, a leading fertilizer production company.

The problem and system requirements were inspired by real-world challenges in industrial acid sampling, with a focus on improving operator safety and process consistency.

All designs and implementations in this repository are for educational and prototype purposes only and do not represent production systems.
---

## Features
- Automated extend → dwell → retract sampling cycle
- Real-time system monitoring through an HMI interface
- Fault detection with color-coded feedback
- Event logging with timestamps for debugging and traceability
- Pneumatic actuation for controlled movement
- Sensor integration (limit switches, encoder feedback)

---

## System Architecture
The system consists of:

- **Microcontroller:** Arduino Uno Rev3  
- **HMI Display:** Nextion touchscreen interface  
- **Actuation:** Pneumatic system (air input + relief valves)  
- **Sensors:**
  - Limit switches (end-of-travel detection)
  - Encoder (depth tracking)
- **Optional Modules:**
  - RTC module (DS3231) for timestamps

---

## HMI Interface
The HMI was designed to reflect real-time system behavior and improve operator interaction.

### Pages:
- **Main Page:** Displays system state and controls (Start/Stop/Reset)
- **Logs Page:** Shows timestamped system events
- **Diagnostics Page:** Displays live sensor and actuator states

Faults are highlighted visually, and logs automatically update during operation.

---

## Control Logic
The system operates as a state machine:

1. **Idle**
2. **Extend** – Pneumatic system lowers the sampler
3. **Dwell** – Sample is collected at depth
4. **Retract** – System returns to initial position
5. **Fault Handling** – Triggered by sensor or system errors

Safety interlocks ensure proper sequencing and prevent unsafe operation.

---

## How to Run

### Requirements
- Arduino IDE
- Nextion Editor (for HMI)
- Required libraries:
  - SoftwareSerial (for HMI communication)
  - RTClib (if using RTC module)

### Steps
1. Upload the Arduino code to the Arduino Uno
2. Upload the HMI design to the Nextion display
3. Connect hardware components (sensors, valves, display)
4. Power the system
5. Use the HMI to start and monitor the system

---

## Testing Without Hardware
- Serial print statements can be used to verify logic flow
- Simulated inputs can be manually triggered in code
- TinkerCAD can be used for basic circuit testing (limited for full system)

---

## Screenshots

### Main Interface
![Main UI]("C:\Users\Joseph\Documents\MainUI.PNG")

### Logs Page
![Logs UI]("C:\Users\Joseph\Documents\LogsUI.PNG")

### Diagnostics Page
![Diagnostics UI]("C:\Users\Joseph\Documents\DiagnosticsUI.PNG")

---

## Future Improvements
- PLC-based implementation for industrial deployment
- Remote operation capability
- Improved enclosure for harsh environments (IP-rated)
- Enhanced sensor reliability

---

## Contributors
- Joseph Williams 
-  

---

## License
This project is for academic purposes.
