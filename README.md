# PIX Control Framework (PCF)

A complete modular autonomous vehicle control framework that is independent of any specific vehicle manufacturer.

## Overview
PIX Control Framework provides a clean, scalable, modular architecture where autonomous driving algorithms remain completely independent from hardware-specific implementations. The framework supports multiple drive-by-wire (DBW) vehicles using a common API and standardized interfaces.

## Architecture

1. **Algorithm API**: High-level interface for autonomous algorithms.
2. **Command Manager**: Prioritizes and manages commands from algorithms.
3. **State Manager**: Manages the state machine (Init, Autonomous, Manual, Fault).
4. **Safety Manager**: Arbitrates commands based on limits and emergency states (AEB).
5. **DBW Manager**: Maps standardized DBW commands to vehicle interfaces.
6. **Vehicle Interface**: Abstract hardware interface for specific platforms.
7. **CAN Codec**: Encodes/decodes CAN frames.
8. **CAN Driver**: Manages SocketCAN communication.
9. **Vehicle Hardware**: Physical DBW system.

## Build Instructions
```bash
mkdir -p pcf_ws/src
cd pcf_ws/src
git clone https://github.com/manish-gupta-in/pix_control_framework.git
cd ..
colcon build --symlink-install
```
