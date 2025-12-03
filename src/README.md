# src/ - Raspberry Pi Source Code

This directory contains source code designed to run on the **Raspberry Pi** (TurtleBot).

## Main Programs

### rb3_node_cpp.cpp
ROS node for the TurtleBot. Main integration point for the robot control system.

### protocol_receiver.cpp
Protocol-aware CRC-decoding chord receiver for robot control. Continuously listens for commands, verifies CRC, and interprets protocol commands.

### velocityProvider.cpp
Velocity management module for robot motion control.
- Header: `../INCLUDE/velocityProvider.hpp`

## Building for Raspberry Pi

These programs are designed to be compiled on the Raspberry Pi itself or cross-compiled for ARM architecture.

### Requirements
- ROS (for rb3_node_cpp.cpp)
- PortAudio library
- FFTW3 library
- Standard C++17 compiler

### Example Build Commands
```bash
# Protocol receiver
g++ -std=c++17 -I../LIB -I../INCLUDE protocol_receiver.cpp \
	../LIB/audio_receiver_lib.cpp ../LIB/audio_comm.cpp \
	../LIB/frequency_detector_lib.cpp ../SRC/CRC.cpp ../SRC/command_protocol.cpp \
	-o protocol_receiver -lportaudio -lfftw3 -lm -lpthread

# Velocity provider (as part of larger program)
g++ -std=c++17 -I../INCLUDE velocityProvider.cpp -c -o velocityProvider.o
```

## Notes
- This directory is separate from `SRC/` (uppercase) which contains desktop/laptop programs
- Programs here are optimized for embedded ARM systems
- Test programs have been moved to `TESTS/` directory
