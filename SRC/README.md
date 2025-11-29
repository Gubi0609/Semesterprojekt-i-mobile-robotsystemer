# SRC/ - Desktop/Laptop Source Code

This directory contains source code designed to run on **desktop/laptop computers** for development and testing.

## Main Programs

### main.cpp
CRC-encoded chord sender. Main transmission program with UI and database logging.

### protocol_sender.cpp
Protocol-aware sender for transmitting commands via audio.

### command_protocol.cpp
Protocol implementation for command encoding/decoding.
- Header: `../INCLUDE/command_protocol.h`

### CRC.cpp
CRC (Cyclic Redundancy Check) implementation for error detection.
- Header: `../INCLUDE/CRC.h`

### UI.cpp
User interface module.
- Header: `../INCLUDE/UI.h`

## Build System

### Using Makefile
```bash
cd SRC
make sender              # Build sender program
make protocol_sender     # Build protocol sender
make receiver            # Build receiver (uses ../src/receiver.cpp)
make protocol_receiver   # Build protocol receiver (uses ../src/protocol_receiver.cpp)
make all                 # Build main programs
```

### Dependencies
- PortAudio library: `sudo apt-get install libportaudio2 portaudio19-dev`
- FFTW3 library: `sudo apt-get install libfftw3-dev`
- SQLite3: `sudo apt-get install libsqlite3-dev`
- Or use: `./install_deps.sh`

### Manual Compilation Example
```bash
# Protocol sender
g++ -std=c++17 -I../LIB -I../INCLUDE protocol_sender.cpp \
	../LIB/audio_transmitter_lib.cpp ../LIB/audio_comm.cpp \
	../LIB/tone_generator_lib.cpp CRC.cpp command_protocol.cpp \
	-o ../BUILD/protocol_sender -lportaudio -lm -lpthread
```

## Files

### Source Files
- `main.cpp` - Main sender with database logging
- `protocol_sender.cpp` - Protocol-aware sender
- `command_protocol.cpp` - Protocol implementation
- `CRC.cpp` - Error detection
- `UI.cpp` - User interface
- `install_deps.sh` - Dependency installation script

### Build Configuration
- `Makefile` - Build system (not in git, local only)

## Notes
- This directory is separate from `src/` (lowercase) which contains Raspberry Pi programs
- Build output goes to `BUILD/` directory
- Test programs have been moved to `TESTS/` directory
- Documentation moved to `DOCS/` directory
