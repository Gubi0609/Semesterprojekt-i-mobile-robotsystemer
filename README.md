# Semesterprojekt-i-mobile-robotsystemer

**Sound-based Communication Protocol for Mobile Robot Systems**

This project implements a communication protocol using sound as the transmission medium. A TurtleBot with Raspberry Pi 3 running ROS receives commands transmitted as audio signals.

We've designed a layered protocol focused on short commands and robust error handling to ensure stable and correct data transfer.

---

## 🎯 Features

- **CRC-encoded chord transmission** - Reliable 16-bit commands using 4-tone chords
- **Voice control** - Natural speech recognition using Vosk STT
- **Error detection** - CRC checksums with automatic verification
- **Duplicate prevention** - Lockout mechanism to prevent repeated commands
- **Auto-restart** - Automatic microphone recovery on failure
- **Frequency tolerance** - Adaptive FFT detection for robust communication

---

## 📁 Repository Structure

```
.
├── SRC/                    # Desktop/laptop source code
│   ├── main.cpp           # Main sender program
│   ├── protocol_sender.cpp
│   ├── command_protocol.cpp
│   ├── CRC.cpp
│   └── UI.cpp
│
├── src/                    # Raspberry Pi source code
│   ├── rb3_node_cpp.cpp   # ROS node for TurtleBot
│   ├── protocol_receiver.cpp
│   └── velocityProvider.cpp
│
├── LIB/                    # Reusable audio communication libraries
│   ├── audio_comm.cpp/h
│   ├── audio_transmitter.h/cpp
│   ├── audio_receiver.h/cpp
│   ├── frequency_detector.h/cpp
│   ├── tone_generator.h/cpp
│   └── examples/          # Library usage examples
│
├── INCLUDE/                # Header files
│   ├── command_protocol.h
│   ├── CRC.h
│   ├── UI.h
│   └── velocityProvider.hpp
│
├── DATABASE/               # Database and logging
│   ├── Database.cpp/h
│   └── Logger.cpp/h
│
├── TESTS/                  # Test and diagnostic programs
│   ├── frequency_test.cpp
│   ├── chord_diagnostic_test.cpp
│   ├── protocol_test.cpp
│   └── (more test programs...)
│
├── TEST_RESULTS/           # Test data and results (committed to git)
│   └── frequency_test_results_*.csv
│
├── DOCS/                   # Documentation
│   ├── INDEX.md           # Documentation index
│   ├── guides/            # User guides
│   ├── api/               # API documentation
│   └── features/          # Feature documentation
│
├── MATLAB/                 # MATLAB analysis scripts
│   ├── FFT.mlx
│   └── Checksum.mlx
│
├── BUILD/                  # Build output (not in git)
└── ASSETS/                 # Project assets

```

### Directory Purpose

- **`SRC/`** - Programs for desktop/laptop (transmitter, testing)
- **`src/`** - Programs for Raspberry Pi (receiver, ROS node)
- **`LIB/`** - Shared libraries for audio communication
- **`INCLUDE/`** - Header files
- **`TESTS/`** - All test and diagnostic programs
- **`TEST_RESULTS/`** - Test data (committed for team access)
- **`DOCS/`** - Complete documentation

---

## 🚀 Quick Start

### Prerequisites

Install dependencies (Ubuntu/Debian):
```bash
sudo apt-get update
sudo apt-get install build-essential cmake libportaudio2 portaudio19-dev libfftw3-dev libsqlite3-dev
```

### Building the Sender (Desktop/Laptop)

**Option 1: Using build script (recommended)**
```bash
./build_desktop.sh
cd BUILD && ./protocol_sender
```

**Option 2: Using CMake**
```bash
mkdir -p build && cd build
cmake .. -DBUILD_DESKTOP=ON -DBUILD_PI=OFF
make -j$(nproc)
cd ../BUILD && ./protocol_sender
```

**Option 3: Using Makefile**
```bash
cd SRC
make protocol_sender
cd ../BUILD && ./protocol_sender
```

### Building the Receiver (Raspberry Pi)

**Option 1: Using build script (recommended)**
```bash
./build_pi.sh
cd BUILD && ./protocol_receiver
```

**Option 2: Using CMake**
```bash
mkdir -p build && cd build
cmake .. -DBUILD_DESKTOP=OFF -DBUILD_PI=ON
make -j$(nproc)
cd ../BUILD && ./protocol_receiver
```

See [BUILD_GUIDE.md](BUILD_GUIDE.md) for complete build instructions.

---

## 📚 Documentation

Complete documentation is available in the `DOCS/` directory:

- **[DOCS/INDEX.md](DOCS/INDEX.md)** - Documentation index and overview
- **[DOCS/guides/CRC_USAGE_GUIDE.md](DOCS/guides/CRC_USAGE_GUIDE.md)** - Quick start guide
- **[DOCS/guides/VOICE_RECEIVER_GUIDE.md](DOCS/guides/VOICE_RECEIVER_GUIDE.md)** - Voice control setup
- **[DOCS/api/README_LIBRARY.md](DOCS/api/README_LIBRARY.md)** - Complete API reference

See `DOCS/INDEX.md` for the full documentation structure.

---

## 🧪 Testing

Test programs are located in `TESTS/` directory. Examples:

```bash
# Frequency detection test
cd TESTS
g++ -std=c++17 -I../LIB frequency_test.cpp [...] -o frequency_test

# Chord diagnostic test
g++ -std=c++17 -I../LIB chord_diagnostic_test.cpp [...] -o chord_diagnostic_test
```

See `TESTS/README.md` for more details on available tests.

Test results are stored in `TEST_RESULTS/` and committed to git for team access.

---

## 🔧 Dependencies

- **PortAudio** - Audio I/O
- **FFTW3** - Fast Fourier Transform
- **SQLite3** - Database (for sender)
- **ROS** - Robot Operating System (for rb3_node_cpp)
- **Vosk** - Speech recognition (optional, for voice control)

### Installation (Ubuntu/Debian)
```bash
sudo apt-get install libportaudio2 portaudio19-dev libfftw3-dev libsqlite3-dev
```

---

## 👥 Developers

- [Asmus Rise](https://github.com/AsmusRise)
- [August Tranberg](https://github.com/Gubi0609)
- [Elias Alstrup](https://github.com/Sputnikboi)
- [Emil Gramstrup](https://github.com/EmilGrams)
- [Frederik Wilkens](https://github.com/FrederikWilkens)

---

## 📝 License

See [LICENSE](LICENSE) file for details.

---

## 🗺️ Project Status

This is an active semester project. Some test programs and documentation may be outdated.
Review and update as needed when making changes.
