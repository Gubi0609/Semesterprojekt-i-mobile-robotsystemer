# Build Guide

This project supports multiple build systems and configurations for different targets.

---

## 🚀 Quick Start

### Using Build Scripts (Recommended)

```bash
# Desktop/laptop programs
./build_desktop.sh

# Raspberry Pi programs
./build_pi.sh

# Test programs
./build_tests.sh
```

### Using Makefile (Alternative - SRC/ only)

```bash
cd SRC
make protocol_sender   # Build specific target
make all              # Build main programs
make help             # Show all targets
```

---

## 📦 Build Systems

This project has **TWO** build systems:

### 1. CMake (Modern, Recommended)
- Located: `CMakeLists.txt` (root)
- Supports: Desktop, Pi, Tests, Examples
- Independent builds for each target
- Output: `BUILD/` directory

### 2. Makefile (Legacy, SRC/ only)
- Located: `SRC/Makefile`
- Supports: Desktop programs and some tests
- More manual control
- Output: `BUILD/` directory

**Recommendation:** Use CMake for new development. Makefile is kept for compatibility.

---

## 🖥️ Building for Desktop/Laptop

### Option 1: Using Build Script
```bash
./build_desktop.sh
```

### Option 2: Using CMake Manually
```bash
mkdir -p build && cd build
cmake .. -DBUILD_DESKTOP=ON -DBUILD_PI=OFF
make -j$(nproc)
cd ..
```

### Option 3: Using Makefile
```bash
cd SRC
make sender           # Main sender with UI
make protocol_sender  # Protocol sender
```

### Option 4: Manual Compilation (One-liner)
```bash
# Protocol sender
g++ -std=c++17 -Wall -Wextra -O2 -I../LIB -I../INCLUDE \
	SRC/protocol_sender.cpp SRC/command_protocol.cpp SRC/CRC.cpp \
	LIB/audio_transmitter_lib.cpp LIB/audio_comm.cpp LIB/tone_generator_lib.cpp \
	-o BUILD/protocol_sender -lportaudio -lm -lpthread
```

### Built Programs:
- `BUILD/sender` - Main sender with database logging
- `BUILD/protocol_sender` - Protocol-aware sender

---

## 🤖 Building for Raspberry Pi

### Option 1: Using Build Script
```bash
./build_pi.sh
```

### Option 2: Using CMake Manually
```bash
mkdir -p build && cd build
cmake .. -DBUILD_DESKTOP=OFF -DBUILD_PI=ON
make -j$(nproc)
cd ..
```

### Option 3: Manual Compilation (One-liner)
```bash
# Protocol receiver
g++ -std=c++17 -Wall -Wextra -O2 -I../LIB -I../INCLUDE \
	src/protocol_receiver.cpp SRC/command_protocol.cpp SRC/CRC.cpp \
	LIB/audio_receiver_lib.cpp LIB/audio_comm.cpp LIB/frequency_detector_lib.cpp \
	-o BUILD/protocol_receiver -lportaudio -lfftw3 -lm -lpthread
```

### Built Programs:
- `BUILD/protocol_receiver` - Protocol-aware receiver
- `BUILD/rb3_node_cpp` - ROS node (if ROS available)

---

## 🧪 Building Tests

### Option 1: Using Build Script
```bash
./build_tests.sh
```

### Option 2: Using CMake Manually
```bash
mkdir -p build && cd build
cmake .. -DBUILD_TESTS=ON -DBUILD_DESKTOP=OFF -DBUILD_PI=OFF
make -j$(nproc)
cd ..
```

### Option 3: Using Makefile (Some tests)
```bash
cd SRC
make frequency_response_test
make detection_count_test
make chord_accuracy_test
make protocol_test
```

### Built Test Programs:
- `BUILD/frequency_test` - Tests all 64 frequencies
- `BUILD/frequency_test_single` - Single-tone detection test
- `BUILD/frequency_transmitter_test` - Transmitter test
- `BUILD/chord_diagnostic_test` - Chord detection diagnostics
- `BUILD/chord_transmitter_test` - Chord transmitter test
- `BUILD/wav_recorder` - Records microphone to WAV
- `BUILD/protocol_test` - Protocol layer test

---

## 📚 Building Library Examples

```bash
mkdir -p build && cd build
cmake .. -DBUILD_EXAMPLES=ON -DBUILD_DESKTOP=OFF -DBUILD_PI=OFF
make -j$(nproc)
cd ..
```

### Built Examples:
- `BUILD/chord_to_bits` - Chord transmission example
- `BUILD/onlytones` - Tone generation only
- `BUILD/onlydetection` - Detection only
- `BUILD/tones_and_detection` - Combined example
- `BUILD/crc_full_test` - Full CRC test

---

## 🔧 CMake Options

```bash
cmake .. \
	-DCMAKE_BUILD_TYPE=Release \     # or Debug
	-DBUILD_DESKTOP=ON/OFF \         # Desktop programs (SRC/)
	-DBUILD_PI=ON/OFF \              # Raspberry Pi programs (src/)
	-DBUILD_TESTS=ON/OFF \           # Test programs (TESTS/)
	-DBUILD_EXAMPLES=ON/OFF          # Library examples (LIB/examples/)
```

### Build Types:
- **Release** - Optimized for performance (default)
- **Debug** - Includes debug symbols, no optimization

### Common Configurations:

```bash
# Desktop development
cmake .. -DBUILD_DESKTOP=ON -DBUILD_PI=OFF -DBUILD_TESTS=ON

# Raspberry Pi deployment
cmake .. -DBUILD_DESKTOP=OFF -DBUILD_PI=ON

# Testing only
cmake .. -DBUILD_TESTS=ON -DBUILD_DESKTOP=OFF -DBUILD_PI=OFF

# Everything (not recommended - choose what you need)
cmake .. -DBUILD_DESKTOP=ON -DBUILD_PI=ON -DBUILD_TESTS=ON -DBUILD_EXAMPLES=ON
```

---

## 📋 Dependencies

### Required for All:
- C++17 compiler (g++ 7.0+)
- PortAudio library
- FFTW3 library (for frequency detection)

### Desktop Only:
- SQLite3 (for database logging)

### Raspberry Pi ROS Node Only:
- ROS (roscpp, std_msgs, geometry_msgs)

### Installation:

#### Ubuntu/Debian:
```bash
sudo apt-get update
sudo apt-get install \
	build-essential \
	cmake \
	libportaudio2 \
	portaudio19-dev \
	libfftw3-dev \
	libsqlite3-dev
```

#### Or use install script (SRC/ only):
```bash
cd SRC
./install_deps.sh
```

---

## 🗂️ Build Output

All executables are placed in the `BUILD/` directory:

```
BUILD/
├── sender                      # Desktop sender
├── protocol_sender             # Desktop protocol sender
├── protocol_receiver           # Pi receiver
├── rb3_node_cpp               # ROS node (if ROS available)
├── frequency_test             # Test programs
├── chord_diagnostic_test      # Test programs
└── ... (more executables)
```

**Note:** The `BUILD/` directory is in `.gitignore` and not committed to git.

---

## 🧹 Cleaning

### CMake builds:
```bash
rm -rf build/
rm -rf BUILD/
```

### Makefile builds:
```bash
cd SRC
make clean
```

---

## ⚠️ Common Issues

### "PortAudio not found"
```bash
sudo apt-get install libportaudio2 portaudio19-dev
```

### "FFTW3 not found"
```bash
sudo apt-get install libfftw3-dev
```

### "SQLite3 not found"
```bash
sudo apt-get install libsqlite3-dev
```

### CMake can't find dependencies
Try specifying paths manually:
```bash
cmake .. \
	-DPortAudio_INCLUDE_DIR=/usr/include \
	-DPortAudio_LIBRARY=/usr/lib/x86_64-linux-gnu/libportaudio.so
```

### ROS not found (for rb3_node_cpp)
Make sure ROS is installed and sourced:
```bash
source /opt/ros/noetic/setup.bash  # or your ROS version
```

---

## 🚀 Running Programs

### Desktop Sender:
```bash
cd BUILD
./protocol_sender
```

### Raspberry Pi Receiver:
```bash
cd BUILD
./protocol_receiver
```

### Tests:
```bash
cd BUILD
./frequency_test
./chord_diagnostic_test
./wav_recorder
```

---

## 📝 Notes

- Build scripts create a `build/` directory for CMake intermediate files
- Final executables go to `BUILD/` directory
- You can use either CMake or Makefile, but CMake is recommended
- The Makefile in `SRC/` is kept for legacy compatibility
- Each build configuration (desktop/pi/tests) can be built independently
- Cross-compilation for Pi from desktop is possible but not configured yet

---

## 🔗 See Also

- [README.md](README.md) - Project overview
- [DOCS/guides/CRC_USAGE_GUIDE.md](DOCS/guides/CRC_USAGE_GUIDE.md) - Usage guide
- [TESTS/README.md](TESTS/README.md) - Test programs documentation
- [SRC/README.md](SRC/README.md) - Desktop source documentation
- [src/README.md](src/README.md) - Raspberry Pi source documentation
