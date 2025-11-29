# CMake Build System Update

**Date:** 2024-11-29
**Status:** ✅ Complete and Tested

---

## 🎯 What Was Done

Created a modern, comprehensive CMake build system that supports independent builds for:
- ✅ Desktop/laptop programs (SRC/)
- ✅ Raspberry Pi programs (src/)
- ✅ Test programs (TESTS/)
- ✅ Library examples (LIB/examples/)

---

## ✨ New Files

1. **CMakeLists.txt** (updated) - Modern CMake configuration
2. **build_desktop.sh** - Quick build script for desktop programs
3. **build_pi.sh** - Quick build script for Raspberry Pi programs
4. **build_tests.sh** - Quick build script for test programs
5. **BUILD_GUIDE.md** - Comprehensive build documentation

---

## 🔧 CMake Features

### Independent Build Configurations
```bash
# Desktop only
cmake .. -DBUILD_DESKTOP=ON -DBUILD_PI=OFF

# Raspberry Pi only
cmake .. -DBUILD_DESKTOP=OFF -DBUILD_PI=ON

# Tests only
cmake .. -DBUILD_TESTS=ON -DBUILD_DESKTOP=OFF -DBUILD_PI=OFF

# Mix and match as needed
cmake .. -DBUILD_DESKTOP=ON -DBUILD_TESTS=ON
```

### Automatic Dependency Detection
- PortAudio (required for all)
- FFTW3 (required for frequency detection)
- SQLite3 (required for desktop programs)
- ROS/catkin (optional, for rb3_node_cpp)

### Smart Library Linking
- Desktop programs link: PortAudio, FFTW3, SQLite3
- Pi programs link: PortAudio, FFTW3
- ROS node links: ROS libraries + audio libraries
- Tests link: Appropriate libraries based on functionality

### Build Output
All executables go to `BUILD/` directory, regardless of build method.

---

## 📦 Build Options

### Using Build Scripts (Easiest)
```bash
./build_desktop.sh    # Desktop programs
./build_pi.sh         # Raspberry Pi programs
./build_tests.sh      # Test programs
```

### Using CMake Directly
```bash
mkdir -p build && cd build
cmake .. -DBUILD_DESKTOP=ON [options]
make -j$(nproc)
```

### Using Makefile (Legacy)
```bash
cd SRC
make protocol_sender
```

**Note:** Makefile is kept for compatibility but CMake is recommended.

---

## 🏗️ What Gets Built

### Desktop Programs (BUILD_DESKTOP=ON)
- `BUILD/sender` - Main sender with database logging
- `BUILD/protocol_sender` - Protocol-aware sender

### Raspberry Pi Programs (BUILD_PI=ON)
- `BUILD/protocol_receiver` - Protocol-aware receiver
- `BUILD/rb3_node_cpp` - ROS node (if ROS detected)
- `libvelocityProvider.a` - Static library

### Test Programs (BUILD_TESTS=ON)
- `BUILD/frequency_test`
- `BUILD/frequency_test_single`
- `BUILD/frequency_transmitter_test`
- `BUILD/chord_diagnostic_test`
- `BUILD/chord_transmitter_test`
- `BUILD/wav_recorder`
- `BUILD/protocol_test`

### Library Examples (BUILD_EXAMPLES=ON)
- `BUILD/chord_to_bits`
- `BUILD/onlytones`
- `BUILD/onlydetection`
- `BUILD/tones_and_detection`
- `BUILD/crc_full_test`

---

## ✅ Testing Results

### Desktop Build
```bash
./build_desktop.sh
✓ Built successfully
✓ protocol_sender: 421 KB
✓ sender: 483 KB
```

### Pi Build
```bash
./build_pi.sh
✓ Built successfully
✓ protocol_receiver: 443 KB
✓ libvelocityProvider.a created
✓ ROS node skipped (ROS not detected on test system)
```

---

## 🔄 Migration from Old CMakeLists.txt

### Old Behavior
- Built everything into single "main" executable
- Used `GLOB_RECURSE` to find all .cpp files
- No options for different targets
- No proper dependency management

### New Behavior
- Independent build targets
- Explicit source file lists
- Options to build only what you need
- Proper dependency detection and linking
- Separate desktop/Pi/test builds

### For Developers
If you were using:
```bash
cmake . && make
```

Now use:
```bash
./build_desktop.sh
# or
./build_pi.sh
```

---

## 📝 Documentation Updates

1. **BUILD_GUIDE.md** - Complete build documentation
	 - All build methods explained
	 - CMake options documented
	 - Common issues and solutions
	 - Examples for every scenario

2. **README.md** - Updated quick start section
	 - References new build scripts
	 - Points to BUILD_GUIDE.md

3. **SRC/README.md** - Updated with CMake info
4. **src/README.md** - Updated with CMake info

---

## 🚀 Quick Reference

### Desktop Development
```bash
./build_desktop.sh
cd BUILD && ./protocol_sender
```

### Raspberry Pi Deployment
```bash
./build_pi.sh
cd BUILD && ./protocol_receiver
```

### Running Tests
```bash
./build_tests.sh
cd BUILD && ./frequency_test
```

### Clean Build
```bash
rm -rf build/ BUILD/
./build_desktop.sh
```

---

## 📊 Benefits

1. **Independent Builds** - No need to build everything
2. **Faster Compilation** - Only build what you need
3. **Clearer Dependencies** - Explicit library requirements
4. **Better Organization** - Separate targets for desktop/Pi/tests
5. **Easy to Use** - Simple build scripts
6. **Cross-Platform Ready** - CMake handles platform differences
7. **IDE Support** - Works with CLion, VSCode CMake extensions
8. **Maintainable** - Clear structure, easy to extend

---

## ⚠️ Important Notes

- **BUILD/ directory** - All executables go here (in .gitignore)
- **build/ directory** - CMake intermediate files (in .gitignore)
- **Makefile still works** - In SRC/ directory for legacy compatibility
- **No conflicts** - CMake and Makefile use same output directory
- **ROS node** - Only built if ROS is detected
- **SQLite3** - Only required for desktop builds

---

## 🔮 Future Improvements

Possible enhancements (not yet implemented):

1. **Cross-compilation** - Build Pi programs from desktop
2. **Install target** - `make install` to system directories
3. **Package generation** - Create .deb packages
4. **Unit tests** - CTest integration
5. **Code coverage** - Coverage reports
6. **Static analysis** - Clang-tidy integration
7. **Documentation generation** - Doxygen integration

---

## 🐛 Troubleshooting

### CMake can't find PortAudio
```bash
sudo apt-get install libportaudio2 portaudio19-dev
```

### CMake can't find FFTW3
```bash
sudo apt-get install libfftw3-dev
```

### "build/ directory not empty"
```bash
rm -rf build/ && mkdir build
```

### Want to see verbose compilation
```bash
cd build
cmake .. [options]
make VERBOSE=1
```

---

## ✅ Checklist for Team

- [x] CMakeLists.txt created and tested
- [x] Build scripts created and made executable
- [x] BUILD_GUIDE.md documentation written
- [x] README.md updated with new build instructions
- [x] Desktop build tested ✓
- [x] Pi build tested ✓
- [ ] Test builds on actual Raspberry Pi
- [ ] Test ROS node build (when ROS available)
- [ ] Team members informed of new build system
- [ ] Update any CI/CD pipelines

---

**The new CMake build system is ready to use! 🎉**

See [BUILD_GUIDE.md](BUILD_GUIDE.md) for complete documentation.
