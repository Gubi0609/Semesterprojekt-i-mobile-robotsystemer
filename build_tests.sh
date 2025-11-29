#!/bin/bash
# Build script for test programs

set -e  # Exit on error

echo "=========================================="
echo "Building Test Programs (TESTS/)"
echo "=========================================="
echo ""

# Create build directory
mkdir -p build
cd build

# Run CMake with test options
cmake .. \
	-DCMAKE_BUILD_TYPE=Release \
	-DBUILD_DESKTOP=OFF \
	-DBUILD_PI=OFF \
	-DBUILD_TESTS=ON \
	-DBUILD_EXAMPLES=OFF

# Build
make -j$(nproc)

echo ""
echo "=========================================="
echo "Build complete!"
echo "=========================================="
echo ""
echo "Test executables are in BUILD/ directory:"
echo "  - BUILD/frequency_test"
echo "  - BUILD/frequency_test_single"
echo "  - BUILD/frequency_transmitter_test"
echo "  - BUILD/chord_diagnostic_test"
echo "  - BUILD/chord_transmitter_test"
echo "  - BUILD/wav_recorder"
echo "  - BUILD/protocol_test"
echo ""
echo "Run with:"
echo "  cd BUILD && ./<test_name>"
echo ""
