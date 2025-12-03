#!/bin/bash
# Build script for desktop/laptop programs

set -e  # Exit on error

echo "=========================================="
echo "Building Desktop Programs (SRC/)"
echo "=========================================="
echo ""

# Create build directory
mkdir -p build
cd build

# Run CMake with desktop options
cmake .. \
	-DCMAKE_BUILD_TYPE=Release \
	-DBUILD_DESKTOP=ON \
	-DBUILD_PI=OFF \
	-DBUILD_TESTS=OFF \
	-DBUILD_EXAMPLES=OFF

# Build
make -j$(nproc)

echo ""
echo "=========================================="
echo "Build complete!"
echo "=========================================="
echo ""
echo "Executables are in BUILD/ directory:"
echo "  - BUILD/sender"
echo "  - BUILD/protocol_sender"
echo ""
echo "Run with:"
echo "  cd BUILD && ./protocol_sender"
echo ""
