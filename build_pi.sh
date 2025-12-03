#!/bin/bash
# Build script for Raspberry Pi programs

set -e  # Exit on error

echo "=========================================="
echo "Building Raspberry Pi Programs (src/)"
echo "=========================================="
echo ""

# Create build directory
mkdir -p build
cd build

# Run CMake with Pi options
cmake .. \
	-DCMAKE_BUILD_TYPE=Release \
	-DBUILD_DESKTOP=OFF \
	-DBUILD_PI=ON \
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
echo "  - BUILD/protocol_receiver"
if [ -f "../BUILD/rb3_node_cpp" ]; then
	echo "  - BUILD/rb3_node_cpp (ROS node)"
fi
echo ""
echo "Run with:"
echo "  cd BUILD && ./protocol_receiver"
echo ""
