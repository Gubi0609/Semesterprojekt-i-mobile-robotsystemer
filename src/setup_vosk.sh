#!/bin/bash
# Setup script for Vosk speech recognition

echo "=== Vosk Setup Script ==="
echo ""

# Check if running on Pi or x86
ARCH=$(uname -m)
echo "Architecture: $ARCH"

# Install PortAudio if not already installed (needed for microphone)
echo ""
echo "Checking PortAudio..."
if ! dpkg -l | grep -q libportaudio2; then
	echo "Installing PortAudio..."
	sudo apt-get update
	sudo apt-get install -y libportaudio2 portaudio19-dev
else
	echo "PortAudio already installed"
fi

# Download Vosk API
echo ""
echo "Downloading Vosk API..."
cd /tmp

if [ "$ARCH" = "aarch64" ] || [ "$ARCH" = "armv7l" ]; then
	# Raspberry Pi (ARM)
	echo "Downloading Vosk for ARM..."
	wget https://github.com/alphacep/vosk-api/releases/download/v0.3.45/vosk-linux-aarch64-0.3.45.zip -O vosk.zip
else
	# x86_64
	echo "Downloading Vosk for x86_64..."
	wget https://github.com/alphacep/vosk-api/releases/download/v0.3.45/vosk-linux-x86_64-0.3.45.zip -O vosk.zip
fi

echo "Extracting Vosk..."
unzip -o vosk.zip
VOSK_DIR=$(ls -d vosk-* | head -n 1)

echo "Installing Vosk library..."
sudo cp $VOSK_DIR/libvosk.so /usr/local/lib/
sudo ldconfig

echo "Installing Vosk headers..."
sudo mkdir -p /usr/local/include/vosk
sudo cp $VOSK_DIR/vosk_api.h /usr/local/include/vosk/

# Download a small English model
echo ""
echo "Downloading Vosk English model (small - ~40MB)..."
cd /tmp
wget https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip -O model.zip

echo "Extracting model..."
unzip -o model.zip
MODEL_DIR=$(ls -d vosk-model-* | head -n 1)

# Move model to home directory
echo "Installing model..."
mkdir -p ~/vosk-models
mv $MODEL_DIR ~/vosk-models/
echo "Model installed to: ~/vosk-models/$MODEL_DIR"

# Cleanup
echo ""
echo "Cleaning up..."
cd /tmp
rm -f vosk.zip model.zip
rm -rf vosk-*

echo ""
echo "=== Setup Complete! ==="
echo ""
echo "Vosk library installed to: /usr/local/lib/libvosk.so"
echo "Vosk headers installed to: /usr/local/include/vosk/"
echo "Model installed to: ~/vosk-models/$MODEL_DIR"
echo ""
echo "To use the model, set the path in your code:"
echo "  const char* model_path = \"$HOME/vosk-models/$MODEL_DIR\";"
echo ""
echo "Now rebuild voice_receiver with: make voice_receiver"
