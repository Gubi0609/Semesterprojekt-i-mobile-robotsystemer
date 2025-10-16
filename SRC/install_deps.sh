#!/bin/bash
# Installation script for audio processing dependencies

echo "Installing audio processing dependencies..."
echo ""

# Detect OS
if [ -f /etc/os-release ]; then
	. /etc/os-release
	OS=$ID
else
	OS=$(uname -s)
fi

case "$OS" in
	ubuntu|debian)
		echo "Detected Debian/Ubuntu system"
		sudo apt-get update
		sudo apt-get install -y libportaudio2 portaudio19-dev libfftw3-dev libmp3lame-dev build-essential
		;;
	fedora|rhel|centos)
		echo "Detected Fedora/RHEL system"
		sudo dnf install -y portaudio portaudio-devel fftw3-devel lame-devel gcc-c++
		;;
	arch|manjaro)
		echo "Detected Arch Linux system"
		sudo pacman -S --noconfirm portaudio fftw lame base-devel
		;;
	Darwin|darwin)
		echo "Detected macOS system"
		if ! command -v brew &> /dev/null; then
			echo "Homebrew not found. Please install from https://brew.sh"
			exit 1
		fi
		brew install portaudio fftw lame
		;;
	*)
		echo "Unknown OS: $OS"
		echo "Please manually install: portaudio, fftw3, and lame development libraries"
		exit 1
		;;
esac

echo ""
echo "Dependencies installed successfully!"
echo "Now run: make all"
