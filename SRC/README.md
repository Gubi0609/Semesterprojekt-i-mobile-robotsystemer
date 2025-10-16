# Audio Tone Detection System

This directory contains three audio processing tools:

## Tools

1. **tones** - Generate sine wave tones at specified frequencies
2. **mp3_recorder** - Record audio from microphone and save as MP3
3. **frequency_detector** - Real-time frequency detection using FFT

## Installation

### Install Required Dependencies

#### Ubuntu/Debian:
```bash
sudo apt-get update
sudo apt-get install libportaudio2 portaudio19-dev libfftw3-dev libmp3lame-dev
```

#### Fedora/RHEL:
```bash
sudo dnf install portaudio portaudio-devel fftw3-devel lame-devel
```

#### Arch Linux:
```bash
sudo pacman -S portaudio fftw lame
```

#### macOS (Homebrew):
```bash
brew install portaudio fftw lame
```

## Building

```bash
make all
```

Or build individually:
```bash
make tones
make mp3_recorder
make frequency_detector
```

## Usage

### 1. Tone Generator

Generate tones at specific frequencies:

```bash
# Play 1000 Hz tone for 5 seconds
./tones 1000

# Play multiple tones simultaneously
./tones 1000 5000 10000 -d 5

# Custom parameters
./tones 440 880 -d 3 -sr 48000 -gain 0.5 -ch 2
```

**Options:**
- `-d <seconds>` - Duration (default: 5)
- `-sr <rate>` - Sample rate (default: 48000)
- `-gain <0-1>` - Output gain (default: 0.8)
- `-ch <1|2>` - Channels: 1=mono, 2=stereo (default: 2)

### 2. MP3 Recorder

Record audio to MP3 file:

```bash
./mp3_recorder
```

Records 5 seconds of audio to `out.mp3` by default.

### 3. Frequency Detector

**Real-time frequency detection** - listens to microphone and identifies dominant frequencies:

```bash
# Basic usage - press Enter to stop
./frequency_detector

# Custom parameters
./frequency_detector -sr 48000 -fft 8192 -peaks 10
```

**Options:**
- `-sr <rate>` - Sample rate (default: 48000 Hz)
- `-fft <size>` - FFT size, higher = better resolution (default: 4096)
	- 2048 = ~23 Hz resolution, faster
	- 4096 = ~12 Hz resolution, balanced
	- 8192 = ~6 Hz resolution, best accuracy
- `-peaks <n>` - Number of peaks to display (default: 5)
- `-h, --help` - Show help

**Example Output:**
```
========================================
	Real-time Frequency Detector
	Sample Rate: 48000 Hz
	FFT Size: 4096
	Frequency Resolution: 11.7 Hz
========================================
Press Ctrl+C to stop...

Dominant Frequencies: 1000.0 Hz (100%) | 2000.0 Hz (45%) | 3000.0 Hz (23%)
```

## Testing the System

### Test 1: Generate and Detect a Single Tone

**Terminal 1:**
```bash
./tones 1000 -d 60  # Generate 1000 Hz tone for 60 seconds
```

**Terminal 2:**
```bash
./frequency_detector  # Should detect ~1000 Hz
```

### Test 2: Multiple Frequencies

**Terminal 1:**
```bash
./tones 440 880 1320 -d 60  # A4, A5, E6 notes
```

**Terminal 2:**
```bash
./frequency_detector -peaks 10  # Should detect all three frequencies
```

### Test 3: High Resolution Detection

**Terminal 1:**
```bash
./tones 1000 1050 -d 60  # Two close frequencies (50 Hz apart)
```

**Terminal 2:**
```bash
./frequency_detector -fft 8192  # Higher FFT size to resolve close frequencies
```

## How It Works

### Frequency Detector

The frequency detector uses the following technique:

1. **Audio Capture**: Streams audio from microphone using PortAudio
2. **Windowing**: Applies Hann window to reduce spectral leakage
3. **FFT**: Performs Fast Fourier Transform using FFTW3
4. **Peak Detection**: Identifies local maxima in frequency spectrum
5. **Display**: Shows top N frequencies with their relative magnitudes

**Key Features:**
- Real-time processing with low latency
- Overlapping windows for smooth detection
- Automatic peak finding with threshold
- Frequency resolution = Sample Rate / FFT Size

### FFT Size Selection

- **Smaller FFT (2048)**: Faster processing, lower frequency resolution
- **Larger FFT (8192)**: Better frequency resolution, more CPU intensive
- **Trade-off**: For most applications, 4096 is a good balance

**Frequency Resolution Examples:**
- 48000 Hz / 2048 = 23.4 Hz resolution
- 48000 Hz / 4096 = 11.7 Hz resolution
- 48000 Hz / 8192 = 5.9 Hz resolution

## Troubleshooting

### No audio input device
```
Error: No default input device.
```
**Solution**: Check that your microphone is connected and recognized:
```bash
arecord -l  # List audio input devices
```

### FFTW3 not found during compilation
```
Error: fftw3.h: No such file or directory
```
**Solution**: Install FFTW3 development package:
```bash
sudo apt-get install libfftw3-dev
```

### PortAudio errors
```
Error: Pa_Initialize failed
```
**Solution**: Check PortAudio installation:
```bash
pkg-config --modversion portaudio-2.0
```

### Low detection accuracy
- Increase FFT size for better frequency resolution
- Ensure good signal-to-noise ratio
- Position microphone closer to sound source
- Reduce background noise

## Clean Up

```bash
make clean
```

## Notes

- The Makefile in this directory is for local testing only (git ignored)
- All tools use PortAudio for cross-platform audio I/O
- Frequency detector uses FFTW3 for optimal FFT performance
- Sample rate of 48000 Hz allows detection up to 24000 Hz (Nyquist frequency)
