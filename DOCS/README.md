# Audio Communication System

Complete library for audio-based data transmission between devices (computer ↔ robot).

## What You Have

### 1. **Core Libraries** (in LIB/)
- `audio_comm.h/cpp` - Frequency encoding/decoding
- `audio_transmitter.h/lib.cpp` - Transmitter classes
- `audio_receiver.h/lib.cpp` - Receiver classes
- `tone_generator.h/lib.cpp` - Audio playback
- `frequency_detector.h/lib.cpp` - Audio analysis with FFT

### 2. **Command-Line Tools** (in BUILD/)
- `transmitter` - Single tone transmitter (4-bit: 0-15)
- `receiver` - Single tone receiver
- `chord_transmitter` - Multi-tone transmitter (16-bit: 0-65535)
- `chord_receiver` - Multi-tone receiver

### 3. **Example Code**
- `examples/simple_chord_example.cpp` - Complete usage example
- See `README_LIBRARY.md` for API documentation

## Quick Test

### Single Tone (4 bits)

**Terminal 1:**
```bash
cd BUILD
./receiver
```

**Terminal 2:**
```bash
cd BUILD
./transmitter 7
```

### Chord (16 bits)

**Terminal 1:**
```bash
cd BUILD
./chord_receiver
```

**Terminal 2:**
```bash
cd BUILD
./chord_transmitter 1234
```

## How It Works

### Single Tone Mode
- **Data**: 4 bits (0-15)
- **Method**: Frequency Shift Keying (FSK)
- **Range**: 5000-8000 Hz (200 Hz steps)
- **Use case**: Simple commands

| Value | Frequency |
|-------|-----------|
| 0     | 5000 Hz   |
| 7     | 6400 Hz   |
| 15    | 8000 Hz   |

### Chord Mode (Recommended)
- **Data**: 16 bits (0-65535)
- **Method**: 4 simultaneous tones (chord)
- **Ranges**:
	- Tone 1: 5000-8000 Hz (bits 0-3)
	- Tone 2: 8500-11500 Hz (bits 4-7)
	- Tone 3: 12000-15000 Hz (bits 8-11)
	- Tone 4: 15500-18500 Hz (bits 12-15)
- **Use case**: Complex data, robot control

Example: Value 1234 (0x04D2) = 4 tones at specific frequencies

### Robustness Features

1. **Bandpass Filtering**: Removes out-of-range noise
2. **Magnitude Thresholding**: Ignores weak signals
3. **Peak Detection**: Only considers local maxima
4. **Frequency Validation**: Must match expected values
5. **Consistency Checking**: Requires multiple detections (chord mode)
6. **All-or-Nothing**: Chord requires all 4 tones present

These features make false positives extremely rare!

## Using as a Library

### Simple Example

```cpp
#include "audio_transmitter.h"
#include "audio_receiver.h"

int main() {
		// Transmit
		AudioComm::ChordTransmitter tx;
		tx.startTransmitting(1234);

		// Receive
		AudioComm::ChordReceiver rx;
		rx.startReceiving({}, [](const auto& det) {
				std::cout << "Received: " << det.value << "\n";
		});

		std::this_thread::sleep_for(std::chrono::seconds(5));
		return 0;
}
```

### Build Your App

```makefile
CXX = g++
CXXFLAGS = -std=c++17 -I../LIB
LIBS = -lportaudio -lfftw3 -lpthread

AUDIO_OBJS = tone_generator_lib.o frequency_detector_lib.o \
						 audio_comm_lib.o audio_transmitter_lib.o audio_receiver_lib.o

myapp: myapp.cpp $(AUDIO_OBJS)
	$(CXX) $(CXXFLAGS) -o myapp myapp.cpp $(AUDIO_OBJS) $(LIBS)
```

## Documentation

- **README_LIBRARY.md** - Complete API reference
- **README_AUDIO_COMM.md** - Single tone details
- **README_CHORD.md** - Chord mode details

## Architecture

```
Application
		↓
AudioTransmitter / AudioReceiver (High-level API)
		↓
ToneGenerator / FrequencyDetector (Audio I/O)
		↓
PortAudio (Hardware)
```

Encoding/Decoding is handled by `audio_comm` classes.

## Configuration

All parameters are adjustable via Config structs:

```cpp
AudioComm::ChordTransmitter::Config config;
config.tone1MinFreq = 5000.0;   // Adjust frequency ranges
config.toneDuration = 2.0;       // Transmission time
config.gain = 0.6;              // Volume

AudioComm::ChordReceiver::Config rxConfig;
rxConfig.fftSize = 16384;        // Better resolution
rxConfig.minDetections = 3;      // More stringent
rxConfig.detectionTolerance = 150.0;  // More forgiving
```

## Building Everything

```bash
cd SRC
make clean
make all        # Original tools + audio communication
make libs       # Just the library objects
make examples   # Example programs
```

## Files Overview

### Library Files
```
LIB/
├── audio_comm.h/cpp              # Core encoding/decoding
├── audio_transmitter.h/lib.cpp   # Transmitter classes
├── audio_receiver.h/lib.cpp      # Receiver classes
├── tone_generator.h/lib.cpp      # Audio output
├── frequency_detector.h/lib.cpp  # Audio input + FFT
├── transmitter.cpp               # CLI tool
├── receiver.cpp                  # CLI tool
├── chord_transmitter.cpp         # CLI tool
├── chord_receiver.cpp            # CLI tool
└── examples/
		└── simple_chord_example.cpp  # Usage example
```

### Generated Files
```
BUILD/
├── *.o                           # Library objects
├── transmitter                   # Single tone TX
├── receiver                      # Single tone RX
├── chord_transmitter             # Chord TX
├── chord_receiver                # Chord RX
└── simple_chord_example          # Example program
```

## Next Steps

1. **Test the system**: Use the CLI tools to verify it works in your environment
2. **Integrate into your project**: Use the library classes
3. **Optimize settings**: Adjust frequencies, FFT size, tolerances for your hardware
4. **Add protocol layer**: Implement start/stop markers, checksums, packets
5. **Robot integration**: Use for remote control, telemetry, etc.

## Performance

- **Latency**: 100-200ms detection delay
- **Range**: 1-5 meters (depends on speaker/microphone)
- **Data rate**:
	- Single tone: ~5 values/second
	- Chord: ~5 values/second (16 bits each = 80 bits/s)
- **Reliability**: Very high with consistency checking

## Troubleshooting

### Issue: No detection
- Check volume
- Verify microphone is working
- Try lower frequencies
- Check `detectionTolerance`

### Issue: False positives
- Increase `minDetections`
- Reduce `consistencyWindow`
- Use chord mode (more robust)

### Issue: High frequencies don't work
- Some hardware can't handle >15 kHz
- Lower all frequency ranges in config
- Test with single tone first

## License & Credits

Part of the Semester Project in Mobile Robot Systems.

Uses:
- **PortAudio** - Cross-platform audio I/O
- **FFTW3** - Fast Fourier Transform
