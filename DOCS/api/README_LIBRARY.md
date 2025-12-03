# Audio Communication Library API

Clean, reusable library for audio-based communication between devices.

## Overview

The library provides two main classes for transmitting and receiving data via audio:

- **SingleToneTransmitter / SingleToneReceiver**: 4-bit communication (0-15)
- **ChordTransmitter / ChordReceiver**: 16-bit communication (0-65535)

## Quick Example

```cpp
#include "audio_transmitter.h"
#include "audio_receiver.h"

// Transmit a 16-bit value
AudioComm::ChordTransmitter transmitter;
transmitter.startTransmitting(1234);

// Receive with callback
AudioComm::ChordReceiver receiver;
receiver.startReceiving({}, [](const AudioComm::ChordReceiver::Detection& det) {
		std::cout << "Received value: " << det.value << "\n";
});
```

## API Reference

### ChordTransmitter

Transmits 16-bit values using 4 simultaneous tones.

#### Configuration

```cpp
struct Config {
		double tone1MinFreq = 5000.0;     // Tone 1 range
		double tone1MaxFreq = 8000.0;
		double tone2MinFreq = 8500.0;     // Tone 2 range
		double tone2MaxFreq = 11500.0;
		double tone3MinFreq = 12000.0;    // Tone 3 range
		double tone3MaxFreq = 15000.0;
		double tone4MinFreq = 15500.0;    // Tone 4 range
		double tone4MaxFreq = 18500.0;
		int bitsPerTone = 4;              // Don't change
		double toneDuration = 0.0;        // 0 = infinite
		double fadeTime = 0.05;           // Smooth transitions
		double sampleRate = 48000.0;
		double gain = 0.4;                // Volume (0.0-1.0)
};
```

#### Methods

```cpp
// Start transmitting (blocking)
bool startTransmitting(uint16_t value);
bool startTransmitting(uint16_t value, const Config& config);

// Start transmitting (non-blocking)
bool startTransmittingAsync(uint16_t value,
													 std::function<void()> onComplete = nullptr);
bool startTransmittingAsync(uint16_t value, const Config& config,
													 std::function<void()> onComplete = nullptr);

// Control
void stop();
bool isTransmitting() const;
std::vector<double> getCurrentFrequencies() const;
void waitForCompletion();
```

#### Example

```cpp
AudioComm::ChordTransmitter transmitter;

// Use default config
transmitter.startTransmitting(12345);
transmitter.waitForCompletion();

// Or customize
AudioComm::ChordTransmitter::Config config;
config.toneDuration = 2.0;  // Transmit for 2 seconds
config.gain = 0.6;          // Louder

transmitter.startTransmitting(42, config);
```

### ChordReceiver

Receives 16-bit values from 4-tone chords.

#### Configuration

```cpp
struct Config {
		double tone1MinFreq = 5000.0;     // Must match transmitter
		double tone1MaxFreq = 8000.0;
		double tone2MinFreq = 8500.0;
		double tone2MaxFreq = 11500.0;
		double tone3MinFreq = 12000.0;
		double tone3MaxFreq = 15000.0;
		double tone4MinFreq = 15500.0;
		double tone4MaxFreq = 18500.0;
		int bitsPerTone = 4;
		double sampleRate = 48000.0;
		int fftSize = 8192;               // Higher = better resolution
		double detectionTolerance = 100.0; // Frequency tolerance (Hz)
		double updateRate = 10.0;         // Detections per second

		// Consistency checking (prevents false positives)
		int minDetections = 2;            // Require N detections
		double consistencyWindow = 1.0;   // Within N seconds
};
```

#### Detection Structure

```cpp
struct Detection {
		uint16_t value;                   // Decoded value (0-65535)
		std::vector<double> frequencies;  // All 4 detected frequencies
		std::vector<uint8_t> toneValues;  // 4-bit value per tone
		double magnitude;                 // Signal strength
		int detectionCount;               // How many times detected
		double detectionTime;             // Time span of detections
};
```

#### Methods

```cpp
// Start receiving with callback
bool startReceiving(const Config& config,
									 std::function<void(const Detection&)> onDetection);

// Control
void stop();
bool isReceiving() const;
bool getLastDetection(Detection& detection) const;
void waitForCompletion();
```

#### Example

```cpp
AudioComm::ChordReceiver receiver;
AudioComm::ChordReceiver::Config config;

// Optional: adjust settings
config.minDetections = 3;  // More stringent
config.fftSize = 16384;    // Better resolution

// Start receiving
receiver.startReceiving(config,
		[](const AudioComm::ChordReceiver::Detection& det) {
				std::cout << "Received: " << det.value << "\n";
				std::cout << "Detected " << det.detectionCount << " times\n";
				std::cout << "Over " << det.detectionTime << " seconds\n";

				// Show individual tones
				for (size_t i = 0; i < 4; ++i) {
						std::cout << "Tone " << (i+1) << ": "
											<< static_cast<int>(det.toneValues[i]) << "\n";
				}
		});

// Run until Ctrl+C
while (receiver.isReceiving()) {
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
}
```

### SingleToneTransmitter

Transmits 4-bit values (0-15) using a single tone.

#### Configuration

```cpp
struct Config {
		double minFreq = 5000.0;
		double maxFreq = 8000.0;
		int bitsPerTone = 4;
		double toneDuration = 0.0;
		double fadeTime = 0.05;
		double sampleRate = 48000.0;
		double gain = 0.5;
};
```

#### Methods

```cpp
bool startTransmitting(uint8_t value);
bool startTransmitting(uint8_t value, const Config& config);
bool startTransmittingAsync(uint8_t value,
													 std::function<void()> onComplete = nullptr);
void stop();
bool isTransmitting() const;
double getCurrentFrequency() const;
void waitForCompletion();
```

#### Example

```cpp
AudioComm::SingleToneTransmitter transmitter;
transmitter.startTransmitting(7);  // Transmit value 7 (6400 Hz)
```

### SingleToneReceiver

Receives 4-bit values from single tones.

#### Configuration

```cpp
struct Config {
		double minFreq = 5000.0;
		double maxFreq = 8000.0;
		int bitsPerTone = 4;
		double sampleRate = 48000.0;
		int fftSize = 4096;
		double detectionTolerance = 100.0;
		double updateRate = 10.0;
};
```

#### Detection Structure

```cpp
struct Detection {
		uint8_t value;        // Decoded value (0-15)
		double frequency;     // Detected frequency
		double magnitude;     // Signal strength
		int detectionCount;   // For consistency
};
```

#### Example

```cpp
AudioComm::SingleToneReceiver receiver;
receiver.startReceiving({},
		[](const AudioComm::SingleToneReceiver::Detection& det) {
				std::cout << "Received: " << static_cast<int>(det.value) << "\n";
				std::cout << "Frequency: " << det.frequency << " Hz\n";
		});
```

## Building Your Application

### Makefile

```makefile
CXX = g++
CXXFLAGS = -std=c++17 -Wall -O2 -I/path/to/LIB
LIBS = -lportaudio -lfftw3 -lpthread

OBJS = tone_generator_lib.o \
			 frequency_detector_lib.o \
			 audio_comm_lib.o \
			 audio_transmitter_lib.o \
			 audio_receiver_lib.o

myapp: myapp.cpp $(OBJS)
	$(CXX) $(CXXFLAGS) -o myapp myapp.cpp $(OBJS) $(LIBS)
```

### Required Libraries

- **PortAudio**: Audio I/O
- **FFTW3**: FFT for frequency detection
- **pthread**: Threading

Install on Ubuntu/Debian:
```bash
sudo apt-get install libportaudio2 libportaudio-ocaml-dev libfftw3-dev
```

## Design Patterns

### Fire and Forget

```cpp
AudioComm::ChordTransmitter tx;
AudioComm::ChordTransmitter::Config config;
config.toneDuration = 1.0;  // Transmit for 1 second
tx.startTransmitting(1234, config);
// Continue doing other work...
```

### Continuous Listening

```cpp
AudioComm::ChordReceiver rx;
rx.startReceiving({}, [](const auto& det) {
		processReceivedValue(det.value);
});

// Runs forever
while (true) {
		doOtherWork();
}

rx.stop();
```

### Request-Response

```cpp
// Transmitter side
void sendCommand(uint16_t cmd) {
		AudioComm::ChordTransmitter tx;
		AudioComm::ChordTransmitter::Config config;
		config.toneDuration = 0.5;
		tx.startTransmitting(cmd, config);
		tx.waitForCompletion();
}

// Receiver side
AudioComm::ChordReceiver rx;
rx.startReceiving({}, [](const auto& det) {
		handleCommand(det.value);
		sendResponse(calculateResponse(det.value));
});
```

## Error Handling

All `start*()` methods return `bool`:
- `true`: Successfully started
- `false`: Failed (no audio device, already running, etc.)

```cpp
if (!transmitter.startTransmitting(value)) {
		std::cerr << "Failed to start transmission\n";
		return;
}
```

## Thread Safety

- Callbacks are called from audio processing threads
- Keep callbacks short and non-blocking
- Use mutex if modifying shared state

```cpp
std::mutex dataMutex;
std::vector<uint16_t> receivedValues;

receiver.startReceiving({}, [&](const auto& det) {
		std::lock_guard<std::mutex> lock(dataMutex);
		receivedValues.push_back(det.value);
});
```

## Performance Tips

1. **FFT Size**: Larger = better resolution but slower
	 - 4096: Good for single tone
	 - 8192: Good for chords
	 - 16384: Best quality, slower

2. **Update Rate**: How often to check for signals
	 - 10 Hz: Good balance
	 - 20 Hz: Faster response, more CPU

3. **Consistency Checking**: Prevents false positives
	 - `minDetections = 2`: Good default
	 - Increase for noisy environments

4. **Tone Duration**: Longer = more reliable
	 - 0.5s: Quick but may miss
	 - 1.0s: Good balance
	 - 2.0s: Very reliable

## Troubleshooting

### No detection

- Check audio devices are working
- Verify frequency ranges match
- Increase volume
- Check `detectionTolerance`

### False positives

- Increase `minDetections`
- Reduce `consistencyWindow`
- Use bandpass filtering (automatic)

### Poor accuracy

- Increase `fftSize`
- Reduce background noise
- Adjust frequency ranges for your hardware

## Next Steps

1. Run the example: `./simple_chord_example`
2. Check out the command-line tools for testing
3. Integrate into your robot/application
4. Add error correction/checksums for reliability
