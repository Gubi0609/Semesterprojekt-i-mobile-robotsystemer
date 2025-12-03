# Audio Library Usage Guide

This guide shows how to use the ToneGenerator and FrequencyDetector libraries in your own projects.

## Library Files

- `tone_generator.h` - ToneGenerator header
- `tone_generator_lib.cpp` - ToneGenerator implementation
- `frequency_detector.h` - FrequencyDetector header
- `frequency_detector_lib.cpp` - FrequencyDetector implementation

## Building

### Compile the libraries:
```bash
make libs
```

### Build example programs:
```bash
make examples
```

This creates:
- `tone_example` - ToneGenerator examples
- `detector_example` - FrequencyDetector examples
- `combined_example` - Using both together

## ToneGenerator API

### Basic Usage

```cpp
#include "tone_generator.h"

ToneGenerator generator;

// Configure tone generation
ToneGenerator::Config config;
config.frequencies = {1000.0, 2000.0};  // Hz
config.duration = 5.0;                   // seconds (0 = infinite)
config.gain = 0.5;                       // 0.0 - 1.0
config.channels = 2;                     // 1=mono, 2=stereo
config.sampleRate = 48000.0;             // Hz
config.fadeTime = 0.01;                  // fade in/out seconds

// Blocking mode (waits until complete)
generator.start(config);
generator.waitForCompletion();

// Async mode (non-blocking)
generator.startAsync(config, []() {
	std::cout << "Playback completed!\n";
});

// Do other work...
while (generator.isPlaying()) {
	double pos = generator.getPlaybackPosition();
	std::cout << "Position: " << pos << "s\n";
}

// Stop manually
generator.stop();
```

### Configuration Options

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `frequencies` | `vector<double>` | Required | Frequencies in Hz to generate |
| `sampleRate` | `double` | 48000.0 | Sample rate in Hz |
| `duration` | `double` | 5.0 | Duration in seconds (0 = infinite) |
| `gain` | `double` | 0.8 | Overall gain (0.0 - 1.0) |
| `channels` | `int` | 2 | 1=mono, 2=stereo |
| `fadeTime` | `double` | 0.01 | Fade in/out time in seconds |

### Methods

- `bool start(const Config& config)` - Start playback (blocking)
- `bool startAsync(const Config& config, callback)` - Start playback (non-blocking)
- `void stop()` - Stop playback
- `bool isPlaying()` - Check if currently playing
- `double getPlaybackPosition()` - Get current position in seconds
- `void waitForCompletion()` - Wait until playback completes

### Examples

**Single tone for 3 seconds:**
```cpp
ToneGenerator::Config config;
config.frequencies = {440.0};  // A4 note
config.duration = 3.0;
generator.start(config);
generator.waitForCompletion();
```

**Multiple tones (chord):**
```cpp
ToneGenerator::Config config;
config.frequencies = {440.0, 554.0, 659.0};  // A major chord
config.duration = 5.0;
config.gain = 0.3;  // Lower gain for multiple tones
generator.startAsync(config);
```

**Continuous tone (stop manually):**
```cpp
ToneGenerator::Config config;
config.frequencies = {1000.0};
config.duration = 0.0;  // Infinite
generator.startAsync(config);

// Do work...
std::this_thread::sleep_for(std::chrono::seconds(5));

generator.stop();
```

## FrequencyDetector API

### Basic Usage

```cpp
#include "frequency_detector.h"

FrequencyDetector detector;

// Configure detection
FrequencyDetector::Config config;
config.sampleRate = 48000;              // Hz
config.fftSize = 4096;                  // FFT size (power of 2)
config.numPeaks = 5;                    // Number of peaks to detect
config.duration = 10.0;                 // seconds (0 = infinite)
config.bandpassLow = 800.0;             // Hz (0 = disabled)
config.bandpassHigh = 1200.0;           // Hz (0 = disabled)
config.updateRate = 10.0;               // updates per second

// Blocking mode (returns final result)
auto peaks = detector.start(config);
for (const auto& peak : peaks) {
	std::cout << peak.frequency << " Hz ("
			  << (peak.magnitude * 100) << "%)\n";
}

// Async mode with callback
detector.startAsync(config, [](const auto& peaks) {
	std::cout << "Detected: ";
	for (const auto& peak : peaks) {
		std::cout << peak.frequency << " Hz ";
	}
	std::cout << "\n";
});

detector.waitForCompletion();
```

### Configuration Options

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `sampleRate` | `int` | 48000 | Sample rate in Hz |
| `fftSize` | `int` | 4096 | FFT size (must be power of 2) |
| `numPeaks` | `int` | 5 | Number of peaks to return |
| `duration` | `double` | 0.0 | Duration in seconds (0 = infinite) |
| `bandpassLow` | `double` | 0.0 | Low cutoff Hz (0 = disabled) |
| `bandpassHigh` | `double` | 0.0 | High cutoff Hz (0 = disabled) |
| `updateRate` | `double` | 20.0 | Callback updates per second |

### FrequencyPeak Structure

```cpp
struct FrequencyPeak {
	double frequency;    // Frequency in Hz
	double magnitude;    // Normalized magnitude (0.0 - 1.0)
	double magnitudeDB;  // Magnitude in decibels
};
```

### Methods

- `vector<FrequencyPeak> start(const Config&)` - Start detection (blocking)
- `bool startAsync(const Config&, callback)` - Start detection (non-blocking)
- `void stop()` - Stop detection
- `bool isDetecting()` - Check if currently detecting
- `vector<FrequencyPeak> getDetectedFrequencies()` - Get latest results
- `void waitForCompletion()` - Wait until detection completes

### Examples

**Detect for 5 seconds:**
```cpp
FrequencyDetector::Config config;
config.duration = 5.0;
config.numPeaks = 3;
auto peaks = detector.start(config);
```

**Real-time detection with callback:**
```cpp
FrequencyDetector::Config config;
config.duration = 10.0;
config.updateRate = 5.0;  // 5 updates/sec
config.numPeaks = 5;

detector.startAsync(config, [](const auto& peaks) {
	for (const auto& peak : peaks) {
		std::cout << peak.frequency << " Hz ";
	}
	std::cout << "\n";
});
```

**With bandpass filter:**
```cpp
FrequencyDetector::Config config;
config.duration = 5.0;
config.bandpassLow = 800.0;
config.bandpassHigh = 1200.0;  // Only detect 800-1200 Hz
config.numPeaks = 3;
auto peaks = detector.start(config);
```

**High resolution detection:**
```cpp
FrequencyDetector::Config config;
config.fftSize = 8192;  // Better resolution
config.duration = 5.0;
// Resolution = 48000 / 8192 = 5.86 Hz per bin
auto peaks = detector.start(config);
```

## Using Both Together

### Generate and detect simultaneously:

```cpp
#include "tone_generator.h"
#include "frequency_detector.h"

ToneGenerator generator;
FrequencyDetector detector;

// Start generating tone
ToneGenerator::Config genConfig;
genConfig.frequencies = {1000.0};
genConfig.duration = 10.0;
generator.startAsync(genConfig);

// Wait for it to stabilize
std::this_thread::sleep_for(std::chrono::milliseconds(500));

// Start detecting
FrequencyDetector::Config detConfig;
detConfig.duration = 10.0;
detConfig.updateRate = 2.0;

detector.startAsync(detConfig, [](const auto& peaks) {
	std::cout << "Detected: ";
	for (const auto& peak : peaks) {
		std::cout << peak.frequency << " Hz ";
	}
	std::cout << "\n";
});

// Wait for both to finish
detector.waitForCompletion();
generator.stop();
```

## Integration into Your Project

### Option 1: Compile object files and link

```bash
# Compile libraries
g++ -std=c++17 -O2 -c tone_generator_lib.cpp -o tone_generator_lib.o
g++ -std=c++17 -O2 -c frequency_detector_lib.cpp -o frequency_detector_lib.o

# Compile your program and link
g++ -std=c++17 -O2 -o myprogram myprogram.cpp \
	tone_generator_lib.o frequency_detector_lib.o \
	-lportaudio -lfftw3 -lm -lpthread
```

### Option 2: Include in your Makefile

```makefile
OBJS = myprogram.o tone_generator_lib.o frequency_detector_lib.o
LIBS = -lportaudio -lfftw3 -lm -lpthread

myprogram: $(OBJS)
	g++ -o myprogram $(OBJS) $(LIBS)

tone_generator_lib.o: tone_generator_lib.cpp tone_generator.h
	g++ -std=c++17 -O2 -c tone_generator_lib.cpp

frequency_detector_lib.o: frequency_detector_lib.cpp frequency_detector.h
	g++ -std=c++17 -O2 -c frequency_detector_lib.cpp

myprogram.o: myprogram.cpp
	g++ -std=c++17 -O2 -c myprogram.cpp
```

### Option 3: Direct include in CMake

```cmake
add_executable(myprogram
	myprogram.cpp
	tone_generator_lib.cpp
	frequency_detector_lib.cpp
)

target_link_libraries(myprogram
	portaudio
	fftw3
	pthread
	m
)
```

## Complete Example

See `combined_example.cpp` for a full working example that:
1. Generates multiple tones
2. Detects them with bandpass filtering
3. Uses high-resolution FFT for close frequencies
4. Demonstrates both blocking and async modes

Run it with:
```bash
make combined_example
./combined_example
```

## Tips

### Tone Generation
- Use lower gain (0.2-0.4) when generating multiple frequencies
- Set `duration = 0.0` for continuous playback
- Use `fadeTime` to avoid clicks at start/stop

### Frequency Detection
- Higher FFT size = better resolution but more latency
- Use bandpass filter to focus on specific frequency ranges
- Set appropriate `updateRate` for your needs (1-20 Hz typical)
- Frequency resolution = sampleRate / fftSize

### Performance
- Both libraries use separate threads for audio I/O
- Safe to use multiple instances simultaneously
- Callbacks execute on detection thread (keep them fast)
- Both libraries handle PortAudio initialization/cleanup

## Dependencies

- **PortAudio**: Audio I/O (both libraries)
- **FFTW3**: FFT calculations (FrequencyDetector only)
- **C++17**: Required for both libraries
