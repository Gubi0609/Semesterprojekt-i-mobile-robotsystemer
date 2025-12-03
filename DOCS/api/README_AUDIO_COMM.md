# Audio Communication System

Single-tone audio communication system for transmitting 4-bit data values (0-15) between devices using sound.

## Overview

This system uses **Frequency Shift Keying (FSK)** to encode data into audio tones:
- **Frequency Range**: 5000 - 8000 Hz
- **Data Capacity**: 4 bits per tone (16 possible values: 0-15)
- **Frequency Step**: 200 Hz between values
- **Tone Duration**: 0.2 seconds per value

## Architecture

### Components

1. **audio_comm.h/cpp** - Core encoding/decoding library
	 - `SingleToneEncoder`: Converts 4-bit values to frequencies
	 - `SingleToneDecoder`: Converts detected frequencies back to values

2. **transmitter** - Sends data via audio
	 - Uses `ToneGenerator` to play frequencies
	 - Command-line interface for easy testing

3. **receiver** - Receives data via audio
	 - Uses `FrequencyDetector` with FFT analysis
	 - Real-time decoding with visual feedback

## Frequency Mapping

| Value | Frequency | Binary | Hex |
|-------|-----------|--------|-----|
| 0     | 5000 Hz   | 0000   | 0x0 |
| 1     | 5200 Hz   | 0001   | 0x1 |
| 2     | 5400 Hz   | 0010   | 0x2 |
| 3     | 5600 Hz   | 0011   | 0x3 |
| 4     | 5800 Hz   | 0100   | 0x4 |
| 5     | 6000 Hz   | 0101   | 0x5 |
| 6     | 6200 Hz   | 0110   | 0x6 |
| 7     | 6400 Hz   | 0111   | 0x7 |
| 8     | 6600 Hz   | 1000   | 0x8 |
| 9     | 6800 Hz   | 1001   | 0x9 |
| 10    | 7000 Hz   | 1010   | 0xA |
| 11    | 7200 Hz   | 1011   | 0xB |
| 12    | 7400 Hz   | 1100   | 0xC |
| 13    | 7600 Hz   | 1101   | 0xD |
| 14    | 7800 Hz   | 1110   | 0xE |
| 15    | 8000 Hz   | 1111   | 0xF |

## Building

```bash
cd SRC
make audio_comm
```

This creates two executables in the `BUILD` directory:
- `transmitter` - Sends data
- `receiver` - Receives data

## Usage

### Transmitter (Computer/Robot)

Send a single value (0-15):
```bash
./transmitter 7
```

Send multiple values in sequence:
```bash
./transmitter 1 2 3 4 5
```

Send all possible values (0-15):
```bash
make test-sequence
```

### Receiver (Computer/Robot)

Listen continuously (Ctrl+C to stop):
```bash
./receiver
```

Listen for specific duration (e.g., 10 seconds):
```bash
./receiver 10
```

## Testing

### Test Individual Programs

Test transmitter only:
```bash
make test-transmit
```

Test receiver only:
```bash
make test-receive
```

### Full Communication Test

1. **Terminal 1** - Start receiver:
```bash
cd BUILD
./receiver
```

2. **Terminal 2** - Send data:
```bash
cd BUILD
./transmitter 7
```

The receiver should display:
```
Detected: 6400.0 Hz => Value:  7 (0x7) | Magnitude: 85.3% | Binary: 0111
```

### Test Complete Sequence

Send all values 0-15 and observe detection:
```bash
# Terminal 1
./receiver

# Terminal 2
./transmitter 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15
```

## Configuration

You can modify the communication parameters in `audio_comm.h`:

```cpp
struct SingleToneConfig {
		double minFreq = 5000.0;      // Minimum frequency
		double maxFreq = 8000.0;      // Maximum frequency
		int bitsPerTone = 4;          // Bits per tone
		double toneDuration = 0.2;    // Tone duration (seconds)
		double fadeTime = 0.01;       // Fade in/out (seconds)
		double sampleRate = 48000.0;  // Audio sample rate
		int fftSize = 4096;           // FFT size
		double detectionTolerance = 50.0; // Frequency tolerance (Hz)
};
```

## Design Decisions

### Why Start with Single Tone?

1. **Simpler debugging** - Easier to verify frequency detection is working
2. **Better tolerance** - More forgiving with frequency accuracy
3. **Baseline performance** - Establish reliable communication before scaling up
4. **Troubleshooting** - Can isolate issues with the detector before multi-tone complexity

### Future Multi-Tone Extension

Once single-tone communication is reliable, you can extend to 4 simultaneous tones:
- **Tone 1**: 5000-8000 Hz (4 bits)
- **Tone 2**: 8000-12000 Hz (4 bits)
- **Tone 3**: 12000-16000 Hz (4 bits)
- **Tone 4**: 16000-20000 Hz (4 bits)

This would give you **16 bits** (0-65535) of data per "chord".

## Troubleshooting

### Receiver not detecting frequencies

1. **Check audio input device**:
	 - Ensure microphone is working
	 - Check system audio settings
	 - Test with: `./receiver 5` while playing audio

2. **Adjust detection tolerance**:
	 - Increase `detectionTolerance` in config (e.g., 100.0 Hz)
	 - Lower frequencies may work better in noisy environments

3. **Check frequency range**:
	 - Some speakers can't reproduce high frequencies well
	 - Try lower frequency range (e.g., 2000-5000 Hz)

### Poor detection accuracy

1. **Environmental noise**:
	 - Use bandpass filter (already enabled)
	 - Reduce ambient noise
	 - Increase transmitter volume

2. **FFT settings**:
	 - Larger FFT size = better frequency resolution but slower
	 - Try `fftSize = 8192` for more accuracy

3. **Timing**:
	 - Increase `toneDuration` for more reliable detection
	 - Ensure receiver is running before transmitter starts

## API Reference

### Encoder

```cpp
AudioComm::SingleToneEncoder encoder;
double freq = encoder.encodeValue(7);  // Returns 6400.0 Hz
```

### Decoder

```cpp
AudioComm::SingleToneDecoder decoder;
int value = decoder.decodeFrequency(6400.0);  // Returns 7
if (value >= 0) {
		// Valid value detected
}
```

## Performance

- **Detection latency**: ~50-100ms (depends on update rate)
- **Accuracy**: ±50 Hz (configurable)
- **Data rate**: ~5 values/second (with 0.2s tones)
- **Range**: Limited by speaker/microphone capabilities (typically 1-5 meters)

## Next Steps

1. **Test reliability**: Run extended tests to measure accuracy
2. **Optimize parameters**: Adjust frequency range, tone duration for your environment
3. **Add error detection**: Implement checksums or parity bits
4. **Multi-tone support**: Extend to 4-tone chords for 16-bit communication
5. **Packet protocol**: Add start/stop markers for multi-byte messages
