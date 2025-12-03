# Multi-Tone Chord Communication

Transmit 16-bit values (0-65535) using 4 simultaneous audio tones.

## Overview

The chord system plays **4 tones simultaneously** (a "chord"), with each tone carrying 4 bits of data:

| Tone | Frequency Range | Bits | Purpose |
|------|----------------|------|---------|
| Tone 1 | 5000-8000 Hz | 0-3 | LSB (least significant) |
| Tone 2 | 8500-11500 Hz | 4-7 | |
| Tone 3 | 12000-15000 Hz | 8-11 | |
| Tone 4 | 15500-18500 Hz | 12-15 | MSB (most significant) |

**Total capacity**: 4 tones × 4 bits = **16 bits** (0-65535)

## Quick Start

### Build
```bash
cd SRC
make audio_comm
```

### Test Chord Communication

**Terminal 1** - Start receiver:
```bash
cd BUILD
./chord_receiver
```

**Terminal 2** - Send a value:
```bash
cd BUILD
./chord_transmitter 1234
```

The receiver should detect all 4 tones and decode the 16-bit value!

## Usage Examples

### Chord Transmitter

Send a decimal value:
```bash
./chord_transmitter 1234
```

Send a hex value (convert first):
```bash
./chord_transmitter 0x04D2  # 1234 in decimal
./chord_transmitter 4818    # Same thing
```

Send maximum value:
```bash
./chord_transmitter 65535   # 0xFFFF
```

Send minimum value:
```bash
./chord_transmitter 0
```

### Chord Receiver

Listen continuously:
```bash
./chord_receiver
```

Listen for 10 seconds:
```bash
./chord_receiver 10
```

## Example Output

### Transmitter sends 1234:
```
=== Audio Communication Chord Transmitter ===
Encoding: 4 tones x 4 bits = 16 bits per chord
Tone 1: 5000-8000 Hz
Tone 2: 8500-11500 Hz
Tone 3: 12000-15000 Hz
Tone 4: 15500-18500 Hz

Value breakdown for 1234 (0x04D2):
Binary: 0000 0100 1101 0010

Tone 1 (LSB): bits  0- 3 =  2 (0x2) => 5400.0 Hz
Tone 2: bits  4- 7 = 13 (0xD) => 11100.0 Hz
Tone 3: bits  8-11 =  4 (0x4) => 12800.0 Hz
Tone 4 (MSB): bits 12-15 =  0 (0x0) => 15500.0 Hz

Transmitting chord (Press Ctrl+C to stop)...
Frequencies: 5400.0 Hz, 11100.0 Hz, 12800.0 Hz, 15500.0 Hz
```

### Receiver detects:
```
=== CHORD DETECTED ===
Frequencies: 5400.0 Hz, 11100.0 Hz, 12800.0 Hz, 15500.0 Hz
Tone 1 (bits 0-3): 2 (0x2)
Tone 2 (bits 4-7): 13 (0xD)
Tone 3 (bits 8-11): 4 (0x4)
Tone 4 (bits 12-15): 0 (0x0)

Decoded Value: 1234 (0x04D2)
Binary: 0000 0100 1101 0010
```

## How It Works

### Encoding (Transmitter)

1. Take a 16-bit value (e.g., 1234)
2. Split into 4 nibbles (4-bit chunks):
	 - Bits 0-3: 2
	 - Bits 4-7: 13
	 - Bits 8-11: 4
	 - Bits 12-15: 0
3. Map each nibble to a frequency in its range:
	 - Tone 1: 2 → 5400 Hz
	 - Tone 2: 13 → 11100 Hz
	 - Tone 3: 4 → 12800 Hz
	 - Tone 4: 0 → 15500 Hz
4. Play all 4 frequencies simultaneously

### Decoding (Receiver)

1. Perform FFT analysis on incoming audio
2. Identify peaks in each of the 4 frequency ranges
3. Decode each frequency to a 4-bit value
4. Combine the 4 nibbles into a 16-bit value

## Frequency Mapping Details

Each tone range is divided into 16 equal steps (200 Hz for most ranges):

### Tone 1: 5000-8000 Hz (200 Hz steps)
```
Value 0 = 5000 Hz    Value 8 = 6600 Hz
Value 1 = 5200 Hz    Value 9 = 6800 Hz
Value 2 = 5400 Hz    Value 10 = 7000 Hz
...                  ...
Value 15 = 8000 Hz
```

### Tone 2: 8500-11500 Hz (200 Hz steps)
### Tone 3: 12000-15000 Hz (200 Hz steps)
### Tone 4: 15500-18500 Hz (200 Hz steps)

## Configuration

Modify `ChordConfig` in `audio_comm.h`:

```cpp
struct ChordConfig {
		// Adjust frequency ranges if needed
		double tone1MinFreq = 5000.0;
		double tone1MaxFreq = 8000.0;
		// ... etc for other tones

		int bitsPerTone = 4;              // Don't change without updating code
		double toneDuration = 0.0;        // 0 = infinite (for testing)
		double fadeTime = 0.05;           // Smooth transitions
		double sampleRate = 48000.0;      // Audio sample rate
		int fftSize = 8192;               // Larger = better resolution
		double detectionTolerance = 100.0; // Hz tolerance for matching
};
```

## Troubleshooting

### Receiver doesn't detect all 4 tones

1. **Check frequency range**: Some speakers can't produce high frequencies (>15 kHz)
	 - Try lowering all frequency ranges
	 - Or use only 2-3 tones for testing

2. **Increase FFT size**: Better frequency resolution
	 ```cpp
	 int fftSize = 16384;  // Higher resolution, slower
	 ```

3. **Check speaker/microphone**:
	 - Ensure speaker can produce high frequencies
	 - Test with single tone first: `./transmitter 7`

### Only detecting some tones

- **Volume**: Increase speaker volume
- **Tone spacing**: Ensure 500+ Hz gap between tone ranges
- **Environment**: Reduce background noise

### False detections

- **Increase tolerance**: `detectionTolerance = 150.0`
- **Bandpass filter**: Already enabled, covers 4500-19000 Hz
- **Require all 4 tones**: Decoder only outputs when all 4 are present

## Performance

- **Data rate**: 16 bits per chord
- **Detection latency**: 100-200 ms
- **Accuracy**: Depends on environment and hardware
- **Range**: 1-5 meters (typical)

## Advantages of Chord vs Single Tone

1. **16x more data**: 16 bits vs 4 bits
2. **Same time**: All tones transmitted simultaneously
3. **Error detection**: Missing tone = invalid chord
4. **Robustness**: Noise in one frequency range doesn't destroy entire message

## Scaling to More Bits

Want even more data? You could:

1. **Use more tones**: 8 tones × 4 bits = 32 bits
2. **More bits per tone**: 5 bits per tone = 32 values (needs precise detection)
3. **Add time dimension**: Send multiple chords in sequence

## Next Steps

1. **Test in your environment**: See how high frequencies work
2. **Adjust ranges**: Optimize for your speaker/microphone
3. **Add protocol**: Start/stop markers, checksums, packets
4. **Robot integration**: Use for command & control

## Comparison: Single Tone vs Chord

| Feature | Single Tone | Chord (4 tones) |
|---------|-------------|-----------------|
| Data capacity | 4 bits (0-15) | 16 bits (0-65535) |
| Frequency range | 5-8 kHz | 5-18.5 kHz |
| Complexity | Simple | Moderate |
| Hardware requirements | Basic speaker/mic | Good quality needed |
| Debugging | Easy | Harder |
| Use case | Simple commands | Complex data |

Start with single tone to verify your setup, then move to chords for more data!
