# Microphone Frequency Response Test Tool

## Purpose
This tool tests which frequencies your microphone can reliably detect by transmitting tones and measuring detection rates.

## Three Usage Modes

### 1. Single Device Mode (Both TX + RX)
Run everything on one device (Raspberry Pi or PC).

**⚠️ WARNING**: May have feedback/echo issues!

**Requirements**:
- External speaker AND external microphone
- Physically separate them by 1+ meters
- Lower the gain if you hear feedback

**Command**:
```bash
./microphone_frequency_sweep 1 0.5 both
```

---

### 2. Split Mode (RECOMMENDED for best results)

Run transmitter on one device and receiver on another.

#### On Device 1 (PC with good speakers):
```bash
./microphone_frequency_sweep 1 0.8 transmit
```
This will play test tones through the speakers.

#### On Device 2 (Raspberry Pi with microphone):
```bash
./microphone_frequency_sweep 1 0.5 receive
```
This will listen and record results.

**Setup**:
1. Start the RECEIVER first, press ENTER when prompted
2. Start the TRANSMITTER, press ENTER when prompted
3. Keep devices ~1 meter apart
4. Results will be saved on the receiver device

---

## Test Modes

### Mode 1: Quick Test (Recommended for first run)
- Tests 16 frequencies from 4.5 kHz to 16 kHz
- Takes ~1 minute
- Good for quick overview

```bash
./microphone_frequency_sweep 1 0.5
```

### Mode 2: Detailed Test
- Tests 47 frequencies (every 250 Hz)
- Takes ~3 minutes
- Better granularity

```bash
./microphone_frequency_sweep 2 0.5
```

### Mode 3: Fine-Grained Test
- Tests 116 frequencies (every 100 Hz)
- Takes ~7 minutes
- Very detailed frequency response

```bash
./microphone_frequency_sweep 3 0.5
```

### Mode 4: Chord Frequency Test
- Tests all 64 exact chord frequencies used in your protocol
- Tests each of the 16 possible values for all 4 tones
- Takes ~4 minutes
- **Most relevant for your actual application!**

```bash
./microphone_frequency_sweep 4 0.5
```

---

## Command Line Arguments

```bash
./microphone_frequency_sweep [mode] [gain] [run_mode]
```

- `mode`: Test mode (1-4, default: 1)
	- 1 = Quick (16 frequencies)
	- 2 = Detailed (47 frequencies)
	- 3 = Fine-grained (116 frequencies)
	- 4 = Chord frequencies (64 frequencies)

- `gain`: Transmitter volume (0.1 to 1.0, default: 0.5)
	- Lower if you hear distortion or feedback
	- Higher if detection rates are too low
	- Start with 0.5 and adjust

- `run_mode`: Operation mode (default: both)
	- `both` = Transmit and receive on same device
	- `transmit` = Only transmit tones
	- `receive` = Only receive and analyze

---

## Example Usage Scenarios

### Scenario 1: Quick check on single Pi
```bash
# Use external speaker + mic, separate by 1m+
./microphone_frequency_sweep 1 0.4 both
```

### Scenario 2: Thorough test with two devices
```bash
# On PC (transmit):
./microphone_frequency_sweep 4 0.8 transmit

# On Pi (receive):
./microphone_frequency_sweep 4 0.5 receive
```

### Scenario 3: Testing after adjusting microphone
```bash
# Quick test to verify improvement
./microphone_frequency_sweep 1 0.5 both
```

---

## Understanding the Results

### Detection Rate
- **≥80%**: ✓ GOOD - Frequency is reliable
- **<80%**: ✗ POOR - Frequency is unreliable

### CSV Output
Results are saved to `frequency_test_results_<timestamp>.csv`:
```csv
Frequency (Hz),Detection Count,Avg Magnitude,Detection Rate (%),Reliable
4500.0,95,0.6234,95.0,YES
5250.0,82,0.5891,82.0,YES
6000.0,45,0.3211,45.0,NO
...
```

### Summary Report
After testing, you'll see:
```
Total frequencies tested: 16
Reliable frequencies (≥80%): 12 (75.0%)
Unreliable frequencies (<80%): 4

Reliable frequency range: 4500 Hz - 14500 Hz

Problem frequency ranges:
	 • 15000 - 16000 Hz  ← Microphone struggles here!
```

---

## Recommendations Based on Results

### If high frequencies (>15 kHz) are unreliable:
```
⚠️  Microphone upper limit is around 15000 Hz
→ Adjust tone4MaxFreq to stay below this limit
→ Update LIB/audio_comm.h, audio_transmitter.h, audio_receiver.h
```

### If low frequencies (<5 kHz) are unreliable:
```
⚠️  Microphone lower limit is around 5000 Hz
→ Adjust tone1MinFreq to stay above this limit
```

### If <70% of frequencies work:
```
⚠️  General microphone issues detected
→ Check physical connections
→ Test different microphone
→ Reduce background noise
→ Adjust transmitter gain
```

---

## Troubleshooting

### "Failed to start detector"
- Microphone not connected or not detected
- Check: `arecord -l` (should show your mic)
- Try: `arecord -d 3 test.wav` (test recording)

### "Failed to start transmitter"
- Speaker not connected or not detected
- Check: `aplay -l` (should show your speaker)
- Try: `speaker-test -t wav -c 2` (test playback)

### Very low detection rates across all frequencies
- Volume too low → increase gain
- Devices too far apart → move closer
- Background noise → find quieter location
- Wrong audio device selected → check ALSA settings

### Feedback/squealing in "both" mode
- Reduce gain: try 0.3 or 0.2
- Increase physical separation
- Use directional microphone
- Consider using split mode instead

---

## Compilation

```bash
cd /path/to/project
# Compile with appropriate flags for your audio library
g++ -o microphone_frequency_sweep \
		SRC/Test\ programs/microphone_frequency_sweep.cpp \
		LIB/audio_transmitter_lib.cpp \
		LIB/audio_receiver_lib.cpp \
		LIB/frequency_detector_lib.cpp \
		-lportaudio -lfftw3 -pthread -std=c++17
```

Adjust linker flags based on your audio backend (PortAudio, ALSA, etc.)

---

## What to do with Results

1. **Run Mode 4** (chord frequency test) first
2. Check which of the 64 chord frequencies are unreliable
3. If any frequencies show <80% detection:
	 - Note the frequency range
	 - Adjust your ChordConfig in the library headers
	 - Rebuild your project
	 - Test again to verify improvement

Example: If Tone 4 frequencies (13500-16000 Hz) are mostly unreliable:
```cpp
// In LIB/audio_comm.h, change:
double tone4MinFreq = 13500.0;
double tone4MaxFreq = 16000.0;

// To something lower:
double tone4MinFreq = 11500.0;
double tone4MaxFreq = 13500.0;
```
