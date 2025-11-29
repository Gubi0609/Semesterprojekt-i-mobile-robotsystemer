# CRC-Encoded Chord Communication - Usage Guide

Quick guide for using the CRC-encoded sender/receiver system for robot control.

---

## Overview

The system uses CRC-4 error detection to transmit reliable 12-bit commands via audio chords:
- **Sender** (SRC/main.cpp): Encodes and transmits commands
- **Receiver** (src/receiver.cpp): Receives, verifies, and processes commands on the Pi

**Flow:** `12-bit command → CRC encode → 16-bit chord → Transmit → Receive → CRC verify → Extract 12-bit command`

---

## Quick Start

### Building

```bash
cd SRC
make sender    # Build sender
make receiver  # Build receiver for Pi
make all       # Build both
```

### Running

**Sender** (transmit a command):
```bash
cd BUILD
./sender 2047
```
- Takes a 12-bit value (0-4095)
- Automatically adds CRC checksum
- Transmits as 16-bit chord

**Receiver** (listen for commands):
```bash
cd BUILD
./receiver
```
- Continuously listens for chords
- Verifies CRC on each received command
- Rejects corrupted transmissions
- Press Ctrl+C to stop

---

## Example Session

**Terminal 1 (Receiver on Pi):**
```bash
$ ./receiver
=== CRC-Decoding Chord Receiver ===
Robot Control System

Initial robot state: IDLE

Starting receiver...
Listening for CRC-encoded commands...
Press Ctrl+C to stop
============================================================
```

**Terminal 2 (Sender):**
```bash
$ ./sender 2047
=== CRC-Encoded Chord Transmitter ===

Original data (12-bit): 2047 (0x7FF)
CRC encoded (16-bit): 32767 (0x7FFF)
CRC checksum (4-bit): 15 (0xF)

Transmitting CRC-encoded value...
Transmission complete!
```

**Terminal 1 (Receiver output):**
```
>>> CHORD RECEIVED <<<
Timestamp: 2.15s
Raw value: 32767 (0x7FFF)
Detection count: 12
Frequencies: 1700.0 Hz, 2600.0 Hz, 3500.0 Hz, 4400.0 Hz

--- CRC VERIFICATION ---
✓ CRC CHECK PASSED - Valid command received
Decoded command: 2047 (0x7FF)

--- COMMAND PROCESSING ---
Current state: IDLE
Action: Command acknowledged (IDLE state)
	Command value: 2047
	(Robot ready for future state implementations)

============================================================
Listening for next command...
```

---

## File Organization

```
SRC/                    # Sender files (uppercase)
	main.cpp              # Sender program
	Makefile              # Build configuration

src/                    # Receiver files for Pi (lowercase)
	receiver.cpp          # Receiver program
	rb3_node_cpp.cpp      # Other Pi code

BUILD/                  # Compiled binaries
	sender                # Sender executable
	receiver              # Receiver executable

LIB/examples/           # Communication layer tests
	chord_to_bits.cpp     # Full transmission test
	onlytones.cpp         # Tone generation only
	onlydetection.cpp     # Detection only
	crc_chord_transmission.cpp  # Integrated CRC test
```

---

## Testing Communication Layers

Test individual components before running the full system:

### Test 1: Tone Generation
```bash
make onlytones
cd ../BUILD && ./onlytones
```
Verifies audio output works.

### Test 2: Frequency Detection
```bash
make onlydetection
cd ../BUILD && ./onlydetection
```
Verifies audio input and FFT detection works.

### Test 3: Full Chord Test
```bash
make chord_to_bits
cd ../BUILD && ./chord_to_bits
```
Tests transmission + reception in one process.

### Test 4: CRC Integration Test
```bash
make crc_full_test
cd ../BUILD && ./crc_full_test
```
Tests CRC encoding, transmission, reception, and verification.

---

## Command Format

### Valid Range
- **Input:** 0 to 4095 (12-bit unsigned integer)
- **Transmitted:** 0 to 65535 (16-bit with CRC)

### CRC Protection
- **Data bits:** 12 bits (your command)
- **CRC bits:** 4 bits (error detection)
- **Generator polynomial:** `{1, 0, 0, 1, 1}` (default)

The CRC can detect:
- Single-bit errors
- Most multi-bit errors
- All burst errors up to 4 bits

---

## Robot State (Expandable)

The receiver currently implements an `IDLE` state. To add more states:

1. **Edit enum in receiver.cpp:**
```cpp
enum class RobotState {
		IDLE,
		MOVING_FORWARD,
		MOVING_BACKWARD,
		TURNING_LEFT,
		TURNING_RIGHT
		// Add more as needed
};
```

2. **Add command definitions:**
```cpp
const uint16_t CMD_MOVE_FORWARD = 1;
const uint16_t CMD_MOVE_BACKWARD = 2;
const uint16_t CMD_TURN_LEFT = 3;
const uint16_t CMD_TURN_RIGHT = 4;
```

3. **Process commands in IDLE state:**
```cpp
case RobotState::IDLE:
		if (decodedData.value() == CMD_MOVE_FORWARD) {
				currentState = RobotState::MOVING_FORWARD;
				// Start motors
		}
		break;
```

---

## Error Handling

### CRC Check Failed
```
✗ CRC CHECK FAILED - Command rejected!
	Error detected in transmission.
	Command ignored for safety.
```
**Action:** Command is discarded. Sender should retransmit.

### Invalid Input Range
```bash
$ ./sender 5000
Error: Value must be between 0 and 4095 (12-bit)
	Received: 5000
```
**Action:** Fix input value to be within 0-4095.

### No Audio Detection
If receiver shows no output:
1. Check microphone permissions
2. Verify audio devices: `arecord -l`
3. Test with simpler examples first (see Testing section)

---

## Makefile Targets

```bash
make sender          # Build sender only
make receiver        # Build receiver only
make all             # Build both (default)
make examples        # Build all test examples
make libs            # Build library objects only
make clean           # Remove all built files
make help            # Show all targets
```

---

## Additional Documentation

For detailed information about the underlying system, see:
- **API Reference:** `README_LIBRARY.md`
- **Chord Mode:** `README_CHORD.md`
- **Audio System:** `README_AUDIO_COMM.md`
- **Main Overview:** `README.md`

---

## Troubleshooting

**Problem:** Receiver doesn't detect sender
- Ensure both use same audio system
- Check volume levels (not too quiet, not clipping)
- Test with `crc_full_test` first (integrated test)

**Problem:** Frequent CRC errors
- Reduce background noise
- Increase transmission duration
- Check for audio device issues

**Problem:** Build errors
- Ensure all dependencies installed: `./install_deps.sh`
- Check that PortAudio and FFTW3 are available
- Verify library object files exist in `BUILD/`

---

**Last Updated:** 2025-11-05
