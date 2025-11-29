# Robot Control System Documentation

This folder contains documentation for the robot control systems.
These files should be uploaded to the project wiki for team access.

## Documentation Structure

```
DOCS/
├── INDEX.md (this file)
├── README.md - Main overview
├── SRC_README.md - SRC directory documentation
│
├── guides/               # User guides
│   ├── CRC_USAGE_GUIDE.md
│   ├── LIBRARY_USAGE.md
│   └── VOICE_RECEIVER_GUIDE.md
│
├── api/                  # API documentation
│   ├── README_AUDIO_COMM.md
│   ├── README_CHORD.md
│   └── README_LIBRARY.md
│
└── features/             # Feature documentation
	├── AUTO_RESTART_FEATURE.md
	├── DUPLICATE_DETECTION_LOCKOUT.md
	└── FREQUENCY_TOLERANCE_UPDATE.md
```

## Getting Started

### 1. Quick Start Guides

#### **CRC-Encoded Chord Control** (START HERE!)
📄 `guides/CRC_USAGE_GUIDE.md`
- Quick start for robot control via audio chords
- Build and run instructions
- Command format and error handling
- Testing individual layers

#### **Voice Control System** (NEW!)
📄 `guides/VOICE_RECEIVER_GUIDE.md`
- Voice command recognition using Vosk
- Natural speech control for robot
- Installation and setup instructions
- Supported voice commands
- Troubleshooting

#### **Library Usage**
📄 `guides/LIBRARY_USAGE.md`
- How to use the audio communication libraries
- Integration examples

---

### 2. API Reference

#### **Complete API Documentation**
📄 `api/README_LIBRARY.md`
- All classes and methods
- Configuration options
- Code examples
- Design patterns
- Thread safety

#### **Chord Mode Details**
📄 `api/README_CHORD.md`
- 16-bit communication (4 tones)
- Frequency mapping
- Usage examples
- Configuration guide

#### **Single Tone Details**
📄 `api/README_AUDIO_COMM.md`
- 4-bit communication (1 tone)
- Frequency mapping
- Design decisions
- Future extensions

---

### 3. Feature Documentation

#### **Auto-Restart Feature**
📄 `features/AUTO_RESTART_FEATURE.md`
- Automatic microphone restart on failure
- Implementation details

#### **Duplicate Detection Prevention**
📄 `features/DUPLICATE_DETECTION_LOCKOUT.md`
- Prevents duplicate command execution
- Lockout mechanism

#### **Frequency Tolerance Updates**
📄 `features/FREQUENCY_TOLERANCE_UPDATE.md`
- FFT tolerance improvements
- Detection accuracy enhancements

---

## System Overview

The robot supports two independent control modes:

### Voice Control System
- Natural speech recognition using Vosk STT
- Intuitive commands like "move forward", "turn left", "stop"
- Runs directly on Raspberry Pi with microphone
- See: `guides/VOICE_RECEIVER_GUIDE.md`

### Chord Control System
- CRC-encoded audio chord transmission
- Reliable error detection and correction
- Sends commands from sender device to robot
- See: `guides/CRC_USAGE_GUIDE.md`

Both systems publish to the same `/cmd_vel` topic and can be used independently.

---

## Suggested Wiki Structure

When uploading to wiki, create these sections:

```
Home
├── Getting Started
│   ├── Quick Start (CRC_USAGE_GUIDE.md)
│   ├── Voice Control Guide (VOICE_RECEIVER_GUIDE.md)
│   └── Library Usage (LIBRARY_USAGE.md)
│
├── API Reference
│   ├── Complete API (README_LIBRARY.md)
│   ├── Chord Mode (README_CHORD.md)
│   └── Single Tone (README_AUDIO_COMM.md)
│
└── Features
	├── Auto-Restart
	├── Duplicate Detection
	└── Frequency Tolerance
```

---

## Quick Links for Your Team

1. **NEW: Voice Control** → `guides/VOICE_RECEIVER_GUIDE.md`
2. **Quick Start: Chord Control** → `guides/CRC_USAGE_GUIDE.md`
3. **Getting Started** → `README.md`
4. **Using the Library** → `guides/LIBRARY_USAGE.md`
5. **API Reference** → `api/README_LIBRARY.md`
6. **Advanced: Chord Mode** → `api/README_CHORD.md`
7. **Advanced: Single Tone** → `api/README_AUDIO_COMM.md`

---

## Keeping Docs in Sync

When updating the system:
1. Edit markdown files in this DOCS folder
2. Test changes locally
3. Upload updated files to wiki
4. Update wiki's last modified date
5. Update INDEX.md if structure changes

---

## Notes

⚠️ Some documentation may be outdated. Review and update as needed when making significant changes to the codebase.
