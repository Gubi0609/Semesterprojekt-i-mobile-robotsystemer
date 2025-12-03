# Voice Command Receiver Guide

## Overview

The voice receiver enables control of the robot through natural voice commands using the Vosk speech recognition library. It runs on the Raspberry Pi and can operate in two modes:

- **TEST_MODE**: Runs without ROS2, prints commands to terminal (for development)
- **ROS2 MODE**: Publishes velocity commands to `/cmd_vel` topic (for production)

## Features

- **Offline Speech Recognition**: Uses Vosk for local, lightweight STT
- **Continuous & Timed Modes**: Commands can stay active until "stop" or run for 2 seconds
- **Standard Robot Commands**: Forward, backward, left, right, spin, stop
- **Velocity Limits**: Matches rb3_node_cpp controller (0.22 m/s linear, 2.84 rad/s angular)

## Installation

### 1. Install Vosk Library and Model

Run the provided setup script on the Raspberry Pi:

```bash
cd src
chmod +x setup_vosk.sh
./setup_vosk.sh
```

This will:
- Install PortAudio (for microphone input)
- Download and install Vosk library (ARM or x86_64)
- Download English model (~40MB)
- Install to standard locations

The model will be installed to: `~/vosk-models/vosk-model-small-en-us-0.15`

### 2. Build Voice Receiver

From the `SRC` directory:

```bash
make voice_receiver
```

This compiles with Vosk and PortAudio libraries linked.

## Configuration

### Compile-Time Flags

Edit `src/voice_receiver.cpp` to configure:

```cpp
// Run without ROS2 (test mode - prints to terminal)
#define TEST_MODE true

// Use keyboard input instead of microphone (for parser testing only)
#define USE_KEYBOARD_INPUT false
```

### Model Path

The program looks for the Vosk model in this order:
1. `VOSK_MODEL_PATH` environment variable
2. `~/vosk-models/vosk-model-small-en-us-0.15` (default from setup script)
3. `vosk-model-small-en-us-0.15` (current directory)

To use a custom model:

```bash
export VOSK_MODEL_PATH="/path/to/your/model"
```

### Command Modes

The system supports two execution modes:

- **CONTINUOUS**: Command stays active until you say "stop"
- **TIMED**: Command runs for 2 seconds then automatically stops

Default mode is CONTINUOUS. Mode can be toggled in keyboard test mode by typing "mode".

### Velocity Configuration

Default velocities (defined in code):

```cpp
struct VelocityConfig {
		float max_linear = 0.22f;       // m/s (max forward/backward)
		float max_angular = 2.84f;      // rad/s (max turning)
		float default_linear = 0.15f;   // Normal forward/backward speed
		float default_angular = 1.5f;   // Normal turning speed
};
```

## Usage

### Test Mode (No ROS2)

Test the voice recognition without ROS2:

```bash
cd BUILD
./voice_receiver
```

The program will:
1. Load the Vosk model
2. Open the microphone
3. Listen for voice commands
4. Print recognized commands and parsed states to terminal

Press Ctrl+C to exit.

### Keyboard Test Mode

For testing the command parser without a microphone, enable keyboard input:

1. Set `USE_KEYBOARD_INPUT true` in voice_receiver.cpp
2. Rebuild: `make voice_receiver`
3. Run and type commands

### ROS2 Mode (Production)

1. Set `TEST_MODE false` in voice_receiver.cpp
2. Rebuild: `make voice_receiver`
3. Run on the robot:

```bash
cd BUILD
./voice_receiver
```

Commands will be published to `/cmd_vel` as `geometry_msgs/TwistStamped` messages.

## Voice Commands

### Supported Commands

| Voice Command | Robot Action | State |
|--------------|--------------|-------|
| "move forward", "go", "ahead" | Move forward at default speed | MOVING_FORWARD |
| "move backward", "back", "reverse" | Move backward at default speed | MOVING_BACKWARD |
| "turn left", "left" | Turn left in place | TURNING_LEFT |
| "turn right", "right" | Turn right in place | TURNING_RIGHT |
| "spin", "rotate" | Spin at max angular velocity | SPINNING |
| "stop", "halt", "idle" | Stop all movement | IDLE |

### Command Examples

**Natural phrases that work:**
- "go forward"
- "turn to the left"
- "spin around"
- "please stop"
- "move back"

The parser extracts keywords from your speech, so many natural variations work.

### Command Behavior

**CONTINUOUS Mode:**
- Say "go forward" → Robot moves forward continuously
- Say "stop" → Robot stops
- Say "turn left" → Robot turns left continuously
- Say "stop" → Robot stops

**TIMED Mode:**
- Say "go forward" → Robot moves forward for 2 seconds then stops
- Say "turn left" → Robot turns left for 2 seconds then stops
- No need to say "stop" (but you can for immediate stop)

## Troubleshooting

### "Failed to load Vosk model"

**Solution:** Make sure the model is installed and path is correct:

```bash
ls ~/vosk-models/vosk-model-small-en-us-0.15
```

If not found, run `setup_vosk.sh` again.

### "No default input device found"

**Solution:** Check that a microphone is connected:

```bash
arecord -l
```

Test microphone:

```bash
arecord -d 5 test.wav
aplay test.wav
```

### "Failed to initialize PortAudio"

**Solution:** Install PortAudio:

```bash
sudo apt-get install libportaudio2 portaudio19-dev
```

### "Can't read from a callback stream"

This error occurred in the initial implementation and has been fixed. The stream is now configured for blocking reads instead of callback mode. If you see this error, make sure you have the latest version of voice_receiver.cpp.

### Recognition Not Working

**Check microphone levels:**

```bash
alsamixer
```

Press F4 to select capture device and adjust gain.

**Test Vosk directly:** The program prints recognized text before parsing. If you see empty text or no output, the microphone or model may need adjustment.

**Try the keyboard test mode:** Set `USE_KEYBOARD_INPUT true` to verify the command parser works independently.

### Compilation Errors

**Missing vosk_api.h:**

```bash
ls /usr/local/include/vosk/vosk_api.h
```

If not found, re-run setup script or manually install Vosk.

**Undefined reference to vosk_model_new:**

Check that `-lvosk` is in the link command. Verify libvosk.so exists:

```bash
ls /usr/local/lib/libvosk.so
sudo ldconfig
```

## Architecture

### Program Flow

```
1. Initialize Vosk model and recognizer
2. Initialize PortAudio and open microphone stream (blocking mode)
3. Main loop:
	 - Read audio from microphone (1024 frames)
	 - Feed audio to Vosk recognizer
	 - On complete utterance:
		 * Parse JSON result to extract text
		 * Parse text to determine robot state
		 * Execute state (publish velocity commands)
	 - Check for timed command expiration
	 - Continue publishing in CONTINUOUS mode
4. Cleanup on exit
```

### Key Components

**VoskModel**: Loaded once at startup, contains language model
**VoskRecognizer**: Processes audio stream, outputs recognized text
**PortAudio**: Cross-platform audio I/O library (blocking read mode)
**RobotCommander**: Publishes velocity commands or prints to terminal
**Command Parser**: Converts text to robot states using keyword matching

### Integration with ROS2

When `TEST_MODE false`, the RobotCommander publishes:

```cpp
geometry_msgs::msg::TwistStamped msg;
msg.header.stamp = this->get_clock()->now();
msg.header.frame_id = "base_link";
msg.twist.linear.x = linear_x;   // Forward/backward
msg.twist.angular.z = angular_z;  // Left/right rotation
```

Published to topic: `/cmd_vel`

This matches the interface expected by `rb3_node_cpp.cpp`.

## Extending the System

### Adding New Commands

Edit `parseCommand()` function in `voice_receiver.cpp`:

```cpp
RobotState parseCommand(const string& command) {
		string cmd = command;
		for (auto& c : cmd) c = tolower(c);

		// Add your new command here
		if (cmd.find("dance") != string::npos) {
				return RobotState::DANCING;  // Add new state to enum
		}

		// ... existing commands
}
```

Then add the new state to the enum and handle it in `executeState()`.

### Changing Velocities

Modify the `executeState()` function in RobotCommander class:

```cpp
case RobotState::MOVING_FORWARD:
		linear_x = 0.20f;  // Faster forward speed
		angular_z = 0.0f;
		break;
```

### Adding Voice Feedback

You could add audio feedback using `espeak` or similar:

```cpp
void speakResponse(const string& text) {
		system(("espeak \"" + text + "\" 2>/dev/null &").c_str());
}

// In main loop after recognizing command:
speakResponse("Moving forward");
```

## Performance

**Model Size:** ~40MB (small English model)
**Recognition Latency:** ~200-500ms typical
**CPU Usage:** ~10-15% on Raspberry Pi 4
**Memory:** ~100MB

The small English model provides good accuracy for robot control commands while staying lightweight enough for the Pi.

## Development Notes

This voice control system is separate from the CRC chord-based control system. Both can coexist:

- **Chord System** (`receiver.cpp`): Uses audio chords with CRC encoding for reliable transmission
- **Voice System** (`voice_receiver.cpp`): Uses natural speech recognition for intuitive control

Each has its own executable and can be used independently based on your needs.

## See Also

- [Vosk Documentation](https://alphacephei.com/vosk/)
- [PortAudio Documentation](http://www.portaudio.com/)
- `rb3_node_cpp.cpp` - Robot controller node
- `receiver.cpp` - Chord-based command receiver
- `CRC_USAGE_GUIDE.md` - CRC chord transmission system
- `INDEX.md` - Documentation index
