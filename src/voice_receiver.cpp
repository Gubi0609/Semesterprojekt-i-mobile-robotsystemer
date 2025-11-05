#include <iostream>
#include <iomanip>
#include <string>
#include <thread>
#include <chrono>
#include <atomic>
#include <csignal>
#include <cstring>

// Vosk includes
#include "vosk/vosk_api.h"

// PortAudio includes (for microphone input)
#include <portaudio.h>

// ROS2 includes (commented out for test mode)
// #include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/twist_stamped.hpp"

using namespace std;

// ============================================================================
// CONFIGURATION
// ============================================================================

// Set to true to run without ROS2 (test mode - prints to terminal)
#define TEST_MODE true

// Set to true to use keyboard input instead of microphone (for testing parser only)
#define USE_KEYBOARD_INPUT false

// Command execution modes
enum class CommandMode {
	CONTINUOUS,  // Command stays active until "stop"
	TIMED        // Command runs for fixed duration then stops
};

// Robot states
enum class RobotState {
	IDLE,
	MOVING_FORWARD,
	MOVING_BACKWARD,
	TURNING_LEFT,
	TURNING_RIGHT,
	SPINNING
};

// Robot velocity configuration (matching rb3_node_cpp.cpp limits)
struct VelocityConfig {
	float max_linear = 0.22f;   // m/s (forward/backward)
	float max_angular = 2.84f;  // rad/s (turning)
	float default_linear = 0.15f;
	float default_angular = 1.5f;
};

// Command duration for TIMED mode (seconds)
const float TIMED_COMMAND_DURATION = 2.0f;

// ============================================================================
// GLOBALS
// ============================================================================

atomic<bool> running(true);
atomic<RobotState> currentState(RobotState::IDLE);
CommandMode commandMode = CommandMode::CONTINUOUS;
VelocityConfig velocityConfig;

// ============================================================================
// SIGNAL HANDLER
// ============================================================================

void signalHandler(int signum) {
	cout << "\n\nReceived interrupt signal (" << signum << ")\n";
	cout << "Shutting down voice receiver...\n";
	running = false;
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

const char* stateToString(RobotState state) {
	switch(state) {
		case RobotState::IDLE: return "IDLE";
		case RobotState::MOVING_FORWARD: return "MOVING_FORWARD";
		case RobotState::MOVING_BACKWARD: return "MOVING_BACKWARD";
		case RobotState::TURNING_LEFT: return "TURNING_LEFT";
		case RobotState::TURNING_RIGHT: return "TURNING_RIGHT";
		case RobotState::SPINNING: return "SPINNING";
		default: return "UNKNOWN";
	}
}

const char* modeToString(CommandMode mode) {
	return mode == CommandMode::CONTINUOUS ? "CONTINUOUS" : "TIMED";
}

// ============================================================================
// ROBOT COMMAND PUBLISHER
// ============================================================================

class RobotCommander {
public:
	RobotCommander() {
#if TEST_MODE
		cout << ">>> TEST MODE: Commands will be printed to terminal <<<\n\n";
#else
		// Initialize ROS2 node here
		cout << ">>> ROS2 MODE: Commands will be published to /cmd_vel <<<\n\n";
#endif
	}

	void publishCommand(float linear_x, float angular_z) {
#if TEST_MODE
		// Test mode - print to terminal
		cout << "  [CMD] linear.x: " << fixed << setprecision(2) << linear_x
			 << " m/s, angular.z: " << angular_z << " rad/s\n";
#else
		// ROS2 mode - publish to /cmd_vel
		// geometry_msgs::msg::TwistStamped msg;
		// msg.header.stamp = this->get_clock()->now();
		// msg.header.frame_id = "base_link";
		// msg.twist.linear.x = linear_x;
		// msg.twist.linear.y = 0;
		// msg.twist.linear.z = 0;
		// msg.twist.angular.x = 0;
		// msg.twist.angular.y = 0;
		// msg.twist.angular.z = angular_z;
		// publisher_->publish(msg);
#endif
	}

	void executeState(RobotState state) {
		float linear_x = 0.0f;
		float angular_z = 0.0f;

		switch(state) {
			case RobotState::IDLE:
				linear_x = 0.0f;
				angular_z = 0.0f;
				break;

			case RobotState::MOVING_FORWARD:
				linear_x = velocityConfig.default_linear;
				angular_z = 0.0f;
				break;

			case RobotState::MOVING_BACKWARD:
				linear_x = -velocityConfig.default_linear;
				angular_z = 0.0f;
				break;

			case RobotState::TURNING_LEFT:
				linear_x = 0.0f;
				angular_z = velocityConfig.default_angular;
				break;

			case RobotState::TURNING_RIGHT:
				linear_x = 0.0f;
				angular_z = -velocityConfig.default_angular;
				break;

			case RobotState::SPINNING:
				linear_x = 0.0f;
				angular_z = velocityConfig.max_angular;
				break;
		}

		publishCommand(linear_x, angular_z);
	}

	void stop() {
		publishCommand(0.0f, 0.0f);
	}

private:
	// ROS2 publisher (for when TEST_MODE is false)
	// rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
};

// ============================================================================
// VOICE COMMAND PARSER
// ============================================================================

// Helper function to check if a word exists as a complete word (not substring)
bool containsWord(const string& text, const string& word) {
	size_t pos = 0;
	while ((pos = text.find(word, pos)) != string::npos) {
		// Check if character before is not alphanumeric (or we're at start)
		bool startOk = (pos == 0 || !isalnum(text[pos-1]));
		// Check if character after is not alphanumeric (or we're at end)
		bool endOk = (pos + word.length() == text.length() ||
					  !isalnum(text[pos + word.length()]));

		if (startOk && endOk) {
			return true;
		}
		pos++;
	}
	return false;
}

RobotState parseCommand(const string& command) {
	// Convert to lowercase for easier matching
	string cmd = command;
	for (auto& c : cmd) c = tolower(c);

	// Parse commands using word boundary matching
	if (containsWord(cmd, "forward") ||
		containsWord(cmd, "ahead") ||
		containsWord(cmd, "go")) {
		return RobotState::MOVING_FORWARD;
	}
	else if (containsWord(cmd, "backward") ||
			 containsWord(cmd, "back") ||
			 containsWord(cmd, "reverse")) {
		return RobotState::MOVING_BACKWARD;
	}
	else if (containsWord(cmd, "left")) {
		return RobotState::TURNING_LEFT;
	}
	else if (containsWord(cmd, "right")) {
		return RobotState::TURNING_RIGHT;
	}
	else if (containsWord(cmd, "spin") ||
			 containsWord(cmd, "rotate")) {
		return RobotState::SPINNING;
	}
	else if (containsWord(cmd, "stop") ||
			 containsWord(cmd, "halt") ||
			 containsWord(cmd, "idle")) {
		return RobotState::IDLE;
	}

	return RobotState::IDLE; // Default to idle for unknown commands
}

// ============================================================================
// SIMULATED VOICE INPUT (for testing without Vosk)
// ============================================================================

void simulatedVoiceInput() {
	cout << "\n=== SIMULATED VOICE INPUT MODE ===\n";
	cout << "Type commands to test the system:\n";
	cout << "  - 'move forward' / 'go'\n";
	cout << "  - 'move backward' / 'back'\n";
	cout << "  - 'turn left'\n";
	cout << "  - 'turn right'\n";
	cout << "  - 'spin'\n";
	cout << "  - 'stop'\n";
	cout << "  - 'mode' (toggle CONTINUOUS/TIMED)\n";
	cout << "  - 'quit' (exit)\n";
	cout << "\nCurrent mode: " << modeToString(commandMode) << "\n";
	cout << string(60, '=') << "\n\n";

	RobotCommander commander;
	auto lastCommandTime = chrono::steady_clock::now();

	while (running) {
		// Check if we need to stop in TIMED mode
		if (commandMode == CommandMode::TIMED && currentState != RobotState::IDLE) {
			auto now = chrono::steady_clock::now();
			auto elapsed = chrono::duration<float>(now - lastCommandTime).count();

			if (elapsed >= TIMED_COMMAND_DURATION) {
				cout << "\n[TIMED MODE] Command duration expired, stopping...\n";
				currentState = RobotState::IDLE;
				commander.stop();
				cout << "State: " << stateToString(currentState) << "\n\n";
			}
		}

		// Non-blocking input check
		cout << "> ";
		string input;
		getline(cin, input);

		if (!running) break;

		if (input.empty()) continue;

		// Check for mode toggle
		if (input == "mode") {
			commandMode = (commandMode == CommandMode::CONTINUOUS)
						  ? CommandMode::TIMED
						  : CommandMode::CONTINUOUS;
			cout << "\n>>> Switched to " << modeToString(commandMode) << " mode <<<\n\n";
			continue;
		}

		// Check for quit
		if (input == "quit" || input == "exit") {
			running = false;
			break;
		}

		// Parse and execute command
		RobotState newState = parseCommand(input);

		if (newState != currentState || newState != RobotState::IDLE) {
			currentState = newState;
			lastCommandTime = chrono::steady_clock::now();

			cout << "\n>>> VOICE COMMAND RECOGNIZED <<<\n";
			cout << "Command: \"" << input << "\"\n";
			cout << "Parsed as: " << stateToString(currentState) << "\n";
			cout << "Mode: " << modeToString(commandMode) << "\n";

			if (commandMode == CommandMode::TIMED && currentState != RobotState::IDLE) {
				cout << "Duration: " << TIMED_COMMAND_DURATION << " seconds\n";
			}

			commander.executeState(currentState);
			cout << string(60, '=') << "\n\n";
		}

		// In CONTINUOUS mode, continuously publish current state
		if (commandMode == CommandMode::CONTINUOUS && currentState != RobotState::IDLE) {
			this_thread::sleep_for(chrono::milliseconds(100));
			commander.executeState(currentState);
		}
	}
}

// ============================================================================
// VOSK VOICE RECOGNITION
// ============================================================================

void voskVoiceInput() {
	cout << "\n=== VOSK VOICE RECOGNITION MODE ===\n";
	cout << "Initializing Vosk speech recognition...\n";

	// Determine model path - check environment variable or use default
	const char* model_path = getenv("VOSK_MODEL_PATH");
	if (model_path == nullptr) {
		// Try default location
		const char* home = getenv("HOME");
		static string default_path;
		if (home != nullptr) {
			default_path = string(home) + "/vosk-models/vosk-model-small-en-us-0.15";
			model_path = default_path.c_str();
		} else {
			model_path = "vosk-model-small-en-us-0.15";
		}
	}

	cout << "Loading model from: " << model_path << "\n";

	// Initialize Vosk model
	VoskModel *model = vosk_model_new(model_path);
	if (model == nullptr) {
		cerr << "ERROR: Failed to load Vosk model!\n";
		cerr << "Make sure the model exists at: " << model_path << "\n";
		cerr << "Run setup_vosk.sh to install the model.\n";
		cerr << "Or set VOSK_MODEL_PATH environment variable.\n";
		return;
	}

	cout << "Model loaded successfully\n";

	// Create recognizer for 16kHz audio
	VoskRecognizer *recognizer = vosk_recognizer_new(model, 16000.0);
	if (recognizer == nullptr) {
		cerr << "ERROR: Failed to create Vosk recognizer!\n";
		vosk_model_free(model);
		return;
	}

	cout << "Recognizer created\n";

	// Initialize PortAudio
	PaError err = Pa_Initialize();
	if (err != paNoError) {
		cerr << "ERROR: Failed to initialize PortAudio: " << Pa_GetErrorText(err) << "\n";
		vosk_recognizer_free(recognizer);
		vosk_model_free(model);
		return;
	}

	cout << "PortAudio initialized\n";

	// Get default input device
	PaStreamParameters inputParameters;
	inputParameters.device = Pa_GetDefaultInputDevice();
	if (inputParameters.device == paNoDevice) {
		cerr << "ERROR: No default input device found!\n";
		Pa_Terminate();
		vosk_recognizer_free(recognizer);
		vosk_model_free(model);
		return;
	}

	const PaDeviceInfo* deviceInfo = Pa_GetDeviceInfo(inputParameters.device);
	cout << "Using input device: " << deviceInfo->name << "\n";

	inputParameters.channelCount = 1;  // Mono
	inputParameters.sampleFormat = paInt16;
	inputParameters.suggestedLatency = deviceInfo->defaultLowInputLatency;
	inputParameters.hostApiSpecificStreamInfo = nullptr;

	// Open audio stream (blocking mode - no callback)
	PaStream *stream;
	err = Pa_OpenStream(&stream,
						&inputParameters,
						nullptr,  // No output
						16000.0,  // Sample rate (16kHz for Vosk)
						1024,     // Frames per buffer
						paClipOff,
						nullptr,  // No callback - use blocking read
						nullptr);

	if (err != paNoError) {
		cerr << "ERROR: Failed to open audio stream: " << Pa_GetErrorText(err) << "\n";
		Pa_Terminate();
		vosk_recognizer_free(recognizer);
		vosk_model_free(model);
		return;
	}

	cout << "Audio stream opened\n";

	// Start audio stream
	err = Pa_StartStream(stream);
	if (err != paNoError) {
		cerr << "ERROR: Failed to start audio stream: " << Pa_GetErrorText(err) << "\n";
		Pa_CloseStream(stream);
		Pa_Terminate();
		vosk_recognizer_free(recognizer);
		vosk_model_free(model);
		return;
	}

	cout << "Audio stream started\n\n";

	cout << "Listening for voice commands...\n";
	cout << "Say: 'move forward', 'turn left', 'stop', etc.\n";
	cout << "Press Ctrl+C to exit\n";
	cout << string(60, '=') << "\n\n";

	// Create robot commander
	RobotCommander commander;
	auto lastCommandTime = chrono::steady_clock::now();

	// Audio buffer (1024 samples * 2 bytes per sample = 2048 bytes)
	const int BUFFER_SIZE = 2048;
	int16_t audioBuffer[BUFFER_SIZE / 2];

	// Main recognition loop
	while (running) {
		// Read audio data from microphone
		err = Pa_ReadStream(stream, audioBuffer, 1024);

		if (err != paNoError && err != paInputOverflowed) {
			cerr << "ERROR: Failed to read audio stream: " << Pa_GetErrorText(err) << "\n";
			break;
		}

		// Feed audio data to Vosk
		int result = vosk_recognizer_accept_waveform(recognizer,
													 (const char*)audioBuffer,
													 BUFFER_SIZE);

		if (result) {
			// Final result (complete utterance detected)
			const char* resultJson = vosk_recognizer_result(recognizer);

			// Parse JSON to extract text
			// Simple parsing - look for "text" : "..."
			string jsonStr(resultJson);
			size_t textPos = jsonStr.find("\"text\" : \"");
			if (textPos != string::npos) {
				textPos += 10; // Length of "text" : "
				size_t endPos = jsonStr.find("\"", textPos);
				if (endPos != string::npos) {
					string recognizedText = jsonStr.substr(textPos, endPos - textPos);

					// Only process non-empty results
					if (!recognizedText.empty()) {
						cout << "\n>>> VOICE RECOGNIZED <<<\n";
						cout << "Text: \"" << recognizedText << "\"\n";

						// Parse command
						RobotState newState = parseCommand(recognizedText);

						if (newState != currentState || newState != RobotState::IDLE) {
							currentState = newState;
							lastCommandTime = chrono::steady_clock::now();

							cout << "Parsed as: " << stateToString(currentState) << "\n";
							cout << "Mode: " << modeToString(commandMode) << "\n";

							if (commandMode == CommandMode::TIMED && currentState != RobotState::IDLE) {
								cout << "Duration: " << TIMED_COMMAND_DURATION << " seconds\n";
							}

							commander.executeState(currentState);
							cout << string(60, '=') << "\n\n";
						}
					}
				}
			}
		} else {
			// Partial result (for debugging - can be commented out if too verbose)
			// const char* partialJson = vosk_recognizer_partial_result(recognizer);
			// cout << "Partial: " << partialJson << "\n";
		}

		// Check if we need to stop in TIMED mode
		if (commandMode == CommandMode::TIMED && currentState != RobotState::IDLE) {
			auto now = chrono::steady_clock::now();
			auto elapsed = chrono::duration<float>(now - lastCommandTime).count();

			if (elapsed >= TIMED_COMMAND_DURATION) {
				cout << "\n[TIMED MODE] Command duration expired, stopping...\n";
				currentState = RobotState::IDLE;
				commander.stop();
				cout << "State: " << stateToString(currentState) << "\n";
				cout << string(60, '=') << "\n\n";
			}
		}

		// In CONTINUOUS mode, keep publishing current state
		if (commandMode == CommandMode::CONTINUOUS && currentState != RobotState::IDLE) {
			commander.executeState(currentState);
		}
	}

	// Cleanup
	cout << "\nCleaning up...\n";

	// Stop robot
	currentState = RobotState::IDLE;
	commander.stop();

	// Stop and close audio stream
	Pa_StopStream(stream);
	Pa_CloseStream(stream);
	Pa_Terminate();

	// Free Vosk resources
	vosk_recognizer_free(recognizer);
	vosk_model_free(model);

	cout << "Vosk cleanup complete\n";
}

// ============================================================================
// MAIN
// ============================================================================

int main(int argc, char* argv[]) {
	cout << "=== Voice Command Robot Controller ===\n";
	cout << "Initial state: " << stateToString(currentState) << "\n";
	cout << "Command mode: " << modeToString(commandMode) << "\n\n";

	// Register signal handlers
	signal(SIGINT, signalHandler);
	signal(SIGTERM, signalHandler);

#if TEST_MODE
	// Test mode
	#if USE_KEYBOARD_INPUT
		// Keyboard simulation for testing parser
		simulatedVoiceInput();
	#else
		// Use Vosk with microphone but print commands instead of ROS2
		voskVoiceInput();
	#endif
#else
	// Production mode - use Vosk + ROS2
	// Initialize ROS2
	// rclcpp::init(argc, argv);
	voskVoiceInput();
	// rclcpp::shutdown();
#endif

	cout << "\n=== Shutdown complete ===\n";
	cout << "Final state: " << stateToString(currentState) << "\n";

	return 0;
}
