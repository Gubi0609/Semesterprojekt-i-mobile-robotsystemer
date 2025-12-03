#include "../LIB/audio_receiver.h"
#include "../INCLUDE/CRC.h"
#include "../INCLUDE/command_protocol.h"
#include <iostream>
#include <iomanip>
#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>

// Protocol-aware CRC-decoding chord receiver for robot control
// Continuously listens for commands, verifies CRC, and interprets protocol

// Global flag for graceful shutdown
std::atomic<bool> running(true);

// Duplicate detection prevention
struct LastDetection {
	uint16_t value = 0;
	std::chrono::steady_clock::time_point timestamp;
	bool hasValue = false;
};
LastDetection lastDetection;
const double LOCKOUT_PERIOD = 0.8;  // Seconds - based on max latency of 0.755s + margin

// Microphone restart tracking
std::chrono::steady_clock::time_point lastActivityTime;
const double RESTART_INTERVAL = 30.0;  // Restart mic if idle for 30 seconds
bool needsRestart = false;

// Signal handler for Ctrl+C
void signalHandler(int signum) {
	std::cout << "\n\nReceived interrupt signal (" << signum << ")\n";
	std::cout << "Shutting down receiver...\n";
	running = false;
}

// Robot command callbacks (these would control actual motors/hardware)
void executeDriveForDuration(const DriveForDurationCommand& cmd) {
	std::cout << "\n🤖 EXECUTING: " << cmd.toString() << "\n";
	std::cout << "   ► Duration: " << cmd.getDurationSeconds() << " seconds\n";
	std::cout << "   ► Speed: " << cmd.getSpeedPercent() << "%\n";
	std::cout << "   [TODO: Control motors to drive forward]\n";

	// Example motor control (pseudo-code):
	// setMotorSpeed(LEFT_MOTOR, cmd.getSpeedPercent());
	// setMotorSpeed(RIGHT_MOTOR, cmd.getSpeedPercent());
	// startTimer(cmd.getDurationSeconds());
}

void executeTurnForDuration(const TurnForDurationCommand& cmd) {
	std::cout << "\n🤖 EXECUTING: " << cmd.toString() << "\n";
	std::cout << "   ► Duration: " << cmd.getDurationSeconds() << " seconds\n";
	std::cout << "   ► Turn Rate: " << cmd.getTurnRatePercent() << "%\n";

	float turnRate = cmd.getTurnRatePercent();
	if (turnRate < -10.0f) {
		std::cout << "   ► Direction: LEFT TURN\n";
	} else if (turnRate > 10.0f) {
		std::cout << "   ► Direction: RIGHT TURN\n";
	} else {
		std::cout << "   ► Direction: STRAIGHT\n";
	}

	std::cout << "   [TODO: Control motors with differential speed]\n";

	// Example differential drive control (pseudo-code):
	// float baseSpeed = 50.0f;
	// float leftSpeed = baseSpeed * (1.0f - turnRate/100.0f);
	// float rightSpeed = baseSpeed * (1.0f + turnRate/100.0f);
	// setMotorSpeed(LEFT_MOTOR, leftSpeed);
	// setMotorSpeed(RIGHT_MOTOR, rightSpeed);
	// startTimer(cmd.getDurationSeconds());
}

void executeDriveForward(const DriveForwardCommand& cmd) {
	std::cout << "\n🤖 EXECUTING: " << cmd.toString() << "\n";
	std::cout << "   ► Speed: " << cmd.getSpeedPercent() << "% (HIGH RESOLUTION)\n";
	std::cout << "   ► Mode: CONTINUOUS (until new command)\n";
	std::cout << "   [TODO: Control motors to drive continuously]\n";

	// Example motor control (pseudo-code):
	// setMotorSpeed(LEFT_MOTOR, cmd.getSpeedPercent());
	// setMotorSpeed(RIGHT_MOTOR, cmd.getSpeedPercent());
	// enableMotors();
}

void executeTurn(const TurnCommand& cmd) {
	std::cout << "\n🤖 EXECUTING: " << cmd.toString() << "\n";
	std::cout << "   ► Turn Rate: " << cmd.getTurnRatePercent() << "% (HIGH RESOLUTION)\n";

	float turnRate = cmd.getTurnRatePercent();
	if (turnRate < -10.0f) {
		std::cout << "   ► Direction: CONTINUOUS LEFT TURN\n";
	} else if (turnRate > 10.0f) {
		std::cout << "   ► Direction: CONTINUOUS RIGHT TURN\n";
	} else {
		std::cout << "   ► Direction: STRAIGHT\n";
	}

	std::cout << "   ► Mode: CONTINUOUS (until new command)\n";
	std::cout << "   [TODO: Control motors with differential speed]\n";

	// Example differential drive control (pseudo-code):
	// float baseSpeed = 50.0f;
	// float leftSpeed = baseSpeed * (1.0f - turnRate/100.0f);
	// float rightSpeed = baseSpeed * (1.0f + turnRate/100.0f);
	// setMotorSpeed(LEFT_MOTOR, leftSpeed);
	// setMotorSpeed(RIGHT_MOTOR, rightSpeed);
	// enableMotors();
}

void executeStop() {
	std::cout << "\n🤖 EXECUTING: STOP\n";
	std::cout << "   ► Stopping all motors\n";
	std::cout << "   [TODO: Stop all motors immediately]\n";

	// Example motor control (pseudo-code):
	// setMotorSpeed(LEFT_MOTOR, 0);
	// setMotorSpeed(RIGHT_MOTOR, 0);
	// disableMotors();
}

void onModeChange(RobotMode mode) {
	std::cout << "\n📡 MODE CHANGE: " << CommandProtocol::modeToString(mode) << "\n";

	switch(mode) {
		case RobotMode::MODE_SELECT:
			std::cout << "   ► Waiting for mode selection...\n";
			break;
		case RobotMode::DRIVE_FOR_DURATION:
			std::cout << "   ► Ready to receive: Duration-based drive commands\n";
			std::cout << "   ► Format: 6-bit duration (0-8s) + 6-bit speed (0-100%)\n";
			break;
		case RobotMode::TURN_FOR_DURATION:
			std::cout << "   ► Ready to receive: Duration-based turn commands\n";
			std::cout << "   ► Format: 6-bit duration (0-8s) + 6-bit turn rate (-100% to +100%)\n";
			break;
		case RobotMode::DRIVE_FORWARD:
			std::cout << "   ► Ready to receive: Continuous drive commands\n";
			std::cout << "   ► Format: 12-bit speed (0-100%, high resolution)\n";
			break;
		case RobotMode::TURN:
			std::cout << "   ► Ready to receive: Continuous turn commands\n";
			std::cout << "   ► Format: 12-bit turn rate (-100% to +100%, high resolution)\n";
			break;
		case RobotMode::STOP:
			std::cout << "   ► STOP mode activated\n";
			break;
	}
}

int main(int argc, char* argv[]) {
	std::cout << "╔════════════════════════════════════════════════════════╗\n";
	std::cout << "║   Protocol-Aware CRC Receiver - Robot Control         ║\n";
	std::cout << "╚════════════════════════════════════════════════════════╝\n\n";

	// Register signal handler for graceful shutdown
	signal(SIGINT, signalHandler);
	signal(SIGTERM, signalHandler);

	// Select receiver mode
	int mode = 1; // Default to safe mode
	if (argc > 1) {
		mode = std::stoi(argv[1]);
	} else {
		std::cout << "Select receiver mode:\n";
		std::cout << "  1. Safe Mode (Very Fast: 2 detections, 0.3s window, 75Hz)\n";
		std::cout << "  2. Risky Mode (Extreme Speed: 1 detection, 0.15s window, 100Hz)\n";
		std::cout << "  3. Default Mode (Standard: 2 detections, 1.0s window, 10Hz)\n";
		std::cout << "\nEnter mode (1-3, default=1): ";
		std::cin >> mode;
		if (std::cin.fail() || mode < 1 || mode > 3) {
			mode = 1;
			std::cout << "Invalid input, using Safe Mode.\n";
		}
	}

	// Initialize CRC
	CRC crc;

	// Initialize command protocol
	CommandProtocol protocol;
	protocol.setDriveForDurationCallback(executeDriveForDuration);
	protocol.setTurnForDurationCallback(executeTurnForDuration);
	protocol.setDriveForwardCallback(executeDriveForward);
	protocol.setTurnCallback(executeTurn);
	protocol.setStopCallback(executeStop);
	protocol.setModeChangeCallback(onModeChange);

	std::cout << "Protocol initialized. Current mode: "
	          << CommandProtocol::modeToString(protocol.getCurrentMode()) << "\n\n";

	// Create receiver
	AudioComm::ChordReceiver receiver;
	AudioComm::ChordReceiver::Config recvConfig;

	// INCREASED FFT SIZE for better frequency resolution
	// 16384 samples @ 48kHz = ~2.93 Hz resolution (vs 5.86 Hz with 8192)
	// This helps distinguish closely-spaced tones more accurately
	recvConfig.fftSize = 16384;

	// INCREASED DETECTION TOLERANCE for hardware frequency variations
	// Your microphone test showed ±50 Hz variations, so we use 150 Hz tolerance
	// to accommodate potential compounding effects when multiple tones play
	recvConfig.detectionTolerance = 150.0;

	// Configure based on selected mode
	switch(mode) {
		case 1: // Safe Mode
			recvConfig.minDetections = 2;
			recvConfig.consistencyWindow = 0.3;
			recvConfig.updateRate = 75.0;
			std::cout << "Using Safe Mode (Very Fast): 2 detections, 0.3s window, 75Hz\n";
			break;
		case 2: // Risky Mode
			recvConfig.minDetections = 1;
			recvConfig.consistencyWindow = 0.15;
			recvConfig.updateRate = 100.0;
			std::cout << "Using Risky Mode (Extreme Speed): 1 detection, 0.15s window, 100Hz\n";
			break;
		case 3: // Default Mode
		default:
			recvConfig.minDetections = 2;
			recvConfig.consistencyWindow = 1.0;
			recvConfig.updateRate = 10.0;
			std::cout << "Using Default Mode: 2 detections, 1.0s window, 10Hz\n";
			break;
	}

	// Display FFT resolution info
	double freqResolution = recvConfig.sampleRate / recvConfig.fftSize;
	std::cout << "FFT Size: " << recvConfig.fftSize
	          << " (Frequency resolution: " << std::fixed << std::setprecision(2)
	          << freqResolution << " Hz per bin)\n";
	std::cout << "Detection Tolerance: " << recvConfig.detectionTolerance << " Hz\n";
	std::cout << "Duplicate Lockout: " << LOCKOUT_PERIOD << " seconds\n";
	std::cout << "Auto-restart interval: " << RESTART_INTERVAL << " seconds (when idle)\n";

	std::cout << "\nStarting audio receiver...\n";
	std::cout << "Listening for CRC-encoded protocol commands...\n";
	std::cout << "Press Ctrl+C to stop\n\n";
	std::cout << std::string(60, '=') << "\n\n";

	// Initialize activity timestamp
	lastActivityTime = std::chrono::steady_clock::now();

	// Define the detection callback (reusable for restarts)
	auto detectionCallback = [&crc, &protocol](const AudioComm::ChordReceiver::Detection& det) {
			// Check for duplicate detection (lockout period)
			auto now = std::chrono::steady_clock::now();
			if (lastDetection.hasValue && lastDetection.value == det.value) {
				auto timeSinceLast = std::chrono::duration<double>(now - lastDetection.timestamp).count();
				if (timeSinceLast < LOCKOUT_PERIOD) {
					// Duplicate detection within lockout period - ignore silently
					return;
				}
			}

			// Update last detection and activity time
			lastDetection.value = det.value;
			lastDetection.timestamp = now;
			lastDetection.hasValue = true;
			lastActivityTime = now;  // Update activity timestamp

			std::cout << "\n┌─────────────────────────────────────────────────────────\n";
			std::cout << "│ CHORD DETECTED\n";
			std::cout << "├─────────────────────────────────────────────────────────\n";
			std::cout << "│ Timestamp: " << std::fixed << std::setprecision(2)
			          << det.detectionTime << "s\n";
			std::cout << "│ Raw 16-bit value: " << det.value
					  << " (0x" << std::hex << std::uppercase << std::setw(4)
					  << std::setfill('0') << det.value << std::dec << ")\n";
			std::cout << "│ Detection count: " << det.detectionCount << "\n";
			std::cout << "│ Frequencies: ";
			for (size_t i = 0; i < det.frequencies.size(); ++i) {
				std::cout << std::fixed << std::setprecision(1) << det.frequencies[i] << " Hz";
				if (i < det.frequencies.size() - 1) std::cout << ", ";
			}
			std::cout << "\n└─────────────────────────────────────────────────────────\n";

			// Step 1: Verify CRC
			std::cout << "\n[STEP 1: CRC VERIFICATION]\n";
			bool isValid = crc.verify(det.value);

			if (!isValid) {
				std::cout << "CRC CHECK FAILED!\n";
				std::cout << "  → Transmission error detected\n";
				std::cout << "  → Command REJECTED for safety\n";
				std::cout << "\n" << std::string(60, '=') << "\n";
				std::cout << "Listening for next command...\n\n";
				return;
			}

			std::cout << "CRC CHECK PASSED\n";

			// Step 2: Decode to get original 12-bit command
			std::cout << "\n[STEP 2: DECODE COMMAND]\n";
			optional<uint16_t> decodedData = crc.decode1612(det.value);

			if (!decodedData.has_value()) {
				std::cout << "DECODE FAILED!\n";
				std::cout << "\n" << std::string(60, '=') << "\n";
				std::cout << "Listening for next command...\n\n";
				return;
			}

			uint16_t command = decodedData.value();
			std::cout << "✓ Decoded 12-bit command: " << command
			          << " (0x" << std::hex << std::uppercase << std::setw(3)
			          << std::setfill('0') << command << std::dec << ")\n";

			// Step 3: Process command through protocol
			std::cout << "\n[STEP 3: PROTOCOL PROCESSING]\n";
			std::cout << "Current mode: " << CommandProtocol::modeToString(protocol.getCurrentMode()) << "\n";

			if (protocol.isWaitingForModeSelect()) {
				std::cout << "Status: Waiting for mode selection\n";
			}

			// Process the command
			protocol.processCommand(command);

			std::cout << "\n" << std::string(60, '=') << "\n";
			std::cout << "Ready for next command...\n\n";
	};

	// Start receiving with callback
	bool receiverStarted = receiver.startReceiving(recvConfig, detectionCallback);

	if (!receiverStarted) {
		std::cerr << "✗ ERROR: Failed to start audio receiver!\n";
		return 1;
	}

	// Keep running until interrupted
	while (running) {
		std::this_thread::sleep_for(std::chrono::milliseconds(100));

		// Check if we need to restart the microphone (only when idle)
		auto now = std::chrono::steady_clock::now();
		auto timeSinceActivity = std::chrono::duration<double>(now - lastActivityTime).count();

		if (timeSinceActivity >= RESTART_INTERVAL && !needsRestart) {
			needsRestart = true;
			std::cout << "\n Idle for " << RESTART_INTERVAL << "s - Restarting microphone...\n";

			// Stop receiver
			receiver.stop();
			std::this_thread::sleep_for(std::chrono::milliseconds(200)); // Brief pause

			// Restart receiver with same config and callback
			receiverStarted = receiver.startReceiving(recvConfig, detectionCallback);

			if (receiverStarted) {
				std::cout << "Microphone restarted successfully\n";
				std::cout << "Listening for commands...\n\n";
			} else {
				std::cerr << "ERROR: Failed to restart microphone!\n";
			}

			// Reset activity timer and restart flag
			lastActivityTime = std::chrono::steady_clock::now();
			needsRestart = false;
		}
	}

	// Cleanup
	std::cout << "\nStopping receiver...\n";
	receiver.stop();

	std::cout << "Receiver stopped.\n";
	std::cout << "Final mode: " << CommandProtocol::modeToString(protocol.getCurrentMode()) << "\n";
	std::cout << "Shutdown complete.\n";

	return 0;
}
