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

	std::cout << "Starting audio receiver...\n";
	std::cout << "Listening for CRC-encoded protocol commands...\n";
	std::cout << "Press Ctrl+C to stop\n\n";
	std::cout << std::string(60, '=') << "\n\n";

	// Start receiving with callback
	bool receiverStarted = receiver.startReceiving(recvConfig,
		[&crc, &protocol](const AudioComm::ChordReceiver::Detection& det) {
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
				std::cout << "✗ CRC CHECK FAILED!\n";
				std::cout << "  → Transmission error detected\n";
				std::cout << "  → Command REJECTED for safety\n";
				std::cout << "\n" << std::string(60, '=') << "\n";
				std::cout << "Listening for next command...\n\n";
				return;
			}

			std::cout << "✓ CRC CHECK PASSED\n";

			// Step 2: Decode to get original 12-bit command
			std::cout << "\n[STEP 2: DECODE COMMAND]\n";
			optional<uint16_t> decodedData = crc.decode1612(det.value);

			if (!decodedData.has_value()) {
				std::cout << "✗ DECODE FAILED!\n";
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
		});

	if (!receiverStarted) {
		std::cerr << "✗ ERROR: Failed to start audio receiver!\n";
		return 1;
	}

	// Keep running until interrupted
	while (running) {
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}

	// Cleanup
	std::cout << "\nStopping receiver...\n";
	receiver.stop();

	std::cout << "Receiver stopped.\n";
	std::cout << "Final mode: " << CommandProtocol::modeToString(protocol.getCurrentMode()) << "\n";
	std::cout << "Shutdown complete.\n";

	return 0;
}
