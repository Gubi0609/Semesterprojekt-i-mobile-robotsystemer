#include "../LIB/audio_receiver.h"
#include "../INCLUDE/CRC.h"
#include <iostream>
#include <iomanip>
#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>

// CRC-decoding chord receiver for robot control
// Continuously listens for commands and verifies them using CRC

// Robot state enum (to be expanded later)
enum class RobotState {
	IDLE
	// Add more states as needed:
	// MOVING_FORWARD,
	// MOVING_BACKWARD,
	// TURNING_LEFT,
	// TURNING_RIGHT,
	// etc.
};

// Global flag for graceful shutdown
std::atomic<bool> running(true);

// Signal handler for Ctrl+C
void signalHandler(int signum) {
	std::cout << "\n\nReceived interrupt signal (" << signum << ")\n";
	std::cout << "Shutting down receiver...\n";
	running = false;
}

// Function to convert state to string for display
const char* stateToString(RobotState state) {
	switch(state) {
		case RobotState::IDLE: return "IDLE";
		default: return "UNKNOWN";
	}
}

int main() {
	std::cout << "=== CRC-Decoding Chord Receiver ===\n";
	std::cout << "Robot Control System\n\n";

	// Register signal handler for graceful shutdown
	signal(SIGINT, signalHandler);
	signal(SIGTERM, signalHandler);

	// Initialize CRC with default generator key
	CRC crc;

	float currentSignal;
	// Robot state
	RobotState currentState = RobotState::IDLE;
	std::cout << "Initial robot state: " << stateToString(currentState) << "\n\n";

	// Create receiver
	AudioComm::ChordReceiver receiver;
	AudioComm::ChordReceiver::Config recvConfig;

	std::cout << "Starting receiver...\n";
	std::cout << "Listening for CRC-encoded commands...\n";
	std::cout << "Press Ctrl+C to stop\n\n";
	std::cout << std::string(60, '=') << "\n\n";

	// Start receiving with callback that performs CRC verification
	bool receiverStarted = receiver.startReceiving(recvConfig,
		[&crc, &currentState](const AudioComm::ChordReceiver::Detection& det) {
			std::cout << "\n>>> CHORD RECEIVED <<<\n";
			std::cout << "Timestamp: " << std::fixed << std::setprecision(2)
			          << det.detectionTime << "s\n";
			std::cout << "Raw value: " << det.value
					  << " (0x" << std::hex << std::uppercase << std::setw(4)
					  << std::setfill('0') << det.value << std::dec << ")\n";
			std::cout << "Detection count: " << det.detectionCount << "\n";
			std::cout << "Frequencies: ";
			for (size_t i = 0; i < det.frequencies.size(); ++i) {
				std::cout << std::fixed << std::setprecision(1) << det.frequencies[i] << " Hz";
				if (i < det.frequencies.size() - 1) std::cout << ", ";
			}
			std::cout << "\n\n";

			// Verify CRC
			std::cout << "--- CRC VERIFICATION ---\n";
			bool isValid = crc.verify(det.value);

			if (isValid) {
				std::cout << "✓ CRC CHECK PASSED - Valid command received\n";

				// Decode to get original 12-bit data
				optional<uint16_t> decodedData = crc.decode1612(det.value);
				if (decodedData.has_value()) {
					std::cout << "Decoded command: " << decodedData.value()
					          << " (0x" << std::hex << std::uppercase << std::setw(3)
					          << std::setfill('0') << decodedData.value() << std::dec << ")\n";

					// Process command based on current state
					std::cout << "\n--- COMMAND PROCESSING ---\n";
					std::cout << "Current state: " << stateToString(currentState) << "\n";

					switch(currentState) {
						case RobotState::IDLE:
							// In IDLE state, just log the received command
							// Later this will trigger state transitions
							std::cout << "Action: Command acknowledged (IDLE state)\n";
							std::cout << "  Command value: " << decodedData.value() << "\n";
							std::cout << "  (Robot ready for future state implementations)\n";

							// Example of how to expand later:
							// if (decodedData.value() == MOVE_FORWARD_CMD) {
							//     currentState = RobotState::MOVING_FORWARD;
							//     // Start motors, etc.
							// }
							break;

						// Add more state handlers here as needed
						default:
							std::cout << "Action: Unknown state\n";
							break;
					}
				} else {
					std::cout << "✗ ERROR: Failed to decode data\n";
				}
			} else {
				std::cout << "✗ CRC CHECK FAILED - Command rejected!\n";
				std::cout << "  Error detected in transmission.\n";
				std::cout << "  Command ignored for safety.\n";
			}

			std::cout << "\n" << std::string(60, '=') << "\n";
			std::cout << "Listening for next command...\n\n";
		});

	if (!receiverStarted) {
		std::cerr << "Error: Failed to start receiver!\n";
		return 1;
	}

	// Keep running until interrupted
	while (running) {
		// Sleep to avoid busy waiting
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}

	// Stop receiver
	std::cout << "\nStopping receiver...\n";
	receiver.stop();

	std::cout << "Receiver stopped.\n";
	std::cout << "Final robot state: " << stateToString(currentState) << "\n";
	std::cout << "Shutdown complete.\n";

	return 0;
}
