#include "../INCLUDE/command_protocol.h"
#include "../LIB/audio_transmitter.h"
#include "../INCLUDE/CRC.h"
#include <iostream>
#include <iomanip>
#include <string>
#include <thread>
#include <chrono>

// Helper to print command information
void printCommandInfo(const std::string& description, uint16_t bits) {
	std::cout << "\n┌────────────────────────────────────────────────────\n";
	std::cout << "│ " << description << "\n";
	std::cout << "│ Raw value: " << bits << " (0x" << std::hex << std::setfill('0')
			  << std::setw(3) << bits << std::dec << ")\n";
	std::cout << "└────────────────────────────────────────────────────\n";
}

// Function to send a command through the audio system
void sendCommand(AudioComm::ChordTransmitter& transmitter, CRC& crc, uint16_t command,
				 const std::string& description, double duration = 1.0, bool waitAfter = true) {
	printCommandInfo(description, command);

	// Encode with CRC
	std::vector<uint16_t> data = {command};
	std::vector<uint16_t> encoded = crc.encode1216(data);

	std::cout << "Encoded (with CRC): " << encoded[0] << " (0x" << std::hex
			  << std::setfill('0') << std::setw(4) << encoded[0] << std::dec << ")\n";

	// Send via audio
	std::cout << "Sending via audio for " << duration << " seconds...\n";
	AudioComm::ChordTransmitter::Config config;
	config.toneDuration = duration;
	if (!transmitter.startTransmitting(encoded[0], config)) {
		std::cerr << "Failed to start transmission!\n";
		return;
	}

	if (waitAfter) {
		std::cout << "Waiting for transmission to complete...\n";
		transmitter.waitForCompletion();
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}
}

void printMenu() {
	std::cout << "\n╔════════════════════════════════════════════════════════╗\n";
	std::cout << "║         Robot Command Sender (Audio Protocol)         ║\n";
	std::cout << "╚════════════════════════════════════════════════════════╝\n";
	std::cout << "\nProtocol Commands:\n";
	std::cout << "  R. Send RESET signal (enter mode select)\n";
	std::cout << "  M. Send Mode Select\n";
	std::cout << "\nRobot Commands:\n";
	std::cout << "  1. Drive for Duration (speed + duration)\n";
	std::cout << "  2. Turn for Duration (turn rate + duration)\n";
	std::cout << "  3. Drive Forward (continuous, high resolution)\n";
	std::cout << "  4. Turn (continuous, high resolution)\n";
	std::cout << "  5. Stop\n";
	std::cout << "  6. Send Preset Sequence (demo)\n";
	std::cout << "\n  0. Exit\n";
	std::cout << "\nEnter choice: ";
}

int main() {
	AudioComm::ChordTransmitter transmitter;
	CRC crc;

	std::cout << "╔════════════════════════════════════════════════════════╗\n";
	std::cout << "║   Protocol-Based Audio Command Sender                 ║\n";
	std::cout << "╚════════════════════════════════════════════════════════╝\n\n";

	std::cout << "Audio transmitter ready.\n";

	std::string choice;
	while (true) {
		printMenu();
		std::cin >> choice;

		if (std::cin.fail()) {
			std::cin.clear();
			std::cin.ignore(10000, '\n');
			std::cout << "Invalid input.\n";
			continue;
		}

		if (choice == "0") {
			std::cout << "\nExiting...\n";
			break;
		}

		// Handle reset command
		if (choice == "R" || choice == "r") {
			std::cout << "\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
			std::cout << "Sending RESET signal\n";
			std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
			uint16_t reset = encodeReset();
			sendCommand(transmitter, crc, reset, "RESET (enter mode select)", 0.8, false);
			continue;
		}

		// Handle mode select command
		if (choice == "M" || choice == "m") {
			std::cout << "\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
			std::cout << "Select Mode\n";
			std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
			std::cout << "Available modes:\n";
			std::cout << "  1. DRIVE_FOR_DURATION\n";
			std::cout << "  2. TURN_FOR_DURATION\n";
			std::cout << "  3. DRIVE_FORWARD\n";
			std::cout << "  4. TURN\n";
			std::cout << "  5. STOP\n";
			std::cout << "Enter mode number: ";
			int modeChoice;
			std::cin >> modeChoice;

			RobotMode mode;
			switch(modeChoice) {
				case 1: mode = RobotMode::DRIVE_FOR_DURATION; break;
				case 2: mode = RobotMode::TURN_FOR_DURATION; break;
				case 3: mode = RobotMode::DRIVE_FORWARD; break;
				case 4: mode = RobotMode::TURN; break;
				case 5: mode = RobotMode::STOP; break;
				default:
					std::cout << "Invalid mode selection.\n";
					continue;
			}

			uint16_t modeCmd = encodeModeSelect(mode);
			sendCommand(transmitter, crc, modeCmd, "Mode: " + CommandProtocol::modeToString(mode), 0.8, false);
			continue;
		}

		// Convert to int for numbered choices
		int numChoice;
		try {
			numChoice = std::stoi(choice);
		} catch (...) {
			std::cout << "Invalid choice. Please try again.\n";
			continue;
		}

		// Always send reset first to enter mode select (for numbered commands)
		std::cout << "\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
		std::cout << "Step 1: Sending RESET signal\n";
		std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
		uint16_t reset = encodeReset();
		sendCommand(transmitter, crc, reset, "RESET (enter mode select)", 0.8, true);

		switch (numChoice) {
			case 1: {
				std::cout << "\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
				std::cout << "Step 2: Select DRIVE_FOR_DURATION mode\n";
				std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
				uint16_t modeSelect = encodeModeSelect(RobotMode::DRIVE_FOR_DURATION);
				sendCommand(transmitter, crc, modeSelect, "Mode: DRIVE_FOR_DURATION", 0.8, true);

				float duration, speed;
				std::cout << "\nEnter duration (0-8 seconds): ";
				std::cin >> duration;
				std::cout << "Enter speed (0-100%): ";
				std::cin >> speed;

				std::cout << "\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
				std::cout << "Step 3: Send drive command\n";
				std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
				uint16_t cmd = encodeDriveForDuration(duration, speed);
				DriveForDurationCommand decoded = DriveForDurationCommand::decode(cmd);
				sendCommand(transmitter, crc, cmd, "Drive: " + decoded.toString(), 0.8, false);
				break;
			}

			case 2: {
				std::cout << "\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
				std::cout << "Step 2: Select TURN_FOR_DURATION mode\n";
				std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
				uint16_t modeSelect = encodeModeSelect(RobotMode::TURN_FOR_DURATION);
				sendCommand(transmitter, crc, modeSelect, "Mode: TURN_FOR_DURATION", 0.8, true);

				float duration, turnRate;
				std::cout << "\nEnter duration (0-8 seconds): ";
				std::cin >> duration;
				std::cout << "Enter turn rate (-100 to +100%, negative=left, positive=right): ";
				std::cin >> turnRate;

				std::cout << "\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
				std::cout << "Step 3: Send turn command\n";
				std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
				uint16_t cmd = encodeTurnForDuration(duration, turnRate);
				TurnForDurationCommand decoded = TurnForDurationCommand::decode(cmd);
				sendCommand(transmitter, crc, cmd, "Turn: " + decoded.toString(), 0.8, false);
				break;
			}

			case 3: {
				std::cout << "\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
				std::cout << "Step 2: Select DRIVE_FORWARD mode\n";
				std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
				uint16_t modeSelect = encodeModeSelect(RobotMode::DRIVE_FORWARD);
				sendCommand(transmitter, crc, modeSelect, "Mode: DRIVE_FORWARD (continuous)", 0.8, true);

				float speed;
				std::cout << "\nEnter speed (0-100%): ";
				std::cin >> speed;

				std::cout << "\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
				std::cout << "Step 3: Send continuous drive command\n";
				std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
				uint16_t cmd = encodeDriveForward(speed);
				DriveForwardCommand decoded = DriveForwardCommand::decode(cmd);
				sendCommand(transmitter, crc, cmd, "Drive: " + decoded.toString(), 0.8, false);
				std::cout << "\nNote: Robot will drive continuously until stopped or new command received.\n";
				break;
			}

			case 4: {
				std::cout << "\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
				std::cout << "Step 2: Select TURN mode\n";
				std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
				uint16_t modeSelect = encodeModeSelect(RobotMode::TURN);
				sendCommand(transmitter, crc, modeSelect, "Mode: TURN (continuous)", 0.8, true);

				float turnRate;
				std::cout << "\nEnter turn rate (-100 to +100%, negative=left, positive=right): ";
				std::cin >> turnRate;

				std::cout << "\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
				std::cout << "Step 3: Send continuous turn command\n";
				std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
				uint16_t cmd = encodeTurn(turnRate);
				TurnCommand decoded = TurnCommand::decode(cmd);
				sendCommand(transmitter, crc, cmd, "Turn: " + decoded.toString(), 0.8, false);
				std::cout << "\nNote: Robot will turn continuously until stopped or new command received.\n";
				break;
			}

			case 5: {
				std::cout << "\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
				std::cout << "Step 2: Send STOP command\n";
				std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
				uint16_t stopCmd = encodeStop();
				sendCommand(transmitter, crc, stopCmd, "STOP command", 0.8, false);
				break;
			}

			case 6: {
				std::cout << "\n╔════════════════════════════════════════════════════════╗\n";
				std::cout << "║   Demo Sequence: Square Pattern                        ║\n";
				std::cout << "╚════════════════════════════════════════════════════════╝\n";
				std::cout << "\nSending sequence to make robot drive in a square...\n";

				// Forward for 2 seconds at 50%
				std::cout << "\n[1/4] Forward\n";
				sendCommand(transmitter, crc, encodeReset(), "RESET", 0.8, true);
				sendCommand(transmitter, crc, encodeModeSelect(RobotMode::DRIVE_FOR_DURATION), "Mode: DRIVE", 0.8, true);
				sendCommand(transmitter, crc, encodeDriveForDuration(2.0f, 50.0f), "Drive 2s at 50%", 0.8, true);
				std::this_thread::sleep_for(std::chrono::seconds(3));

				// Turn right 90 degrees (1 second at 50%)
				std::cout << "\n[2/4] Turn right\n";
				sendCommand(transmitter, crc, encodeReset(), "RESET", 0.8, true);
				sendCommand(transmitter, crc, encodeModeSelect(RobotMode::TURN_FOR_DURATION), "Mode: TURN", 0.8, true);
				sendCommand(transmitter, crc, encodeTurnForDuration(1.0f, 50.0f), "Turn 1s right", 0.8, true);
				std::this_thread::sleep_for(std::chrono::seconds(2));

				// Forward again
				std::cout << "\n[3/4] Forward\n";
				sendCommand(transmitter, crc, encodeReset(), "RESET", 0.8, true);
				sendCommand(transmitter, crc, encodeModeSelect(RobotMode::DRIVE_FOR_DURATION), "Mode: DRIVE", 0.8, true);
				sendCommand(transmitter, crc, encodeDriveForDuration(2.0f, 50.0f), "Drive 2s at 50%", 0.8, true);
				std::this_thread::sleep_for(std::chrono::seconds(3));

				// Turn right again
				std::cout << "\n[4/4] Turn right\n";
				sendCommand(transmitter, crc, encodeReset(), "RESET", 0.8, true);
				sendCommand(transmitter, crc, encodeModeSelect(RobotMode::TURN_FOR_DURATION), "Mode: TURN", 0.8, true);
				sendCommand(transmitter, crc, encodeTurnForDuration(1.0f, 50.0f), "Turn 1s right", 0.8, true);

				std::cout << "\nSequence complete!\n";
				break;
			}

			default:
				std::cout << "Invalid choice. Please try again.\n";
				break;
		}
	}

	std::cout << "\nShutdown complete.\n";
	return 0;
}
