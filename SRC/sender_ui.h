#ifndef SENDER_UI_H
#define SENDER_UI_H

#include "../INCLUDE/command_protocol.h"
#include "../LIB/audio_transmitter.h"
#include "../INCLUDE/CRC.h"
#include <iostream>
#include <iomanip>
#include <string>

// Forward declarations
void sendCommandWithRetry(AudioComm::ChordTransmitter& transmitter, CRC& crc, uint16_t command,
                          const std::string& description, double duration , 
                          bool waitForFeedback , bool retryOnce );

// State tracking
enum class RobotState {
	NO_MODE,
	DRIVE_FOR_DURATION,
	TURN_FOR_DURATION,
	DRIVE_CONTINUOUS,
	TURN_CONTINUOUS,
	STOP_MODE
};

std::string stateToString(RobotState state) {
	switch(state) {
		case RobotState::NO_MODE: return "NO MODE (Mode Select)";
		case RobotState::DRIVE_FOR_DURATION: return "DRIVE FOR DURATION";
		case RobotState::TURN_FOR_DURATION: return "TURN FOR DURATION";
		case RobotState::DRIVE_CONTINUOUS: return "DRIVE CONTINUOUS";
		case RobotState::TURN_CONTINUOUS: return "TURN CONTINUOUS";
		case RobotState::STOP_MODE: return "STOP";
		default: return "UNKNOWN";
	}
}

void printModeSelectMenu() {
	std::cout << "\n╔════════════════════════════════════════════════════════╗\n";
	std::cout << "║               SELECT ROBOT MODE    																						       	                  ║\n";
	std::cout << "╚════════════════════════════════════════════════════════╝\n";
	std::cout << "\n  1. Drive for Duration\n";
	std::cout << "  2. Turn for Duration\n";
	std::cout << "  3. Drive Forward (continuous)\n";
	std::cout << "  4. Turn (continuous)\n";
	std::cout << "  5. Stop Mode\n";
	std::cout << "\n  0. Exit Program\n";
	std::cout << "\n All modes use smart feedback with retry\n";
	std::cout << "\nSelect mode: ";
}

void printInModeMenu(RobotState state) {
	std::cout << "\n╔════════════════════════════════════════════════════════╗\n";
	std::cout << "║  Current Mode: " << std::left << std::setfill(' ') << std::setw(37) << stateToString(state) << std::setfill('0') << "║\n";
	std::cout << "╚════════════════════════════════════════════════════════╝\n";
	std::cout << "\n  C. Send Command (for this mode)\n";
	std::cout << "  S. Send STOP\n";
	std::cout << "  R. RESET (return to mode select)\n";
	std::cout << "  0. Exit Program\n";
	std::cout << "\nChoice: ";
}

void runStateMachineUI(AudioComm::ChordTransmitter& transmitter, CRC& crc) {
	RobotState currentState = RobotState::NO_MODE;
	std::string choice;
	
	while (true) {
		if (currentState == RobotState::NO_MODE) {
			// ===== MODE SELECT STATE =====
			printModeSelectMenu();
			std::cin >> choice;

			if (std::cin.fail()) {
				std::cin.clear();
				std::cin.ignore(10000, '\n');
				std::cout << "Invalid input.\n";
				continue;
			}

			if (choice == "0") {
				std::cout << "\n Exiting...\n";
				break;
			}

			// Determine which mode was selected
			RobotMode selectedMode;
			RobotState nextState;
			bool validChoice = true;

			if (choice == "1") {
				selectedMode = RobotMode::DRIVE_FOR_DURATION;
				nextState = RobotState::DRIVE_FOR_DURATION;
			} else if (choice == "2") {
				selectedMode = RobotMode::TURN_FOR_DURATION;
				nextState = RobotState::TURN_FOR_DURATION;
			} else if (choice == "3") {
				selectedMode = RobotMode::DRIVE_FORWARD;
				nextState = RobotState::DRIVE_CONTINUOUS;
			} else if (choice == "4") {
				selectedMode = RobotMode::TURN;
				nextState = RobotState::TURN_CONTINUOUS;
			} else if (choice == "5") {
				selectedMode = RobotMode::STOP;
				nextState = RobotState::STOP_MODE;
			} else {
				std::cout << "❌ Invalid choice.\n";
				validChoice = false;
			}

			if (!validChoice) continue;

			// Send RESET + MODE SELECT with feedback
			std::cout << "\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
			std::cout << " Entering mode: " << stateToString(nextState) << "\n";
			std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
			
			sendCommandWithRetry(transmitter, crc, encodeReset(), "RESET", 0.8, true, true);
			sendCommandWithRetry(transmitter, crc, encodeModeSelect(selectedMode), 
			                     "Mode: " + stateToString(nextState), 0.8, true, true);
			
			std::cout << "\n Robot is now in " << stateToString(nextState) << " mode\n";
			currentState = nextState;

		} else {
			// ===== IN-MODE STATE =====
			printInModeMenu(currentState);
			std::cin >> choice;

			if (std::cin.fail()) {
				std::cin.clear();
				std::cin.ignore(10000, '\n');
				std::cout << "Invalid input.\n";
				continue;
			}

			if (choice == "0") {
				std::cout << "\n Exiting...\n";
				break;
			}

			// Handle RESET - return to mode select
			if (choice == "R" || choice == "r") {
				std::cout << "\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
				std::cout << " Sending RESET (returning to mode select)\n";
				std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
				sendCommandWithRetry(transmitter, crc, encodeReset(), "RESET", 0.8, true, true);
				currentState = RobotState::NO_MODE;
				std::cout << "\n Back to mode select\n";
				continue;
			}

			// Handle STOP
			if (choice == "S" || choice == "s") {
				std::cout << "\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
				std::cout << " Sending STOP\n";
				std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
				sendCommandWithRetry(transmitter, crc, encodeStop(), "STOP", 0.8, true, true);
				std::cout << "\n STOP sent (still in " << stateToString(currentState) << " mode)\n";
				continue;
			}

			// Handle COMMAND for current mode
			if (choice == "C" || choice == "c") {
				std::cout << "\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
				
				if (currentState == RobotState::DRIVE_FOR_DURATION) {
					float duration, speed;
					std::cout << "Enter duration (seconds): ";
					std::cin >> duration;
					std::cout << "Enter speed percentage (0-100): ";
					std::cin >> speed;
					std::cin.ignore(10000, '\n');  // Clear input buffer

					sendCommandWithRetry(transmitter, crc, encodeDriveForDuration(duration, speed),
					                     "Drive " + std::to_string(duration) + "s at " + std::to_string((int)speed) + "%",
					                     0.8, true, true);

				} else if (currentState == RobotState::TURN_FOR_DURATION) {
					float duration, turnRate;
					std::cout << "Enter duration (seconds): ";
					std::cin >> duration;
					std::cout << "Enter turn rate percentage (-100 to 100): ";
					std::cin >> turnRate;
					std::cin.ignore(10000, '\n');  // Clear input buffer

					sendCommandWithRetry(transmitter, crc, encodeTurnForDuration(duration, turnRate),
					                     "Turn " + std::to_string(duration) + "s at " + std::to_string((int)turnRate) + "%",
					                     0.8, true, true);

				} else if (currentState == RobotState::DRIVE_CONTINUOUS) {
					float speed;
					std::cout << "Enter speed percentage (0-100): ";
					std::cin >> speed;
					std::cin.ignore(10000, '\n');  // Clear input buffer

					sendCommandWithRetry(transmitter, crc, encodeDriveForward(speed),
					                     "Drive forward at " + std::to_string((int)speed) + "%",
					                     0.8, true, true);

				} else if (currentState == RobotState::TURN_CONTINUOUS) {
					float turnRate;
					std::cout << "Enter turn rate percentage (-100 to 100): ";
					std::cin >> turnRate;
					std::cin.ignore(10000, '\n');  // Clear input buffer

					sendCommandWithRetry(transmitter, crc, encodeTurn(turnRate),
					                     "Turn at " + std::to_string((int)turnRate) + "%",
					                     0.8, true, true);
					
				} else if (currentState == RobotState::STOP_MODE) {
					std::cout << "Already in STOP mode. Use 'R' to return to mode select.\n";
				}
				
				std::cout << "\n Command sent\n";
				continue;
			}

			std::cout << "Invalid choice.\n";
		}
	}
}

#endif // SENDER_UI_H
