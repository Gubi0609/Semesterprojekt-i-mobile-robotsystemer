#ifndef SENDER_UI_H
#define SENDER_UI_H

#include "../INCLUDE/command_protocol.h"
#include "../LIB/audio_transmitter.h"
#include "../INCLUDE/CRC.h"
#include "../INCLUDE/Database.h"
#include <iostream>
#include <iomanip>
#include <string>
#include <thread>
#include <chrono>

// Forward declarations
void sendCommandWithRetry(AudioComm::ChordTransmitter& transmitter, CRC& crc, uint16_t command,
                          const std::string& description, double duration , 
                          bool waitForFeedback , bool retryOnce , Database* db ,
                          float speed , float turnSpeed , float cmdDuration );

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
	std::cout << "║               SELECT ROBOT MODE    	\n";
	std::cout << "╚════════════════════════════════════════════════════════╝\n";
	std::cout << "\n  1. Drive for Duration\n";
	std::cout << "  2. Turn for Duration\n";
	std::cout << "  3. Drive Forward (continuous)\n";
	std::cout << "  4. Turn (continuous)\n";
	std::cout << "  5. Stop Mode\n";
	std::cout << "\n  T. Run Automated Test Suite\n";
	std::cout << "\n  0. Exit Program\n";
	std::cout << "\n All modes use smart feedback with retry\n";
	std::cout << "\nSelect mode: ";
}

// ============================================================================
// Automated Test Suite
// ============================================================================

struct TestCommand {
	std::string description;
	RobotMode mode;
	float speed;
	float turnSpeed;
	float duration;
	bool isReset;
	bool isStop;
};

void runAutomatedTestSuite(AudioComm::ChordTransmitter& transmitter, CRC& crc, Database* db) {
	std::cout << "\n╔════════════════════════════════════════════════════════╗\n";
	std::cout << "║           AUTOMATED TEST SUITE                         ║\n";
	std::cout << "╚════════════════════════════════════════════════════════╝\n";
	std::cout << "\nThis will run a comprehensive test of all command types.\n";
	std::cout << "The test will send multiple commands and log all results.\n";
	std::cout << "\nTest includes:\n";
	std::cout << "  - 5x RESET commands\n";
	std::cout << "  - 5x Drive for Duration (various speeds/durations)\n";
	std::cout << "  - 5x Turn for Duration (various rates/durations)\n";
	std::cout << "  - 3x Drive Continuous (various speeds)\n";
	std::cout << "  - 3x Turn Continuous (various rates)\n";
	std::cout << "  - 3x STOP commands\n";
	std::cout << "\nTotal: ~24 command sequences (~50+ individual transmissions)\n";

	// Ask about clearing database
	std::cout << "\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
	std::cout << "  Clear database before test? (recommended for fresh stats)\n";
	std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
	std::cout << "  Y. Yes, clear all previous data\n";
	std::cout << "  N. No, keep existing data\n";
	std::cout << "\nChoice (Y/N): ";

	std::string clearChoice;
	std::getline(std::cin, clearChoice);

	if (clearChoice == "Y" || clearChoice == "y") {
		if (db != nullptr) {
			std::cout << "\nClearing database...\n";
			db->clearAllData();
			std::cout << "Database cleared.\n";
		} else {
			std::cout << "\nNo database connection - skipping clear.\n";
		}
	}

	std::cout << "\nMake sure the robot is ready and listening!\n";
	std::cout << "\nPress ENTER to start the test suite (or 'q' to cancel): ";

	std::string input;
	std::getline(std::cin, input);
	if (input == "q" || input == "Q") {
		std::cout << "Test cancelled.\n";
		return;
	}

	// Define test sequence
	std::vector<TestCommand> testSequence = {
		// === RESET Tests ===
		{"RESET #1", RobotMode::STOP, 0, 0, 0, true, false},
		{"RESET #2", RobotMode::STOP, 0, 0, 0, true, false},
		{"RESET #3", RobotMode::STOP, 0, 0, 0, true, false},
		{"RESET #4", RobotMode::STOP, 0, 0, 0, true, false},
		{"RESET #5", RobotMode::STOP, 0, 0, 0, true, false},

		// === Drive for Duration Tests ===
		{"Drive 2s @ 25%", RobotMode::DRIVE_FOR_DURATION, 25, 0, 2.0f, false, false},
		{"Drive 3s @ 50%", RobotMode::DRIVE_FOR_DURATION, 50, 0, 3.0f, false, false},
		{"Drive 1s @ 75%", RobotMode::DRIVE_FOR_DURATION, 75, 0, 1.0f, false, false},
		{"Drive 2s @ 100%", RobotMode::DRIVE_FOR_DURATION, 100, 0, 2.0f, false, false},
		{"Drive 4s @ 30%", RobotMode::DRIVE_FOR_DURATION, 30, 0, 4.0f, false, false},

		// === Turn for Duration Tests ===
		{"Turn 2s @ 25% (right)", RobotMode::TURN_FOR_DURATION, 0, 25, 2.0f, false, false},
		{"Turn 2s @ -25% (left)", RobotMode::TURN_FOR_DURATION, 0, -25, 2.0f, false, false},
		{"Turn 3s @ 50% (right)", RobotMode::TURN_FOR_DURATION, 0, 50, 3.0f, false, false},
		{"Turn 1s @ -75% (left)", RobotMode::TURN_FOR_DURATION, 0, -75, 1.0f, false, false},
		{"Turn 2s @ 100% (right)", RobotMode::TURN_FOR_DURATION, 0, 100, 2.0f, false, false},

		// === Drive Continuous Tests ===
		{"Drive continuous @ 25%", RobotMode::DRIVE_FORWARD, 25, 0, 0, false, false},
		{"STOP after drive", RobotMode::STOP, 0, 0, 0, false, true},
		{"Drive continuous @ 50%", RobotMode::DRIVE_FORWARD, 50, 0, 0, false, false},
		{"STOP after drive", RobotMode::STOP, 0, 0, 0, false, true},
		{"Drive continuous @ 75%", RobotMode::DRIVE_FORWARD, 75, 0, 0, false, false},
		{"STOP after drive", RobotMode::STOP, 0, 0, 0, false, true},

		// === Turn Continuous Tests ===
		{"Turn continuous @ 30%", RobotMode::TURN, 0, 30, 0, false, false},
		{"STOP after turn", RobotMode::STOP, 0, 0, 0, false, true},
		{"Turn continuous @ -50%", RobotMode::TURN, 0, -50, 0, false, false},
		{"STOP after turn", RobotMode::STOP, 0, 0, 0, false, true},
		{"Turn continuous @ 80%", RobotMode::TURN, 0, 80, 0, false, false},
		{"STOP after turn", RobotMode::STOP, 0, 0, 0, false, true},
	};

	int totalTests = testSequence.size();
	int currentTest = 0;
	int successCount = 0;
	int failCount = 0;

	auto startTime = std::chrono::steady_clock::now();

	std::cout << "\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
	std::cout << "  STARTING AUTOMATED TEST SUITE\n";
	std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n";

	for (const auto& test : testSequence) {
		currentTest++;

		std::cout << "\n┌──────────────────────────────────────────────────────────┐\n";
		std::cout << "│ TEST " << currentTest << "/" << totalTests << ": " << test.description << "\n";
		std::cout << "└──────────────────────────────────────────────────────────┘\n";

		if (test.isReset) {
			// Just send RESET
			sendCommandWithRetry(transmitter, crc, encodeReset(),
			                     "RESET", 0.8, true, true, db, 0.0f, 0.0f, 0.0f);
		} else if (test.isStop) {
			// Send RESET then STOP
			sendCommandWithRetry(transmitter, crc, encodeReset(),
			                     "RESET (preparing for STOP)", 0.8, true, true, db, 0.0f, 0.0f, 0.0f);
			std::this_thread::sleep_for(std::chrono::milliseconds(300));
			sendCommandWithRetry(transmitter, crc, encodeStop(),
			                     "STOP", 0.8, true, true, db, 0.0f, 0.0f, 0.0f);
		} else {
			// Send RESET, then MODE, then COMMAND
			sendCommandWithRetry(transmitter, crc, encodeReset(),
			                     "RESET", 0.8, true, true, db, 0.0f, 0.0f, 0.0f);
			std::this_thread::sleep_for(std::chrono::milliseconds(300));

			// Send mode select
			std::string modeStr;
			switch (test.mode) {
				case RobotMode::DRIVE_FOR_DURATION: modeStr = "Mode: DRIVE FOR DURATION"; break;
				case RobotMode::TURN_FOR_DURATION: modeStr = "Mode: TURN FOR DURATION"; break;
				case RobotMode::DRIVE_FORWARD: modeStr = "Mode: DRIVE CONTINUOUS"; break;
				case RobotMode::TURN: modeStr = "Mode: TURN CONTINUOUS"; break;
				default: modeStr = "Mode: UNKNOWN"; break;
			}
			sendCommandWithRetry(transmitter, crc, encodeModeSelect(test.mode),
			                     modeStr, 0.8, true, true, db, 0.0f, 0.0f, 0.0f);
			std::this_thread::sleep_for(std::chrono::milliseconds(300));

			// Send the actual command
			uint16_t cmdBits;
			std::string cmdDesc = test.description;

			switch (test.mode) {
				case RobotMode::DRIVE_FOR_DURATION:
					cmdBits = encodeDriveForDuration(test.duration, test.speed);
					break;
				case RobotMode::TURN_FOR_DURATION:
					cmdBits = encodeTurnForDuration(test.duration, test.turnSpeed);
					break;
				case RobotMode::DRIVE_FORWARD:
					cmdBits = encodeDriveForward(test.speed);
					break;
				case RobotMode::TURN:
					cmdBits = encodeTurn(test.turnSpeed);
					break;
				default:
					cmdBits = 0;
					break;
			}

			sendCommandWithRetry(transmitter, crc, cmdBits, cmdDesc, 0.8, true, true, db,
			                     test.speed, test.turnSpeed, test.duration);
		}

		// Brief pause between tests
		std::cout << "\n Waiting before next test...\n";
		std::this_thread::sleep_for(std::chrono::milliseconds(2000));
	}

	auto endTime = std::chrono::steady_clock::now();
	auto totalDuration = std::chrono::duration_cast<std::chrono::seconds>(endTime - startTime).count();

	std::cout << "\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
	std::cout << "  TEST SUITE COMPLETE\n";
	std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
	std::cout << "\n  Total tests:     " << totalTests << "\n";
	std::cout << "  Total duration:  " << totalDuration << " seconds\n";
	std::cout << "\n  Results are logged to the database.\n";
	std::cout << "  Run transmission_statistics.py to analyze results.\n";
	std::cout << "\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";

	std::cout << "\nPress ENTER to continue...";
	std::getline(std::cin, input);
}

void printInModeMenu(RobotState state) {
	std::cout << "\n╔════════════════════════════════════════════════════════╗\n";
	std::cout << "║  Current Mode: " << std::left << std::setfill(' ') << std::setw(37) << stateToString(state) << std::setfill('0') << "\n";
	std::cout << "╚════════════════════════════════════════════════════════╝\n";
	std::cout << "\n  C. Send Command (for this mode)\n";
	std::cout << "  S. Send STOP\n";
	std::cout << "  R. RESET (return to mode select)\n";
	std::cout << "  0. Exit Program\n";
	std::cout << "\nChoice: ";
}

void runStateMachineUI(AudioComm::ChordTransmitter& transmitter, CRC& crc, Database* db = nullptr) {
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

			// Handle automated test suite
			if (choice == "T" || choice == "t") {
				std::cin.ignore(10000, '\n');  // Clear input buffer before test
				runAutomatedTestSuite(transmitter, crc, db);
				continue;
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
				std::cout << "Invalid choice.\n";
				validChoice = false;
			}

			if (!validChoice) continue;

			// Send RESET + MODE SELECT with feedback
			std::cout << "\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
			std::cout << " Entering mode: " << stateToString(nextState) << "\n";
			std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
			
			sendCommandWithRetry(transmitter, crc, encodeReset(), "RESET", 0.8, true, true, db, 0.0f, 0.0f, 0.0f);
			sendCommandWithRetry(transmitter, crc, encodeModeSelect(selectedMode), 
			                     "Mode: " + stateToString(nextState), 0.8, true, true, db, 0.0f, 0.0f, 0.0f);
			
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
				sendCommandWithRetry(transmitter, crc, encodeReset(), "RESET", 0.8, true, true, db, 0.0f, 0.0f, 0.0f);
				currentState = RobotState::NO_MODE;
				std::cout << "\n Back to mode select\n";
				continue;
			}

			// Handle STOP
			if (choice == "S" || choice == "s") {
				std::cout << "\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
				std::cout << " Sending STOP\n";
				std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
				// First send RESET to enter MODE_SELECT state
				sendCommandWithRetry(transmitter, crc, encodeReset(), "RESET (preparing for STOP)", 0.8, true, true, db, 0.0f, 0.0f, 0.0f);
				std::this_thread::sleep_for(std::chrono::milliseconds(200));  // Brief pause
				// Then send STOP mode
				sendCommandWithRetry(transmitter, crc, encodeStop(), "STOP", 0.8, true, true, db, 0.0f, 0.0f, 0.0f);
				std::cout << "\n✓ STOP sequence sent (RESET → STOP)\n";
				std::cout << "   Robot should now be stopped and in MODE_SELECT\n";
				// Update state to reflect we're back in mode selection
				currentState = RobotState::NO_MODE;
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
					                     0.8, true, true, db, speed, 0.0f, duration);

				} else if (currentState == RobotState::TURN_FOR_DURATION) {
					float duration, turnRate;
					std::cout << "Enter duration (seconds): ";
					std::cin >> duration;
					std::cout << "Enter turn rate percentage (-100 to 100): ";
					std::cin >> turnRate;
					std::cin.ignore(10000, '\n');  // Clear input buffer

					sendCommandWithRetry(transmitter, crc, encodeTurnForDuration(duration, turnRate),
					                     "Turn " + std::to_string(duration) + "s at " + std::to_string((int)turnRate) + "%",
					                     0.8, true, true, db, 0.0f, turnRate, duration);

				} else if (currentState == RobotState::DRIVE_CONTINUOUS) {
					float speed;
					std::cout << "Enter speed percentage (0-100): ";
					std::cin >> speed;
					std::cin.ignore(10000, '\n');  // Clear input buffer

					sendCommandWithRetry(transmitter, crc, encodeDriveForward(speed),
					                     "Drive forward at " + std::to_string((int)speed) + "%",
					                     0.8, true, true, db, speed, 0.0f, 0.0f);

				} else if (currentState == RobotState::TURN_CONTINUOUS) {
					float turnRate;
					std::cout << "Enter turn rate percentage (-100 to 100): ";
					std::cin >> turnRate;
					std::cin.ignore(10000, '\n');  // Clear input buffer

					sendCommandWithRetry(transmitter, crc, encodeTurn(turnRate),
					                     "Turn at " + std::to_string((int)turnRate) + "%",
					                     0.8, true, true, db, 0.0f, turnRate, 0.0f);
					
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
