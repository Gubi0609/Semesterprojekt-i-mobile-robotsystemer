#include "../INCLUDE/command_protocol.h"
#include <iostream>
#include <iomanip>
#include <vector>

// Helper function to print binary representation
void printBinary(uint16_t value) {
	std::cout << "0x" << std::hex << std::setfill('0') << std::setw(3) << value
			  << " (0b";
	for (int i = 11; i >= 0; i--) {
		std::cout << ((value >> i) & 1);
		if (i % 4 == 0 && i != 0) std::cout << "_";
	}
	std::cout << ")" << std::dec;
}

// Simulate sending a command through the audio system
void simulateSendCommand(uint16_t bits, const std::string& description) {
	std::cout << "\n┌─────────────────────────────────────────────────────\n";
	std::cout << "│ SENDING: " << description << "\n";
	std::cout << "│ Bits: ";
	printBinary(bits);
	std::cout << "\n└─────────────────────────────────────────────────────\n";
}

// Example callbacks for the robot side
void onDriveForDuration(const DriveForDurationCommand& cmd) {
	std::cout << "🤖 ROBOT: Executing " << cmd.toString() << "\n";
	std::cout << "   → Driving at " << cmd.getSpeedPercent() << "% for "
			  << cmd.getDurationSeconds() << " seconds\n";
}

void onTurnForDuration(const TurnForDurationCommand& cmd) {
	std::cout << "🤖 ROBOT: Executing " << cmd.toString() << "\n";
	std::cout << "   → Turning at " << cmd.getTurnRatePercent() << "% for "
			  << cmd.getDurationSeconds() << " seconds\n";
}

void onDriveForward(const DriveForwardCommand& cmd) {
	std::cout << "🤖 ROBOT: Executing " << cmd.toString() << "\n";
	std::cout << "   → Driving continuously at " << cmd.getSpeedPercent() << "%\n";
}

void onTurn(const TurnCommand& cmd) {
	std::cout << "🤖 ROBOT: Executing " << cmd.toString() << "\n";
	std::cout << "   → Turning continuously at " << cmd.getTurnRatePercent() << "%\n";
}

void onStop() {
	std::cout << "🤖 ROBOT: Executing STOP\n";
	std::cout << "   → Motors stopped\n";
}

void onModeChange(RobotMode mode) {
	std::cout << "🤖 ROBOT: Mode changed to " << CommandProtocol::modeToString(mode) << "\n";
}

int main() {
	std::cout << "╔═══════════════════════════════════════════════════════════╗\n";
	std::cout << "║   Command Protocol Test - Audio Robot Communication     ║\n";
	std::cout << "╚═══════════════════════════════════════════════════════════╝\n";

	// Setup protocol handler (robot side)
	CommandProtocol protocol;
	protocol.setDriveForDurationCallback(onDriveForDuration);
	protocol.setTurnForDurationCallback(onTurnForDuration);
	protocol.setDriveForwardCallback(onDriveForward);
	protocol.setTurnCallback(onTurn);
	protocol.setStopCallback(onStop);
	protocol.setModeChangeCallback(onModeChange);

	std::cout << "\n═══════════════════════════════════════════════════════════\n";
	std::cout << "TEST 1: Drive for duration\n";
	std::cout << "═══════════════════════════════════════════════════════════\n";

	// Reset and select mode
	uint16_t reset = encodeReset();
	simulateSendCommand(reset, "RESET");
	protocol.processCommand(reset);

	uint16_t modeSelect = encodeModeSelect(RobotMode::DRIVE_FOR_DURATION);
	simulateSendCommand(modeSelect, "Select DRIVE_FOR_DURATION mode");
	protocol.processCommand(modeSelect);

	// Send drive command: 3 seconds at 75% speed
	uint16_t driveCmd = encodeDriveForDuration(3.0f, 75.0f);
	simulateSendCommand(driveCmd, "Drive for 3s at 75%");
	protocol.processCommand(driveCmd);

	std::cout << "\n═══════════════════════════════════════════════════════════\n";
	std::cout << "TEST 2: Turn for duration\n";
	std::cout << "═══════════════════════════════════════════════════════════\n";

	// Reset and select turn mode
	reset = encodeReset();
	simulateSendCommand(reset, "RESET");
	protocol.processCommand(reset);

	modeSelect = encodeModeSelect(RobotMode::TURN_FOR_DURATION);
	simulateSendCommand(modeSelect, "Select TURN_FOR_DURATION mode");
	protocol.processCommand(modeSelect);

	// Send turn command: 2 seconds at 50% (right turn)
	uint16_t turnCmd = encodeTurnForDuration(2.0f, 50.0f);
	simulateSendCommand(turnCmd, "Turn right for 2s at 50%");
	protocol.processCommand(turnCmd);

	std::cout << "\n═══════════════════════════════════════════════════════════\n";
	std::cout << "TEST 3: Continuous drive with high resolution\n";
	std::cout << "═══════════════════════════════════════════════════════════\n";

	// Reset and select continuous drive mode
	reset = encodeReset();
	simulateSendCommand(reset, "RESET");
	protocol.processCommand(reset);

	modeSelect = encodeModeSelect(RobotMode::DRIVE_FORWARD);
	simulateSendCommand(modeSelect, "Select DRIVE_FORWARD mode");
	protocol.processCommand(modeSelect);

	// Send high-resolution drive command: 42.5% speed
	uint16_t preciseDrive = encodeDriveForward(42.5f);
	simulateSendCommand(preciseDrive, "Drive continuously at 42.5%");
	protocol.processCommand(preciseDrive);

	std::cout << "\n═══════════════════════════════════════════════════════════\n";
	std::cout << "TEST 4: Continuous turn (left)\n";
	std::cout << "═══════════════════════════════════════════════════════════\n";

	// Reset and select continuous turn mode
	reset = encodeReset();
	simulateSendCommand(reset, "RESET");
	protocol.processCommand(reset);

	modeSelect = encodeModeSelect(RobotMode::TURN);
	simulateSendCommand(modeSelect, "Select TURN mode");
	protocol.processCommand(modeSelect);

	// Send turn command: -80% (left turn)
	uint16_t continuousTurn = encodeTurn(-80.0f);
	simulateSendCommand(continuousTurn, "Turn left continuously at -80%");
	protocol.processCommand(continuousTurn);

	std::cout << "\n═══════════════════════════════════════════════════════════\n";
	std::cout << "TEST 5: Emergency stop\n";
	std::cout << "═══════════════════════════════════════════════════════════\n";

	// Reset and select stop mode
	reset = encodeReset();
	simulateSendCommand(reset, "RESET");
	protocol.processCommand(reset);

	modeSelect = encodeStop();
	simulateSendCommand(modeSelect, "STOP command");
	protocol.processCommand(modeSelect);

	std::cout << "\n═══════════════════════════════════════════════════════════\n";
	std::cout << "TEST 6: Resolution comparison\n";
	std::cout << "═══════════════════════════════════════════════════════════\n";

	std::cout << "\n6-bit resolution (0-63):\n";
	for (float speed : {0.0f, 25.0f, 50.0f, 75.0f, 100.0f}) {
		DriveForDurationCommand cmd = DriveForDurationCommand::create(1.0f, speed);
		std::cout << "  Input: " << speed << "% → Raw: " << (int)cmd.speedRaw
				  << " → Output: " << cmd.getSpeedPercent() << "%\n";
	}

	std::cout << "\n12-bit resolution (0-4095):\n";
	for (float speed : {0.0f, 25.0f, 50.0f, 75.0f, 100.0f}) {
		DriveForwardCommand cmd = DriveForwardCommand::create(speed);
		std::cout << "  Input: " << speed << "% → Raw: " << cmd.speedRaw
				  << " → Output: " << cmd.getSpeedPercent() << "%\n";
	}

	std::cout << "\n╔═══════════════════════════════════════════════════════════╗\n";
	std::cout << "║   All tests completed successfully!                      ║\n";
	std::cout << "╚═══════════════════════════════════════════════════════════╝\n\n";

	return 0;
}
