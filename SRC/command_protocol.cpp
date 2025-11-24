#include "../INCLUDE/command_protocol.h"
#include <sstream>
#include <iomanip>
#include <cmath>
#include <algorithm>

// ============================================================================
// DriveForDurationCommand Implementation
// ============================================================================

DriveForDurationCommand DriveForDurationCommand::create(float durationSec, float speedPercent) {
	DriveForDurationCommand cmd;

	// Clamp and convert duration (0-8 seconds to 0-63)
	durationSec = std::max(0.0f, std::min(8.0f, durationSec));
	cmd.durationRaw = static_cast<uint8_t>((durationSec / 8.0f) * 63.0f + 0.5f);

	// Clamp and convert speed (0-100% to 0-63)
	speedPercent = std::max(0.0f, std::min(100.0f, speedPercent));
	cmd.speedRaw = static_cast<uint8_t>((speedPercent / 100.0f) * 63.0f + 0.5f);

	return cmd;
}

uint16_t DriveForDurationCommand::encode() const {
	// [6 bits duration] [6 bits speed]
	return ((durationRaw & 0x3F) << 6) | (speedRaw & 0x3F);
}

DriveForDurationCommand DriveForDurationCommand::decode(uint16_t bits) {
	DriveForDurationCommand cmd;
	cmd.durationRaw = (bits >> 6) & 0x3F;
	cmd.speedRaw = bits & 0x3F;
	return cmd;
}

std::string DriveForDurationCommand::toString() const {
	std::ostringstream oss;
	oss << "DriveForDuration(duration=" << std::fixed << std::setprecision(2)
		<< getDurationSeconds() << "s, speed=" << getSpeedPercent() << "%)";
	return oss.str();
}

// ============================================================================
// TurnForDurationCommand Implementation
// ============================================================================

TurnForDurationCommand TurnForDurationCommand::create(float durationSec, float turnRatePercent) {
	TurnForDurationCommand cmd;

	// Clamp and convert duration (0-8 seconds to 0-63)
	durationSec = std::max(0.0f, std::min(8.0f, durationSec));
	cmd.durationRaw = static_cast<uint8_t>((durationSec / 8.0f) * 63.0f + 0.5f);

	// Clamp and convert turn rate (-100% to +100% to 0-63)
	turnRatePercent = std::max(-100.0f, std::min(100.0f, turnRatePercent));
	cmd.turnRateRaw = static_cast<uint8_t>(((turnRatePercent / 100.0f) + 1.0f) * 31.5f + 0.5f);

	return cmd;
}

uint16_t TurnForDurationCommand::encode() const {
	// [6 bits duration] [6 bits turn rate]
	return ((durationRaw & 0x3F) << 6) | (turnRateRaw & 0x3F);
}

TurnForDurationCommand TurnForDurationCommand::decode(uint16_t bits) {
	TurnForDurationCommand cmd;
	cmd.durationRaw = (bits >> 6) & 0x3F;
	cmd.turnRateRaw = bits & 0x3F;
	return cmd;
}

std::string TurnForDurationCommand::toString() const {
	std::ostringstream oss;
	oss << "TurnForDuration(duration=" << std::fixed << std::setprecision(2)
		<< getDurationSeconds() << "s, turnRate=" << getTurnRatePercent() << "%)";
	return oss.str();
}

// ============================================================================
// DriveForwardCommand Implementation
// ============================================================================

DriveForwardCommand DriveForwardCommand::create(float speedPercent) {
	DriveForwardCommand cmd;

	// Clamp and convert speed (0-100% to 0-4095)
	speedPercent = std::max(0.0f, std::min(100.0f, speedPercent));
	cmd.speedRaw = static_cast<uint16_t>((speedPercent / 100.0f) * 4095.0f + 0.5f);

	return cmd;
}

uint16_t DriveForwardCommand::encode() const {
	return speedRaw & 0xFFF;  // 12 bits
}

DriveForwardCommand DriveForwardCommand::decode(uint16_t bits) {
	DriveForwardCommand cmd;
	cmd.speedRaw = bits & 0xFFF;
	return cmd;
}

std::string DriveForwardCommand::toString() const {
	std::ostringstream oss;
	oss << "DriveForward(speed=" << std::fixed << std::setprecision(2)
		<< getSpeedPercent() << "%)";
	return oss.str();
}

// ============================================================================
// TurnCommand Implementation
// ============================================================================

TurnCommand TurnCommand::create(float turnRatePercent) {
	TurnCommand cmd;

	// Clamp and convert turn rate (-100% to +100% to 0-4095)
	turnRatePercent = std::max(-100.0f, std::min(100.0f, turnRatePercent));
	cmd.turnRateRaw = static_cast<uint16_t>(((turnRatePercent / 100.0f) + 1.0f) * 2047.5f + 0.5f);

	return cmd;
}

uint16_t TurnCommand::encode() const {
	return turnRateRaw & 0xFFF;  // 12 bits
}

TurnCommand TurnCommand::decode(uint16_t bits) {
	TurnCommand cmd;
	cmd.turnRateRaw = bits & 0xFFF;
	return cmd;
}

std::string TurnCommand::toString() const {
	std::ostringstream oss;
	oss << "Turn(turnRate=" << std::fixed << std::setprecision(2)
		<< getTurnRatePercent() << "%)";
	return oss.str();
}

// ============================================================================
// CommandProtocol Implementation
// ============================================================================

CommandProtocol::CommandProtocol()
	: currentMode(RobotMode::MODE_SELECT)
	, waitingForModeSelect(false)
{
}

void CommandProtocol::processCommand(uint16_t bits) {
	// Check for reset signal
	if (bits == RESET_SIGNAL) {
		reset();
		return;
	}

	// If waiting for mode select, interpret bits as mode selection
	if (waitingForModeSelect) {
		RobotMode newMode = static_cast<RobotMode>(bits);

		// Validate mode selection
		if (newMode >= RobotMode::MODE_SELECT && newMode <= RobotMode::STOP) {
			currentMode = newMode;
			waitingForModeSelect = false;

			if (modeChangeCallback) {
				modeChangeCallback(newMode);
			}

			// If STOP mode was selected, execute immediately
			if (newMode == RobotMode::STOP && stopCallback) {
				stopCallback();
			}
		} else {
			// Invalid mode, go back to mode select
			reset();
		}
		return;
	}

	// Process command based on current mode
	switch (currentMode) {
		case RobotMode::DRIVE_FOR_DURATION:
			if (driveForDurationCallback) {
				DriveForDurationCommand cmd = DriveForDurationCommand::decode(bits);
				driveForDurationCallback(cmd);
			}
			break;

		case RobotMode::TURN_FOR_DURATION:
			if (turnForDurationCallback) {
				TurnForDurationCommand cmd = TurnForDurationCommand::decode(bits);
				turnForDurationCallback(cmd);
			}
			break;

		case RobotMode::DRIVE_FORWARD:
			if (driveForwardCallback) {
				DriveForwardCommand cmd = DriveForwardCommand::decode(bits);
				driveForwardCallback(cmd);
			}
			break;

		case RobotMode::TURN:
			if (turnCallback) {
				TurnCommand cmd = TurnCommand::decode(bits);
				turnCallback(cmd);
			}
			break;

		case RobotMode::STOP:
			if (stopCallback) {
				stopCallback();
			}
			break;

		case RobotMode::MODE_SELECT:
			// Should not reach here, but treat as mode selection
			processCommand(bits);  // Recursive call will handle mode selection
			break;
	}
}

void CommandProtocol::setDriveForDurationCallback(std::function<void(const DriveForDurationCommand&)> callback) {
	driveForDurationCallback = callback;
}

void CommandProtocol::setTurnForDurationCallback(std::function<void(const TurnForDurationCommand&)> callback) {
	turnForDurationCallback = callback;
}

void CommandProtocol::setDriveForwardCallback(std::function<void(const DriveForwardCommand&)> callback) {
	driveForwardCallback = callback;
}

void CommandProtocol::setTurnCallback(std::function<void(const TurnCommand&)> callback) {
	turnCallback = callback;
}

void CommandProtocol::setStopCallback(std::function<void()> callback) {
	stopCallback = callback;
}

void CommandProtocol::setModeChangeCallback(std::function<void(RobotMode)> callback) {
	modeChangeCallback = callback;
}

void CommandProtocol::reset() {
	currentMode = RobotMode::MODE_SELECT;
	waitingForModeSelect = true;

	if (modeChangeCallback) {
		modeChangeCallback(RobotMode::MODE_SELECT);
	}
}

std::string CommandProtocol::modeToString(RobotMode mode) {
	switch (mode) {
		case RobotMode::MODE_SELECT:        return "MODE_SELECT";
		case RobotMode::DRIVE_FOR_DURATION: return "DRIVE_FOR_DURATION";
		case RobotMode::TURN_FOR_DURATION:  return "TURN_FOR_DURATION";
		case RobotMode::DRIVE_FORWARD:      return "DRIVE_FORWARD";
		case RobotMode::TURN:               return "TURN";
		case RobotMode::STOP:               return "STOP";
		default:                            return "UNKNOWN";
	}
}

// ============================================================================
// Utility Functions
// ============================================================================

uint16_t encodeDriveForDuration(float durationSec, float speedPercent) {
	return DriveForDurationCommand::create(durationSec, speedPercent).encode();
}

uint16_t encodeTurnForDuration(float durationSec, float turnRatePercent) {
	return TurnForDurationCommand::create(durationSec, turnRatePercent).encode();
}

uint16_t encodeDriveForward(float speedPercent) {
	return DriveForwardCommand::create(speedPercent).encode();
}

uint16_t encodeTurn(float turnRatePercent) {
	return TurnCommand::create(turnRatePercent).encode();
}

uint16_t encodeModeSelect(RobotMode mode) {
	return static_cast<uint16_t>(mode);
}

uint16_t encodeStop() {
	return static_cast<uint16_t>(RobotMode::STOP);
}

uint16_t encodeReset() {
	return RESET_SIGNAL;
}
