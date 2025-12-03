#ifndef COMMAND_PROTOCOL_H
#define COMMAND_PROTOCOL_H

#include <cstdint>
#include <string>
#include <functional>

// Protocol constants
constexpr uint16_t RESET_SIGNAL = 0x000;  // All zeros - triggers mode select
constexpr uint16_t MAX_12BIT_VALUE = 0xFFF;

// Operating modes (used after reset signal)
enum class RobotMode : uint16_t {
	MODE_SELECT         = 0x000,  // Waiting for mode selection
	DRIVE_FOR_DURATION  = 0x001,  // Drive at speed for duration
	TURN_FOR_DURATION   = 0x002,  // Turn at rate for duration
	DRIVE_FORWARD       = 0x003,  // Continuous drive at speed
	TURN                = 0x004,  // Continuous turn at rate
	STOP                = 0x005,  // Stop command (no additional data needed)
};

// Drive For Duration command (Mode 0x001)
// Format: [6 bits duration] [6 bits speed]
// Duration: 0-63 maps to 0-8 seconds (63/63 * 8 = 8.0 seconds)
// Speed: 0-63 maps to 0-100% (63/63 * 100 = 100%)
struct DriveForDurationCommand {
	uint8_t durationRaw;  // 0-63 (6 bits)
	uint8_t speedRaw;     // 0-63 (6 bits)

	// Convert to actual values
	float getDurationSeconds() const { return (durationRaw / 63.0f) * 8.0f; }
	float getSpeedPercent() const { return (speedRaw / 63.0f) * 100.0f; }

	// Create from actual values
	static DriveForDurationCommand create(float durationSec, float speedPercent);

	// Encode to 12-bit value
	uint16_t encode() const;

	// Decode from 12-bit value
	static DriveForDurationCommand decode(uint16_t bits);

	std::string toString() const;
};

// Turn For Duration command (Mode 0x002)
// Format: [6 bits duration] [6 bits turn rate]
// Duration: 0-63 maps to 0-8 seconds
// Turn rate: 0-63 maps to -100% to +100% (0=full left, 31=straight, 63=full right)
struct TurnForDurationCommand {
	uint8_t durationRaw;  // 0-63 (6 bits)
	uint8_t turnRateRaw;  // 0-63 (6 bits)

	// Convert to actual values
	float getDurationSeconds() const { return (durationRaw / 63.0f) * 8.0f; }
	float getTurnRatePercent() const { return ((turnRateRaw / 31.5f) - 1.0f) * 100.0f; }  // Maps 0-63 to -100 to +100

	// Create from actual values
	static TurnForDurationCommand create(float durationSec, float turnRatePercent);

	// Encode to 12-bit value
	uint16_t encode() const;

	// Decode from 12-bit value
	static TurnForDurationCommand decode(uint16_t bits);

	std::string toString() const;
};

// Drive Forward command (Mode 0x003)
// Format: [12 bits speed] - full resolution
// Speed: 0-4095 maps to 0-100%
struct DriveForwardCommand {
	uint16_t speedRaw;  // 0-4095 (12 bits)

	// Convert to actual value
	float getSpeedPercent() const { return (speedRaw / 4095.0f) * 100.0f; }

	// Create from actual value
	static DriveForwardCommand create(float speedPercent);

	// Encode to 12-bit value
	uint16_t encode() const;

	// Decode from 12-bit value
	static DriveForwardCommand decode(uint16_t bits);

	std::string toString() const;
};

// Turn command (Mode 0x004)
// Format: [12 bits turn rate] - full resolution
// Turn rate: 0-4095 maps to -100% to +100% (0=full left, 2047=straight, 4095=full right)
struct TurnCommand {
	uint16_t turnRateRaw;  // 0-4095 (12 bits)

	// Convert to actual valuegetCurrentMode
	float getTurnRatePercent() const { return ((turnRateRaw / 2047.5f) - 1.0f) * 100.0f; }  // Maps 0-4095 to -100 to +100

	// Create from actual value
	static TurnCommand create(float turnRatePercent);

	// Encode to 12-bit value
	uint16_t encode() const;

	// Decode from 12-bit value
	static TurnCommand decode(uint16_t bits);

	std::string toString() const;
};

// Main protocol state machine
class CommandProtocol {
private:
	RobotMode currentMode;
	bool waitingForModeSelect;

	// Callback functions for each mode
	std::function<void(const DriveForDurationCommand&)> driveForDurationCallback;
	std::function<void(const TurnForDurationCommand&)> turnForDurationCallback;
	std::function<void(const DriveForwardCommand&)> driveForwardCallback;
	std::function<void(const TurnCommand&)> turnCallback;
	std::function<void()> stopCallback;
	std::function<void(RobotMode)> modeChangeCallback;

public:
	CommandProtocol();

	// Process incoming 12-bit command
	void processCommand(uint16_t bits);

	// Get current mode
	RobotMode getCurrentMode() const { return currentMode; }

	// Check if waiting for mode selection
	bool isWaitingForModeSelect() const { return waitingForModeSelect; }

	// Register callbacks for different command types
	void setDriveForDurationCallback(std::function<void(const DriveForDurationCommand&)> callback);
	void setTurnForDurationCallback(std::function<void(const TurnForDurationCommand&)> callback);
	void setDriveForwardCallback(std::function<void(const DriveForwardCommand&)> callback);
	void setTurnCallback(std::function<void(const TurnCommand&)> callback);
	void setStopCallback(std::function<void()> callback);
	void setModeChangeCallback(std::function<void(RobotMode)> callback);

	// Reset to mode select state
	void reset();

	// Utility: Get mode name as string
	static std::string modeToString(RobotMode mode);
};

// Utility functions for encoding commands (for sender side)
uint16_t encodeDriveForDuration(float durationSec, float speedPercent);
uint16_t encodeTurnForDuration(float durationSec, float turnRatePercent);
uint16_t encodeDriveForward(float speedPercent);
uint16_t encodeTurn(float turnRatePercent);
uint16_t encodeModeSelect(RobotMode mode);
uint16_t encodeStop();  // Returns mode code for STOP
uint16_t encodeReset(); // Returns RESET_SIGNAL

#endif // COMMAND_PROTOCOL_H
