#include "../../INCLUDE/command_protocol.h"
#include "../../LIB/audio_transmitter.h"
#include "../../LIB/audio_receiver.h"
#include "../../INCLUDE/CRC.h"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <vector>
#include <fstream>
#include <atomic>
#include <mutex>

// Structure to hold test results
struct TestResult {
	std::string commandType;
	uint16_t rawValue;
	uint16_t encodedValue;
	double transmitStartTime;
	double detectionTime;
	double latency;
	bool detected;
	bool crcValid;

	std::string toString() const {
		std::ostringstream oss;
		oss << std::fixed << std::setprecision(3);
		oss << commandType << ": ";
		if (detected) {
			oss << "Detected in " << latency << "s";
			if (!crcValid) oss << " (CRC FAILED)";
		} else {
			oss << "NOT DETECTED";
		}
		return oss.str();
	}
};

// Global variables for synchronization
std::vector<TestResult> results;
std::mutex resultsMutex;
std::atomic<bool> waitingForDetection(false);
std::atomic<uint16_t> expectedValue(0);
std::atomic<double> transmitStartTime(0.0);

// Get current time in seconds
double getCurrentTime() {
	auto now = std::chrono::high_resolution_clock::now();
	auto duration = now.time_since_epoch();
	return std::chrono::duration<double>(duration).count();
}

// Send a command and start timing
void sendCommandTimed(AudioComm::ChordTransmitter& transmitter, CRC& crc,
					  uint16_t command, const std::string& commandType, double duration) {
	std::cout << "\n>>> Sending: " << commandType << " (0x" << std::hex << std::setw(3)
			  << std::setfill('0') << command << std::dec << ")\n";

	// Encode with CRC
	std::vector<uint16_t> data = {command};
	std::vector<uint16_t> encoded = crc.encode1216(data);

	// Setup test tracking
	TestResult result;
	result.commandType = commandType;
	result.rawValue = command;
	result.encodedValue = encoded[0];
	result.detected = false;
	result.crcValid = false;

	// Add placeholder result (will be updated when detected)
	{
		std::lock_guard<std::mutex> lock(resultsMutex);
		results.push_back(result);
	}

	// Set up detection expectation
	expectedValue = encoded[0];
	transmitStartTime = getCurrentTime();
	result.transmitStartTime = transmitStartTime;
	waitingForDetection = true;

	// Send the command
	AudioComm::ChordTransmitter::Config config;
	config.toneDuration = duration;

	std::cout << "    Transmitting for " << duration << "s...\n";
	if (!transmitter.startTransmitting(encoded[0], config)) {
		std::cerr << "    ERROR: Failed to start transmission!\n";
		waitingForDetection = false;
		return;
	}

	// Wait for transmission to complete
	transmitter.waitForCompletion();

	// Give extra time for detection
	std::this_thread::sleep_for(std::chrono::milliseconds(500));

	// Check if it was detected
	if (waitingForDetection) {
		std::cout << "    ⚠ NOT DETECTED within timeout\n";
		std::lock_guard<std::mutex> lock(resultsMutex);
		results.back().detected = false;
	}

	waitingForDetection = false;
}

void printResults() {
	std::cout << "\n╔════════════════════════════════════════════════════════════════╗\n";
	std::cout << "║                    LATENCY TEST RESULTS                        ║\n";
	std::cout << "╚════════════════════════════════════════════════════════════════╝\n\n";

	double totalLatency = 0.0;
	int detectedCount = 0;
	int failedCRC = 0;

	std::cout << std::fixed << std::setprecision(3);
	std::cout << std::left;
	std::cout << std::setw(30) << "Command Type"
			  << std::setw(12) << "Raw Value"
			  << std::setw(12) << "Latency (s)"
			  << std::setw(10) << "Status\n";
	std::cout << std::string(64, '─') << "\n";

	for (const auto& result : results) {
		std::cout << std::setw(30) << result.commandType
				  << std::setw(12) << result.rawValue;

		if (result.detected) {
			std::cout << std::setw(12) << result.latency;
			if (result.crcValid) {
				std::cout << "✓ OK\n";
				totalLatency += result.latency;
				detectedCount++;
			} else {
				std::cout << "✗ CRC FAIL\n";
				failedCRC++;
			}
		} else {
			std::cout << std::setw(12) << "N/A"
					  << "✗ MISSED\n";
		}
	}

	std::cout << std::string(64, '─') << "\n";
	std::cout << "\nSummary:\n";
	std::cout << "  Total commands sent: " << results.size() << "\n";
	std::cout << "  Successfully detected: " << detectedCount << " ("
			  << (100.0 * detectedCount / results.size()) << "%)\n";
	std::cout << "  CRC failures: " << failedCRC << "\n";
	std::cout << "  Missed: " << (results.size() - detectedCount - failedCRC) << "\n";

	if (detectedCount > 0) {
		std::cout << "\n  Average latency: " << (totalLatency / detectedCount) << "s\n";
		std::cout << "  Min latency: " << std::fixed << std::setprecision(3);

		double minLat = 999.0, maxLat = 0.0;
		for (const auto& r : results) {
			if (r.detected && r.crcValid) {
				minLat = std::min(minLat, r.latency);
				maxLat = std::max(maxLat, r.latency);
			}
		}
		std::cout << minLat << "s\n";
		std::cout << "  Max latency: " << maxLat << "s\n";
	}
	std::cout << "\n";
}

void saveResultsToFile(const std::string& filename) {
	std::ofstream file(filename);
	if (!file.is_open()) {
		std::cerr << "Failed to open file: " << filename << "\n";
		return;
	}

	file << "Command Type,Raw Value,Encoded Value,Latency (s),Detected,CRC Valid\n";
	for (const auto& result : results) {
		file << result.commandType << ","
			 << result.rawValue << ","
			 << result.encodedValue << ",";

		if (result.detected) {
			file << std::fixed << std::setprecision(3) << result.latency;
		} else {
			file << "N/A";
		}

		file << "," << (result.detected ? "Yes" : "No")
			 << "," << (result.crcValid ? "Yes" : "No") << "\n";
	}

	file.close();
	std::cout << "Results saved to: " << filename << "\n";
}

int main() {
	std::cout << "╔════════════════════════════════════════════════════════════════╗\n";
	std::cout << "║            Protocol Latency & Reliability Test                ║\n";
	std::cout << "╚════════════════════════════════════════════════════════════════╝\n\n";

	// Initialize transmitter and receiver
	AudioComm::ChordTransmitter transmitter;
	AudioComm::ChordReceiver receiver;
	CRC crc;

	std::cout << "Initializing audio receiver...\n";

	// Setup receiver callback
	AudioComm::ChordReceiver::Config recvConfig;
	bool receiverStarted = receiver.startReceiving(recvConfig,
		[&crc](const AudioComm::ChordReceiver::Detection& det) {
			if (!waitingForDetection) {
				return; // Not currently testing
			}

			// Check if this is the expected value
			if (det.value == expectedValue) {
				double detectionTime = getCurrentTime();
				double latency = detectionTime - transmitStartTime;

				// Verify CRC
				bool isValid = crc.verify(det.value);

				std::cout << "    ✓ Detected after " << std::fixed << std::setprecision(3)
						  << latency << "s";
				if (!isValid) {
					std::cout << " (CRC FAILED)";
				}
				std::cout << "\n";

				// Update the result
				{
					std::lock_guard<std::mutex> lock(resultsMutex);
					if (!results.empty()) {
						results.back().detected = true;
						results.back().crcValid = isValid;
						results.back().detectionTime = detectionTime;
						results.back().latency = latency;
					}
				}

				waitingForDetection = false;
			}
		});

	if (!receiverStarted) {
		std::cerr << "ERROR: Failed to start receiver!\n";
		return 1;
	}

	std::cout << "Audio receiver started.\n";
	std::cout << "Audio transmitter ready.\n\n";

	std::cout << "Starting latency test sequence...\n";
	std::cout << "Testing each command type multiple times.\n";
	std::cout << std::string(64, '=') << "\n";

	// Test sequence
	std::cout << "\n[TEST 1] RESET Signal (3 repetitions)\n";
	std::cout << std::string(64, '-') << "\n";
	for (int i = 0; i < 3; i++) {
		sendCommandTimed(transmitter, crc, encodeReset(), "RESET", 1.0);
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}

	std::cout << "\n[TEST 2] Mode Select Commands (2 repetitions each)\n";
	std::cout << std::string(64, '-') << "\n";
	RobotMode modes[] = {
		RobotMode::DRIVE_FOR_DURATION,
		RobotMode::TURN_FOR_DURATION,
		RobotMode::DRIVE_FORWARD,
		RobotMode::TURN,
		RobotMode::STOP
	};

	for (auto mode : modes) {
		for (int i = 0; i < 2; i++) {
			sendCommandTimed(transmitter, crc, encodeModeSelect(mode),
						   "Mode: " + CommandProtocol::modeToString(mode), 2.0);
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
		}
	}

	std::cout << "\n[TEST 3] Drive For Duration Commands (various values)\n";
	std::cout << std::string(64, '-') << "\n";
	float driveTests[][2] = {{2.0f, 50.0f}, {4.0f, 75.0f}, {1.0f, 25.0f}};
	for (auto test : driveTests) {
		uint16_t cmd = encodeDriveForDuration(test[0], test[1]);
		DriveForDurationCommand decoded = DriveForDurationCommand::decode(cmd);
		sendCommandTimed(transmitter, crc, cmd,
						"Drive(" + std::to_string((int)test[0]) + "s, " +
						std::to_string((int)test[1]) + "%)", 1.0);
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}

	std::cout << "\n[TEST 4] Turn For Duration Commands (various values)\n";
	std::cout << std::string(64, '-') << "\n";
	float turnTests[][2] = {{2.0f, 50.0f}, {1.5f, -75.0f}, {3.0f, 0.0f}};
	for (auto test : turnTests) {
		uint16_t cmd = encodeTurnForDuration(test[0], test[1]);
		sendCommandTimed(transmitter, crc, cmd,
						"Turn(" + std::to_string((int)test[0]) + "s, " +
						std::to_string((int)test[1]) + "%)", 1.0);
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}

	std::cout << "\n[TEST 5] Drive Forward Commands (high resolution)\n";
	std::cout << std::string(64, '-') << "\n";
	float driveForwardTests[] = {25.5f, 50.0f, 75.5f};
	for (auto speed : driveForwardTests) {
		uint16_t cmd = encodeDriveForward(speed);
		sendCommandTimed(transmitter, crc, cmd,
						"DriveForward(" + std::to_string((int)speed) + "%)", 1.0);
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}

	std::cout << "\n[TEST 6] Turn Commands (high resolution)\n";
	std::cout << std::string(64, '-') << "\n";
	float turnContinuousTests[] = {-80.0f, 0.0f, 80.0f};
	for (auto rate : turnContinuousTests) {
		uint16_t cmd = encodeTurn(rate);
		sendCommandTimed(transmitter, crc, cmd,
						"Turn(" + std::to_string((int)rate) + "%)", 1.0);
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}

	// Stop receiver
	std::cout << "\n" << std::string(64, '=') << "\n";
	std::cout << "Test sequence complete. Stopping receiver...\n";
	receiver.stop();

	// Display and save results
	printResults();

	std::string filename = "../BUILD/protocol_latency_results.csv";
	saveResultsToFile(filename);

	return 0;
}
