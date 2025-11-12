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
	int detectionCount;

	std::string toString() const {
		std::ostringstream oss;
		oss << std::fixed << std::setprecision(3);
		oss << commandType << ": ";
		if (detected) {
			oss << "Detected in " << latency << "s (count: " << detectionCount << ")";
			if (!crcValid) oss << " (CRC FAILED)";
		} else {
			oss << "NOT DETECTED";
		}
		return oss.str();
	}
};

// Configuration preset
struct ReceiverPreset {
	std::string name;
	int minDetections;
	double consistencyWindow;
	double updateRate;
	std::string description;
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
	std::cout << ">>> " << commandType << " (0x" << std::hex << std::setw(3)
			  << std::setfill('0') << command << std::dec << ")... " << std::flush;

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
	result.detectionCount = 0;

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

	if (!transmitter.startTransmitting(encoded[0], config)) {
		std::cerr << "TX FAILED\n";
		waitingForDetection = false;
		return;
	}

	// Wait for transmission to complete
	transmitter.waitForCompletion();

	// Give extra time for detection
	std::this_thread::sleep_for(std::chrono::milliseconds(500));

	// Check if it was detected
	if (waitingForDetection) {
		std::cout << "MISSED\n";
		std::lock_guard<std::mutex> lock(resultsMutex);
		results.back().detected = false;
	}

	waitingForDetection = false;
}

void printResults(const ReceiverPreset& preset) {
	std::cout << "\n╔════════════════════════════════════════════════════════════════╗\n";
	std::cout << "║                  TEST RESULTS - " << std::left << std::setw(30) << preset.name << "║\n";
	std::cout << "╚════════════════════════════════════════════════════════════════╝\n";
	std::cout << "Config: minDetections=" << preset.minDetections
			  << ", window=" << preset.consistencyWindow << "s"
			  << ", updateRate=" << preset.updateRate << "Hz\n\n";

	double totalLatency = 0.0;
	int detectedCount = 0;
	int failedCRC = 0;

	std::cout << std::fixed << std::setprecision(3);

	for (const auto& result : results) {
		if (result.detected) {
			if (result.crcValid) {
				totalLatency += result.latency;
				detectedCount++;
			} else {
				failedCRC++;
			}
		}
	}

	std::cout << "Summary:\n";
	std::cout << "  Total commands: " << results.size() << "\n";
	std::cout << "  Detected: " << detectedCount << " ("
			  << std::fixed << std::setprecision(1)
			  << (100.0 * detectedCount / results.size()) << "%)\n";
	std::cout << "  CRC failures: " << failedCRC << "\n";
	std::cout << "  Missed: " << (results.size() - detectedCount - failedCRC) << "\n";

	if (detectedCount > 0) {
		std::cout << std::fixed << std::setprecision(3);
		std::cout << "\nLatency Statistics:\n";
		std::cout << "  Average: " << (totalLatency / detectedCount) << "s\n";

		double minLat = 999.0, maxLat = 0.0;
		for (const auto& r : results) {
			if (r.detected && r.crcValid) {
				minLat = std::min(minLat, r.latency);
				maxLat = std::max(maxLat, r.latency);
			}
		}
		std::cout << "  Min: " << minLat << "s\n";
		std::cout << "  Max: " << maxLat << "s\n";
		std::cout << "  Range: " << (maxLat - minLat) << "s\n";
	}
	std::cout << "\n";
}

void saveResultsToFile(const std::string& filename, const ReceiverPreset& preset) {
	std::ofstream file(filename);
	if (!file.is_open()) {
		std::cerr << "Failed to open file: " << filename << "\n";
		return;
	}

	file << "# Configuration: " << preset.name << "\n";
	file << "# minDetections=" << preset.minDetections
		 << ", consistencyWindow=" << preset.consistencyWindow
		 << ", updateRate=" << preset.updateRate << "\n";
	file << "Command Type,Raw Value,Encoded Value,Latency (s),Detection Count,Detected,CRC Valid\n";

	for (const auto& result : results) {
		file << result.commandType << ","
			 << result.rawValue << ","
			 << result.encodedValue << ",";

		if (result.detected) {
			file << std::fixed << std::setprecision(4) << result.latency;
		} else {
			file << "N/A";
		}

		file << "," << result.detectionCount
			 << "," << (result.detected ? "Yes" : "No")
			 << "," << (result.crcValid ? "Yes" : "No") << "\n";
	}

	file.close();
	std::cout << "Results saved to: " << filename << "\n";
}

void runTestSequence(AudioComm::ChordTransmitter& transmitter, CRC& crc) {
	std::cout << "\nRunning test sequence...\n";
	std::cout << std::string(64, '=') << "\n";

	// Quick test with various command types
	std::cout << "[RESET Signals]\n";
	for (int i = 0; i < 3; i++) {
		sendCommandTimed(transmitter, crc, encodeReset(), "RESET", 1.0);
	}

	std::cout << "\n[Mode Selects]\n";
	sendCommandTimed(transmitter, crc, encodeModeSelect(RobotMode::DRIVE_FOR_DURATION), "Mode:DRIVE_DUR", 2.0);
	sendCommandTimed(transmitter, crc, encodeModeSelect(RobotMode::TURN_FOR_DURATION), "Mode:TURN_DUR", 2.0);
	sendCommandTimed(transmitter, crc, encodeModeSelect(RobotMode::DRIVE_FORWARD), "Mode:DRIVE_FWD", 2.0);

	std::cout << "\n[Data Commands]\n";
	sendCommandTimed(transmitter, crc, encodeDriveForDuration(2.0f, 50.0f), "Drive(2s,50%)", 1.0);
	sendCommandTimed(transmitter, crc, encodeTurnForDuration(1.5f, -75.0f), "Turn(1.5s,-75%)", 1.0);
	sendCommandTimed(transmitter, crc, encodeDriveForward(42.5f), "DriveFwd(42.5%)", 1.0);
	sendCommandTimed(transmitter, crc, encodeTurn(80.0f), "Turn(80%)", 1.0);

	std::cout << std::string(64, '=') << "\n";
}

int main(int argc, char* argv[]) {
	std::cout << "╔════════════════════════════════════════════════════════════════╗\n";
	std::cout << "║      Configurable Protocol Latency & Reliability Test         ║\n";
	std::cout << "╚════════════════════════════════════════════════════════════════╝\n\n";

	// Define receiver configuration presets
	std::vector<ReceiverPreset> presets = {
		{"Default (Reliable)", 2, 1.0, 10.0, "Standard settings, good reliability"},
		{"Fast", 2, 0.5, 50.0, "Fast updates, double verification"},
		{"Very Fast", 2, 0.3, 75.0, "Very fast updates, still safe"},
		{"Ultra Fast", 1, 0.2, 100.0, "Single detection, maximum speed"},
		{"Aggressive", 3, 0.4, 50.0, "Triple verify with high update rate (tested working)"},
		{"Balanced", 2, 0.6, 40.0, "Good balance between speed and reliability"},
		{"Extreme Speed", 1, 0.15, 100.0, "Absolute fastest, may have false positives"},
		{"Safe & Fast", 3, 0.5, 60.0, "Triple check but still fast"}
	};

	// Show available presets
	std::cout << "Available receiver configurations:\n\n";
	for (size_t i = 0; i < presets.size(); i++) {
		std::cout << "  " << (i + 1) << ". " << presets[i].name << "\n";
		std::cout << "     " << presets[i].description << "\n";
		std::cout << "     minDetections=" << presets[i].minDetections
				  << ", window=" << presets[i].consistencyWindow << "s"
				  << ", updateRate=" << presets[i].updateRate << "Hz\n\n";
	}
	std::cout << "  9. Test ALL presets (comparative analysis)\n";
	std::cout << "  0. Custom configuration\n\n";

	int choice;
	if (argc > 1) {
		choice = std::stoi(argv[1]);
	} else {
		std::cout << "Enter choice (0-9): ";
		std::cin >> choice;
	}

	if (choice < 0 || choice > 9) {
		std::cerr << "Invalid choice!\n";
		return 1;
	}

	// Handle "test all" option
	if (choice == 9) {
		std::cout << "\n╔════════════════════════════════════════════════════════════════╗\n";
		std::cout << "║              COMPARATIVE ANALYSIS - ALL PRESETS                ║\n";
		std::cout << "╚════════════════════════════════════════════════════════════════╝\n";

		AudioComm::ChordTransmitter transmitter;
		CRC crc;

		std::ofstream compareFile("../BUILD/protocol_latency_comparison.csv");
		compareFile << "Preset,MinDetections,Window,UpdateRate,TotalTests,Detected,DetectionRate,AvgLatency,MinLatency,MaxLatency\n";

		for (const auto& preset : presets) {
			std::cout << "\n" << std::string(64, '=') << "\n";
			std::cout << "Testing preset: " << preset.name << "\n";
			std::cout << std::string(64, '=') << "\n";

			// Clear previous results
			results.clear();

			// Setup receiver with this preset
			AudioComm::ChordReceiver receiver;
			AudioComm::ChordReceiver::Config recvConfig;
			recvConfig.minDetections = preset.minDetections;
			recvConfig.consistencyWindow = preset.consistencyWindow;
			recvConfig.updateRate = preset.updateRate;

			bool receiverStarted = receiver.startReceiving(recvConfig,
				[&crc](const AudioComm::ChordReceiver::Detection& det) {
					if (!waitingForDetection) return;

					if (det.value == expectedValue) {
						double detectionTime = getCurrentTime();
						double latency = detectionTime - transmitStartTime;
						bool isValid = crc.verify(det.value);

						std::cout << "OK (" << std::fixed << std::setprecision(3)
								  << latency << "s, " << det.detectionCount << "x)\n";

						{
							std::lock_guard<std::mutex> lock(resultsMutex);
							if (!results.empty()) {
								results.back().detected = true;
								results.back().crcValid = isValid;
								results.back().detectionTime = detectionTime;
								results.back().latency = latency;
								results.back().detectionCount = det.detectionCount;
							}
						}

						waitingForDetection = false;
					}
				});

			if (!receiverStarted) {
				std::cerr << "Failed to start receiver!\n";
				continue;
			}

			// Run test sequence
			runTestSequence(transmitter, crc);

			// Stop receiver
			receiver.stop();

			// Print results
			printResults(preset);

			// Save individual results
			std::string filename = "../BUILD/protocol_latency_" + preset.name + ".csv";
			// Replace spaces with underscores
			std::replace(filename.begin(), filename.end(), ' ', '_');
			saveResultsToFile(filename, preset);

			// Add to comparison file
			int detected = 0;
			double totalLat = 0.0, minLat = 999.0, maxLat = 0.0;
			for (const auto& r : results) {
				if (r.detected && r.crcValid) {
					detected++;
					totalLat += r.latency;
					minLat = std::min(minLat, r.latency);
					maxLat = std::max(maxLat, r.latency);
				}
			}

			compareFile << preset.name << ","
					   << preset.minDetections << ","
					   << preset.consistencyWindow << ","
					   << preset.updateRate << ","
					   << results.size() << ","
					   << detected << ","
					   << (100.0 * detected / results.size()) << ","
					   << (detected > 0 ? totalLat / detected : 0.0) << ","
					   << (detected > 0 ? minLat : 0.0) << ","
					   << (detected > 0 ? maxLat : 0.0) << "\n";

			std::this_thread::sleep_for(std::chrono::seconds(2));
		}

		compareFile.close();
		std::cout << "\n╔════════════════════════════════════════════════════════════════╗\n";
		std::cout << "║         COMPARISON COMPLETE - Results saved to CSV            ║\n";
		std::cout << "╚════════════════════════════════════════════════════════════════╝\n";
		std::cout << "Comparison file: ../BUILD/protocol_latency_comparison.csv\n\n";

		return 0;
	}

	// Single preset test
	ReceiverPreset selectedPreset;

	if (choice == 0) {
		// Custom configuration
		selectedPreset.name = "Custom";
		std::cout << "\nEnter custom configuration:\n";
		std::cout << "  Min detections (1-10): ";
		std::cin >> selectedPreset.minDetections;
		std::cout << "  Consistency window (0.05-3.0 seconds): ";
		std::cin >> selectedPreset.consistencyWindow;
		std::cout << "  Update rate (5-200 Hz): ";
		std::cin >> selectedPreset.updateRate;
		selectedPreset.description = "User-defined configuration";
	} else {
		selectedPreset = presets[choice - 1];
	}

	std::cout << "\n╔════════════════════════════════════════════════════════════════╗\n";
	std::cout << "║  Testing with: " << std::left << std::setw(47) << selectedPreset.name << "║\n";
	std::cout << "╚════════════════════════════════════════════════════════════════╝\n";

	// Initialize transmitter and receiver
	AudioComm::ChordTransmitter transmitter;
	AudioComm::ChordReceiver receiver;
	CRC crc;

	// Setup receiver with selected configuration
	AudioComm::ChordReceiver::Config recvConfig;
	recvConfig.minDetections = selectedPreset.minDetections;
	recvConfig.consistencyWindow = selectedPreset.consistencyWindow;
	recvConfig.updateRate = selectedPreset.updateRate;

	std::cout << "\nStarting receiver with configuration:\n";
	std::cout << "  minDetections: " << recvConfig.minDetections << "\n";
	std::cout << "  consistencyWindow: " << recvConfig.consistencyWindow << "s\n";
	std::cout << "  updateRate: " << recvConfig.updateRate << " Hz\n\n";

	bool receiverStarted = receiver.startReceiving(recvConfig,
		[&crc](const AudioComm::ChordReceiver::Detection& det) {
			if (!waitingForDetection) return;

			if (det.value == expectedValue) {
				double detectionTime = getCurrentTime();
				double latency = detectionTime - transmitStartTime;
				bool isValid = crc.verify(det.value);

				std::cout << "OK (" << std::fixed << std::setprecision(3)
						  << latency << "s, " << det.detectionCount << "x)";
				if (!isValid) std::cout << " CRC FAIL";
				std::cout << "\n";

				{
					std::lock_guard<std::mutex> lock(resultsMutex);
					if (!results.empty()) {
						results.back().detected = true;
						results.back().crcValid = isValid;
						results.back().detectionTime = detectionTime;
						results.back().latency = latency;
						results.back().detectionCount = det.detectionCount;
					}
				}

				waitingForDetection = false;
			}
		});

	if (!receiverStarted) {
		std::cerr << "ERROR: Failed to start receiver!\n";
		return 1;
	}

	// Run test sequence
	runTestSequence(transmitter, crc);

	// Stop receiver
	receiver.stop();

	// Display and save results
	printResults(selectedPreset);

	std::string filename = "../BUILD/protocol_latency_results.csv";
	saveResultsToFile(filename, selectedPreset);

	return 0;
}
