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
#include <map>

// Test program to identify which chords are reliably detected and which fail
// Uses increased FFT resolution for better frequency discrimination

// Structure to hold test results
struct ChordTestResult {
	uint16_t sentValue;         // 16-bit value sent
	uint16_t detectedValue;     // Value that was detected (may differ if misdetected)
	bool detected;              // Was anything detected?
	bool correctValue;          // Was the correct value detected?
	double latency;             // Detection latency in seconds
	int detectionCount;         // Number of consistent detections
	std::vector<double> sentFrequencies;      // Frequencies we transmitted
	std::vector<double> detectedFrequencies;  // Frequencies that were detected
};

// Global variables for synchronization
std::map<uint16_t, ChordTestResult> testResults;
std::mutex resultsMutex;
std::atomic<bool> waitingForDetection(false);
std::atomic<uint16_t> expectedValue(0);
std::atomic<double> transmitStartTime(0.0);

// Detection log for troubleshooting
struct DetectionLogEntry {
	double timestamp;
	uint16_t value;
	int detectionCount;
	std::vector<double> frequencies;
};
std::vector<DetectionLogEntry> detectionLog;
std::mutex detectionLogMutex;

// Get current time in seconds
double getCurrentTime() {
	auto now = std::chrono::high_resolution_clock::now();
	auto duration = now.time_since_epoch();
	return std::chrono::duration<double>(duration).count();
}

// Decode 16-bit chord back to 4 frequencies (for debugging)
std::vector<double> decodeChordToFrequencies(uint16_t value) {
	std::vector<double> freqs;

	// Extract 4-bit values
	uint8_t tone1 = (value >> 12) & 0x0F;
	uint8_t tone2 = (value >> 8) & 0x0F;
	uint8_t tone3 = (value >> 4) & 0x0F;
	uint8_t tone4 = value & 0x0F;

	// Convert to frequencies (matching AudioComm::ChordTransmitter defaults)
	freqs.push_back(5000.0 + tone1 * 200.0);  // 5000-8000 Hz
	freqs.push_back(8500.0 + tone2 * 200.0);  // 8500-11500 Hz
	freqs.push_back(12000.0 + tone3 * 200.0); // 12000-15000 Hz
	freqs.push_back(15500.0 + tone4 * 200.0); // 15500-18500 Hz

	return freqs;
}

// Send a chord and wait for detection
void testChord(AudioComm::ChordTransmitter& transmitter, uint16_t value, double duration) {
	ChordTestResult result;
	result.sentValue = value;
	result.detectedValue = 0;
	result.detected = false;
	result.correctValue = false;
	result.latency = 0.0;
	result.detectionCount = 0;
	result.sentFrequencies = decodeChordToFrequencies(value);

	// Store initial result
	{
		std::lock_guard<std::mutex> lock(resultsMutex);
		testResults[value] = result;
	}

	// Clear detection log
	{
		std::lock_guard<std::mutex> logLock(detectionLogMutex);
		detectionLog.clear();
	}

	// Set up detection expectation
	expectedValue = value;
	transmitStartTime = getCurrentTime();
	waitingForDetection = true;

	// Send the chord
	AudioComm::ChordTransmitter::Config config;
	config.toneDuration = duration;

	if (!transmitter.startTransmitting(value, config)) {
		std::cerr << "TX FAILED for value " << value << "\n";
		waitingForDetection = false;
		return;
	}

	// Wait for transmission to complete
	transmitter.waitForCompletion();

	// Give extra time for detection
	std::this_thread::sleep_for(std::chrono::milliseconds(300));

	waitingForDetection = false;
}

void printDetailedResults(int fftSize) {
	std::cout << "\n╔════════════════════════════════════════════════════════════════╗\n";
	std::cout << "║           CHORD DETECTION ACCURACY TEST RESULTS               ║\n";
	std::cout << "╚════════════════════════════════════════════════════════════════╝\n";
	std::cout << "FFT Size: " << fftSize << " (Resolution: "
	          << std::fixed << std::setprecision(2)
	          << (48000.0 / fftSize) << " Hz/bin)\n\n";

	int totalTests = testResults.size();
	int detected = 0;
	int correctDetections = 0;
	int wrongValue = 0;
	int missed = 0;

	std::vector<uint16_t> missedChords;
	std::vector<uint16_t> wrongChords;

	for (const auto& [value, result] : testResults) {
		if (result.detected) {
			detected++;
			if (result.correctValue) {
				correctDetections++;
			} else {
				wrongValue++;
				wrongChords.push_back(value);
			}
		} else {
			missed++;
			missedChords.push_back(value);
		}
	}

	std::cout << "Summary:\n";
	std::cout << "  Total chords tested: " << totalTests << "\n";
	std::cout << "  Correctly detected: " << correctDetections
	          << " (" << (100.0 * correctDetections / totalTests) << "%)\n";
	std::cout << "  Wrong value detected: " << wrongValue
	          << " (" << (100.0 * wrongValue / totalTests) << "%)\n";
	std::cout << "  Not detected: " << missed
	          << " (" << (100.0 * missed / totalTests) << "%)\n";

	// Show problematic chords
	if (!missedChords.empty()) {
		std::cout << "\n❌ MISSED CHORDS (" << missedChords.size() << "):\n";
		std::cout << std::hex << std::uppercase;
		for (size_t i = 0; i < missedChords.size(); i++) {
			std::cout << "0x" << std::setw(4) << std::setfill('0') << missedChords[i];

			// Show frequencies
			auto freqs = decodeChordToFrequencies(missedChords[i]);
			std::cout << " (" << std::dec << std::fixed << std::setprecision(0);
			for (size_t j = 0; j < freqs.size(); j++) {
				std::cout << freqs[j];
				if (j < freqs.size() - 1) std::cout << ", ";
			}
			std::cout << " Hz)" << std::hex;

			if ((i + 1) % 3 == 0) std::cout << "\n";
			else std::cout << "  ";
		}
		std::cout << std::dec << "\n";
	}

	if (!wrongChords.empty()) {
		std::cout << "\n⚠️  WRONG VALUE DETECTED (" << wrongChords.size() << "):\n";
		std::cout << std::hex << std::uppercase;
		for (size_t i = 0; i < wrongChords.size() && i < 20; i++) {
			uint16_t sent = wrongChords[i];
			uint16_t detected = testResults[sent].detectedValue;

			std::cout << "Sent: 0x" << std::setw(4) << std::setfill('0') << sent
			          << " → Got: 0x" << std::setw(4) << std::setfill('0') << detected;

			// Show frequency comparison
			auto sentFreqs = decodeChordToFrequencies(sent);
			auto detFreqs = testResults[sent].detectedFrequencies;

			std::cout << " (" << std::dec;
			for (size_t j = 0; j < sentFreqs.size() && j < detFreqs.size(); j++) {
				double diff = detFreqs[j] - sentFreqs[j];
				if (std::abs(diff) > 50) { // Flag large differences
					std::cout << "T" << (j+1) << ": " << std::showpos
					          << std::fixed << std::setprecision(0) << diff << "Hz ";
				}
			}
			std::cout << std::noshowpos << std::hex << ")\n";
		}
		std::cout << std::dec;
		if (wrongChords.size() > 20) {
			std::cout << "... and " << (wrongChords.size() - 20) << " more\n";
		}
	}

	// Calculate latency statistics for successful detections
	if (correctDetections > 0) {
		double totalLatency = 0.0;
		double minLat = 999.0;
		double maxLat = 0.0;

		for (const auto& [value, result] : testResults) {
			if (result.correctValue) {
				totalLatency += result.latency;
				minLat = std::min(minLat, result.latency);
				maxLat = std::max(maxLat, result.latency);
			}
		}

		std::cout << "\n✓ Latency Statistics (successful detections):\n";
		std::cout << std::fixed << std::setprecision(3);
		std::cout << "  Average: " << (totalLatency / correctDetections) << "s\n";
		std::cout << "  Min: " << minLat << "s\n";
		std::cout << "  Max: " << maxLat << "s\n";
	}
}

void saveResultsToCSV(const std::string& filename, int fftSize) {
	std::ofstream file(filename);
	if (!file.is_open()) {
		std::cerr << "Failed to open file: " << filename << "\n";
		return;
	}

	file << "# FFT Size: " << fftSize << ", Resolution: "
	     << (48000.0 / fftSize) << " Hz/bin\n";
	file << "Sent Value (hex),Detected Value (hex),Status,Latency (s),Detection Count,";
	file << "Freq1_Sent,Freq2_Sent,Freq3_Sent,Freq4_Sent,";
	file << "Freq1_Detected,Freq2_Detected,Freq3_Detected,Freq4_Detected\n";

	for (const auto& [value, result] : testResults) {
		file << std::hex << std::uppercase << "0x" << std::setw(4)
		     << std::setfill('0') << result.sentValue << ",";

		if (result.detected) {
			file << "0x" << std::setw(4) << std::setfill('0') << result.detectedValue << ",";

			if (result.correctValue) {
				file << "CORRECT,";
			} else {
				file << "WRONG_VALUE,";
			}

			file << std::dec << std::fixed << std::setprecision(4) << result.latency << ",";
			file << result.detectionCount << ",";
		} else {
			file << "N/A,MISSED,N/A,N/A,";
		}

		// Sent frequencies
		for (size_t i = 0; i < result.sentFrequencies.size(); i++) {
			file << std::fixed << std::setprecision(1) << result.sentFrequencies[i];
			if (i < result.sentFrequencies.size() - 1) file << ",";
		}
		file << ",";

		// Detected frequencies
		if (!result.detectedFrequencies.empty()) {
			for (size_t i = 0; i < result.detectedFrequencies.size(); i++) {
				file << std::fixed << std::setprecision(1) << result.detectedFrequencies[i];
				if (i < result.detectedFrequencies.size() - 1) file << ",";
			}
		}

		file << "\n";
	}

	file.close();
	std::cout << "\n✓ Results saved to: " << filename << "\n";
}

// Keep track of detections for each test
std::map<uint16_t, std::vector<DetectionLogEntry>> detectionsByTest;

void saveDetailedDetectionLog(const std::string& filename) {
	std::ofstream file(filename);
	if (!file.is_open()) {
		std::cerr << "Failed to open detection log file: " << filename << "\n";
		return;
	}

	file << "# Detailed Detection Log - Shows what was detected during failed transmissions\n";
	file << "Sent Value (hex),Sent Freqs (Hz),Detected Value (hex),Detected Freqs (Hz),Timestamp (s),Detection Count\n";

	for (const auto& [sentValue, detections] : detectionsByTest) {
		if (detections.empty()) continue;

		auto sentFreqs = decodeChordToFrequencies(sentValue);

		for (const auto& det : detections) {
			file << std::hex << std::uppercase << "0x" << std::setw(4)
			     << std::setfill('0') << sentValue << ",\"";

			// Sent frequencies
			for (size_t i = 0; i < sentFreqs.size(); i++) {
				file << std::fixed << std::setprecision(0) << sentFreqs[i];
				if (i < sentFreqs.size() - 1) file << " ";
			}
			file << "\",0x" << std::setw(4) << std::setfill('0') << det.value << ",\"";

			// Detected frequencies
			for (size_t i = 0; i < det.frequencies.size(); i++) {
				file << std::fixed << std::setprecision(0) << det.frequencies[i];
				if (i < det.frequencies.size() - 1) file << " ";
			}
			file << "\"," << std::dec << std::fixed << std::setprecision(4)
			     << det.timestamp << "," << det.detectionCount << "\n";
		}
	}

	file.close();
	std::cout << "✓ Detection log saved to: " << filename << "\n";
}

int main(int argc, char* argv[]) {
	std::cout << "╔════════════════════════════════════════════════════════════════╗\n";
	std::cout << "║          Chord Detection Accuracy Test (High-Res FFT)         ║\n";
	std::cout << "╚════════════════════════════════════════════════════════════════╝\n\n";

	std::cout << "This test systematically checks chord detection accuracy\n";
	std::cout << "to identify which chord values are problematic.\n\n";

	// Configuration options
	int fftSize = 16384;  // Higher resolution (default was 8192)
	std::cout << "Select FFT size:\n";
	std::cout << "  1. 8192  (Standard - 5.86 Hz/bin resolution)\n";
	std::cout << "  2. 16384 (High - 2.93 Hz/bin resolution) [RECOMMENDED]\n";
	std::cout << "  3. 32768 (Very High - 1.46 Hz/bin resolution)\n";
	std::cout << "Enter choice (1-3, default=2): ";

	int fftChoice = 2;
	if (argc > 1) {
		fftChoice = std::stoi(argv[1]);
	} else {
		std::cin >> fftChoice;
		if (std::cin.fail()) fftChoice = 2;
	}

	switch (fftChoice) {
		case 1: fftSize = 8192; break;
		case 2: fftSize = 16384; break;
		case 3: fftSize = 32768; break;
		default: fftSize = 16384;
	}

	double freqResolution = 48000.0 / fftSize;
	std::cout << "\nUsing FFT size: " << fftSize
	          << " (Resolution: " << std::fixed << std::setprecision(2)
	          << freqResolution << " Hz per bin)\n";

	// Test mode selection
	std::cout << "\nSelect test mode:\n";
	std::cout << "  1. Quick test (16 protocol commands)\n";
	std::cout << "  2. Sample test (256 random chords)\n";
	std::cout << "  3. Full sweep (all 65536 possible chords) [VERY SLOW]\n";
	std::cout << "Enter choice (1-3, default=1): ";

	int testMode = 1;
	if (argc > 2) {
		testMode = std::stoi(argv[2]);
	} else {
		std::cin >> testMode;
		if (std::cin.fail()) testMode = 1;
	}

	// Initialize components
	AudioComm::ChordTransmitter transmitter;
	AudioComm::ChordReceiver receiver;
	CRC crc;

	// Setup receiver with increased FFT resolution
	AudioComm::ChordReceiver::Config recvConfig;
	recvConfig.fftSize = fftSize;
	recvConfig.minDetections = 2;
	recvConfig.consistencyWindow = 0.3;
	recvConfig.updateRate = 60.0;
	recvConfig.detectionTolerance = 150.0;  // Increased for hardware frequency variations

	std::cout << "\nReceiver configuration:\n";
	std::cout << "  FFT Size: " << recvConfig.fftSize << "\n";
	std::cout << "  Min Detections: " << recvConfig.minDetections << "\n";
	std::cout << "  Consistency Window: " << recvConfig.consistencyWindow << "s\n";
	std::cout << "  Update Rate: " << recvConfig.updateRate << " Hz\n\n";

	// Start receiver with detection callback
	bool receiverStarted = receiver.startReceiving(recvConfig,
		[](const AudioComm::ChordReceiver::Detection& det) {
			double detectionTime = getCurrentTime();
			double latency = detectionTime - transmitStartTime;

			// Log ALL detections during transmission window for debugging
			if (waitingForDetection) {
				std::lock_guard<std::mutex> logLock(detectionLogMutex);
				DetectionLogEntry entry;
				entry.timestamp = latency;
				entry.value = det.value;
				entry.detectionCount = det.detectionCount;
				entry.frequencies = det.frequencies;
				detectionLog.push_back(entry);
			}

			if (!waitingForDetection) return;

			{
				std::lock_guard<std::mutex> lock(resultsMutex);
				uint16_t expected = expectedValue;

				if (testResults.find(expected) != testResults.end()) {
					testResults[expected].detected = true;
					testResults[expected].detectedValue = det.value;
					testResults[expected].correctValue = (det.value == expected);
					testResults[expected].latency = latency;
					testResults[expected].detectionCount = det.detectionCount;
					testResults[expected].detectedFrequencies = det.frequencies;
				}
			}

			waitingForDetection = false;
		});

	if (!receiverStarted) {
		std::cerr << "ERROR: Failed to start receiver!\n";
		return 1;
	}

	std::cout << "Starting test...\n";
	std::cout << std::string(64, '=') << "\n";

	// Run tests based on selected mode
	std::vector<uint16_t> testValues;

	switch (testMode) {
		case 1: { // Protocol commands
			std::cout << "Testing protocol commands...\n";
			CRC tempCrc;

			// Test various protocol commands
			testValues.push_back(tempCrc.encode1216({encodeReset()})[0]);
			testValues.push_back(tempCrc.encode1216({encodeModeSelect(RobotMode::DRIVE_FOR_DURATION)})[0]);
			testValues.push_back(tempCrc.encode1216({encodeModeSelect(RobotMode::TURN_FOR_DURATION)})[0]);
			testValues.push_back(tempCrc.encode1216({encodeModeSelect(RobotMode::DRIVE_FORWARD)})[0]);
			testValues.push_back(tempCrc.encode1216({encodeModeSelect(RobotMode::TURN)})[0]);
			testValues.push_back(tempCrc.encode1216({encodeModeSelect(RobotMode::STOP)})[0]);

			// Add some data commands
			for (int i = 0; i < 10; i++) {
				uint16_t cmd = (rand() % 4096);
				testValues.push_back(tempCrc.encode1216({cmd})[0]);
			}
			break;
		}

		case 2: { // Random sample
			std::cout << "Testing 256 random chords...\n";
			srand(time(NULL));
			for (int i = 0; i < 256; i++) {
				testValues.push_back(rand() % 65536);
			}
			break;
		}

		case 3: { // Full sweep
			std::cout << "Testing ALL 65536 possible chords (this will take a while)...\n";
			for (int i = 0; i < 65536; i++) {
				testValues.push_back(i);
			}
			break;
		}
	}

	// Run tests
	int testNum = 0;
	for (uint16_t value : testValues) {
		testNum++;

		if (testNum % 10 == 0 || testMode == 1) {
			std::cout << "[" << testNum << "/" << testValues.size() << "] ";
			std::cout << "Testing 0x" << std::hex << std::uppercase
			          << std::setw(4) << std::setfill('0') << value
			          << std::dec << "... " << std::flush;
		}

		testChord(transmitter, value, 1.0);

		{
			std::lock_guard<std::mutex> lock(resultsMutex);
			std::lock_guard<std::mutex> logLock(detectionLogMutex);

			// Save detection log for this test
			if (!detectionLog.empty()) {
				detectionsByTest[value] = detectionLog;
			}

			if (testResults[value].correctValue) {
				if (testNum % 10 == 0 || testMode == 1) {
					std::cout << "✓ OK\n";
				}
			} else if (testResults[value].detected) {
				std::cout << "⚠ WRONG (got 0x" << std::hex << std::uppercase
				          << std::setw(4) << std::setfill('0')
				          << testResults[value].detectedValue << std::dec << ")";

				// Show all detections
				if (!detectionLog.empty()) {
					std::cout << " [detected during TX: ";
					for (size_t i = 0; i < detectionLog.size() && i < 3; i++) {
						std::cout << "0x" << std::hex << std::uppercase
						          << std::setw(4) << std::setfill('0')
						          << detectionLog[i].value << std::dec;
						if (i < detectionLog.size() - 1 && i < 2) std::cout << ", ";
					}
					if (detectionLog.size() > 3) {
						std::cout << "... +" << (detectionLog.size() - 3) << " more";
					}
					std::cout << "]";
				}
				std::cout << "\n";
			} else {
				std::cout << "✗ MISSED";

				// Show what was detected during the transmission window
				if (!detectionLog.empty()) {
					std::cout << " [detected: ";
					for (size_t i = 0; i < detectionLog.size() && i < 3; i++) {
						std::cout << "0x" << std::hex << std::uppercase
						          << std::setw(4) << std::setfill('0')
						          << detectionLog[i].value << std::dec;
						if (i < detectionLog.size() - 1 && i < 2) std::cout << ", ";
					}
					if (detectionLog.size() > 3) {
						std::cout << "... +" << (detectionLog.size() - 3) << " more";
					}
					std::cout << "]";
				} else {
					std::cout << " [nothing detected]";
				}
				std::cout << "\n";
			}
		}

		// Brief pause between tests
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}

	std::cout << std::string(64, '=') << "\n";

	// Stop receiver
	receiver.stop();

	// Display and save results
	printDetailedResults(fftSize);

	std::string filename = "../BUILD/chord_accuracy_fft" + std::to_string(fftSize) + ".csv";
	saveResultsToCSV(filename, fftSize);

	// Save detailed detection log if there were any failures
	if (!detectionsByTest.empty()) {
		std::string logFilename = "../BUILD/chord_detection_log_fft" + std::to_string(fftSize) + ".csv";
		saveDetailedDetectionLog(logFilename);
	}

	std::cout << "\nTest complete!\n";

	return 0;
}
