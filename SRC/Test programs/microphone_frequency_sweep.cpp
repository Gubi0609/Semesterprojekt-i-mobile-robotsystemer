#include "../../LIB/audio_transmitter.h"
#include "../../LIB/audio_receiver.h"
#include "../../LIB/frequency_detector.h"
#include <iostream>
#include <iomanip>
#include <vector>
#include <thread>
#include <chrono>
#include <atomic>
#include <cmath>
#include <fstream>
#include <sstream>
#include <csignal>

// [!]  IMPORTANT USAGE NOTES:
//
// This program tests microphone frequency response by transmitting and receiving.
//
// RECOMMENDED SETUP (Best Results):
// ----------------------------------
// 1. Run transmitter on one device (PC with speakers)
// 2. Run receiver on Raspberry Pi (with microphone)
// 3. Keep devices ~1 meter apart
//
// SINGLE DEVICE SETUP (May have issues):
// --------------------------------------
// If running on single Pi with both speaker and microphone:
// - Use EXTERNAL speaker and microphone (not built-in)
// - Physically separate them as much as possible (1+ meters)
// - May experience feedback/echo issues
// - Results may be less reliable
//
// SPLIT MODE (see below): Can run transmit-only or receive-only mode

// Test result for a single frequency
struct FrequencyTestResult {
	double frequency;
	int detectionCount;
	double avgMagnitude;
	double detectionRate; // percentage
	bool reliable;        // considered reliable if detection rate > 80%
};

// Global variables for detection callback
std::atomic<int> detectionCounter{0};
std::atomic<double> totalMagnitude{0.0};
std::atomic<bool> testRunning{false};

void printHeader() {
	std::cout << "\n╔════════════════════════════════════════════════════════════╗\n";
	std::cout << "║     Microphone Frequency Response Test                     ║\n";
	std::cout << "║     Tests individual frequencies for reliability           ║\n";
	std::cout << "╚════════════════════════════════════════════════════════════╝\n\n";
}

void printProgress(int current, int total) {
	int percent = (current * 100) / total;
	int barWidth = 40;
	int pos = (barWidth * current) / total;

	std::cout << "\r[";
	for (int i = 0; i < barWidth; ++i) {
		if (i < pos) std::cout << "█";
		else std::cout << " ";
	}
	std::cout << "] " << percent << "% (" << current << "/" << total << ")";
	std::cout.flush();
}

FrequencyTestResult testSingleFrequency(double frequency, double testDuration, double transmitGain, bool doTransmit, bool doReceive) {
	FrequencyTestResult result;
	result.frequency = frequency;
	result.detectionCount = 0;
	result.avgMagnitude = 0.0;
	result.detectionRate = 0.0;
	result.reliable = false;

	// Reset counters
	detectionCounter.store(0);
	totalMagnitude.store(0.0);
	testRunning.store(true);

	FrequencyDetector detector;
	const double updateRate = 50.0;

	// Setup receiver (if needed)
	if (doReceive) {
		FrequencyDetector::Config detectorConfig;
		detectorConfig.fftSize = 16384;
		detectorConfig.updateRate = updateRate;
		detectorConfig.sampleRate = 48000;
		detectorConfig.numPeaks = 5; // Get top 5 peaks
		detectorConfig.bandpassLow = frequency - 200.0;  // Filter around target frequency
		detectorConfig.bandpassHigh = frequency + 200.0;
		detectorConfig.duration = 0.0; // Infinite, we'll stop manually

		// Detection callback - capture frequency by reference
		auto onDetection = [frequency](const std::vector<FrequencyDetector::FrequencyPeak>& peaks) {
			if (!testRunning.load()) return;

			// Check if any peak is close to our target frequency
			for (const auto& peak : peaks) {
				if (std::abs(peak.frequency - frequency) < 150.0) { // 150 Hz tolerance
					detectionCounter.fetch_add(1);
					totalMagnitude.fetch_add(peak.magnitude);
					break; // Only count once per callback
				}
			}
		};

		// Start receiver
		if (!detector.startAsync(detectorConfig, onDetection)) {
			std::cerr << "\nFailed to start detector for " << frequency << " Hz\n";
			return result;
		}

		// Small delay to let receiver initialize
		std::this_thread::sleep_for(std::chrono::milliseconds(300));
	}

	ToneGenerator transmitter;

	// Setup transmitter (if needed)
	if (doTransmit) {
		ToneGenerator::Config txConfig;
		txConfig.frequencies = {frequency};
		txConfig.duration = testDuration;
		txConfig.gain = transmitGain;
		txConfig.fadeTime = 0.05;
		txConfig.sampleRate = 48000.0;
		txConfig.channels = 2;

		// Start transmitting
		if (!transmitter.start(txConfig)) {
			std::cerr << "\nFailed to start transmitter for " << frequency << " Hz\n";
			if (doReceive) detector.stop();
			return result;
		}

		// Wait for transmission to complete
		transmitter.waitForCompletion();
	} else {
		// If not transmitting, just wait for the test duration
		std::this_thread::sleep_for(std::chrono::duration<double>(testDuration));
	}

	// Small delay to catch any late detections
	std::this_thread::sleep_for(std::chrono::milliseconds(300));

	// Stop detection
	testRunning.store(false);
	if (doReceive) {
		detector.stop();
	}

	// Calculate results (only valid if receiving)
	if (doReceive) {
		result.detectionCount = detectionCounter.load();
		double totalMag = totalMagnitude.load();

		// Expected detections = testDuration * updateRate
		int expectedDetections = static_cast<int>(testDuration * updateRate);
		result.detectionRate = (result.detectionCount * 100.0) / expectedDetections;
		result.avgMagnitude = (result.detectionCount > 0) ? (totalMag / result.detectionCount) : 0.0;
		result.reliable = (result.detectionRate >= 80.0);
	}

	return result;
}

void saveResultsToFile(const std::vector<FrequencyTestResult>& results, const std::string& filename) {
	std::ofstream file(filename);
	if (!file.is_open()) {
		std::cerr << "Failed to open file for writing: " << filename << "\n";
		return;
	}

	file << "Frequency (Hz),Detection Count,Avg Magnitude,Detection Rate (%),Reliable\n";
	for (const auto& result : results) {
		file << std::fixed << std::setprecision(1) << result.frequency << ","
			 << result.detectionCount << ","
			 << std::setprecision(4) << result.avgMagnitude << ","
			 << std::setprecision(1) << result.detectionRate << ","
			 << (result.reliable ? "YES" : "NO") << "\n";
	}

	file.close();
	std::cout << "Results saved to: " << filename << "\n";
}

void printSummary(const std::vector<FrequencyTestResult>& results) {
	std::cout << "\n\n╔════════════════════════════════════════════════════════════╗\n";
	std::cout << "║                    TEST SUMMARY                            ║\n";
	std::cout << "╚════════════════════════════════════════════════════════════╝\n\n";

	// Count reliable frequencies
	int reliableCount = 0;
	int unreliableCount = 0;
	double minReliableFreq = 99999.0;
	double maxReliableFreq = 0.0;

	for (const auto& result : results) {
		if (result.reliable) {
			reliableCount++;
			minReliableFreq = std::min(minReliableFreq, result.frequency);
			maxReliableFreq = std::max(maxReliableFreq, result.frequency);
		} else {
			unreliableCount++;
		}
	}

	std::cout << "Total frequencies tested: " << results.size() << "\n";
	std::cout << "Reliable frequencies (≥80%): " << reliableCount << " ("
			  << std::fixed << std::setprecision(1)
			  << (reliableCount * 100.0 / results.size()) << "%)\n";
	std::cout << "Unreliable frequencies (<80%): " << unreliableCount << "\n";

	if (reliableCount > 0) {
		std::cout << "\nReliable frequency range: "
				  << std::fixed << std::setprecision(0)
				  << minReliableFreq << " Hz - " << maxReliableFreq << " Hz\n";
	}

	// Print detailed results
	std::cout << "\n" << std::string(70, '-') << "\n";
	std::cout << std::left << std::setw(12) << "Frequency"
			  << std::setw(12) << "Detections"
			  << std::setw(15) << "Rate (%)"
			  << std::setw(15) << "Avg Mag"
			  << std::setw(10) << "Status\n";
	std::cout << std::string(70, '-') << "\n";

	for (const auto& result : results) {
		std::cout << std::fixed << std::setprecision(1);
		std::cout << std::left << std::setw(12) << result.frequency
				  << std::setw(12) << result.detectionCount
				  << std::setw(15) << result.detectionRate
				  << std::setw(15) << std::setprecision(4) << result.avgMagnitude
				  << (result.reliable ? "OK GOOD" : "X POOR") << "\n";
	}
	std::cout << std::string(70, '-') << "\n";

	// Identify problem ranges
	std::cout << "\n[*] ANALYSIS:\n";

	std::vector<std::pair<double, double>> problemRanges;
	bool inProblemRange = false;
	double rangeStart = 0.0;

	for (size_t i = 0; i < results.size(); ++i) {
		if (!results[i].reliable && !inProblemRange) {
			inProblemRange = true;
			rangeStart = results[i].frequency;
		} else if (results[i].reliable && inProblemRange) {
			problemRanges.push_back({rangeStart, results[i-1].frequency});
			inProblemRange = false;
		}
	}
	if (inProblemRange) {
		problemRanges.push_back({rangeStart, results.back().frequency});
	}

	if (problemRanges.empty()) {
		std::cout << "[OK] All frequencies responded well!\n";
	} else {
		std::cout << "[!]  Problem frequency ranges detected:\n";
		for (const auto& range : problemRanges) {
			std::cout << "   * " << std::fixed << std::setprecision(0)
					  << range.first << " - " << range.second << " Hz\n";
		}
	}

	// Recommendations
	std::cout << "\n[*] RECOMMENDATIONS:\n";
	if (maxReliableFreq < 16000) {
		std::cout << "   [!]  Microphone upper limit appears to be around "
				  << std::fixed << std::setprecision(0) << maxReliableFreq << " Hz\n";
		std::cout << "   -> Consider adjusting tone ranges to stay below this frequency\n";
	}
	if (minReliableFreq > 5000) {
		std::cout << "   [!]  Microphone lower limit appears to be around "
				  << std::fixed << std::setprecision(0) << minReliableFreq << " Hz\n";
		std::cout << "   -> Consider adjusting tone ranges to stay above this frequency\n";
	}
	if (reliableCount < results.size() * 0.7) {
		std::cout << "   [!]  Less than 70% of frequencies are reliable\n";
		std::cout << "   -> Check microphone hardware and connections\n";
		std::cout << "   -> Try adjusting transmitter gain\n";
		std::cout << "   -> Ensure minimal background noise\n";
	}
}

int main(int argc, char* argv[]) {
	printHeader();

	// Parse command line arguments
	int mode = 1;
	double transmitGain = 0.5;
	std::string runMode = "both"; // "both", "transmit", "receive"

	if (argc > 1) {
		mode = std::atoi(argv[1]);
	}
	if (argc > 2) {
		transmitGain = std::atof(argv[2]);
	}
	if (argc > 3) {
		runMode = argv[3];
	}

	// Check run mode
	bool doTransmit = (runMode == "both" || runMode == "transmit");
	bool doReceive = (runMode == "both" || runMode == "receive");

	if (!doTransmit && !doReceive) {
		std::cerr << "Invalid run mode. Use 'both', 'transmit', or 'receive'\n";
		return 1;
	}

	std::vector<double> frequenciesToTest;
	double testDuration = 2.0; // seconds per frequency

	// Define test modes
	switch(mode) {
		case 1: // Quick test - 16 frequencies
			std::cout << "Mode: QUICK TEST (16 frequencies, ~1 minute)\n";
			for (int i = 0; i < 16; ++i) {
				frequenciesToTest.push_back(4500.0 + i * 750.0); // 4.5 kHz to 16 kHz
			}
			break;

		case 2: // Detailed test - every 250 Hz
			std::cout << "Mode: DETAILED TEST (47 frequencies, ~3 minutes)\n";
			for (double f = 4500.0; f <= 16000.0; f += 250.0) {
				frequenciesToTest.push_back(f);
			}
			break;

		case 3: // Fine-grained test - every 100 Hz
			std::cout << "Mode: FINE-GRAINED TEST (116 frequencies, ~7 minutes)\n";
			for (double f = 4500.0; f <= 16000.0; f += 100.0) {
				frequenciesToTest.push_back(f);
			}
			testDuration = 1.5; // Shorter per-frequency to keep reasonable time
			break;

		case 4: // Chord frequency test - test exact chord frequencies
			std::cout << "Mode: CHORD FREQUENCY TEST (64 frequencies)\n";
			std::cout << "Testing all possible chord tone frequencies...\n";
			// Test all 16 values for each of the 4 tones
			for (int tone = 0; tone < 4; ++tone) {
				double minFreq, maxFreq;
				switch(tone) {
					case 0: minFreq = 4500.0; maxFreq = 7000.0; break;
					case 1: minFreq = 7500.0; maxFreq = 10000.0; break;
					case 2: minFreq = 10500.0; maxFreq = 13000.0; break;
					case 3: minFreq = 13500.0; maxFreq = 16000.0; break;
				}
				double step = (maxFreq - minFreq) / 15.0;
				for (int val = 0; val < 16; ++val) {
					frequenciesToTest.push_back(minFreq + val * step);
				}
			}
			break;

		default:
			std::cerr << "Invalid mode. Use 1, 2, 3, or 4.\n";
			std::cout << "\nUsage: " << argv[0] << " [mode] [gain] [run_mode]\n";
			std::cout << "  mode: 1=Quick (default), 2=Detailed, 3=Fine-grained, 4=Chord frequencies\n";
			std::cout << "  gain: Transmitter gain (0.1-1.0, default=0.5)\n";
			std::cout << "  run_mode: both=TX+RX (default), transmit=TX only, receive=RX only\n";
			std::cout << "\nExamples:\n";
			std::cout << "  " << argv[0] << " 1 0.5 both      # Single device (speaker + mic)\n";
			std::cout << "  " << argv[0] << " 1 0.8 transmit  # Transmit only (run on PC)\n";
			std::cout << "  " << argv[0] << " 1 0.5 receive   # Receive only (run on Pi)\n";
			return 1;
	}

	std::cout << "Run mode: " << runMode << "\n";

	std::cout << "Test duration per frequency: " << testDuration << " seconds\n";
	std::cout << "Transmitter gain: " << transmitGain << "\n";
	std::cout << "Total estimated time: "
			  << std::fixed << std::setprecision(1)
			  << (frequenciesToTest.size() * (testDuration + 0.5)) / 60.0
			  << " minutes\n\n";

	std::cout << "[*] Instructions:\n";
	if (runMode == "both") {
		std::cout << "   [!]  BOTH mode: Using same device for TX and RX\n";
		std::cout << "   * Use EXTERNAL speaker and microphone\n";
		std::cout << "   * Separate them by at least 1 meter\n";
		std::cout << "   * May experience feedback - reduce gain if needed\n";
	} else if (runMode == "transmit") {
		std::cout << "   [TX] TRANSMIT ONLY mode\n";
		std::cout << "   * This device will play test tones through speakers\n";
		std::cout << "   * Start the RECEIVE program on another device first\n";
		std::cout << "   * Place devices ~1 meter apart\n";
	} else {
		std::cout << "   [RX] RECEIVE ONLY mode\n";
		std::cout << "   * This device will listen with microphone\n";
		std::cout << "   * Start the TRANSMIT program on another device\n";
		std::cout << "   * Place devices ~1 meter apart\n";
	}
	std::cout << "   * Ensure quiet environment with minimal background noise\n";
	std::cout << "   * Do not move or touch equipment during test\n";
	if (doReceive) {
		std::cout << "   * Results will be saved to CSV file at end\n";
	}
	std::cout << "\n";

	std::cout << "Press ENTER to start...";
	std::cin.ignore();
	std::cin.get();

	// Run tests
	std::vector<FrequencyTestResult> results;
	int testNum = 0;

	std::cout << "\n🔬 Testing " << frequenciesToTest.size() << " frequencies...\n\n";

	for (double freq : frequenciesToTest) {
		testNum++;

		if (doTransmit) {
			std::cout << "[TX] Transmitting " << std::fixed << std::setprecision(1) << freq << " Hz... ";
		}
		if (doReceive) {
			std::cout << "[RX] Testing " << std::fixed << std::setprecision(1) << freq << " Hz... ";
		}
		std::cout.flush();

		FrequencyTestResult result = testSingleFrequency(freq, testDuration, transmitGain, doTransmit, doReceive);
		results.push_back(result);

		if (doReceive) {
			std::cout << result.detectionRate << "% detected ";
			std::cout << (result.reliable ? "OK" : "X") << "\n";
		} else {
			std::cout << "done\n";
		}

		printProgress(testNum, frequenciesToTest.size());
	}

	std::cout << "\n\n[OK] Testing complete!\n";

	// Only save/display results if we were receiving
	if (!doReceive) {
		std::cout << "Transmit-only mode complete. Check receiver for results.\n";
		return 0;
	}

	// Save results
	std::time_t now = std::time(nullptr);
	std::stringstream filename;
	filename << "frequency_test_results_" << now << ".csv";
	saveResultsToFile(results, filename.str());

	// Print summary
	printSummary(results);

	std::cout << "\n";
	return 0;
}
