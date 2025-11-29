#include "../../LIB/tone_generator.h"
#include "../../LIB/frequency_detector.h"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <map>
#include <cmath>
#include <thread>
#include <chrono>
#include <algorithm>

// Test program to identify which specific frequencies your microphone can't detect
// This helps diagnose hardware frequency response issues

struct FrequencyTestResult {
	double frequency;
	bool detected;
	double detectedFreq;
	double frequencyError;
	double magnitude;
	int detectionCount;
};

// Global results storage
std::vector<FrequencyTestResult> allResults;

// Test a single frequency
FrequencyTestResult testFrequency(double targetFreq, double duration = 0.5) {
	FrequencyTestResult result;
	result.frequency = targetFreq;
	result.detected = false;
	result.detectedFreq = 0.0;
	result.frequencyError = 0.0;
	result.magnitude = 0.0;
	result.detectionCount = 0;

	bool foundFreq = false;
	double bestMatch = 0.0;
	double bestMagnitude = 0.0;
	int totalDetections = 0;

	// Setup frequency detector
	FrequencyDetector detector;
	FrequencyDetector::Config detConfig;
	detConfig.sampleRate = 48000;
	detConfig.fftSize = 16384;
	detConfig.numPeaks = 10;
	detConfig.duration = 0.0;  // Continuous mode
	detConfig.bandpassLow = targetFreq - 500;
	detConfig.bandpassHigh = targetFreq + 500;
	detConfig.updateRate = 20.0;

	// Start detector in background
	bool detectorStarted = detector.startAsync(detConfig,
		[&](const std::vector<FrequencyDetector::FrequencyPeak>& peaks) {
			if (peaks.empty()) return;

			// Find the peak closest to our target frequency
			for (const auto& peak : peaks) {
				double error = std::abs(peak.frequency - targetFreq);

				// Consider it a match if within 50 Hz
				if (error < 50.0) {
					foundFreq = true;
					totalDetections++;
					if (peak.magnitude > bestMagnitude) {
						bestMatch = peak.frequency;
						bestMagnitude = peak.magnitude;
					}
				}
			}
		});

	if (!detectorStarted) {
		std::cerr << "Failed to start detector for " << targetFreq << " Hz\n";
		return result;
	}

	// Give detector time to initialize and start listening
	std::this_thread::sleep_for(std::chrono::milliseconds(200));

	// Generate the tone
	ToneGenerator generator;
	ToneGenerator::Config genConfig;
	genConfig.frequencies = {targetFreq};  // Single frequency
	genConfig.duration = duration;
	genConfig.sampleRate = 48000;
	genConfig.gain = 0.3;

	bool generatorStarted = generator.start(genConfig);
	if (!generatorStarted) {
		std::cerr << "Failed to start generator for " << targetFreq << " Hz\n";
		detector.stop();
		return result;
	}

	generator.waitForCompletion();

	// Give extra time for detection to catch the tail end
	std::this_thread::sleep_for(std::chrono::milliseconds(200));

	// Stop detector
	detector.stop();

	// Fill in results
	result.detected = foundFreq;
	result.detectionCount = totalDetections;
	if (foundFreq) {
		result.detectedFreq = bestMatch;
		result.frequencyError = bestMatch - targetFreq;
		result.magnitude = bestMagnitude;
	}

	return result;
}

void printResults(const std::vector<FrequencyTestResult>& results) {
	std::cout << "\n╔════════════════════════════════════════════════════════════════╗\n";
	std::cout << "║           MICROPHONE FREQUENCY RESPONSE TEST                  ║\n";
	std::cout << "╚════════════════════════════════════════════════════════════════╝\n\n";

	int detected = 0;
	int missed = 0;
	std::vector<double> missedFreqs;
	std::vector<double> weakFreqs;

	std::cout << "Target Freq | Detected | Actual Freq | Error (Hz) | Magnitude | Count\n";
	std::cout << std::string(75, '-') << "\n";

	for (const auto& result : results) {
		std::cout << std::fixed << std::setprecision(1) << std::setw(11) << result.frequency << " | ";

		if (result.detected) {
			detected++;
			std::cout << "   YES   | ";
			std::cout << std::setw(11) << result.detectedFreq << " | ";
			std::cout << std::setw(10) << std::showpos << result.frequencyError << std::noshowpos << " | ";
			std::cout << std::setw(9) << std::setprecision(3) << result.magnitude << " | ";
			std::cout << std::setw(5) << result.detectionCount;

			if (result.magnitude < 0.1) {
				std::cout << " ⚠ WEAK";
				weakFreqs.push_back(result.frequency);
			}
		} else {
			missed++;
			missedFreqs.push_back(result.frequency);
			std::cout << "   NO    |      N/A    |     N/A    |    N/A    |  N/A  ✗ MISSED";
		}

		std::cout << "\n";
	}

	std::cout << std::string(75, '-') << "\n";
	std::cout << "Total: " << results.size()
	          << " | Detected: " << detected
	          << " (" << (100.0 * detected / results.size()) << "%) | "
	          << "Missed: " << missed << "\n\n";

	if (!missedFreqs.empty()) {
		std::cout << "❌ MISSED FREQUENCIES (" << missedFreqs.size() << "):\n";
		for (double freq : missedFreqs) {
			std::cout << "  " << std::fixed << std::setprecision(1) << freq << " Hz\n";
		}
		std::cout << "\n";
	}

	if (!weakFreqs.empty()) {
		std::cout << "⚠️  WEAK SIGNAL FREQUENCIES (" << weakFreqs.size() << "):\n";
		for (double freq : weakFreqs) {
			std::cout << "  " << std::fixed << std::setprecision(1) << freq << " Hz\n";
		}
		std::cout << "\n";
	}

	// Analyze by frequency band
	std::cout << "ANALYSIS BY FREQUENCY BAND:\n";
	std::map<std::string, std::pair<int, int>> bandStats; // band -> (total, detected)

	for (const auto& result : results) {
		std::string band;
		if (result.frequency >= 5000 && result.frequency <= 8000) {
			band = "Band 1 (5000-8000 Hz)";
		} else if (result.frequency >= 8500 && result.frequency <= 11500) {
			band = "Band 2 (8500-11500 Hz)";
		} else if (result.frequency >= 12000 && result.frequency <= 15000) {
			band = "Band 3 (12000-15000 Hz)";
		} else if (result.frequency >= 15500 && result.frequency <= 18500) {
			band = "Band 4 (15500-18500 Hz)";
		} else {
			band = "Other";
		}

		bandStats[band].first++;
		if (result.detected) {
			bandStats[band].second++;
		}
	}

	for (const auto& [band, stats] : bandStats) {
		int total = stats.first;
		int det = stats.second;
		double percent = (100.0 * det / total);
		std::cout << "  " << std::left << std::setw(30) << band
		          << ": " << det << "/" << total
		          << " (" << std::fixed << std::setprecision(1) << percent << "%)";
		if (percent < 80.0) {
			std::cout << " ⚠ PROBLEMATIC";
		}
		std::cout << "\n";
	}
}

void saveResultsToCSV(const std::string& filename, const std::vector<FrequencyTestResult>& results) {
	std::ofstream file(filename);
	if (!file.is_open()) {
		std::cerr << "Failed to open file: " << filename << "\n";
		return;
	}

	file << "Target Frequency (Hz),Detected,Detected Frequency (Hz),Error (Hz),Magnitude,Detection Count\n";

	for (const auto& result : results) {
		file << std::fixed << std::setprecision(1) << result.frequency << ",";
		file << (result.detected ? "YES" : "NO") << ",";

		if (result.detected) {
			file << result.detectedFreq << ",";
			file << std::showpos << result.frequencyError << std::noshowpos << ",";
			file << std::setprecision(4) << result.magnitude << ",";
			file << result.detectionCount;
		} else {
			file << "N/A,N/A,N/A,N/A";
		}

		file << "\n";
	}

	file.close();
	std::cout << "\n✓ Results saved to: " << filename << "\n";
}

// Extract unique frequencies from the quick test protocol commands
std::vector<double> getQuickTestFrequencies() {
	// The 16 chords from quick test use these frequencies:
	// We need to extract all unique frequencies used
	std::vector<uint16_t> testChords = {
		0x0000,  // RESET typically
		0xA310,  // Example problematic
		0x4580,  // Example problematic
		// Add more from your actual quick test
	};

	// For now, let's test all 16 values in each of the 4 bands
	// This gives us 64 unique frequencies to test
	std::vector<double> frequencies;

	// Band 1: 5000-8000 Hz (200 Hz steps)
	for (int i = 0; i < 16; i++) {
		frequencies.push_back(5000.0 + i * 200.0);
	}

	// Band 2: 8500-11500 Hz (200 Hz steps)
	for (int i = 0; i < 16; i++) {
		frequencies.push_back(8500.0 + i * 200.0);
	}

	// Band 3: 12000-15000 Hz (200 Hz steps)
	for (int i = 0; i < 16; i++) {
		frequencies.push_back(12000.0 + i * 200.0);
	}

	// Band 4: 15500-18500 Hz (200 Hz steps)
	for (int i = 0; i < 16; i++) {
		frequencies.push_back(15500.0 + i * 200.0);
	}

	return frequencies;
}

int main(int argc, char* argv[]) {
	std::cout << "╔════════════════════════════════════════════════════════════════╗\n";
	std::cout << "║         Microphone Frequency Response Test                    ║\n";
	std::cout << "╚════════════════════════════════════════════════════════════════╝\n\n";

	std::cout << "This test checks which frequencies your microphone can detect.\n";
	std::cout << "It will play each frequency used in the chord system and verify\n";
	std::cout << "if your microphone can pick it up.\n\n";

	// Get test mode
	int testMode = 1;
	if (argc > 1) {
		testMode = std::stoi(argv[1]);
	} else {
		std::cout << "Select test mode:\n";
		std::cout << "  1. All frequencies (64 total - all 4 bands)\n";
		std::cout << "  2. Quick sample (16 frequencies - one from each position)\n";
		std::cout << "  3. Extended range test (100-20000 Hz sweep)\n";
		std::cout << "Enter choice (1-3, default=1): ";
		std::cin >> testMode;
		if (std::cin.fail()) testMode = 1;
	}

	std::vector<double> testFrequencies;

	switch (testMode) {
		case 1: // All frequencies from chord system
			testFrequencies = getQuickTestFrequencies();
			std::cout << "\nTesting all 64 frequencies used in chord system...\n";
			break;

		case 2: { // Quick sample
			std::cout << "\nTesting sample of 16 key frequencies...\n";
			// Test one from each band position
			for (int i = 0; i < 4; i++) {
				testFrequencies.push_back(5000.0 + i * 1000.0);   // Band 1
				testFrequencies.push_back(8500.0 + i * 1000.0);   // Band 2
				testFrequencies.push_back(12000.0 + i * 1000.0);  // Band 3
				testFrequencies.push_back(15500.0 + i * 1000.0);  // Band 4
			}
			break;
		}

		case 3: { // Extended range
			std::cout << "\nTesting extended frequency range (100-20000 Hz)...\n";
			// Test every 500 Hz from 100 to 20000
			for (int freq = 100; freq <= 20000; freq += 500) {
				testFrequencies.push_back(freq);
			}
			break;
		}

		default:
			testFrequencies = getQuickTestFrequencies();
			break;
	}

	std::cout << "Total frequencies to test: " << testFrequencies.size() << "\n";
	std::cout << "Duration per test: 0.5 seconds\n";
	std::cout << "Estimated time: " << (testFrequencies.size() * 0.8) << " seconds\n\n";
	std::cout << "Press Enter to start...";
	std::cin.ignore();
	std::cin.get();

	std::cout << "\nStarting test...\n";
	std::cout << std::string(75, '=') << "\n";

	int testNum = 0;
	for (double freq : testFrequencies) {
		testNum++;
		std::cout << "[" << testNum << "/" << testFrequencies.size() << "] ";
		std::cout << "Testing " << std::fixed << std::setprecision(1)
		          << freq << " Hz... " << std::flush;

		try {
			FrequencyTestResult result = testFrequency(freq, 0.5);
			allResults.push_back(result);

			if (result.detected) {
				std::cout << "✓ OK";
				if (result.magnitude < 0.1) {
					std::cout << " (weak: " << std::fixed << std::setprecision(3)
					          << result.magnitude << ")";
				}
			} else {
				std::cout << "✗ MISSED";
			}
			std::cout << "\n";
		} catch (const std::exception& e) {
			std::cout << "✗ ERROR: " << e.what() << "\n";
			FrequencyTestResult result;
			result.frequency = freq;
			result.detected = false;
			result.detectedFreq = 0.0;
			result.frequencyError = 0.0;
			result.magnitude = 0.0;
			result.detectionCount = 0;
			allResults.push_back(result);
		}

		// Longer pause between tests to allow audio devices to fully close
		std::this_thread::sleep_for(std::chrono::milliseconds(300));
	}

	std::cout << std::string(75, '=') << "\n";

	// Display and save results
	printResults(allResults);

	std::string filename = "../BUILD/microphone_frequency_response.csv";
	saveResultsToCSV(filename, allResults);

	std::cout << "\n╔════════════════════════════════════════════════════════════════╗\n";
	std::cout << "║                    RECOMMENDATIONS                             ║\n";
	std::cout << "╚════════════════════════════════════════════════════════════════╝\n\n";

	// Count issues per band
	int band1Issues = 0, band2Issues = 0, band3Issues = 0, band4Issues = 0;
	for (const auto& result : allResults) {
		if (!result.detected) {
			if (result.frequency >= 5000 && result.frequency <= 8000) band1Issues++;
			else if (result.frequency >= 8500 && result.frequency <= 11500) band2Issues++;
			else if (result.frequency >= 12000 && result.frequency <= 15000) band3Issues++;
			else if (result.frequency >= 15500 && result.frequency <= 18500) band4Issues++;
		}
	}

	if (band1Issues > 0 || band2Issues > 0 || band3Issues > 0 || band4Issues > 0) {
		std::cout << "⚠️  Your microphone has issues with certain frequency bands.\n\n";

		if (band1Issues > 0) {
			std::cout << "  • Band 1 (5000-8000 Hz): " << band1Issues << " frequencies missed\n";
		}
		if (band2Issues > 0) {
			std::cout << "  • Band 2 (8500-11500 Hz): " << band2Issues << " frequencies missed\n";
		}
		if (band3Issues > 0) {
			std::cout << "  • Band 3 (12000-15000 Hz): " << band3Issues << " frequencies missed\n";
		}
		if (band4Issues > 0) {
			std::cout << "  • Band 4 (15500-18500 Hz): " << band4Issues << " frequencies missed\n";
		}

		std::cout << "\nSuggestions:\n";
		std::cout << "  1. Use your computer's built-in microphone (you said it works better)\n";
		std::cout << "  2. Increase speaker volume\n";
		std::cout << "  3. Position microphone closer to speaker\n";
		std::cout << "  4. Test in quieter environment\n";
		std::cout << "  5. Consider using a different microphone with better frequency response\n";
	} else {
		std::cout << "✓ Your microphone can detect all test frequencies!\n";
		std::cout << "  The detection issues may be due to:\n";
		std::cout << "  - Environmental noise\n";
		std::cout << "  - Signal strength (try increasing volume)\n";
		std::cout << "  - Multiple tones interfering with each other in chords\n";
	}

	std::cout << "\nTest complete!\n";

	return 0;
}
