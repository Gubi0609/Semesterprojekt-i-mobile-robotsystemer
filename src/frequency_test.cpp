#include "../LIB/audio_receiver.h"
#include "../LIB/audio_comm.h"
#include <iostream>
#include <iomanip>
#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>
#include <map>
#include <vector>
#include <cmath>

// Frequency test tool for diagnosing microphone detection issues
// Tests all 64 frequencies (16 values × 4 tones) used in the protocol

std::atomic<bool> running(true);

void signalHandler(int signum) {
	std::cout << "\n\nReceived interrupt signal (" << signum << ")\n";
	std::cout << "Shutting down test...\n";
	running = false;
}

// Structure to track detection statistics for a frequency
struct FrequencyStats {
	double targetFreq = 0.0;
	int toneIndex = -1;
	int toneValue = -1;
	int detectionCount = 0;
	double totalMagnitude = 0.0;
	double avgMagnitude = 0.0;
	std::vector<double> detectedFrequencies;
	double avgDetectedFreq = 0.0;
	double freqDeviation = 0.0;
	std::chrono::steady_clock::time_point lastDetection;
};

// Global statistics map
std::map<int, FrequencyStats> frequencyStats; // Key: toneIndex * 16 + toneValue

void printTableHeader() {
	std::cout << "\n┌──────┬───────┬─────────┬────────┬────────────┬───────────┬─────────────┐\n";
	std::cout << "│ Tone │ Value │ Target  │ Detect │    Avg     │    Avg    │  Frequency  │\n";
	std::cout << "│      │       │   (Hz)  │ Count  │  Magnitude │ Detected  │  Deviation  │\n";
	std::cout << "├──────┼───────┼─────────┼────────┼────────────┼───────────┼─────────────┤\n";
}

void printTableFooter() {
	std::cout << "└──────┴───────┴─────────┴────────┴────────────┴───────────┴─────────────┘\n";
}

void printStatsRow(const FrequencyStats& stats, bool warning = false) {
	std::string marker = warning ? "⚠ " : "  ";

	std::cout << "│ " << marker << stats.toneIndex << "   │ "
			  << std::setw(5) << stats.toneValue << " │ "
			  << std::setw(7) << std::fixed << std::setprecision(1) << stats.targetFreq << " │ "
			  << std::setw(6) << stats.detectionCount << " │ "
			  << std::setw(10) << std::fixed << std::setprecision(4) << stats.avgMagnitude << " │ "
			  << std::setw(9) << std::fixed << std::setprecision(1) << stats.avgDetectedFreq << " │ "
			  << std::setw(11) << std::fixed << std::setprecision(1) << stats.freqDeviation << " │\n";
}

void printCurrentStatistics() {
	std::cout << "\n\n═══════════════════════════════════════════════════════════════════════════\n";
	std::cout << "                        DETECTION STATISTICS\n";
	std::cout << "═══════════════════════════════════════════════════════════════════════════\n";

	// Organize by tone
	for (int tone = 0; tone < 4; ++tone) {
		std::cout << "\n┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓\n";
		std::cout << "┃                         TONE " << tone << " STATISTICS                          ┃\n";
		std::cout << "┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛\n";

		printTableHeader();

		int detectedCount = 0;
		int missedCount = 0;
		double totalDeviation = 0.0;
		int deviationCount = 0;

		for (int value = 0; value < 16; ++value) {
			int key = tone * 16 + value;
			if (frequencyStats.find(key) != frequencyStats.end()) {
				const auto& stats = frequencyStats[key];
				bool warning = (stats.detectionCount == 0 || std::abs(stats.freqDeviation) > 100.0);
				printStatsRow(stats, warning);

				if (stats.detectionCount > 0) {
					detectedCount++;
					totalDeviation += std::abs(stats.freqDeviation);
					deviationCount++;
				} else {
					missedCount++;
				}
			}
		}

		printTableFooter();

		// Tone summary
		std::cout << "\nTone " << tone << " Summary:\n";
		std::cout << "  ✓ Detected: " << detectedCount << "/16 frequencies\n";
		if (missedCount > 0) {
			std::cout << "  ✗ Missing:  " << missedCount << "/16 frequencies\n";
		}
		if (deviationCount > 0) {
			double avgDeviation = totalDeviation / deviationCount;
			std::cout << "  ⟳ Avg Freq Deviation: " << std::fixed << std::setprecision(1)
					  << avgDeviation << " Hz\n";
			if (avgDeviation > 50.0) {
				std::cout << "  ⚠ WARNING: High frequency deviation detected!\n";
			}
		}
	}

	// Overall summary
	std::cout << "\n═══════════════════════════════════════════════════════════════════════════\n";
	std::cout << "                          OVERALL SUMMARY\n";
	std::cout << "═══════════════════════════════════════════════════════════════════════════\n";

	int totalDetected = 0;
	int totalMissed = 0;
	std::vector<int> problemTones;

	for (int tone = 0; tone < 4; ++tone) {
		int toneDetected = 0;
		int toneMissed = 0;

		for (int value = 0; value < 16; ++value) {
			int key = tone * 16 + value;
			if (frequencyStats.find(key) != frequencyStats.end()) {
				if (frequencyStats[key].detectionCount > 0) {
					toneDetected++;
				} else {
					toneMissed++;
				}
			}
		}

		totalDetected += toneDetected;
		totalMissed += toneMissed;

		if (toneMissed > 3) { // More than 3 missing = problem tone
			problemTones.push_back(tone);
		}
	}

	std::cout << "\nTotal Frequencies Detected: " << totalDetected << "/64\n";
	std::cout << "Total Frequencies Missing:  " << totalMissed << "/64\n";
	std::cout << "Detection Rate: " << std::fixed << std::setprecision(1)
			  << (100.0 * totalDetected / 64.0) << "%\n";

	if (!problemTones.empty()) {
		std::cout << "\n⚠ PROBLEM TONES: ";
		for (size_t i = 0; i < problemTones.size(); ++i) {
			std::cout << "Tone " << problemTones[i];
			if (i < problemTones.size() - 1) std::cout << ", ";
		}
		std::cout << "\n";
	}

	std::cout << "\n═══════════════════════════════════════════════════════════════════════════\n\n";
}

void printRecommendations() {
	std::cout << "\n┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓\n";
	std::cout << "┃                         RECOMMENDATIONS                                 ┃\n";
	std::cout << "┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛\n\n";

	// Analyze results
	int totalMissed = 0;
	std::map<int, int> toneMissed;
	std::map<int, double> toneAvgDeviation;
	std::map<int, int> toneDeviationCount;

	for (int tone = 0; tone < 4; ++tone) {
		toneMissed[tone] = 0;
		toneAvgDeviation[tone] = 0.0;
		toneDeviationCount[tone] = 0;

		for (int value = 0; value < 16; ++value) {
			int key = tone * 16 + value;
			if (frequencyStats.find(key) != frequencyStats.end()) {
				const auto& stats = frequencyStats[key];
				if (stats.detectionCount == 0) {
					toneMissed[tone]++;
					totalMissed++;
				} else {
					toneAvgDeviation[tone] += std::abs(stats.freqDeviation);
					toneDeviationCount[tone]++;
				}
			}
		}

		if (toneDeviationCount[tone] > 0) {
			toneAvgDeviation[tone] /= toneDeviationCount[tone];
		}
	}

	// Provide recommendations
	if (totalMissed > 10) {
		std::cout << "❌ CRITICAL: Many frequencies not detected (" << totalMissed << "/64)\n\n";
		std::cout << "Possible causes:\n";
		std::cout << "  1. Microphone not working or not enabled\n";
		std::cout << "  2. Speaker volume too low\n";
		std::cout << "  3. Too much ambient noise\n";
		std::cout << "  4. Microphone frequency response limited\n";
		std::cout << "  5. Distance between speaker and microphone too large\n\n";
		std::cout << "Try:\n";
		std::cout << "  • Check microphone settings and permissions\n";
		std::cout << "  • Increase speaker volume\n";
		std::cout << "  • Test in a quieter environment\n";
		std::cout << "  • Move speaker closer to microphone\n";
	} else if (totalMissed > 0) {
		std::cout << "⚠ WARNING: Some frequencies not detected (" << totalMissed << "/64)\n\n";

		// Check which tones have issues
		for (int tone = 0; tone < 4; ++tone) {
			if (toneMissed[tone] > 0) {
				std::cout << "  Tone " << tone << ": " << toneMissed[tone] << " frequencies missing\n";

				// Provide tone-specific advice
				if (tone == 3) {
					std::cout << "    → High frequency range (13.5-16 kHz)\n";
					std::cout << "    → May be outside microphone's frequency response\n";
				} else if (tone == 0) {
					std::cout << "    → Low frequency range (4.5-7 kHz)\n";
					std::cout << "    → Check for ambient noise interference\n";
				}
			}
		}
		std::cout << "\n";
	} else {
		std::cout << "✓ GOOD: All frequencies detected!\n\n";
	}

	// Check frequency deviation
	bool highDeviation = false;
	for (int tone = 0; tone < 4; ++tone) {
		if (toneAvgDeviation[tone] > 75.0) {
			if (!highDeviation) {
				std::cout << "⚠ WARNING: High frequency deviation detected\n\n";
				highDeviation = true;
			}
			std::cout << "  Tone " << tone << ": " << std::fixed << std::setprecision(1)
					  << toneAvgDeviation[tone] << " Hz average deviation\n";
		}
	}

	if (highDeviation) {
		std::cout << "\nCurrent detection tolerance: 150 Hz\n";
		std::cout << "Consider:\n";
		std::cout << "  • Increasing detectionTolerance in receiver config\n";
		std::cout << "  • Checking speaker frequency accuracy\n";
		std::cout << "  • Testing with a different speaker/microphone\n\n";
	} else if (totalMissed == 0) {
		double maxDeviation = 0.0;
		for (int tone = 0; tone < 4; ++tone) {
			if (toneAvgDeviation[tone] > maxDeviation) {
				maxDeviation = toneAvgDeviation[tone];
			}
		}
		std::cout << "✓ Frequency deviation: " << std::fixed << std::setprecision(1)
				  << maxDeviation << " Hz (GOOD)\n";
		std::cout << "\nYour system is working well!\n";
		std::cout << "Current tolerance (150 Hz) is appropriate.\n\n";
	}

	std::cout << "┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛\n\n";
}

int main(int argc, char* argv[]) {
	std::cout << "╔════════════════════════════════════════════════════════════════════╗\n";
	std::cout << "║           FREQUENCY DETECTION TEST TOOL                            ║\n";
	std::cout << "║   Tests all 64 frequencies used in the communication protocol      ║\n";
	std::cout << "╚════════════════════════════════════════════════════════════════════╝\n\n";

	// Register signal handler
	signal(SIGINT, signalHandler);
	signal(SIGTERM, signalHandler);

	// Create encoder to know what frequencies to expect
	AudioComm::ChordConfig config;
	AudioComm::ChordEncoder encoder(config);

	// Initialize frequency statistics for all 64 frequencies
	std::cout << "Initializing test for 64 frequencies (4 tones × 16 values)...\n\n";

	for (int tone = 0; tone < 4; ++tone) {
		std::cout << "Tone " << tone << " frequency range: ";
		switch(tone) {
			case 0: std::cout << "4.5 - 7.0 kHz\n"; break;
			case 1: std::cout << "7.5 - 10.0 kHz\n"; break;
			case 2: std::cout << "10.5 - 13.0 kHz\n"; break;
			case 3: std::cout << "13.5 - 16.0 kHz\n"; break;
		}

		for (int value = 0; value < 16; ++value) {
			int key = tone * 16 + value;
			FrequencyStats stats;
			stats.targetFreq = encoder.getFrequency(tone, value);
			stats.toneIndex = tone;
			stats.toneValue = value;
			frequencyStats[key] = stats;
		}
	}

	std::cout << "\n";

	// Create decoder for detection
	AudioComm::ChordDecoder decoder(config);

	// Create receiver with high sensitivity
	AudioComm::ChordReceiver receiver;
	AudioComm::ChordReceiver::Config recvConfig;

	// High-resolution FFT for accurate frequency detection
	recvConfig.fftSize = 16384;
	recvConfig.detectionTolerance = 150.0;
	recvConfig.minDetections = 1;  // Accept single detections for testing
	recvConfig.consistencyWindow = 0.5;
	recvConfig.updateRate = 50.0;  // Fast updates

	std::cout << "Receiver Configuration:\n";
	std::cout << "  FFT Size: " << recvConfig.fftSize << " samples\n";
	std::cout << "  Frequency Resolution: " << std::fixed << std::setprecision(2)
			  << (recvConfig.sampleRate / recvConfig.fftSize) << " Hz/bin\n";
	std::cout << "  Detection Tolerance: " << recvConfig.detectionTolerance << " Hz\n";
	std::cout << "  Update Rate: " << recvConfig.updateRate << " Hz\n\n";

	std::cout << "═══════════════════════════════════════════════════════════════════════\n";
	std::cout << "                    TEST INSTRUCTIONS\n";
	std::cout << "═══════════════════════════════════════════════════════════════════════\n\n";
	std::cout << "1. Use the transmitter to send test signals\n";
	std::cout << "2. Try to transmit various values to test different frequencies\n";
	std::cout << "3. Let the test run for at least 30-60 seconds\n";
	std::cout << "4. The tool will track which frequencies are detected\n";
	std::cout << "5. Press Ctrl+C when done to see the full report\n\n";
	std::cout << "TIP: Use a protocol transmitter or generate test tones covering\n";
	std::cout << "     the full range of values (0x000 to 0xFFF)\n\n";
	std::cout << "═══════════════════════════════════════════════════════════════════════\n\n";

	std::cout << "Starting receiver...\n";
	std::cout << "Listening for frequencies...\n\n";

	auto startTime = std::chrono::steady_clock::now();
	int totalDetections = 0;

	// Detection callback
	auto detectionCallback = [&](const AudioComm::ChordReceiver::Detection& det) {
		totalDetections++;
		auto now = std::chrono::steady_clock::now();

		// Process each detected frequency
		for (size_t i = 0; i < det.frequencies.size() && i < det.toneValues.size(); ++i) {
			double detectedFreq = det.frequencies[i];
			uint8_t toneValue = det.toneValues[i];

			// Find which tone this is
			auto result = decoder.decodeSingleFrequency(detectedFreq);
			int toneIndex = result.first;
			int decodedValue = result.second;

			if (toneIndex >= 0 && toneIndex < 4) {
				int key = toneIndex * 16 + decodedValue;

				if (frequencyStats.find(key) != frequencyStats.end()) {
					FrequencyStats& stats = frequencyStats[key];
					stats.detectionCount++;
					stats.totalMagnitude += det.magnitude;
					stats.avgMagnitude = stats.totalMagnitude / stats.detectionCount;
					stats.detectedFrequencies.push_back(detectedFreq);
					stats.lastDetection = now;

					// Calculate average detected frequency and deviation
					double sum = 0.0;
					for (double f : stats.detectedFrequencies) {
						sum += f;
					}
					stats.avgDetectedFreq = sum / stats.detectedFrequencies.size();
					stats.freqDeviation = stats.avgDetectedFreq - stats.targetFreq;

					// Print live detection
					std::cout << "[" << std::fixed << std::setprecision(2)
							  << std::chrono::duration<double>(now - startTime).count()
							  << "s] Detected: Tone " << toneIndex
							  << ", Value " << std::setw(2) << decodedValue
							  << " | Target: " << std::setw(7) << std::setprecision(1) << stats.targetFreq
							  << " Hz | Detected: " << std::setw(7) << detectedFreq
							  << " Hz | Deviation: " << std::setw(6) << std::showpos << stats.freqDeviation
							  << std::noshowpos << " Hz | Count: " << stats.detectionCount << "\n";
				}
			}
		}
	};

	// Start receiving
	bool receiverStarted = receiver.startReceiving(recvConfig, detectionCallback);

	if (!receiverStarted) {
		std::cerr << "✗ ERROR: Failed to start audio receiver!\n";
		return 1;
	}

	std::cout << "✓ Receiver started successfully\n";
	std::cout << "Monitoring for frequency detections...\n\n";

	// Keep running until interrupted
	auto lastStatsUpdate = std::chrono::steady_clock::now();
	const double STATS_INTERVAL = 10.0; // Print stats every 10 seconds

	while (running) {
		std::this_thread::sleep_for(std::chrono::milliseconds(100));

		auto now = std::chrono::steady_clock::now();
		auto timeSinceLastStats = std::chrono::duration<double>(now - lastStatsUpdate).count();

		// Print periodic statistics
		if (timeSinceLastStats >= STATS_INTERVAL) {
			printCurrentStatistics();
			lastStatsUpdate = now;
		}
	}

	// Stop receiver
	std::cout << "\n\nStopping receiver...\n";
	receiver.stop();

	// Print final statistics
	std::cout << "\n\n";
	std::cout << "╔════════════════════════════════════════════════════════════════════╗\n";
	std::cout << "║                      FINAL TEST RESULTS                            ║\n";
	std::cout << "╚════════════════════════════════════════════════════════════════════╝\n";

	auto endTime = std::chrono::steady_clock::now();
	double testDuration = std::chrono::duration<double>(endTime - startTime).count();

	std::cout << "\nTest Duration: " << std::fixed << std::setprecision(1) << testDuration << " seconds\n";
	std::cout << "Total Detections: " << totalDetections << "\n";

	printCurrentStatistics();
	printRecommendations();

	std::cout << "\nTest complete!\n";

	return 0;
}
