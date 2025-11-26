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

// Frequency test tool for SINGLE TONE detection (matches single-tone transmitter)
// Tests individual frequencies one at a time

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
	}

	std::cout << "\nTotal Frequencies Detected: " << totalDetected << "/64\n";
	std::cout << "Total Frequencies Missing:  " << totalMissed << "/64\n";
	std::cout << "Detection Rate: " << std::fixed << std::setprecision(1)
			  << (100.0 * totalDetected / 64.0) << "%\n\n";
}

int main(int argc, char* argv[]) {
	std::cout << "╔════════════════════════════════════════════════════════════════════╗\n";
	std::cout << "║       SINGLE-TONE FREQUENCY DETECTION TEST TOOL                    ║\n";
	std::cout << "║   Tests all 64 frequencies used in the communication protocol      ║\n";
	std::cout << "╚════════════════════════════════════════════════════════════════════╝\n\n";

	// Register signal handler
	signal(SIGINT, signalHandler);
	signal(SIGTERM, signalHandler);

	// Parse performance mode
	int perfMode = 1;
	if (argc > 1) {
		perfMode = std::stoi(argv[1]);
	}

	int fftSize;
	double updateRate;
	std::string modeName;

	switch(perfMode) {
		case 0: // Extreme speed
			fftSize = 2048;
			updateRate = 5.0;
			modeName = "Extreme Speed (Pi Optimized)";
			break;
		case 1: // Ultra-fast (default)
			fftSize = 4096;
			updateRate = 10.0;
			modeName = "Ultra-Fast (Default)";
			break;
		case 2: // Balanced
			fftSize = 8192;
			updateRate = 20.0;
			modeName = "Balanced";
			break;
		case 3: // Accurate
			fftSize = 16384;
			updateRate = 30.0;
			modeName = "Accurate";
			break;
		default:
			fftSize = 4096;
			updateRate = 10.0;
			modeName = "Ultra-Fast (Default)";
			break;
	}

	std::cout << "Performance Mode: " << modeName << "\n";
	std::cout << "  FFT Size: " << fftSize << "\n";
	std::cout << "  Target Update Rate: " << updateRate << " Hz\n\n";

	// Create encoder to know what frequencies to expect
	AudioComm::ChordConfig config;
	AudioComm::ChordEncoder encoder(config);

	// Initialize frequency statistics
	std::cout << "Initializing test for 64 frequencies (4 tones × 16 values)...\n\n";

	for (int tone = 0; tone < 4; ++tone) {
		for (int value = 0; value < 16; ++value) {
			int key = tone * 16 + value;
			FrequencyStats stats;
			stats.targetFreq = encoder.getFrequency(tone, value);
			stats.toneIndex = tone;
			stats.toneValue = value;
			frequencyStats[key] = stats;
		}
	}

	// Create decoder
	AudioComm::ChordDecoder decoder(config);

	// Create SINGLE-TONE receiver
	AudioComm::SingleToneReceiver receiver;
	AudioComm::SingleToneReceiver::Config recvConfig;

	// Wide frequency range to catch all tones
	recvConfig.minFreq = 4000.0;
	recvConfig.maxFreq = 17000.0;
	recvConfig.fftSize = fftSize;
	recvConfig.detectionTolerance = 200.0;  // Wide tolerance
	recvConfig.updateRate = updateRate;

	std::cout << "Receiver Configuration:\n";
	std::cout << "  FFT Size: " << recvConfig.fftSize << " samples\n";
	std::cout << "  Frequency Range: " << recvConfig.minFreq << " - " << recvConfig.maxFreq << " Hz\n";
	std::cout << "  Detection Tolerance: " << recvConfig.detectionTolerance << " Hz\n";
	std::cout << "  Update Rate: " << recvConfig.updateRate << " Hz\n\n";

	std::cout << "Starting receiver...\n";
	std::cout << "(Each '.' represents a detected frequency)\n\n";

	auto startTime = std::chrono::steady_clock::now();
	std::atomic<int> totalDetections(0);
	std::atomic<int> totalCallbacks(0);

	// Detection callback
	auto detectionCallback = [&](const AudioComm::SingleToneReceiver::Detection& det) {
		totalCallbacks++;
		auto now = std::chrono::steady_clock::now();

		double detectedFreq = det.frequency;

		// Find which tone this is
		auto result = decoder.decodeSingleFrequency(detectedFreq);
		int toneIndex = result.first;
		int decodedValue = result.second;

		if (toneIndex >= 0 && toneIndex < 4 && decodedValue >= 0 && decodedValue < 16) {
			int key = toneIndex * 16 + decodedValue;

			if (frequencyStats.find(key) != frequencyStats.end()) {
				FrequencyStats& stats = frequencyStats[key];
				stats.detectionCount++;
				stats.totalMagnitude += det.magnitude;
				stats.avgMagnitude = stats.totalMagnitude / stats.detectionCount;
				stats.lastDetection = now;

				// Calculate average detected frequency (running average)
				stats.avgDetectedFreq = (stats.avgDetectedFreq * (stats.detectionCount - 1) + detectedFreq) / stats.detectionCount;
				stats.freqDeviation = stats.avgDetectedFreq - stats.targetFreq;

				totalDetections++;
				std::cout << "." << std::flush;
			}
		}
	};

	// Start receiving
	bool receiverStarted = receiver.startReceiving(recvConfig, detectionCallback);

	if (!receiverStarted) {
		std::cerr << "✗ ERROR: Failed to start audio receiver!\n";
		return 1;
	}

	std::cout << "✓ Receiver started successfully\n\n";

	// Monitor performance
	auto lastRateCheck = std::chrono::steady_clock::now();
	int lastCallbackCount = 0;

	while (running) {
		std::this_thread::sleep_for(std::chrono::milliseconds(500));

		auto now = std::chrono::steady_clock::now();
		auto elapsed = std::chrono::duration<double>(now - lastRateCheck).count();

		if (elapsed >= 5.0) {
			int callbacksSinceLastCheck = totalCallbacks - lastCallbackCount;
			double detectionRate = callbacksSinceLastCheck / elapsed;

			std::cout << "\n[Detection Rate: " << std::fixed << std::setprecision(1)
					  << detectionRate << " Hz] ";
			std::cout.flush();

			lastRateCheck = now;
			lastCallbackCount = totalCallbacks;
		}
	}

	// Stop receiver
	std::cout << "\n\nStopping receiver...\n";
	receiver.stop();

	// Print final statistics
	auto endTime = std::chrono::steady_clock::now();
	double testDuration = std::chrono::duration<double>(endTime - startTime).count();

	std::cout << "\n╔════════════════════════════════════════════════════════════════════╗\n";
	std::cout << "║                      FINAL TEST RESULTS                            ║\n";
	std::cout << "╚════════════════════════════════════════════════════════════════════╝\n";

	std::cout << "\nTest Duration: " << std::fixed << std::setprecision(1) << testDuration << " seconds\n";
	std::cout << "Total Callbacks: " << totalCallbacks.load() << "\n";
	std::cout << "Average Detection Rate: " << std::fixed << std::setprecision(2)
			  << (totalCallbacks.load() / testDuration) << " Hz\n";
	std::cout << "Total Valid Detections: " << totalDetections.load() << "\n";

	printCurrentStatistics();

	std::cout << "\nTest complete!\n";

	return 0;
}
