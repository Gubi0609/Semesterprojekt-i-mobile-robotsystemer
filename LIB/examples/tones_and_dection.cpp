#include "tone_generator.h"
#include "frequency_detector.h"
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>

// Combined example: Generate tones and detect them simultaneously

int main() {
	std::cout << "=== Combined Tone Generation and Detection ===\n\n";

	ToneGenerator generator;
	FrequencyDetector detector;

	// Test 1: Generate and detect a single tone
	std::cout << "Test 1: Generate 1000 Hz and detect it\n";
	std::cout << "----------------------------------------\n";

	ToneGenerator::Config genConfig1;
	genConfig1.frequencies = {1000.0};
	genConfig1.duration = 5.0;
	genConfig1.gain = 0.5;

	FrequencyDetector::Config detConfig1;
	detConfig1.duration = 5.0;
	detConfig1.numPeaks = 3;
	detConfig1.updateRate = 2.0;  // 2 updates per second

	// Start generator
	generator.startAsync(genConfig1);
	std::this_thread::sleep_for(std::chrono::milliseconds(500));  // Let it stabilize

	// Start detector with callback
	detector.startAsync(detConfig1, [](const std::vector<FrequencyDetector::FrequencyPeak>& peaks) {
		std::cout << "Detected: ";
		for (size_t i = 0; i < peaks.size() && i < 3; ++i) {
			std::cout << std::fixed << std::setprecision(1)
					  << peaks[i].frequency << " Hz ("
					  << std::setprecision(0)
					  << (peaks[i].magnitude * 100.0) << "%)";
			if (i < peaks.size() - 1 && i < 2) std::cout << " | ";
		}
		std::cout << "\n";
	});

	detector.waitForCompletion();
	generator.stop();
	std::cout << "\n";

	// Test 2: Multiple frequencies with bandpass filter
	std::cout << "Test 2: Generate 500, 1000, 5000 Hz, filter 800-1200 Hz\n";
	std::cout << "--------------------------------------------------------\n";

	ToneGenerator::Config genConfig2;
	genConfig2.frequencies = {500.0, 1000.0, 5000.0};
	genConfig2.duration = 5.0;
	genConfig2.gain = 0.3;

	FrequencyDetector::Config detConfig2;
	detConfig2.duration = 5.0;
	detConfig2.bandpassLow = 800.0;
	detConfig2.bandpassHigh = 1200.0;
	detConfig2.numPeaks = 5;
	detConfig2.updateRate = 2.0;

	generator.startAsync(genConfig2);
	std::this_thread::sleep_for(std::chrono::milliseconds(500));

	detector.startAsync(detConfig2, [](const std::vector<FrequencyDetector::FrequencyPeak>& peaks) {
		std::cout << "Detected in 800-1200 Hz band: ";
		if (peaks.empty()) {
			std::cout << "None";
		}
		for (size_t i = 0; i < peaks.size(); ++i) {
			std::cout << std::fixed << std::setprecision(1)
					  << peaks[i].frequency << " Hz";
			if (i < peaks.size() - 1) std::cout << " | ";
		}
		std::cout << "\n";
	});

	detector.waitForCompletion();
	generator.stop();
	std::cout << "Expected: Should only detect ~1000 Hz (others filtered out)\n\n";

	// Test 3: Close frequencies with high resolution
	std::cout << "Test 3: Generate 1000 and 1050 Hz (50 Hz apart), high res FFT\n";
	std::cout << "--------------------------------------------------------------\n";

	ToneGenerator::Config genConfig3;
	genConfig3.frequencies = {1000.0, 1050.0};
	genConfig3.duration = 5.0;
	genConfig3.gain = 0.4;

	FrequencyDetector::Config detConfig3;
	detConfig3.duration = 5.0;
	detConfig3.fftSize = 8192;  // High resolution
	detConfig3.numPeaks = 5;
	detConfig3.updateRate = 1.0;

	std::cout << "Frequency resolution: " << (48000.0 / 8192.0) << " Hz\n";

	generator.startAsync(genConfig3);
	std::this_thread::sleep_for(std::chrono::milliseconds(500));

	detector.startAsync(detConfig3, [](const std::vector<FrequencyDetector::FrequencyPeak>& peaks) {
		std::cout << "Detected: ";
		for (size_t i = 0; i < peaks.size() && i < 5; ++i) {
			std::cout << std::fixed << std::setprecision(1)
					  << peaks[i].frequency << " Hz";
			if (i < peaks.size() - 1 && i < 4) std::cout << " | ";
		}
		std::cout << "\n";
	});

	detector.waitForCompletion();
	generator.stop();
	std::cout << "Expected: Should distinguish both 1000 and 1050 Hz\n\n";

	std::cout << "All tests completed!\n";
	return 0;
}
