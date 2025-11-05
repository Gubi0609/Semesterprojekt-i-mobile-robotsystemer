#include "frequency_detector.h"
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>

// Example usage of FrequencyDetector library

void printPeaks(const std::vector<FrequencyDetector::FrequencyPeak>& peaks) {
	if (peaks.empty()) {
		std::cout << "No frequencies detected\n";
		return;
	}

	for (size_t i = 0; i < peaks.size(); ++i) {
		std::cout << std::fixed << std::setprecision(1)
				  << peaks[i].frequency << " Hz ("
				  << std::setprecision(0)
				  << (peaks[i].magnitude * 100.0) << "%)";
		if (i < peaks.size() - 1) std::cout << " | ";
	}
	std::cout << "\n";
}

int main() {
	FrequencyDetector detector;

	std::cout << "=== FrequencyDetector Library Examples ===\n\n";

	// Example 1: Detect for 5 seconds (blocking)
	std::cout << "1. Detecting frequencies for 5 seconds (blocking)...\n";
	std::cout << "   Make some noise or play a tone!\n";
	FrequencyDetector::Config config1;
	config1.duration = 5.0;
	config1.numPeaks = 5;
	auto peaks = detector.start(config1);
	std::cout << "   Final detected frequencies: ";
	printPeaks(peaks);
	std::cout << "\n";

	// Example 2: Async detection with callback
	std::cout << "2. Detecting with real-time callback for 5 seconds...\n";
	FrequencyDetector::Config config2;
	config2.duration = 5.0;
	config2.numPeaks = 3;
	config2.updateRate = 10.0;  // 10 updates per second

	detector.startAsync(config2, [](const std::vector<FrequencyDetector::FrequencyPeak>& peaks) {
		std::cout << "   Detected: ";
		printPeaks(peaks);
	});

	detector.waitForCompletion();
	std::cout << "\n";

	// Example 3: With bandpass filter
	std::cout << "3. Detecting with bandpass filter (800-1200 Hz) for 5 seconds...\n";
	FrequencyDetector::Config config3;
	config3.duration = 5.0;
	config3.bandpassLow = 800.0;
	config3.bandpassHigh = 1200.0;
	config3.numPeaks = 3;

	peaks = detector.start(config3);
	std::cout << "   Detected frequencies in range: ";
	printPeaks(peaks);
	std::cout << "\n";

	// Example 4: High resolution detection
	std::cout << "4. High resolution detection (FFT 8192) for 3 seconds...\n";
	FrequencyDetector::Config config4;
	config4.duration = 3.0;
	config4.fftSize = 8192;
	config4.numPeaks = 5;

	peaks = detector.start(config4);
	std::cout << "   Resolution: " << (48000.0 / 8192.0) << " Hz per bin\n";
	std::cout << "   Detected: ";
	printPeaks(peaks);
	std::cout << "\n";

	// Example 5: Continuous detection (manual stop)
	std::cout << "5. Continuous detection (will stop after 3 seconds)...\n";
	FrequencyDetector::Config config5;
	config5.duration = 0.0;  // Continuous
	config5.numPeaks = 3;
	config5.updateRate = 5.0;

	detector.startAsync(config5, [](const std::vector<FrequencyDetector::FrequencyPeak>& peaks) {
		std::cout << "   ";
		printPeaks(peaks);
	});

	std::this_thread::sleep_for(std::chrono::seconds(3));
	detector.stop();
	std::cout << "   Stopped!\n\n";

	std::cout << "All examples completed!\n";
	return 0;
}
