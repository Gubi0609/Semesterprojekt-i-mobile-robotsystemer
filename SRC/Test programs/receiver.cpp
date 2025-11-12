#include "frequency_detector.h"
#include "audio_comm.h"
#include <iostream>
#include <iomanip>
#include <vector>
#include <chrono>
#include <atomic>
#include <thread>

std::atomic<bool> running(true);

void printUsage(const char* progName) {
	std::cout << "Usage: " << progName << " [duration]\n"
			  << "  duration: Time to listen in seconds (default: continuous)\n"
			  << "\nThe receiver will detect tones in the 5000-8000 Hz range\n"
			  << "and decode them to 4-bit values (0-15).\n"
			  << "\nPress Ctrl+C to stop continuous listening.\n";
}

int main(int argc, char* argv[]) {
	double duration = 0.0; // 0 = continuous

	if (argc > 1) {
		if (std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help") {
			printUsage(argv[0]);
			return 0;
		}
		duration = std::atof(argv[1]);
		if (duration < 0) {
			std::cerr << "Error: Duration must be positive\n";
			return 1;
		}
	}

	// Setup decoder
	AudioComm::SingleToneConfig config;
	AudioComm::SingleToneDecoder decoder(config);

	std::cout << "\n=== Audio Communication Receiver ===\n";
	std::cout << "Frequency range: " << config.minFreq << " - " << config.maxFreq << " Hz\n";
	std::cout << "Detection tolerance: +/- " << config.detectionTolerance << " Hz\n";
	std::cout << "FFT size: " << config.fftSize << "\n";

	if (duration > 0) {
		std::cout << "Listening for " << duration << " seconds...\n";
	} else {
		std::cout << "Listening continuously (Press Ctrl+C to stop)...\n";
	}
	std::cout << "\nWaiting for tones...\n\n";

	FrequencyDetector detector;

	// Setup detector configuration
	FrequencyDetector::Config detectorConfig;
	detectorConfig.sampleRate = static_cast<int>(config.sampleRate);
	detectorConfig.fftSize = config.fftSize;
	detectorConfig.numPeaks = 5; // Detect top 5 peaks
	detectorConfig.duration = duration;
	detectorConfig.bandpassLow = config.minFreq - 500;  // Bandpass filter
	detectorConfig.bandpassHigh = config.maxFreq + 500;
	detectorConfig.updateRate = 10.0; // 10 updates per second

	// Track last decoded value to avoid duplicates
	int lastDecodedValue = -1;
	auto lastDecodeTime = std::chrono::steady_clock::now();
	const double MIN_REPEAT_TIME = 0.15; // Minimum time between repeats (seconds)

	// Detection callback
	auto callback = [&](const std::vector<FrequencyDetector::FrequencyPeak>& peaks) {
		if (peaks.empty()) return;

		auto now = std::chrono::steady_clock::now();
		double timeSinceLastDecode = std::chrono::duration<double>(now - lastDecodeTime).count();

		// Find the strongest peak in our frequency range
		for (const auto& peak : peaks) {
			if (!decoder.isValidFrequency(peak.frequency)) continue;

			int decodedValue = decoder.decodeFrequency(peak.frequency);
			if (decodedValue < 0) continue;

			// Avoid printing duplicate values too quickly
			if (decodedValue != lastDecodedValue || timeSinceLastDecode > MIN_REPEAT_TIME) {
				std::cout << std::fixed << std::setprecision(1);
				std::cout << "Detected: " << std::setw(6) << peak.frequency << " Hz"
						  << " => Value: " << std::setw(2) << decodedValue
						  << " (0x" << std::hex << std::uppercase << decodedValue << std::dec << ")"
						  << " | Magnitude: " << std::setw(4) << std::setprecision(2)
						  << (peak.magnitude * 100.0) << "%";

				// Show binary representation
				std::cout << " | Binary: ";
				for (int bit = 3; bit >= 0; --bit) {
					std::cout << ((decodedValue >> bit) & 1);
				}
				std::cout << "\n";

				lastDecodedValue = decodedValue;
				lastDecodeTime = now;
			}
			break; // Only process the first valid peak
		}
	};

	// Start detection
	if (!detector.startAsync(detectorConfig, callback)) {
		std::cerr << "Error: Failed to start frequency detector\n";
		return 1;
	}

	// Wait for completion or Ctrl+C
	if (duration > 0) {
		detector.waitForCompletion();
	} else {
		// Continuous mode - wait for Ctrl+C
		std::cout << "\n(Ctrl+C to stop)\n";
		while (detector.isDetecting()) {
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
	}

	detector.stop();
	std::cout << "\nReceiver stopped.\n";

	return 0;
}
