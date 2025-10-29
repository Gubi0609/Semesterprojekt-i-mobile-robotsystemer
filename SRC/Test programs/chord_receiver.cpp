#include "frequency_detector.h"
#include "audio_comm.h"
#include <iostream>
#include <iomanip>
#include <vector>
#include <chrono>
#include <atomic>
#include <thread>
#include <map>

std::atomic<bool> running(true);

void printUsage(const char* progName) {
	std::cout << "Usage: " << progName << " [duration]\n"
			  << "  duration: Time to listen in seconds (default: continuous)\n"
			  << "\nThe receiver will detect 4-tone chords and decode them\n"
			  << "to 16-bit values (0-65535).\n"
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
	AudioComm::ChordConfig config;
	AudioComm::ChordDecoder decoder(config);

	std::cout << "\n=== Audio Communication Chord Receiver ===\n";
	std::cout << "Expecting 4 tones per chord (16 bits total):\n";
	std::cout << "Tone 1: " << config.tone1MinFreq << "-" << config.tone1MaxFreq << " Hz (bits 0-3)\n";
	std::cout << "Tone 2: " << config.tone2MinFreq << "-" << config.tone2MaxFreq << " Hz (bits 4-7)\n";
	std::cout << "Tone 3: " << config.tone3MinFreq << "-" << config.tone3MaxFreq << " Hz (bits 8-11)\n";
	std::cout << "Tone 4: " << config.tone4MinFreq << "-" << config.tone4MaxFreq << " Hz (bits 12-15)\n";
	std::cout << "Detection tolerance: +/- " << config.detectionTolerance << " Hz\n";
	std::cout << "FFT size: " << config.fftSize << "\n";

	if (duration > 0) {
		std::cout << "Listening for " << duration << " seconds...\n";
	} else {
		std::cout << "Listening continuously (Press Ctrl+C to stop)...\n";
	}
	std::cout << "\nWaiting for chords...\n\n";

	FrequencyDetector detector;

	// Setup detector configuration
	FrequencyDetector::Config detectorConfig;
	detectorConfig.sampleRate = static_cast<int>(config.sampleRate);
	detectorConfig.fftSize = config.fftSize;
	detectorConfig.numPeaks = 10; // Detect more peaks to find all 4 tones
	detectorConfig.duration = duration;
	detectorConfig.bandpassLow = config.tone1MinFreq - 500;  // Cover all 4 tone ranges
	detectorConfig.bandpassHigh = config.tone4MaxFreq + 500;
	detectorConfig.updateRate = 10.0; // 10 updates per second

	// Track detected tones
	std::map<int, int> currentTones; // toneIndex -> toneValue
	auto lastDecodeTime = std::chrono::steady_clock::now();
	int32_t lastDecodedValue = -1;

	// Chord consistency checking
	struct ChordCandidate {
		int32_t value;
		int count;
		std::chrono::steady_clock::time_point firstSeen;
		std::chrono::steady_clock::time_point lastSeen;
	};
	std::map<int32_t, ChordCandidate> chordCandidates; // value -> candidate info
	const int MIN_DETECTIONS = 2;  // Require at least 2 detections
	const double CONSISTENCY_WINDOW = 1.0; // Within 1 second

	// Detection callback
	auto callback = [&](const std::vector<FrequencyDetector::FrequencyPeak>& peaks) {
		if (peaks.empty()) return;

		auto now = std::chrono::steady_clock::now();
		double timeSinceLastDecode = std::chrono::duration<double>(now - lastDecodeTime).count();

		// Clear current tones
		currentTones.clear();

		// Try to identify which tones are present
		for (const auto& peak : peaks) {
			if (!decoder.isValidFrequency(peak.frequency)) continue;

			auto [toneIndex, toneValue] = decoder.decodeSingleFrequency(peak.frequency);
			if (toneIndex >= 0 && toneValue >= 0) {
				// Store the tone (only keep strongest if duplicates)
				if (currentTones.find(toneIndex) == currentTones.end()) {
					currentTones[toneIndex] = toneValue;
				}
			}
		}

		// Check if we have all 4 tones
		if (currentTones.size() == 4) {
			// Build frequency vector in order
			std::vector<double> detectedFreqs(4);
			bool hasAll = true;

			for (int i = 0; i < 4; ++i) {
				if (currentTones.find(i) == currentTones.end()) {
					hasAll = false;
					break;
				}
				// Reconstruct frequency from tone index and value
				int toneValue = currentTones[i];
				double minFreq, maxFreq;
				switch(i) {
					case 0: minFreq = config.tone1MinFreq; maxFreq = config.tone1MaxFreq; break;
					case 1: minFreq = config.tone2MinFreq; maxFreq = config.tone2MaxFreq; break;
					case 2: minFreq = config.tone3MinFreq; maxFreq = config.tone3MaxFreq; break;
					case 3: minFreq = config.tone4MinFreq; maxFreq = config.tone4MaxFreq; break;
					default: minFreq = maxFreq = 0;
				}
				double freqStep = (maxFreq - minFreq) / 15.0;
				detectedFreqs[i] = minFreq + (toneValue * freqStep);
			}

			if (hasAll) {
				// Decode the full chord
				int32_t decodedValue = decoder.decodeFrequencies(detectedFreqs);

				if (decodedValue >= 0) {
					// Clean up old candidates that are outside the consistency window
					auto it = chordCandidates.begin();
					while (it != chordCandidates.end()) {
						double age = std::chrono::duration<double>(now - it->second.firstSeen).count();
						if (age > CONSISTENCY_WINDOW) {
							it = chordCandidates.erase(it);
						} else {
							++it;
						}
					}

					// Update or create candidate
					if (chordCandidates.find(decodedValue) == chordCandidates.end()) {
						// New candidate
						chordCandidates[decodedValue] = {decodedValue, 1, now, now};
					} else {
						// Existing candidate - increment count
						chordCandidates[decodedValue].count++;
						chordCandidates[decodedValue].lastSeen = now;
					}

					// Check if this candidate has enough detections
					auto& candidate = chordCandidates[decodedValue];
					if (candidate.count >= MIN_DETECTIONS &&
						(decodedValue != lastDecodedValue || timeSinceLastDecode > 0.5)) {

						double detectionTime = std::chrono::duration<double>(
							candidate.lastSeen - candidate.firstSeen).count();

						std::cout << "\n=== CHORD CONFIRMED ===\n";
						std::cout << "Detected " << candidate.count << " times over "
								  << std::fixed << std::setprecision(2) << detectionTime << " seconds\n";
						std::cout << "Frequencies: ";
						for (size_t i = 0; i < detectedFreqs.size(); ++i) {
							std::cout << std::fixed << std::setprecision(1) << detectedFreqs[i] << " Hz";
							if (i < detectedFreqs.size() - 1) std::cout << ", ";
						}
						std::cout << "\n";

						// Show tone values
						for (int i = 0; i < 4; ++i) {
							std::cout << "Tone " << (i+1) << " (bits " << (i*4) << "-" << (i*4+3) << "): "
									  << currentTones[i] << " (0x" << std::hex << std::uppercase
									  << currentTones[i] << std::dec << ")\n";
						}

						// Show decoded value
						std::cout << "\nDecoded Value: " << decodedValue
								  << " (0x" << std::hex << std::uppercase << std::setw(4)
								  << std::setfill('0') << decodedValue << std::dec << ")\n";

						// Show binary
						std::cout << "Binary: ";
						for (int i = 15; i >= 0; --i) {
							std::cout << ((decodedValue >> i) & 1);
							if (i % 4 == 0 && i > 0) std::cout << " ";
						}
						std::cout << "\n";

						lastDecodedValue = decodedValue;
						lastDecodeTime = now;

						// Reset this candidate so we don't print it repeatedly
						chordCandidates.erase(decodedValue);
					}
				}
			}
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
