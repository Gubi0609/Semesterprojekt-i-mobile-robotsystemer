#include "../LIB/audio_receiver.h"
#include "../LIB/audio_comm.h"
#include "../LIB/frequency_detector.h"
#include <iostream>
#include <iomanip>
#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>
#include <map>
#include <vector>

// Chord receiver diagnostic tool - shows what's happening inside the detection process

std::atomic<bool> running(true);

void signalHandler(int signum) {
	std::cout << "\n\nReceived interrupt signal (" << signum << ")\n";
	std::cout << "Shutting down diagnostic...\n";
	running = false;
}

// Performance statistics
struct PerfStats {
	std::atomic<int> fftCallbacks{0};
	std::atomic<int> peaksDetected{0};
	std::atomic<int> validFreqsFound{0};
	std::atomic<int> tone0Found{0};
	std::atomic<int> tone1Found{0};
	std::atomic<int> tone2Found{0};
	std::atomic<int> tone3Found{0};
	std::atomic<int> fullChordsFound{0};
	std::atomic<int> confirmedChords{0};

	std::chrono::steady_clock::time_point startTime;

	void reset() {
		fftCallbacks = 0;
		peaksDetected = 0;
		validFreqsFound = 0;
		tone0Found = 0;
		tone1Found = 0;
		tone2Found = 0;
		tone3Found = 0;
		fullChordsFound = 0;
		confirmedChords = 0;
		startTime = std::chrono::steady_clock::now();
	}

	void printReport() const {
		auto now = std::chrono::steady_clock::now();
		double elapsed = std::chrono::duration<double>(now - startTime).count();

		std::cout << "\n╔════════════════════════════════════════════════════════════════════╗\n";
		std::cout << "║                  CHORD DETECTION DIAGNOSTICS                       ║\n";
		std::cout << "╚════════════════════════════════════════════════════════════════════╝\n\n";

		std::cout << "Time Elapsed: " << std::fixed << std::setprecision(1) << elapsed << " seconds\n\n";

		std::cout << "FFT Processing:\n";
		std::cout << "  Total FFT callbacks: " << fftCallbacks.load() << "\n";
		std::cout << "  FFT rate: " << std::fixed << std::setprecision(2)
				  << (fftCallbacks.load() / elapsed) << " Hz\n\n";

		std::cout << "Peak Detection:\n";
		std::cout << "  Total peaks detected: " << peaksDetected.load() << "\n";
		std::cout << "  Valid frequencies: " << validFreqsFound.load() << "\n";
		std::cout << "  Avg peaks per FFT: " << std::fixed << std::setprecision(2)
				  << (fftCallbacks.load() > 0 ? (double)peaksDetected.load() / fftCallbacks.load() : 0) << "\n\n";

		std::cout << "Tone Detection by Range:\n";
		std::cout << "  Tone 0 (4.5-7 kHz):    " << tone0Found.load() << " detections\n";
		std::cout << "  Tone 1 (7.5-10 kHz):   " << tone1Found.load() << " detections\n";
		std::cout << "  Tone 2 (10.5-13 kHz):  " << tone2Found.load() << " detections\n";
		std::cout << "  Tone 3 (13.5-16 kHz):  " << tone3Found.load() << " detections\n\n";

		std::cout << "Chord Detection:\n";
		std::cout << "  Full chords (all 4 tones): " << fullChordsFound.load() << "\n";
		std::cout << "  Confirmed chords (passed consistency): " << confirmedChords.load() << "\n";

		if (fftCallbacks.load() > 0) {
			std::cout << "  Chord detection rate: "
					  << std::fixed << std::setprecision(4)
					  << (100.0 * fullChordsFound.load() / fftCallbacks.load()) << "% of FFT callbacks\n";
		}

		std::cout << "\n";

		// Bottleneck analysis
		std::cout << "Bottleneck Analysis:\n";
		if (fftCallbacks.load() < elapsed * 5) {
			std::cout << "  ⚠ FFT rate is very low (" << (fftCallbacks.load() / elapsed) << " Hz)\n";
			std::cout << "    → CPU may be too slow or FFT size too large\n";
		} else if (peaksDetected.load() < fftCallbacks.load() * 0.1) {
			std::cout << "  ⚠ Very few peaks detected\n";
			std::cout << "    → Signal may be too weak or noisy\n";
		} else if (validFreqsFound.load() < peaksDetected.load() * 0.5) {
			std::cout << "  ⚠ Most peaks are outside valid frequency ranges\n";
			std::cout << "    → Lots of noise or wrong frequency ranges configured\n";
		} else if (fullChordsFound.load() == 0) {
			std::cout << "  ⚠ NO full chords detected (all 4 tones never present together)\n";
			std::cout << "    → Transmitter may be sending single tones, not chords\n";
			std::cout << "    → Or tones aren't being transmitted simultaneously\n";

			int maxTone = std::max({tone0Found.load(), tone1Found.load(),
									tone2Found.load(), tone3Found.load()});
			if (maxTone > 0) {
				std::cout << "    → Individual tones ARE being detected\n";
				std::cout << "    → Use frequency_test_single for single-tone testing\n";
			}
		} else if (confirmedChords.load() < fullChordsFound.load() * 0.1) {
			std::cout << "  ⚠ Full chords found but few confirmed\n";
			std::cout << "    → Consistency requirements may be too strict\n";
			std::cout << "    → Try reducing minDetections or increasing consistencyWindow\n";
		} else {
			std::cout << "  ✓ System appears to be working normally\n";
		}

		std::cout << "\n";
	}
};

PerfStats stats;

int main(int argc, char* argv[]) {
	std::cout << "╔════════════════════════════════════════════════════════════════════╗\n";
	std::cout << "║           CHORD RECEIVER DIAGNOSTIC TOOL                           ║\n";
	std::cout << "║   Shows detailed information about chord detection process         ║\n";
	std::cout << "╚════════════════════════════════════════════════════════════════════╝\n\n";

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
		case 0:
			fftSize = 2048;
			updateRate = 5.0;
			modeName = "Extreme Speed";
			break;
		case 1:
			fftSize = 4096;
			updateRate = 10.0;
			modeName = "Ultra-Fast (Default)";
			break;
		case 2:
			fftSize = 8192;
			updateRate = 20.0;
			modeName = "Balanced";
			break;
		case 3:
			fftSize = 16384;
			updateRate = 30.0;
			modeName = "Accurate";
			break;
		default:
			fftSize = 4096;
			updateRate = 10.0;
			modeName = "Ultra-Fast";
			break;
	}

	std::cout << "Performance Mode: " << modeName << "\n";
	std::cout << "  FFT Size: " << fftSize << "\n";
	std::cout << "  Target Update Rate: " << updateRate << " Hz\n\n";

	// Create decoder for analysis
	AudioComm::ChordConfig chordConfig;
	AudioComm::ChordDecoder decoder(chordConfig);

	// Create a custom frequency detector to intercept callbacks
	FrequencyDetector::Config detConfig;
	detConfig.sampleRate = 48000;
	detConfig.fftSize = fftSize;
	detConfig.numPeaks = 10;
	detConfig.duration = 0.0;
	detConfig.bandpassLow = 4000.0;
	detConfig.bandpassHigh = 17000.0;
	detConfig.updateRate = updateRate;

	FrequencyDetector detector;

	std::cout << "Starting diagnostic...\n";
	std::cout << "(Live updates every 5 seconds, full report on Ctrl+C)\n\n";

	stats.reset();

	// Detection callback with diagnostics
	auto callback = [&decoder](const std::vector<FrequencyDetector::FrequencyPeak>& peaks) {
		stats.fftCallbacks++;
		stats.peaksDetected += peaks.size();

		// Analyze peaks
		std::map<int, int> tonesFound;

		for (const auto& peak : peaks) {
			if (!decoder.isValidFrequency(peak.frequency)) {
				continue;
			}

			stats.validFreqsFound++;

			auto [toneIndex, toneValue] = decoder.decodeSingleFrequency(peak.frequency);
			if (toneIndex >= 0 && toneValue >= 0) {
				tonesFound[toneIndex] = toneValue;

				switch(toneIndex) {
					case 0: stats.tone0Found++; break;
					case 1: stats.tone1Found++; break;
					case 2: stats.tone2Found++; break;
					case 3: stats.tone3Found++; break;
				}
			}
		}

		// Check if all 4 tones present
		if (tonesFound.size() == 4) {
			stats.fullChordsFound++;
			std::cout << "!" << std::flush;  // Exclamation for full chord
		} else if (tonesFound.size() > 0) {
			std::cout << tonesFound.size() << std::flush;  // Show number of tones
		} else {
			std::cout << "." << std::flush;  // Dot for no valid tones
		}
	};

	bool started = detector.startAsync(detConfig, callback);

	if (!started) {
		std::cerr << "✗ ERROR: Failed to start frequency detector!\n";
		return 1;
	}

	std::cout << "✓ Detector started successfully\n";
	std::cout << "\nLive Output:\n";
	std::cout << "  . = No valid tones detected\n";
	std::cout << "  1,2,3 = Number of tones detected\n";
	std::cout << "  ! = Full chord (all 4 tones detected)\n\n";

	// Monitor and print stats
	auto lastStatsUpdate = std::chrono::steady_clock::now();
	const double STATS_INTERVAL = 5.0;

	while (running) {
		std::this_thread::sleep_for(std::chrono::milliseconds(500));

		auto now = std::chrono::steady_clock::now();
		auto elapsed = std::chrono::duration<double>(now - lastStatsUpdate).count();

		if (elapsed >= STATS_INTERVAL) {
			std::cout << "\n\n[" << std::fixed << std::setprecision(1)
					  << std::chrono::duration<double>(now - stats.startTime).count()
					  << "s] Quick Stats:\n";

			double rate = stats.fftCallbacks.load() /
						 std::chrono::duration<double>(now - stats.startTime).count();
			std::cout << "  FFT Rate: " << std::fixed << std::setprecision(1) << rate << " Hz | ";
			std::cout << "Tone0:" << stats.tone0Found.load() << " ";
			std::cout << "Tone1:" << stats.tone1Found.load() << " ";
			std::cout << "Tone2:" << stats.tone2Found.load() << " ";
			std::cout << "Tone3:" << stats.tone3Found.load() << " | ";
			std::cout << "Full Chords:" << stats.fullChordsFound.load() << "\n\n";

			lastStatsUpdate = now;
		}
	}

	// Stop detector
	std::cout << "\n\nStopping detector...\n";
	detector.stop();

	// Print final report
	stats.printReport();

	std::cout << "Diagnostic complete!\n";
	return 0;
}
