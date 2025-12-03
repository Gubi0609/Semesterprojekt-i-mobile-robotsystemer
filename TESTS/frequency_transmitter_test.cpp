#include "../LIB/audio_transmitter.h"
#include "../LIB/audio_comm.h"
#include <iostream>
#include <iomanip>
#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>

// Frequency transmitter test tool
// Systematically transmits all 64 frequencies to test microphone detection

std::atomic<bool> running(true);

void signalHandler(int signum) {
	std::cout << "\n\nReceived interrupt signal (" << signum << ")\n";
	std::cout << "Shutting down transmitter...\n";
	running = false;
}

void printFrequencyTable(const AudioComm::ChordEncoder& encoder) {
	std::cout << "\n┌──────┬───────┬─────────────┐\n";
	std::cout << "│ Tone │ Value │  Frequency  │\n";
	std::cout << "│      │       │     (Hz)    │\n";
	std::cout << "├──────┼───────┼─────────────┤\n";

	for (int tone = 0; tone < 4; ++tone) {
		for (int value = 0; value < 16; ++value) {
			double freq = encoder.getFrequency(tone, value);
			std::cout << "│  " << tone << "   │  "
					  << std::setw(2) << std::setfill(' ') << value << "   │  "
					  << std::setw(8) << std::fixed << std::setprecision(1) << freq
					  << "   │\n";
		}
		if (tone < 3) {
			std::cout << "├──────┼───────┼─────────────┤\n";
		}
	}
	std::cout << "└──────┴───────┴─────────────┘\n\n";
}

int main(int argc, char* argv[]) {
	std::cout << "╔════════════════════════════════════════════════════════════════════╗\n";
	std::cout << "║         FREQUENCY TRANSMITTER TEST TOOL                            ║\n";
	std::cout << "║   Transmits all 64 frequencies for microphone testing             ║\n";
	std::cout << "╚════════════════════════════════════════════════════════════════════╝\n\n";

	// Register signal handler
	signal(SIGINT, signalHandler);
	signal(SIGTERM, signalHandler);

	// Parse command line arguments
	double durationPerFreq = 2.0;  // Default: 2 seconds per frequency
	int cycles = 1;                 // Default: 1 full cycle through all frequencies

	if (argc > 1) {
		durationPerFreq = std::stod(argv[1]);
	}
	if (argc > 2) {
		cycles = std::stoi(argv[2]);
	}

	std::cout << "Configuration:\n";
	std::cout << "  Duration per frequency: " << durationPerFreq << " seconds\n";
	std::cout << "  Number of cycles: " << (cycles == 0 ? "infinite" : std::to_string(cycles)) << "\n";
	std::cout << "  Total frequencies: 64 (4 tones × 16 values)\n";

	if (cycles > 0) {
		double totalTime = 64 * durationPerFreq * cycles;
		std::cout << "  Estimated total time: " << std::fixed << std::setprecision(1)
				  << totalTime << " seconds (" << (totalTime / 60.0) << " minutes)\n";
	}
	std::cout << "\n";

	// Create encoder
	AudioComm::ChordConfig config;
	AudioComm::ChordEncoder encoder(config);

	// Display frequency table
	std::cout << "Frequency Table:\n";
	printFrequencyTable(encoder);

	// Create transmitter
	AudioComm::ChordTransmitter transmitter;
	AudioComm::ChordTransmitter::Config txConfig;
	txConfig.toneDuration = durationPerFreq;
	txConfig.fadeTime = 0.05;  // Short fade for cleaner transitions

	std::cout << "═══════════════════════════════════════════════════════════════════════\n";
	std::cout << "                    TRANSMISSION STARTING\n";
	std::cout << "═══════════════════════════════════════════════════════════════════════\n\n";

	std::cout << "Instructions:\n";
	std::cout << "  1. Start the frequency_test receiver in another terminal\n";
	std::cout << "  2. This transmitter will cycle through all frequencies\n";
	std::cout << "  3. Press Ctrl+C to stop early\n";
	std::cout << "  4. Check receiver output for detection statistics\n\n";

	std::cout << "Starting transmission in 3 seconds...\n";
	std::this_thread::sleep_for(std::chrono::seconds(3));

	auto startTime = std::chrono::steady_clock::now();
	int currentCycle = 0;
	int totalTransmissions = 0;

	// Main transmission loop
	while (running && (cycles == 0 || currentCycle < cycles)) {
		currentCycle++;

		std::cout << "\n┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓\n";
		std::cout << "┃  CYCLE " << std::setw(2) << currentCycle;
		if (cycles > 0) {
			std::cout << " / " << cycles;
		}
		std::cout << std::string(56, ' ') << "┃\n";
		std::cout << "┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛\n\n";

		// Transmit each tone separately (single frequency at a time)
		for (int tone = 0; tone < 4 && running; ++tone) {
			std::cout << "\n── Tone " << tone << " ──────────────────────────────────────────────────────\n\n";

			for (int value = 0; value < 16 && running; ++value) {
				double freq = encoder.getFrequency(tone, value);

				auto now = std::chrono::steady_clock::now();
				double elapsed = std::chrono::duration<double>(now - startTime).count();

				std::cout << "[" << std::fixed << std::setprecision(1) << elapsed << "s] "
						  << "Transmitting: Tone " << tone
						  << ", Value " << std::setw(2) << value
						  << " → " << std::setw(8) << std::setprecision(1) << freq << " Hz";
				std::cout << std::flush;

				// Create a chord with just this one frequency
				// (We'll encode it as a 16-bit value where only this tone's bits are set)
				uint16_t testValue = value << (tone * 4);

				// For single frequency testing, we'll use single tone transmitter instead
				AudioComm::SingleToneTransmitter singleTx;
				AudioComm::SingleToneTransmitter::Config singleConfig;
				singleConfig.minFreq = freq - 100;  // Narrow range around target
				singleConfig.maxFreq = freq + 100;
				singleConfig.toneDuration = durationPerFreq;
				singleConfig.fadeTime = 0.05;

				// Transmit single tone
				bool success = singleTx.startTransmitting(value, singleConfig);

				if (success) {
					totalTransmissions++;
					std::cout << " ✓\n";

					// Wait for transmission to complete
					singleTx.waitForCompletion();
				} else {
					std::cout << " ✗ FAILED\n";
				}

				// Small pause between frequencies
				if (running) {
					std::this_thread::sleep_for(std::chrono::milliseconds(100));
				}
			}
		}

		if (cycles > 0 && currentCycle < cycles) {
			std::cout << "\n\nCompleted cycle " << currentCycle << " / " << cycles << "\n";
			std::cout << "Pausing 2 seconds before next cycle...\n";
			std::this_thread::sleep_for(std::chrono::seconds(2));
		}
	}

	auto endTime = std::chrono::steady_clock::now();
	double totalTime = std::chrono::duration<double>(endTime - startTime).count();

	std::cout << "\n\n═══════════════════════════════════════════════════════════════════════\n";
	std::cout << "                    TRANSMISSION COMPLETE\n";
	std::cout << "═══════════════════════════════════════════════════════════════════════\n\n";

	std::cout << "Statistics:\n";
	std::cout << "  Cycles completed: " << currentCycle << "\n";
	std::cout << "  Total frequencies transmitted: " << totalTransmissions << "\n";
	std::cout << "  Total time: " << std::fixed << std::setprecision(1) << totalTime
			  << " seconds (" << (totalTime / 60.0) << " minutes)\n";
	std::cout << "  Average time per frequency: "
			  << (totalTransmissions > 0 ? totalTime / totalTransmissions : 0) << " seconds\n\n";

	std::cout << "Check the frequency_test receiver output for detection results!\n\n";

	return 0;
}
