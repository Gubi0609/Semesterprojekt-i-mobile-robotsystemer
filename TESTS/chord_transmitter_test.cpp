#include "../LIB/audio_transmitter.h"
#include "../LIB/audio_comm.h"
#include <iostream>
#include <iomanip>
#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>

// Chord transmitter test - sends actual 4-tone chords for testing chord receiver

std::atomic<bool> running(true);

void signalHandler(int signum) {
	std::cout << "\n\nReceived interrupt signal (" << signum << ")\n";
	std::cout << "Shutting down transmitter...\n";
	running = false;
}

int main(int argc, char* argv[]) {
	std::cout << "╔════════════════════════════════════════════════════════════════════╗\n";
	std::cout << "║         CHORD TRANSMITTER TEST TOOL                                ║\n";
	std::cout << "║   Transmits 4-tone chords for chord receiver testing              ║\n";
	std::cout << "╚════════════════════════════════════════════════════════════════════╝\n\n";

	signal(SIGINT, signalHandler);
	signal(SIGTERM, signalHandler);

	// Parse command line
	double durationPerChord = 2.0;
	int numChords = 10;

	if (argc > 1) {
		durationPerChord = std::stod(argv[1]);
	}
	if (argc > 2) {
		numChords = std::stoi(argv[2]);
	}

	std::cout << "Configuration:\n";
	std::cout << "  Duration per chord: " << durationPerChord << " seconds\n";
	std::cout << "  Number of chords: " << (numChords == 0 ? "infinite" : std::to_string(numChords)) << "\n\n";

	// Create transmitter and encoder
	AudioComm::ChordTransmitter transmitter;
	AudioComm::ChordTransmitter::Config txConfig;
	txConfig.toneDuration = durationPerChord;
	txConfig.fadeTime = 0.05;

	AudioComm::ChordEncoder encoder;

	// Test values to transmit - generate test values covering different bit patterns
	std::vector<uint16_t> testValues = {
		0x0000,  // All zeros
		0x1111,  // Low nibbles
		0x2222,
		0x4444,  // Mid range
		0x8888,  // High nibbles
		0xAAAA,  // Alternating
		0x5555,  // Alternating opposite
		0xF0F0,  // Byte pattern
		0x0F0F,  // Byte pattern opposite
		0xFFFF,  // All ones
	};

	std::cout << "Generating raw 16-bit test values...\n";
	for (const auto& val : testValues) {
		std::cout << "  0x" << std::hex << std::uppercase << std::setw(4)
				  << std::setfill('0') << val << std::dec << "\n";
	}

	std::cout << "\n═══════════════════════════════════════════════════════════════════════\n";
	std::cout << "                    TRANSMISSION STARTING\n";
	std::cout << "═══════════════════════════════════════════════════════════════════════\n\n";

	std::cout << "Instructions:\n";
	std::cout << "  1. Start chord_diagnostic_test or frequency_test in another terminal\n";
	std::cout << "  2. This will transmit chords (all 4 tones simultaneously)\n";
	std::cout << "  3. Press Ctrl+C to stop early\n\n";

	std::cout << "Starting transmission in 3 seconds...\n";
	std::this_thread::sleep_for(std::chrono::seconds(3));

	auto startTime = std::chrono::steady_clock::now();
	int transmitted = 0;

	// Transmit loop
	int cycle = 0;
	while (running && (numChords == 0 || transmitted < numChords)) {
		// Select value to transmit
		uint16_t value = testValues[transmitted % testValues.size()];

		auto now = std::chrono::steady_clock::now();
		double elapsed = std::chrono::duration<double>(now - startTime).count();

		// Get the 4 frequencies for this chord
		std::vector<double> frequencies = encoder.encodeValue(value);

		std::cout << "\n[" << std::fixed << std::setprecision(1) << elapsed << "s] ";
		std::cout << "Transmitting chord #" << (transmitted + 1);
		if (numChords > 0) {
			std::cout << " / " << numChords;
		}
		std::cout << "\n";
		std::cout << "  Value: 0x" << std::hex << std::uppercase << std::setw(4)
				  << std::setfill('0') << value << std::dec << "\n";
		std::cout << "  Frequencies:\n";
		for (int i = 0; i < 4 && i < frequencies.size(); ++i) {
			std::cout << "    Tone " << i << ": " << std::fixed << std::setprecision(1)
					  << frequencies[i] << " Hz";

			// Show which nibble
			uint8_t nibble = encoder.getToneValue(value, i);
			std::cout << " (value: " << (int)nibble << ")\n";
		}

		// Transmit the chord
		bool success = transmitter.startTransmitting(value, txConfig);

		if (success) {
			std::cout << "  Status: ✓ Transmitting...\n";
			transmitter.waitForCompletion();
			transmitted++;
		} else {
			std::cout << "  Status: ✗ FAILED\n";
		}

		// Small pause between chords
		if (running && (numChords == 0 || transmitted < numChords)) {
			std::this_thread::sleep_for(std::chrono::milliseconds(200));
		}
	}

	auto endTime = std::chrono::steady_clock::now();
	double totalTime = std::chrono::duration<double>(endTime - startTime).count();

	std::cout << "\n\n═══════════════════════════════════════════════════════════════════════\n";
	std::cout << "                    TRANSMISSION COMPLETE\n";
	std::cout << "═══════════════════════════════════════════════════════════════════════\n\n";

	std::cout << "Statistics:\n";
	std::cout << "  Chords transmitted: " << transmitted << "\n";
	std::cout << "  Total time: " << std::fixed << std::setprecision(1) << totalTime
			  << " seconds\n";
	std::cout << "  Average time per chord: "
			  << (transmitted > 0 ? totalTime / transmitted : 0) << " seconds\n\n";

	std::cout << "Check the receiver output for detection results!\n\n";

	return 0;
}
