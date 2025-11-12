#include "tone_generator.h"
#include "audio_comm.h"
#include <iostream>
#include <vector>
#include <string>
#include <iomanip>
#include <thread>
#include <chrono>

void printUsage(const char* progName) {
	std::cout << "Usage: " << progName << " <value>\n"
			  << "  value: 16-bit value to transmit (0-65535)\n"
			  << "         or multiple values separated by spaces\n"
			  << "\nExamples:\n"
			  << "  " << progName << " 1234      # Send value 1234\n"
			  << "  " << progName << " 0 255 65535   # Send sequence\n"
			  << "\nChord encoding:\n"
			  << "  Each 16-bit value is encoded as 4 simultaneous tones:\n"
			  << "  - Tone 1 (bits 0-3):   5000-8000 Hz\n"
			  << "  - Tone 2 (bits 4-7):   8500-11500 Hz\n"
			  << "  - Tone 3 (bits 8-11):  12000-15000 Hz\n"
			  << "  - Tone 4 (bits 12-15): 15500-18500 Hz\n";
}

void printValueBreakdown(uint16_t value, const AudioComm::ChordEncoder& encoder) {
	std::cout << "\nValue breakdown for " << value << " (0x"
			  << std::hex << std::uppercase << std::setw(4) << std::setfill('0')
			  << value << std::dec << "):\n";

	// Show binary
	std::cout << "Binary: ";
	for (int i = 15; i >= 0; --i) {
		std::cout << ((value >> i) & 1);
		if (i % 4 == 0 && i > 0) std::cout << " ";
	}
	std::cout << "\n\n";

	// Show each tone
	auto frequencies = encoder.encodeValue(value);
	const char* toneNames[] = {"Tone 1 (LSB)", "Tone 2", "Tone 3", "Tone 4 (MSB)"};

	for (int i = 0; i < 4; ++i) {
		uint8_t toneValue = encoder.getToneValue(value, i);
		std::cout << toneNames[i] << ": bits " << std::setw(2) << (i*4) << "-" << (i*4+3)
				  << " = " << std::setw(2) << static_cast<int>(toneValue)
				  << " (0x" << std::hex << std::uppercase << static_cast<int>(toneValue) << std::dec << ")"
				  << " => " << std::fixed << std::setprecision(1) << frequencies[i] << " Hz\n";
	}
	std::cout << "\n";
}

int main(int argc, char* argv[]) {
	if (argc < 2) {
		printUsage(argv[0]);
		return 1;
	}

	// Parse values from command line
	std::vector<uint16_t> values;
	for (int i = 1; i < argc; ++i) {
		long val = std::atol(argv[i]);
		if (val < 0 || val > 65535) {
			std::cerr << "Error: Value '" << argv[i] << "' out of range (must be 0-65535)\n";
			return 1;
		}
		values.push_back(static_cast<uint16_t>(val));
	}

	// Setup encoder
	AudioComm::ChordConfig config;
	AudioComm::ChordEncoder encoder(config);

	std::cout << "\n=== Audio Communication Chord Transmitter ===\n";
	std::cout << "Encoding: 4 tones x 4 bits = 16 bits per chord\n";
	std::cout << "Tone 1: " << config.tone1MinFreq << "-" << config.tone1MaxFreq << " Hz\n";
	std::cout << "Tone 2: " << config.tone2MinFreq << "-" << config.tone2MaxFreq << " Hz\n";
	std::cout << "Tone 3: " << config.tone3MinFreq << "-" << config.tone3MaxFreq << " Hz\n";
	std::cout << "Tone 4: " << config.tone4MinFreq << "-" << config.tone4MaxFreq << " Hz\n";

	ToneGenerator generator;

	// Transmit each value
	for (size_t i = 0; i < values.size(); ++i) {
		uint16_t value = values[i];
		auto frequencies = encoder.encodeValue(value);

		printValueBreakdown(value, encoder);

		std::cout << "Transmitting chord (Press Ctrl+C to stop)...\n";
		std::cout << "Frequencies: ";
		for (size_t j = 0; j < frequencies.size(); ++j) {
			std::cout << std::fixed << std::setprecision(1) << frequencies[j] << " Hz";
			if (j < frequencies.size() - 1) std::cout << ", ";
		}
		std::cout << "\n\n";

		// Configure tone generator for chord (multiple frequencies)
		ToneGenerator::Config toneConfig;
		toneConfig.frequencies = frequencies;
		toneConfig.duration = config.toneDuration; // 0 = infinite
		toneConfig.gain = 0.4; // Lower gain when playing multiple tones
		toneConfig.fadeTime = config.fadeTime;
		toneConfig.sampleRate = config.sampleRate;

		// Play the chord (blocking until Ctrl+C)
		if (!generator.start(toneConfig)) {
			std::cerr << "\nError: Failed to start tone generator\n";
			return 1;
		}

		generator.waitForCompletion();
		std::cout << "\nStopped.\n";

		// If multiple values, wait before next
		if (i < values.size() - 1) {
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
		}
	}

	std::cout << "\nTransmission complete!\n";
	return 0;
}
