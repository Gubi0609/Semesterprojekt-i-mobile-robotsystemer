#include "tone_generator.h"
#include "audio_comm.h"
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <thread>
#include <chrono>

void printUsage(const char* progName) {
	std::cout << "Usage: " << progName << " <value>\n"
			  << "  value: 4-bit value to transmit (0-15)\n"
			  << "         or multiple values separated by spaces\n"
			  << "\nExamples:\n"
			  << "  " << progName << " 5         # Send value 5\n"
			  << "  " << progName << " 1 2 3 4   # Send sequence 1,2,3,4\n"
			  << "\nFrequency mapping (5000-8000 Hz):\n"
			  << "  Value 0  = 5000 Hz\n"
			  << "  Value 1  = 5200 Hz\n"
			  << "  Value 2  = 5400 Hz\n"
			  << "  ...\n"
			  << "  Value 15 = 8000 Hz\n";
}

int main(int argc, char* argv[]) {
	if (argc < 2) {
		printUsage(argv[0]);
		return 1;
	}

	// Parse values from command line
	std::vector<uint8_t> values;
	for (int i = 1; i < argc; ++i) {
		int val = std::atoi(argv[i]);
		if (val < 0 || val > 15) {
			std::cerr << "Error: Value '" << argv[i] << "' out of range (must be 0-15)\n";
			return 1;
		}
		values.push_back(static_cast<uint8_t>(val));
	}

	// Setup encoder
	AudioComm::SingleToneConfig config;
	AudioComm::SingleToneEncoder encoder(config);

	std::cout << "\n=== Audio Communication Transmitter ===\n";
	std::cout << "Frequency range: " << config.minFreq << " - " << config.maxFreq << " Hz\n";
	std::cout << "Frequency step: " << encoder.getFrequencyStep() << " Hz\n";
	std::cout << "Tone duration: continuous (Ctrl+C to stop)\n";
	std::cout << "\nTransmitting " << values.size() << " value(s)...\n\n";

	ToneGenerator generator;

	// Transmit each value
	for (size_t i = 0; i < values.size(); ++i) {
		uint8_t value = values[i];
		double frequency = encoder.encodeValue(value);

		std::cout << "Transmitting value " << static_cast<int>(value)
				  << " at " << frequency << " Hz (Press Ctrl+C to stop)...\n" << std::flush;

		// Configure tone generator
		ToneGenerator::Config toneConfig;
		toneConfig.frequencies = {frequency};
		toneConfig.duration = config.toneDuration; // 0 = infinite
		toneConfig.gain = 0.5; // Moderate volume
		toneConfig.fadeTime = config.fadeTime;
		toneConfig.sampleRate = config.sampleRate;

		// Play the tone (blocking until Ctrl+C)
		if (!generator.start(toneConfig)) {
			std::cerr << "\nError: Failed to start tone generator\n";
			return 1;
		}

		generator.waitForCompletion(); // Will run forever until Ctrl+C
		std::cout << "\nStopped.\n";

		// If multiple values, wait before next (though user would have pressed Ctrl+C)
		if (i < values.size() - 1) {
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
		}
	}

	std::cout << "\nTransmission complete!\n";
	return 0;
}
