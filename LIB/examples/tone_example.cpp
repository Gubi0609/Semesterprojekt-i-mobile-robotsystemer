#include "tone_generator.h"
#include <iostream>
#include <thread>
#include <chrono>

// Example usage of ToneGenerator library

int main() {
	ToneGenerator generator;

	std::cout << "=== ToneGenerator Library Examples ===\n\n";

	// Example 1: Play a single tone for 2 seconds (blocking)
	std::cout << "1. Playing 1000 Hz for 2 seconds (blocking)...\n";
	ToneGenerator::Config config1;
	config1.frequencies = {1000.0};
	config1.duration = 2.0;
	config1.gain = 0.5;
	generator.start(config1);
	generator.waitForCompletion();
	std::cout << "   Done!\n\n";

	// Example 2: Play multiple tones asynchronously
	std::cout << "2. Playing chord (440, 554, 659 Hz) for 3 seconds (async)...\n";
	ToneGenerator::Config config2;
	config2.frequencies = {440.0, 554.0, 659.0};  // A, C#, E (A major chord)
	config2.duration = 3.0;
	config2.gain = 0.4;

	bool completed = false;
	generator.startAsync(config2, [&]() {
		std::cout << "   Playback completed!\n";
		completed = true;
	});

	// Do other work while playing
	while (!completed) {
		std::cout << "   Position: " << generator.getPlaybackPosition() << "s\r" << std::flush;
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	std::cout << "\n\n";

	// Example 3: Continuous tone (stop manually)
	std::cout << "3. Playing continuous 500 Hz tone...\n";
	std::cout << "   Will stop after 2 seconds...\n";
	ToneGenerator::Config config3;
	config3.frequencies = {500.0};
	config3.duration = 0.0;  // Continuous
	config3.gain = 0.3;
	generator.startAsync(config3);

	std::this_thread::sleep_for(std::chrono::seconds(2));
	generator.stop();
	std::cout << "   Stopped!\n\n";

	// Example 4: Multiple simultaneous frequencies
	std::cout << "4. Playing 5 frequencies for 2 seconds...\n";
	ToneGenerator::Config config4;
	config4.frequencies = {200.0, 400.0, 800.0, 1600.0, 3200.0};
	config4.duration = 2.0;
	config4.gain = 0.3;
	generator.start(config4);
	generator.waitForCompletion();
	std::cout << "   Done!\n\n";

	std::cout << "All examples completed!\n";
	return 0;
}
