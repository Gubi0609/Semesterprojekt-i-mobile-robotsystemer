#include "../INCLUDE/command_protocol.h"
#include "../LIB/audio_transmitter.h"
#include "../LIB/frequency_detector.h"
#include "../INCLUDE/CRC.h"
#include "sender_ui.h"
#include <iostream>
#include <iomanip>
#include <string>
#include <thread>
#include <chrono>
#include <atomic>

// Helper to print command information
void printCommandInfo(const std::string& description, uint16_t bits) {
	std::cout << "\n┌────────────────────────────────────────────────────\n";
	std::cout << "│ " << description << "\n";
	std::cout << "│ Raw value: " << bits << " (0x" << std::hex << std::setfill('0')
			  << std::setw(3) << bits << std::dec << ")\n";
	std::cout << "└────────────────────────────────────────────────────\n";
}

// Global flag for feedback detection
std::atomic<bool> feedbackReceived{false};

// Function to send a command with feedback detection and retry
void sendCommandWithRetry(AudioComm::ChordTransmitter& transmitter, CRC& crc, uint16_t command,
						  const std::string& description, double duration = 1.0,
						  bool waitForFeedback = true, bool retryOnce = true) {
	printCommandInfo(description, command);

	// Encode with CRC
	std::vector<uint16_t> data = {command};
	std::vector<uint16_t> encoded = crc.encode1216(data);

	std::cout << "Encoded (with CRC): " << encoded[0] << " (0x" << std::hex
			  << std::setfill('0') << std::setw(4) << encoded[0] << std::dec << ")\n";

	int attempts = 0;
	const int maxAttempts = retryOnce ? 2 : 1;
	bool success = false;

	while (attempts < maxAttempts && !success) {
		attempts++;
		if (attempts > 1) {
			std::cout << "\n⟳ Retry attempt " << attempts << "/" << maxAttempts << "...\n";
		}

		feedbackReceived.store(false);

		// Start feedback listener FIRST (no delay - catch feedback immediately)
		FrequencyDetector detector;
		std::atomic<bool> stopListening{false};

		if (waitForFeedback) {
			FrequencyDetector::Config detConfig;
			detConfig.sampleRate = 48000;
			detConfig.fftSize = 4096;
			detConfig.numPeaks = 5;
			detConfig.duration = 0.0;  // Continuous
			detConfig.updateRate = 20.0;

			auto callback = [&](const std::vector<FrequencyDetector::FrequencyPeak>& peaks) {
				if (stopListening.load()) return;

				// Look for success tone (3.5 kHz ± 100 Hz)
				for (const auto& peak : peaks) {
					if (peak.frequency >= 3400.0 && peak.frequency <= 3600.0 && peak.magnitude > 0.01) {
						std::cout << "SUCCESS feedback detected! (" << peak.frequency << " Hz)\n";
						feedbackReceived.store(true);
						stopListening.store(true);
						return;
					}
				}
			};

			std::cout << "Listening for feedback (3.5 kHz)...\n";
			detector.startAsync(detConfig, callback);
		}

		// Send via audio AFTER starting listener
		std::cout << "Sending via audio for " << duration << " seconds...\n";
		AudioComm::ChordTransmitter::Config config;
		config.toneDuration = duration;
		if (!transmitter.startTransmitting(encoded[0], config)) {
			std::cerr << "Failed to start transmission!\n";
			if (waitForFeedback) detector.stop();
			return;
		}

		transmitter.waitForCompletion();

		if (waitForFeedback) {
			// Wait additional 0.4s for feedback (0.15s tone + 0.25s grace period)
			std::cout << "Waiting for feedback...\n";
			auto startWait = std::chrono::steady_clock::now();
			while (std::chrono::duration<double>(std::chrono::steady_clock::now() - startWait).count() < 0.4) {
				if (feedbackReceived.load()) {
					success = true;
					break;
				}
				std::this_thread::sleep_for(std::chrono::milliseconds(50));
			}

			stopListening.store(true);
			detector.stop();

			if (!success && attempts < maxAttempts) {
				std::cout << "No feedback received\n";
			} else if (success) {
				std::cout << "Command confirmed by robot!\n";
			}
		} else {
			success = true;  // Don't wait for feedback
		}

		if (!success && attempts < maxAttempts) {
			std::this_thread::sleep_for(std::chrono::milliseconds(200));
		}
	}

	if (!success && waitForFeedback) {
		std::cout << "\n╔════════════════════════════════════════════════════════╗\n";
		std::cout << "║  TRANSMISSION CONFIRMATION FAILED                  																			  ║\n";
		std::cout << "╚════════════════════════════════════════════════════════╝\n";
		std::cout << "No feedback received after " << maxAttempts << " attempts.\n\n";
		std::cout << "Options:\n";
		std::cout << "  R. Retry transmission\n";
		std::cout << "  C. Continue anyway (command may have been received)\n";
		std::cout << "\nChoice: ";

		std::string choice;
		std::cin >> choice;
		std::cin.ignore(10000, '\n');

		if (choice == "R" || choice == "r") {
			std::cout << "\n Retrying transmission...\n";
			// Recursive retry - call itself again
			sendCommandWithRetry(transmitter, crc, command, description, duration, waitForFeedback, retryOnce);
			return;
		} else {
			std::cout << "Continuing without confirmation...\n";
		}
	}

	// No delay needed - listener starts before transmission begins
	// This naturally spaces out commands without explicit delays
}

// Legacy function for compatibility
void sendCommand(AudioComm::ChordTransmitter& transmitter, CRC& crc, uint16_t command,
				 const std::string& description, double duration = 1.0, bool waitAfter = true) {
	sendCommandWithRetry(transmitter, crc, command, description, duration, false, false);
	if (waitAfter) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}
}

// ============================================================================
// Main function
// ============================================================================

int main() {
	AudioComm::ChordTransmitter transmitter;
	CRC crc;

	std::cout << "╔════════════════════════════════════════════════════════╗\n";
	std::cout << "║  Command Sender starting...																											            ║\n";
	std::cout << "╚════════════════════════════════════════════════════════╝\n\n";
	std::cout << "Audio transmitter ready.\n";

	runStateMachineUI(transmitter, crc);

	std::cout << "\n Shutting down...\n";
	return 0;
}
