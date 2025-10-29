#include "audio_transmitter.h"
#include "audio_receiver.h"
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>

// Simple example showing how to use the audio communication library

int main() {
	std::cout << "=== Simple Chord Communication Example ===\n\n";

	// Create receiver
	AudioComm::ChordReceiver receiver;
	AudioComm::ChordReceiver::Config recvConfig;

	std::cout << "Starting receiver...\n";

	// Start receiving with callback
	bool receiverStarted = receiver.startReceiving(recvConfig,
		[](const AudioComm::ChordReceiver::Detection& det) {
			std::cout << "\n>>> CHORD RECEIVED <<<\n";
			std::cout << "Value: " << det.value
					  << " (0x" << std::hex << std::uppercase << std::setw(4)
					  << std::setfill('0') << det.value << std::dec << ")\n";
			std::cout << "Detected " << det.detectionCount << " times over "
					  << std::fixed << std::setprecision(2) << det.detectionTime << "s\n";
			std::cout << "Frequencies: ";
			for (size_t i = 0; i < det.frequencies.size(); ++i) {
				std::cout << std::fixed << std::setprecision(1) << det.frequencies[i] << " Hz";
				if (i < det.frequencies.size() - 1) std::cout << ", ";
			}
			std::cout << "\n\n";
		});

	if (!receiverStarted) {
		std::cerr << "Failed to start receiver!\n";
		return 1;
	}

	std::cout << "Receiver started. Waiting 2 seconds before transmitting...\n\n";
	std::this_thread::sleep_for(std::chrono::seconds(2));

	// Create transmitter
	AudioComm::ChordTransmitter transmitter;
	AudioComm::ChordTransmitter::Config txConfig;
	txConfig.toneDuration = 2.0; // Transmit for 2 seconds

	uint16_t valueToSend = 1234;
	std::cout << "Transmitting value " << valueToSend << "...\n";

	// Start transmitting
	if (!transmitter.startTransmitting(valueToSend, txConfig)) {
		std::cerr << "Failed to start transmitter!\n";
		receiver.stop();
		return 1;
	}

	// Wait for transmission to complete
	transmitter.waitForCompletion();

	std::cout << "Transmission complete. Waiting for final detections...\n";
	std::this_thread::sleep_for(std::chrono::seconds(1));

	// Stop receiver
	receiver.stop();

	std::cout << "\nExample complete!\n";
	return 0;
}
