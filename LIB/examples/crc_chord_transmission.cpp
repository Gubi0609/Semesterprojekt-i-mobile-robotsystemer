#include "audio_transmitter.h"
#include "audio_receiver.h"
#include "../../INCLUDE/CRC.h"
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>

// Example showing CRC encoding, chord transmission, reception, and error detection

int main() {
	std::cout << "=== CRC-Encoded Chord Communication Example ===\n\n";

	// Initialize CRC with generator key (default: {1, 0, 0, 1, 1})
	CRC crc;

	// Data to transmit (12-bit value, max 4095)
	uint16_t originalData = 2047; // Example: 0x7FF (all 12 bits used)
	std::cout << "Original data (12-bit): " << originalData
	          << " (0x" << std::hex << std::uppercase << std::setw(3)
	          << std::setfill('0') << originalData << std::dec << ")\n";

	// Encode the data using CRC (12 bits -> 16 bits)
	vector<uint16_t> dataVector = {static_cast<uint16_t>(originalData & 0x0FFF)}; // Ensure only 12 bits
	vector<uint16_t> encodedVector = crc.encode1216(dataVector);
	uint16_t encodedData = encodedVector[0];

	std::cout << "CRC encoded (16-bit): " << encodedData
	          << " (0x" << std::hex << std::uppercase << std::setw(4)
	          << std::setfill('0') << encodedData << std::dec << ")\n";

	// Extract CRC bits for display
	uint16_t crcBits = encodedData & 0x0F;
	std::cout << "CRC checksum (4-bit): " << crcBits
	          << " (0x" << std::hex << std::uppercase << crcBits << std::dec << ")\n\n";

	// Create receiver
	AudioComm::ChordReceiver receiver;
	AudioComm::ChordReceiver::Config recvConfig;

	std::cout << "Starting receiver...\n";

	// Start receiving with callback that performs CRC verification
	bool receiverStarted = receiver.startReceiving(recvConfig,
		[&crc, originalData](const AudioComm::ChordReceiver::Detection& det) {
			std::cout << "\n>>> CHORD RECEIVED <<<\n";
			std::cout << "Received value: " << det.value
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

			// Verify CRC
			std::cout << "=== CRC VERIFICATION ===\n";
			bool isValid = crc.verify(det.value);

			if (isValid) {
				std::cout << "✓ CRC CHECK PASSED - No errors detected\n";

				// Decode to get original 12-bit data
				optional<uint16_t> decodedData = crc.decode1612(det.value);
				if (decodedData.has_value()) {
					std::cout << "Decoded data (12-bit): " << decodedData.value()
					          << " (0x" << std::hex << std::uppercase << std::setw(3)
					          << std::setfill('0') << decodedData.value() << std::dec << ")\n";

					// Verify data integrity
					if (decodedData.value() == originalData) {
						std::cout << "✓ DATA INTEGRITY CONFIRMED - Matches original data!\n";
					} else {
						std::cout << "✗ DATA MISMATCH - Decoded data differs from original!\n";
					}
				}
			} else {
				std::cout << "✗ CRC CHECK FAILED - Error detected in transmission!\n";
				std::cout << "  The received data is corrupted.\n";
			}
			std::cout << "\n";
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

	std::cout << "Transmitting CRC-encoded value " << encodedData << "...\n";

	// Start transmitting the encoded data
	if (!transmitter.startTransmitting(encodedData, txConfig)) {
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

	std::cout << "\n=== Example complete! ===\n";
	std::cout << "\nSummary:\n";
	std::cout << "- Original 12-bit data was CRC-encoded to 16 bits\n";
	std::cout << "- The 16-bit encoded data was transmitted as a chord\n";
	std::cout << "- The receiver decoded the chord and verified the CRC\n";
	std::cout << "- Error detection confirmed data integrity\n";

	return 0;
}
