#include "../../LIB/audio_receiver.h"
#include "../../LIB/audio_transmitter.h"
#include "../../INCLUDE/CRC.h"
#include "../../INCLUDE/command_protocol.h"
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <atomic>
#include <vector>

// Test to understand minDetections behavior
// Does minDetections=1 mean "first detection" or "wait for 1 confirmation"?

std::atomic<int> callbackCount(0);
std::atomic<double> firstCallbackTime(0.0);

double getCurrentTime() {
	auto now = std::chrono::high_resolution_clock::now();
	auto duration = now.time_since_epoch();
	return std::chrono::duration<double>(duration).count();
}

int main() {
	std::cout << "╔════════════════════════════════════════════════════════════════╗\n";
	std::cout << "║           Detection Count Behavior Test                       ║\n";
	std::cout << "╚════════════════════════════════════════════════════════════════╝\n\n";

	std::cout << "This test will try different minDetections values (0, 1, 2, 3)\n";
	std::cout << "and see when the callback actually fires.\n";
	std::cout << "Test signals will be sent automatically.\n\n";

	// Initialize transmitter and CRC
	AudioComm::ChordTransmitter transmitter;
	CRC crc;

	// Prepare test commands
	std::vector<uint16_t> testCommands = {
		encodeReset(),
		encodeModeSelect(RobotMode::DRIVE_FOR_DURATION),
		encodeDriveForDuration(2.0f, 50.0f)
	};

	std::vector<uint16_t> encodedCommands;
	for (auto cmd : testCommands) {
		std::vector<uint16_t> data = {cmd};
		std::vector<uint16_t> encoded = crc.encode1216(data);
		encodedCommands.push_back(encoded[0]);
	}

	int testValues[] = {0, 1, 2, 3};

	for (int minDet : testValues) {
		std::cout << "\n" << std::string(64, '=') << "\n";
		std::cout << "Testing minDetections = " << minDet << "\n";
		std::cout << std::string(64, '=') << "\n";

		callbackCount = 0;
		firstCallbackTime = 0.0;

		AudioComm::ChordReceiver receiver;
		AudioComm::ChordReceiver::Config config;
		config.minDetections = minDet;
		config.consistencyWindow = 1.0;
		config.updateRate = 50.0;

		std::cout << "Starting receiver...\n";

		double testStartTime = getCurrentTime();

		bool started = receiver.startReceiving(config,
			[&testStartTime, minDet](const AudioComm::ChordReceiver::Detection& det) {
				double now = getCurrentTime();
				double elapsed = now - testStartTime;

				int count = ++callbackCount;

				// Record first callback time
				if (count == 1) {
					firstCallbackTime = elapsed;
				}

				std::cout << "  [" << std::fixed << std::setprecision(3) << elapsed << "s] "
						  << "Callback #" << count
						  << " - Value: 0x" << std::hex << det.value << std::dec
						  << ", DetectionCount: " << det.detectionCount
						  << ", DetectionTime: " << std::fixed << std::setprecision(3)
						  << det.detectionTime << "s\n";
			});

		if (!started) {
			std::cerr << "ERROR: Failed to start receiver with minDetections=" << minDet << "\n";
			if (minDet == 0) {
				std::cout << "→ minDetections=0 is likely INVALID/REJECTED by the receiver\n";
			}
			continue;
		}

		// Send test signals
		std::cout << "Sending test commands...\n";
		for (size_t i = 0; i < encodedCommands.size(); i++) {
			std::cout << "\n  Transmitting command " << (i+1) << "/" << encodedCommands.size()
					  << " (0x" << std::hex << encodedCommands[i] << std::dec << ")...\n";

			AudioComm::ChordTransmitter::Config txConfig;
			txConfig.toneDuration = 2.0;  // 2 second transmission

			double txStart = getCurrentTime();
			if (!transmitter.startTransmitting(encodedCommands[i], txConfig)) {
				std::cerr << "  ERROR: Failed to transmit!\n";
				continue;
			}

			transmitter.waitForCompletion();
			std::cout << "  Transmission complete (took " << std::fixed << std::setprecision(3)
					  << (getCurrentTime() - txStart) << "s)\n";

			// Wait a bit between commands
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
		}

		// Wait for any final detections
		std::cout << "\nWaiting for final detections...\n";
		std::this_thread::sleep_for(std::chrono::seconds(1));

		receiver.stop();

		std::cout << "\nResults:\n";
		std::cout << "  Total callbacks received: " << callbackCount.load() << "\n";

		if (callbackCount == 0) {
			std::cout << "  → No detections";
			if (minDet == 0) {
				std::cout << " - minDetections=0 is INVALID/REJECTED by receiver\n";
			} else {
				std::cout << " - signals may not have been strong enough\n";
			}
		} else {
			std::cout << "  → Receiver DOES work with minDetections=" << minDet << "\n";
			std::cout << "  → First callback at: " << std::fixed << std::setprecision(3)
					  << firstCallbackTime.load() << "s\n";
			double avgTime = firstCallbackTime.load();
			std::cout << "  → Latency to first detection: ~" << avgTime << "s\n";
		}

		std::this_thread::sleep_for(std::chrono::seconds(1));
	}

	std::cout << "\n╔════════════════════════════════════════════════════════════════╗\n";
	std::cout << "║                     TEST COMPLETE                             ║\n";
	std::cout << "╚════════════════════════════════════════════════════════════════╝\n\n";

	std::cout << "Summary:\n";
	std::cout << "  - If minDetections=0 worked, it's treated as \"immediate detection\"\n";
	std::cout << "  - If minDetections=0 failed, the receiver requires at least 1\n";
	std::cout << "  - If minDetections=1 triggered immediately, it means \"first detection\"\n";
	std::cout << "  - If minDetections=1 waited, it means \"wait for 1 confirmation cycle\"\n\n";

	std::cout << "Note: This test requires you to manually play a tone/chord.\n";
	std::cout << "For automated testing, use the protocol_latency_test instead.\n\n";

	return 0;
}
