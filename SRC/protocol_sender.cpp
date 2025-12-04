#include "../INCLUDE/command_protocol.h"
#include "../LIB/audio_transmitter.h"
#include "../LIB/frequency_detector.h"
#include "../INCLUDE/CRC.h"
#include "../INCLUDE/Database.h"
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

// Structure to collect data for database logging
struct TransmissionData {
	int64_t startTime = 0;
	int64_t endTime = 0;
	std::string command = "";
	float speed = 0.0f;
	float turnSpeed = 0.0f;
	float duration = 0.0f;
	uint16_t commandBitDecoded = 0;
	uint16_t commandBitEncoded = 0;
	float tone1 = 0.0f, tone2 = 0.0f, tone3 = 0.0f, tone4 = 0.0f;
	bool hasConfirmation = false; // Whether any confirmation was received
	int confirmationType = 0; // 1 = positive, 2 = negative
};

// Helper to get current timestamp in milliseconds
int64_t getCurrentTimestampMs() {
	auto now = std::chrono::system_clock::now();
	auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
		now.time_since_epoch()).count();
	return ms;
}

// Helper to calculate tone frequencies from encoded command
void calculateToneFrequencies(uint16_t encodedCommand, float& tone1, float& tone2, float& tone3, float& tone4) {
	// Extract the 4 nibbles (4 bits each) from the encoded data
	uint16_t nibble1 = (encodedCommand >> 12) & 0x0F;  // bits 15-12
	uint16_t nibble2 = (encodedCommand >> 8) & 0x0F;   // bits 11-8  
	uint16_t nibble3 = (encodedCommand >> 4) & 0x0F;   // bits 7-4
	uint16_t nibble4 = encodedCommand & 0x0F;          // bits 3-0

	// Calculate actual frequencies based on the ChordTransmitter configuration
	// Tone 1: 4500-7000 Hz range, 16 steps (4 bits)
	double tone1MinFreq = 4500.0;
	double tone1MaxFreq = 7000.0;
	double tone1Step = (tone1MaxFreq - tone1MinFreq) / 15.0;
	tone1 = static_cast<float>(tone1MinFreq + (nibble1 * tone1Step));

	// Tone 2: 7500-10000 Hz range, 16 steps
	double tone2MinFreq = 7500.0;
	double tone2MaxFreq = 10000.0;
	double tone2Step = (tone2MaxFreq - tone2MinFreq) / 15.0;
	tone2 = static_cast<float>(tone2MinFreq + (nibble2 * tone2Step));

	// Tone 3: 10500-13000 Hz range, 16 steps
	double tone3MinFreq = 10500.0;
	double tone3MaxFreq = 13000.0;
	double tone3Step = (tone3MaxFreq - tone3MinFreq) / 15.0;
	tone3 = static_cast<float>(tone3MinFreq + (nibble3 * tone3Step));

	// Tone 4: 13500-16000 Hz range, 16 steps
	double tone4MinFreq = 13500.0;
	double tone4MaxFreq = 16000.0;
	double tone4Step = (tone4MaxFreq - tone4MinFreq) / 15.0;
	tone4 = static_cast<float>(tone4MinFreq + (nibble4 * tone4Step));
}

// Global flags for feedback detection
std::atomic<bool> feedbackReceived{false};
std::atomic<int> feedbackType{0}; // 1 = positive (3.5kHz), 2 = negative (2.5kHz)

// Function to send a command with feedback detection and retry
void sendCommandWithRetry(AudioComm::ChordTransmitter& transmitter, CRC& crc, uint16_t command,
						  const std::string& description, double duration = 1.0,
						  bool waitForFeedback = true, bool retryOnce = true, 
						  Database* db = nullptr, float speed = 0.0f, float turnSpeed = 0.0f, 
						  float cmdDuration = 0.0f) {
	printCommandInfo(description, command);

	// Initialize transmission data collection
	TransmissionData txData;
	txData.startTime = getCurrentTimestampMs();
	txData.command = description;
	txData.speed = speed;
	txData.turnSpeed = turnSpeed; 
	txData.duration = cmdDuration;
	txData.commandBitDecoded = command;

	// Encode with CRC
	std::vector<uint16_t> data = {command};
	std::vector<uint16_t> encoded = crc.encode1216(data);
	txData.commandBitEncoded = encoded[0];

	// Calculate tone frequencies for logging
	calculateToneFrequencies(encoded[0], txData.tone1, txData.tone2, txData.tone3, txData.tone4);

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
		feedbackType.store(0);

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

				// Look for success tone (3.5 kHz ± 100 Hz) or failure tone (2.5 kHz ± 100 Hz)
				for (const auto& peak : peaks) {
					if (peak.frequency >= 3400.0 && peak.frequency <= 3600.0 && peak.magnitude > 0.01) {
						std::cout << "SUCCESS feedback detected! (" << peak.frequency << " Hz)\n";
						feedbackReceived.store(true);
						feedbackType.store(1); // Positive confirmation
						stopListening.store(true);
						return;
					} else if (peak.frequency >= 2400.0 && peak.frequency <= 2600.0 && peak.magnitude > 0.01) {
						std::cout << "FAILURE feedback detected! (" << peak.frequency << " Hz)\n";
						feedbackReceived.store(true);
						feedbackType.store(2); // Negative confirmation
						stopListening.store(true);
						return;
					}
				}
			};

			std::cout << "Listening for feedback (3.5 kHz = success, 2.5 kHz = failure)...\n";
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
				int confirmType = feedbackType.load();
				if (confirmType == 1) {
					std::cout << "Command confirmed by robot! (POSITIVE)\n";
				} else if (confirmType == 2) {
					std::cout << "Command rejected by robot! (NEGATIVE)\n";
				} else {
					std::cout << "Command confirmed by robot!\n";
				}
				txData.endTime = getCurrentTimestampMs();
				txData.hasConfirmation = true;
				txData.confirmationType = confirmType;
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
			// Recursive retry - call itself again (pass through all parameters)
			sendCommandWithRetry(transmitter, crc, command, description, duration, waitForFeedback, retryOnce, db, speed, turnSpeed, cmdDuration);
			return;
		} else {
			std::cout << "Continuing without confirmation...\n";
		}
	}

	// Log to database if database is provided
	if (db != nullptr) {
		// If no confirmation was received, set endTime to 0
		if (txData.endTime == 0 && (!waitForFeedback || !success)) {
			// For commands without feedback or failed commands, we don't set an endTime
			std::cout << "[DB] Logging transmission without confirmation...\n";
		} else if (txData.endTime > 0) {
			std::cout << "[DB] Logging transmission with confirmation (response time: " 
					  << (txData.endTime - txData.startTime) << " ms)...\n";
		}

		bool logged = db->insertSent(txData.startTime, txData.endTime, txData.command,
									  txData.speed, txData.turnSpeed, txData.duration,
									  txData.commandBitDecoded, txData.commandBitEncoded,
									  txData.tone1, txData.tone2, txData.tone3, txData.tone4,
									  txData.hasConfirmation, txData.confirmationType);
		
		if (!logged) {
			std::cerr << "[DB] Failed to log transmission data\n";
		}
	}

	// No delay needed - listener starts before transmission begins
	// This naturally spaces out commands without explicit delays
}

// Legacy function for compatibility
void sendCommand(AudioComm::ChordTransmitter& transmitter, CRC& crc, uint16_t command,
				 const std::string& description, double duration = 1.0, bool waitAfter = true,
				 Database* db = nullptr) {
	sendCommandWithRetry(transmitter, crc, command, description, duration, false, false, db);
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

	// Initialize database
	Database db("protocol_communication.db");
	if (!db.open()) {
		std::cerr << "Failed to open database. Continuing without logging...\n";
	} else if (!db.createTables()) {
		std::cerr << "Failed to create database tables. Continuing without logging...\n";
	} else {
		std::cout << "Database initialized: protocol_communication.db\n";
	}

	std::cout << "╔════════════════════════════════════════════════════════╗\n";
	std::cout << "║  Command Sender starting...																											            ║\n";
	std::cout << "╚════════════════════════════════════════════════════════╝\n\n";
	std::cout << "Audio transmitter ready.\n";

	runStateMachineUI(transmitter, crc, &db);

	std::cout << "\n Shutting down...\n";
	db.close();
	return 0;
}
