#include "../LIB/audio_transmitter.h"
#include "../INCLUDE/CRC.h"
#include "../DATABASE/Database.h"
#include "../DATABASE/Logger.h"
#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <vector>

// CRC-encoded chord transmitter
// Takes a 12-bit value from command line and transmits it with CRC encoding
int main(int argc, char* argv[]) {
    
    // Command line argument check
    
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <value>\n";
        std::cerr << "  value: 12-bit unsigned integer (0-4095)\n\n";
        std::cerr << "Example:\n";
        std::cerr << "  " << argv[0] << " 2047\n";
        return 1;
    }

    // Parse input value
    int inputValue = std::atoi(argv[1]);

    // Validate input range (12-bit)
    if (inputValue < 0 || inputValue > 4095) {
        std::cerr << "Error: Value must be between 0 and 4095 (12-bit)\n";
        std::cerr << "  Received: " << inputValue << "\n";
        return 1;
    }

    uint16_t originalData = static_cast<uint16_t>(inputValue);

    std::cout << "=== CRC-Encoded Chord Transmitter ===\n\n";
    std::cout << "Original data (12-bit): " << originalData
              << " (0x" << std::hex << std::uppercase << std::setw(3)
              << std::setfill('0') << originalData << std::dec << ")\n";

    
    // Initialize CRC encoder
    
    CRC crc;

	// Encode the data using CRC (12 bits -> 16 bits)
	vector<uint16_t> dataVector = {static_cast<uint16_t>(originalData & 0x0FFF)};
	vector<uint16_t> encodedVector = crc.encode1216(dataVector);
	uint16_t encodedData = encodedVector[0];

    std::cout << "CRC encoded (16-bit): " << encodedData
              << " (0x" << std::hex << std::uppercase << std::setw(4)
              << std::setfill('0') << encodedData << std::dec << ")\n";

    // Extract CRC bits for display
    uint16_t crcBits = encodedData & 0x0F;
    std::cout << "CRC checksum (4-bit): " << crcBits
              << " (0x" << std::hex << std::uppercase << crcBits << std::dec << ")\n\n";

    
    // Initialize database
    
    Database db("tracking.db");
    if (!db.open()) {
        std::cerr << "Failed to open database\n";
        return 1;
    }
    db.createTables();

    
    // Log the PC transmission
    
    std::string command = "TRANSMIT_CHORD";
    std::string bits_raw = std::to_string(originalData);
    std::string bits_encoded = std::to_string(encodedData);
    double speed = 0.0; // placeholder, no speed in this context
    double duration = 2.0; // matches the transmitter tone duration

    logPCData(db, command, bits_raw, bits_encoded, speed, duration);

    // -----------------------
    // Create and configure transmitter
    // -----------------------
    AudioComm::ChordTransmitter transmitter;
    AudioComm::ChordTransmitter::Config txConfig;
    txConfig.toneDuration = duration; // transmit for 2 seconds

    std::cout << "Transmitting CRC-encoded value...\n";

    // Start transmitting the encoded data
    if (!transmitter.startTransmitting(encodedData, txConfig)) {
        std::cerr << "Error: Failed to start transmitter!\n";
        db.close();
        return 1;
    }

    // Wait for transmission to complete
    transmitter.waitForCompletion();

    std::cout << "Transmission complete!\n";

    // Close the database
    db.close();

    return 0;
}
