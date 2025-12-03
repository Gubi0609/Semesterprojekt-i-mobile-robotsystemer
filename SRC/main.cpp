#include "../LIB/audio_transmitter.h"
#include "../INCLUDE/CRC.h"
#include "../INCLUDE/Database.h"
#include "../INCLUDE/Logger.h"
#include "../INCLUDE/command_protocol.h"
#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <vector>

using namespace std;

// CRC-encoded chord transmitter with automatic real-time database logging
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
    
    if (!db.createTables()) {
        std::cerr << "Failed to create database tables\n";
        db.close();
        return 1;
    }
    
    // Create and configure transmitter
    AudioComm::ChordTransmitter transmitter;
    AudioComm::ChordTransmitter::Config txConfig;
    txConfig.toneDuration = 2.0; // transmit for 2 seconds
    
    // Extract the 4 nibbles (4 bits each) from the encoded data
    // The ChordTransmitter maps each 4-bit nibble to a frequency
    uint16_t nibble1 = (encodedData >> 12) & 0x0F;  // bits 15-12
    uint16_t nibble2 = (encodedData >> 8) & 0x0F;   // bits 11-8
    uint16_t nibble3 = (encodedData >> 4) & 0x0F;   // bits 7-4
    uint16_t nibble4 = encodedData & 0x0F;          // bits 3-0
    
    // Calculate actual frequencies based on the ChordTransmitter configuration
    // Tone 1: 4500-7000 Hz range, 16 steps (4 bits)
    double tone1MinFreq = 4500.0;
    double tone1MaxFreq = 7000.0;
    double tone1Step = (tone1MaxFreq - tone1MinFreq) / 15.0;
    float tone1 = static_cast<float>(tone1MinFreq + (nibble1 * tone1Step));
    
    // Tone 2: 7500-10000 Hz range, 16 steps
    double tone2MinFreq = 7500.0;
    double tone2MaxFreq = 10000.0;
    double tone2Step = (tone2MaxFreq - tone2MinFreq) / 15.0;
    float tone2 = static_cast<float>(tone2MinFreq + (nibble2 * tone2Step));
    
    // Tone 3: 10500-13000 Hz range, 16 steps
    double tone3MinFreq = 10500.0;
    double tone3MaxFreq = 13000.0;
    double tone3Step = (tone3MaxFreq - tone3MinFreq) / 15.0;
    float tone3 = static_cast<float>(tone3MinFreq + (nibble3 * tone3Step));
    
    // Tone 4: 13500-16000 Hz range, 16 steps
    double tone4MinFreq = 13500.0;
    double tone4MaxFreq = 16000.0;
    double tone4Step = (tone4MaxFreq - tone4MinFreq) / 15.0;
    float tone4 = static_cast<float>(tone4MinFreq + (nibble4 * tone4Step));
    
    std::cout << "Frequencies to transmit:\n";
    std::cout << "  Tone 1: " << tone1 << " Hz (nibble: " << nibble1 << ")\n";
    std::cout << "  Tone 2: " << tone2 << " Hz (nibble: " << nibble2 << ")\n";
    std::cout << "  Tone 3: " << tone3 << " Hz (nibble: " << nibble3 << ")\n";
    std::cout << "  Tone 4: " << tone4 << " Hz (nibble: " << nibble4 << ")\n\n";
    
    // Decode the command protocol to extract command details
    // This is a simple decoder - in practice you might need to track state
    std::string command = "UNKNOWN";
    float speed = 0.0f;
    float turnSpeed = 0.0f;
    float duration = txConfig.toneDuration;
    
    // Check if this is a reset signal
    if (originalData == RESET_SIGNAL) {
        command = "RESET";
    }
    // Check if this is a mode selection command
    else if (originalData <= 0x005) {
        switch (static_cast<RobotMode>(originalData)) {
            case RobotMode::MODE_SELECT:
                command = "MODE_SELECT";
                break;
            case RobotMode::DRIVE_FOR_DURATION:
                command = "DRIVE_FOR_DURATION_MODE";
                break;
            case RobotMode::TURN_FOR_DURATION:
                command = "TURN_FOR_DURATION_MODE";
                break;
            case RobotMode::DRIVE_FORWARD:
                command = "DRIVE_FORWARD_MODE";
                break;
            case RobotMode::TURN:
                command = "TURN_MODE";
                break;
            case RobotMode::STOP:
                command = "STOP";
                break;
        }
    }
    // Otherwise, decode based on expected command format
    // Note: In a real system, you would track the current mode to know how to decode
    // For now, we'll try to decode as different command types and show the possibilities
    else {
        // Try decoding as Drive For Duration
        DriveForDurationCommand driveCmd = DriveForDurationCommand::decode(originalData);
        // Try decoding as Turn For Duration
        TurnForDurationCommand turnCmd = TurnForDurationCommand::decode(originalData);
        // Try decoding as Drive Forward
        DriveForwardCommand driveFwdCmd = DriveForwardCommand::decode(originalData);
        // Try decoding as Turn
        TurnCommand turnOnlyCmd = TurnCommand::decode(originalData);
        
        // For logging, we'll assume it's a drive command if speed is non-zero
        // In practice, you'd track the mode state
        command = "DATA_COMMAND";
        speed = driveCmd.getSpeedPercent();
        duration = driveCmd.getDurationSeconds();
        turnSpeed = turnCmd.getTurnRatePercent();
        
        std::cout << "Command interpretation (mode-dependent):\n";
        std::cout << "  As DRIVE_FOR_DURATION: " << driveCmd.toString() << "\n";
        std::cout << "  As TURN_FOR_DURATION: " << turnCmd.toString() << "\n";
        std::cout << "  As DRIVE_FORWARD: " << driveFwdCmd.toString() << "\n";
        std::cout << "  As TURN: " << turnOnlyCmd.toString() << "\n\n";
    }
    
    // Prepare actual data for logging
    std::string commandBitRaw = std::to_string(originalData);
    std::string commandBitDecoded = std::to_string(originalData);  // Same as raw before encoding
    std::string commandBitEncoded = std::to_string(encodedData);
    
    bool intConfirmationRec = false;  // Will be updated when confirmation is received
    
    // Log the actual transmission data to database
    std::cout << "Logging transmission data to database...\n";
    logPCData(db, command, speed, turnSpeed, duration,
              commandBitRaw, commandBitDecoded, commandBitEncoded,
              tone1, tone2, tone3, tone4, intConfirmationRec);
    
    // Start transmitting the encoded data
    std::cout << "Transmitting CRC-encoded value...\n";
    if (!transmitter.startTransmitting(encodedData, txConfig)) {
        std::cerr << "Error: Failed to start transmitter!\n";
        db.close();
        return 1;
    }
    
    // Wait for transmission to complete
    transmitter.waitForCompletion();
    std::cout << "Transmission complete!\n";
    
    // TODO: If you receive confirmation from Pi, update the database record
    // with intConfirmationRec = true using an UPDATE query
    
    // Close the database
    db.close();
    
    return 0;
}