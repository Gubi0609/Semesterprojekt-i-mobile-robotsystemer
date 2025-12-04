#include "../INCLUDE/Logger.h"
#include <iostream>
#include <chrono>

// Get current timestamp in milliseconds since epoch
int64_t getCurrentTimestampMs() {
    auto now = std::chrono::system_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()).count();
    return ms;
}

// Get current timestamp in ISO 8601 format (for backwards compatibility)
std::string getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_time_t), "%Y-%m-%d %H:%M:%S");
    ss << '.' << std::setfill('0') << std::setw(3) << now_ms.count();
    
    return ss.str();
}

void logSentData(Database& db,
                 const std::string& command,
                 float speed,
                 float turnSpeed,
                 float duration,
                 const std::string& commandBitRaw,
                 const std::string& commandBitDecoded,
                 const std::string& commandBitEncoded,
                 float tone1,
                 float tone2,
                 float tone3,
                 float tone4,
                 int intConfirmationRec)
{
    int64_t startTime = getCurrentTimestampMs();
    int64_t endTime = 0; // Will be updated when confirmation is received
    
    if (!db.insertSent(startTime, endTime, command, speed, turnSpeed, duration,
                       commandBitRaw, commandBitDecoded, commandBitEncoded,
                       tone1, tone2, tone3, tone4, intConfirmationRec))
    {
        std::cerr << "[LOGGER] Failed to insert SentData at timestamp " << startTime << "\n";
    } else {
        std::cout << "[LOGGER] SentData logged - Command: " << command 
                  << ", Timestamp: " << startTime << "\n";
    }
}

void logReceivedData(Database& db,
                     float tone1,
                     float tone2,
                     float tone3,
                     float tone4,
                     const std::string& commandEncoded,
                     bool crc,
                     const std::string& commandDecoded,
                     const std::string& command,
                     float speed,
                     float turnSpeed,
                     float duration,
                     int intConfirmationSen)
{
    int64_t timeStamp = getCurrentTimestampMs();
    
    if (!db.insertReceived(timeStamp, tone1, tone2, tone3, tone4,
                          commandEncoded, crc, commandDecoded, command,
                          speed, turnSpeed, duration, intConfirmationSen))
    {
        std::cerr << "[LOGGER] Failed to insert ReceivedData at timestamp " << timeStamp << "\n";
    } else {
        std::cout << "[LOGGER] ReceivedData logged - Command: " << command 
                  << ", CRC Valid: " << (crc ? "Yes" : "No")
                  << ", Timestamp: " << timeStamp << "\n";
    }
}

// Backwards compatibility wrapper
void logPCData(Database& db,
               const std::string& command,
               float speed,
               float turnSpeed,
               float duration,
               const std::string& commandBitRaw,
               const std::string& commandBitDecoded,
               const std::string& commandBitEncoded,
               float tone1,
               float tone2,
               float tone3,
               float tone4,
               int intConfirmationRec)
{
    logSentData(db, command, speed, turnSpeed, duration,
                commandBitRaw, commandBitDecoded, commandBitEncoded,
                tone1, tone2, tone3, tone4, intConfirmationRec);
}

// Backwards compatibility wrapper
void logPIData(Database& db,
               float tone1,
               float tone2,
               float tone3,
               float tone4,
               const std::string& commandEncoded,
               bool crc,
               const std::string& commandDecoded,
               const std::string& command,
               float speed,
               float turnSpeed,
               float duration,
               int intConfirmationSen)
{
    logReceivedData(db, tone1, tone2, tone3, tone4,
                    commandEncoded, crc, commandDecoded, command,
                    speed, turnSpeed, duration, intConfirmationSen);
}
