#include "../INCLUDE/Logger.h"
#include <iostream>

// Get current timestamp in ISO 8601 format
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
               bool intConfirmationRec)
{
    std::string timeStamp = getCurrentTimestamp();
    
    if (!db.insertPC(timeStamp, command, speed, turnSpeed, duration,
                     commandBitRaw, commandBitDecoded, commandBitEncoded,
                     tone1, tone2, tone3, tone4, intConfirmationRec))
    {
        std::cerr << "[LOGGER] Failed to insert PC data at " << timeStamp << "\n";
    } else {
        std::cout << "[LOGGER] PC data logged at " << timeStamp << "\n";
    }
}

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
               bool intConfirmationSen)
{
    std::string timeStamp = getCurrentTimestamp();
    
    if (!db.insertPI(timeStamp, tone1, tone2, tone3, tone4,
                     commandEncoded, crc, commandDecoded, command,
                     speed, turnSpeed, duration, intConfirmationSen))
    {
        std::cerr << "[LOGGER] Failed to insert PI data at " << timeStamp << "\n";
    } else {
        std::cout << "[LOGGER] PI data logged at " << timeStamp << "\n";
    }
}
