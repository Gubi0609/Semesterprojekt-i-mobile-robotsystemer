#include "../INCLUDE/Logger.h"
#include <iostream>

void logPCData(Database& db,
               const std::string& timeStamp,
               const std::string& command,
               const std::string& movement,
               const std::string& bits,
               const std::string& commandEncoded,
               const std::string& frequencies,
               const std::string& intConfirmationRec)
{
    if (!db.insertPC(timeStamp, command, movement, bits, commandEncoded,
                      frequencies, intConfirmationRec))
    {
        std::cerr << "[LOGGER] Failed to insert PC data.\n";
    }
}

void logPIData(Database& db,
               const std::string& timeStamp,
               const std::string& frequencies,
               const std::string& commandEncoded,
               const bool& crc,
               const std::string& command,
               const std::string& movement,
               const std::string& intConfirmationSen)
{
    if (!db.insertPI(timeStamp, frequencies, commandEncoded, crc, command,
                      movement, intConfirmationSen))
    {
        std::cerr << "[LOGGER] Failed to insert PI data.\n";
    }
}
