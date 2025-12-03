#pragma once
#include "Database.h"
#include <string>

// Log data from PC
void logPCData(Database& db,
               const std::string& timeStamp,
               const std::string& command,
               const std::string& movement,           // structured string: mode, speed, turn, duration
               const std::string& bits,
               const std::string& commandEncoded,
               const std::string& frequencies,       // structured string for 4 tones
               const std::string& intConfirmationRec);

// Log data from PI
void logPIData(Database& db,
               const std::string& timeStamp,
               const std::string& frequencies,
               const std::string& commandEncoded,
               const bool& crc,
               const std::string& command,
               const std::string& movement,           // structured string
               const std::string& intConfirmationSen);
