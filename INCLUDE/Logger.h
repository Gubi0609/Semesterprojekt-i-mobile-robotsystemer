#pragma once
#include "Database.h"
#include <string>
#include <chrono>
#include <iomanip>
#include <sstream>

// Utility function to get current timestamp
std::string getCurrentTimestamp();

// Log data from PC
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
               bool intConfirmationRec);

// Log data from PI
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
               bool intConfirmationSen);
