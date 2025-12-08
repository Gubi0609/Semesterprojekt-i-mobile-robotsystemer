#pragma once
#include "Database.h"
#include <string>
#include <chrono>
#include <iomanip>
#include <sstream>

// Utility functions for timestamps
std::string getCurrentTimestamp();
int64_t getCurrentTimestampMs();

// Log data to SentData table
void logSentData(Database& db,
                 const std::string& command,
                 float speed,
                 float turnSpeed,
                 float duration,
                 uint16_t commandBitDecoded,
                 uint16_t commandBitEncoded,
                 float tone1,
                 float tone2,
                 float tone3,
                 float tone4,
                 bool hasConfirmation = false,
                 int confirmationType = 0);

// Log data to ReceivedData table
void logReceivedData(Database& db,
                     float tone1,
                     float tone2,
                     float tone3,
                     float tone4,
                     uint16_t commandBitEncoded,
                     bool crc,
                     uint16_t commandBitDecoded,
                     const std::string& command,
                     float speed,
                     float turnSpeed,
                     float duration,
                     int intConfirmationSen = 0);

// Deprecated function names for backwards compatibility
void logPCData(Database& db,
               const std::string& command,
               float speed,
               float turnSpeed,
               float duration,
               uint16_t commandBitDecoded,
               uint16_t commandBitEncoded,
               float tone1,
               float tone2,
               float tone3,
               float tone4,
               int intConfirmationRec);

void logPIData(Database& db,
               float tone1,
               float tone2,
               float tone3,
               float tone4,
               uint16_t commandBitEncoded,
               bool crc,
               uint16_t commandBitDecoded,
               const std::string& command,
               float speed,
               float turnSpeed,
               float duration,
               int intConfirmationSen);
