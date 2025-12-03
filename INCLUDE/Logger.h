#pragma once
#include "Database.h"
#include <string>

void logPCData(Database& db,
               const std::string& command,
               const std::string& bits_raw,
               const std::string& bits_encoded,
               double speed,
               double duration);

void logPIData(Database& db,
               const std::string& bits_encoded,
               const std::string& bits_decoded,
               const std::string& command,
               double speed,
               double duration);
