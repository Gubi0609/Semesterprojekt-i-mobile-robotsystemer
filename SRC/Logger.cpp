#include "Logger.h"

void logPCData(Database& db,
               const std::string& command,
               const std::string& bits_raw,
               const std::string& bits_encoded,
               double speed,
               double duration) {
    db.insertPC(command, bits_raw, bits_encoded, speed, duration);
}

void logPIData(Database& db,
               const std::string& bits_encoded,
               const std::string& bits_decoded,
               const std::string& command,
               double speed,
               double duration) {
    db.insertPI(bits_encoded, bits_decoded, command, speed, duration);
}
