#pragma once
#include <sqlite3.h>
#include <string>

class Database {
public:
    Database(const std::string& fileName);
    ~Database();
    bool open();
    void close();
    bool createTables();

    // Insert a record into the PC table
    bool insertPC(const std::string& timeStamp,
                  const std::string& command,
                  const std::string& movement,         // structured string: mode, speed, turn, duration
                  const std::string& bits,
                  const std::string& commandEncoded,
                  const std::string& frequencies,     // structured string for 4 tones
                  const std::string& intConfirmationRec);

    // Insert a record into the PI table
    bool insertPI(const std::string& timeStamp,
                  const std::string& frequencies,
                  const std::string& commandEncoded,
                  bool crc,
                  const std::string& command,
                  const std::string& movement,         // structured string
                  const std::string& intConfirmationSen);

private:
    sqlite3* db = nullptr;
    std::string fileName;
    bool execute(const std::string& sql);
};
