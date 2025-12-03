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
    
    // Insert a record into the PI table
    bool insertPI(const std::string& timeStamp,
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

private:
    sqlite3* db = nullptr;
    std::string fileName;
    bool execute(const std::string& sql);
};