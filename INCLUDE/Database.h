#pragma once
#include <sqlite3.h>
#include <string>
#include <cstdint>

class Database {
public:
    Database(const std::string& fileName);
    ~Database();
    bool open();
    void close();
    bool createTables();
    
    // Insert a record into the SentData table
    bool insertSent(int64_t startTimeStamp,
                    int64_t endTimeStamp,
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
    
    // Insert a record into the ReceivedData table
    bool insertReceived(int64_t timeStamp,
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

private:
    sqlite3* db = nullptr;
    std::string fileName;
    bool execute(const std::string& sql);
};