#include "../INCLUDE/Database.h"
#include <iostream>
#include <sstream>

Database::Database(const std::string& fileName) : fileName(fileName), db(nullptr) {}

Database::~Database() {
    close();
}

// Open connection
bool Database::open() {
    if (sqlite3_open(fileName.c_str(), &db) != SQLITE_OK) {
        std::cerr << "Failed to open database: " << sqlite3_errmsg(db) << "\n";
        db = nullptr;
        return false;
    }
    return true;
}

// Close connection
void Database::close() {
    if (db) {
        sqlite3_close(db);
        db = nullptr;
    }
}

// Execute SQL command
bool Database::execute(const std::string& sql) {
    char* errorMsg = nullptr;
    if (sqlite3_exec(db, sql.c_str(), nullptr, nullptr, &errorMsg) != SQLITE_OK) {
        std::cerr << "SQLite error: " << errorMsg << "\n";
        sqlite3_free(errorMsg);
        return false;
    }
    return true;
}

// Create tables
bool Database::createTables() {
    std::string createSentTable =
        "CREATE TABLE IF NOT EXISTS SentData ("
        "id INTEGER PRIMARY KEY AUTOINCREMENT,"
        "startTimeStamp INTEGER,"
        "endTimeStamp INTEGER,"
        "command TEXT,"
        "speed REAL,"
        "turnSpeed REAL,"
        "duration REAL,"
        "commandBitDecoded INTEGER,"
        "commandBitEncoded INTEGER,"
        "tone1 REAL,"
        "tone2 REAL,"
        "tone3 REAL,"
        "tone4 REAL,"
        "intConfirmationRec INTEGER"
        ");";
    
    std::string createReceivedTable =
        "CREATE TABLE IF NOT EXISTS ReceivedData ("
        "id INTEGER PRIMARY KEY AUTOINCREMENT,"
        "timeStamp INTEGER,"
        "tone1 REAL,"
        "tone2 REAL,"
        "tone3 REAL,"
        "tone4 REAL,"
        "commandBitEncoded INTEGER,"
        "crc INTEGER,"
        "commandBitDecoded INTEGER,"
        "command TEXT,"
        "speed REAL,"
        "turnSpeed REAL,"
        "duration REAL,"
        "intConfirmationSen INTEGER"
        ");";
    
    return execute(createSentTable) && execute(createReceivedTable);
}

// Insert a record into SentData table
bool Database::insertSent(int64_t startTimeStamp,
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
                        int intConfirmationRec)
{
    sqlite3_stmt* stmt;
    std::string sql =
        "INSERT INTO SentData (startTimeStamp, endTimeStamp, command, speed, turnSpeed, duration, "
        "commandBitDecoded, commandBitEncoded, "
        "tone1, tone2, tone3, tone4, intConfirmationRec) "
        "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?);";;
    
    if (sqlite3_prepare_v2(db, sql.c_str(), -1, &stmt, nullptr) != SQLITE_OK) {
        std::cerr << "Failed to prepare SentData statement: " << sqlite3_errmsg(db) << "\n";
        return false;
    }
    
    // Bind parameters
    sqlite3_bind_int64(stmt, 1, startTimeStamp);
    sqlite3_bind_int64(stmt, 2, endTimeStamp);
    sqlite3_bind_text(stmt, 3, command.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_double(stmt, 4, speed);
    sqlite3_bind_double(stmt, 5, turnSpeed);
    sqlite3_bind_double(stmt, 6, duration);
    sqlite3_bind_int(stmt, 7, commandBitDecoded);
    sqlite3_bind_int(stmt, 8, commandBitEncoded);
    sqlite3_bind_double(stmt, 9, tone1);
    sqlite3_bind_double(stmt, 10, tone2);
    sqlite3_bind_double(stmt, 11, tone3);
    sqlite3_bind_double(stmt, 12, tone4);
    sqlite3_bind_int(stmt, 13, intConfirmationRec);
    
    // Execute
    int result = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    
    if (result != SQLITE_DONE) {
        std::cerr << "Failed to execute SentData insert: " << sqlite3_errmsg(db) << "\n";
        return false;
    }
    
    return true;
}

// Insert a record into ReceivedData table
bool Database::insertReceived(int64_t timeStamp,
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
                        int intConfirmationSen)
{
    sqlite3_stmt* stmt;
    std::string sql =
        "INSERT INTO ReceivedData (timeStamp, tone1, tone2, tone3, tone4, "
        "commandBitEncoded, crc, commandBitDecoded, command, "
        "speed, turnSpeed, duration, intConfirmationSen) "
        "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?);";;
    
    if (sqlite3_prepare_v2(db, sql.c_str(), -1, &stmt, nullptr) != SQLITE_OK) {
        std::cerr << "Failed to prepare ReceivedData statement: " << sqlite3_errmsg(db) << "\n";
        return false;
    }
    
    // Bind parameters
    sqlite3_bind_int64(stmt, 1, timeStamp);
    sqlite3_bind_double(stmt, 2, tone1);
    sqlite3_bind_double(stmt, 3, tone2);
    sqlite3_bind_double(stmt, 4, tone3);
    sqlite3_bind_double(stmt, 5, tone4);
    sqlite3_bind_int(stmt, 6, commandBitEncoded);
    sqlite3_bind_int(stmt, 7, crc ? 1 : 0);
    sqlite3_bind_int(stmt, 8, commandBitDecoded);
    sqlite3_bind_text(stmt, 9, command.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_double(stmt, 10, speed);
    sqlite3_bind_double(stmt, 11, turnSpeed);
    sqlite3_bind_double(stmt, 12, duration);
    sqlite3_bind_int(stmt, 13, intConfirmationSen);
    
    // Execute
    int result = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    
    if (result != SQLITE_DONE) {
        std::cerr << "Failed to execute ReceivedData insert: " << sqlite3_errmsg(db) << "\n";
        return false;
    }
    
    return true;
}
