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
    std::string createPCTable =
        "CREATE TABLE IF NOT EXISTS PCData ("
        "id INTEGER PRIMARY KEY AUTOINCREMENT,"
        "timeStamp TEXT,"
        "command TEXT,"
        "speed REAL,"
        "turnSpeed REAL,"
        "duration REAL,"
        "commandBitRaw TEXT,"
        "commandBitDecoded TEXT,"
        "commandBitEncoded TEXT,"
        "tone1 REAL,"
        "tone2 REAL,"
        "tone3 REAL,"
        "tone4 REAL,"
        "intConfirmationRec INTEGER"
        ");";
    
    std::string createPITable =
        "CREATE TABLE IF NOT EXISTS PIData ("
        "id INTEGER PRIMARY KEY AUTOINCREMENT,"
        "timeStamp TEXT,"
        "tone1 REAL,"
        "tone2 REAL,"
        "tone3 REAL,"
        "tone4 REAL,"
        "commandEncoded TEXT,"
        "crc INTEGER,"
        "commandDecoded TEXT,"
        "command TEXT,"
        "speed REAL,"
        "turnSpeed REAL,"
        "duration REAL,"
        "intConfirmationSen INTEGER"
        ");";
    
    return execute(createPCTable) && execute(createPITable);
}

// Insert a record into PC table
bool Database::insertPC(const std::string& timeStamp,
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
                        bool intConfirmationRec)
{
    sqlite3_stmt* stmt;
    std::string sql =
        "INSERT INTO PCData (timeStamp, command, speed, turnSpeed, duration, "
        "commandBitRaw, commandBitDecoded, commandBitEncoded, "
        "tone1, tone2, tone3, tone4, intConfirmationRec) "
        "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?);";
    
    if (sqlite3_prepare_v2(db, sql.c_str(), -1, &stmt, nullptr) != SQLITE_OK) {
        std::cerr << "Failed to prepare PC statement: " << sqlite3_errmsg(db) << "\n";
        return false;
    }
    
    // Bind parameters
    sqlite3_bind_text(stmt, 1, timeStamp.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 2, command.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_double(stmt, 3, speed);
    sqlite3_bind_double(stmt, 4, turnSpeed);
    sqlite3_bind_double(stmt, 5, duration);
    sqlite3_bind_text(stmt, 6, commandBitRaw.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 7, commandBitDecoded.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 8, commandBitEncoded.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_double(stmt, 9, tone1);
    sqlite3_bind_double(stmt, 10, tone2);
    sqlite3_bind_double(stmt, 11, tone3);
    sqlite3_bind_double(stmt, 12, tone4);
    sqlite3_bind_int(stmt, 13, intConfirmationRec ? 1 : 0);
    
    // Execute
    int result = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    
    if (result != SQLITE_DONE) {
        std::cerr << "Failed to execute PC insert: " << sqlite3_errmsg(db) << "\n";
        return false;
    }
    
    return true;
}

// Insert a record into PI table
bool Database::insertPI(const std::string& timeStamp,
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
                        bool intConfirmationSen)
{
    sqlite3_stmt* stmt;
    std::string sql =
        "INSERT INTO PIData (timeStamp, tone1, tone2, tone3, tone4, "
        "commandEncoded, crc, commandDecoded, command, "
        "speed, turnSpeed, duration, intConfirmationSen) "
        "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?);";
    
    if (sqlite3_prepare_v2(db, sql.c_str(), -1, &stmt, nullptr) != SQLITE_OK) {
        std::cerr << "Failed to prepare PI statement: " << sqlite3_errmsg(db) << "\n";
        return false;
    }
    
    // Bind parameters
    sqlite3_bind_text(stmt, 1, timeStamp.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_double(stmt, 2, tone1);
    sqlite3_bind_double(stmt, 3, tone2);
    sqlite3_bind_double(stmt, 4, tone3);
    sqlite3_bind_double(stmt, 5, tone4);
    sqlite3_bind_text(stmt, 6, commandEncoded.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(stmt, 7, crc ? 1 : 0);
    sqlite3_bind_text(stmt, 8, commandDecoded.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 9, command.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_double(stmt, 10, speed);
    sqlite3_bind_double(stmt, 11, turnSpeed);
    sqlite3_bind_double(stmt, 12, duration);
    sqlite3_bind_int(stmt, 13, intConfirmationSen ? 1 : 0);
    
    // Execute
    int result = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    
    if (result != SQLITE_DONE) {
        std::cerr << "Failed to execute PI insert: " << sqlite3_errmsg(db) << "\n";
        return false;
    }
    
    return true;
}