#include "../INCLUDE/Database.h"
#include <iostream>

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
        "movement TEXT,"
        "bits TEXT,"
        "commandEncoded TEXT,"
        "frequencies TEXT,"
        "intConfirmationRec TEXT"
        ");";

    std::string createPITable =
        "CREATE TABLE IF NOT EXISTS PIData ("
        "id INTEGER PRIMARY KEY AUTOINCREMENT,"
        "timeStamp TEXT,"
        "frequencies TEXT,"
        "commandEncoded TEXT,"
        "crc INTEGER,"
        "command TEXT,"
        "movement TEXT,"
        "intConfirmationSen TEXT"
        ");";

    return execute(createPCTable) && execute(createPITable);
}

// Insert a record into PC table
bool Database::insertPC(const std::string& timeStamp,
                        const std::string& command,
                        const std::string& movement,
                        const std::string& bits,
                        const std::string& commandEncoded,
                        const std::string& frequencies,
                        const std::string& intConfirmationRec)
{
    std::string sql =
        "INSERT INTO PCData (timeStamp, command, movement, bits, commandEncoded, frequencies, intConfirmationRec) VALUES ('" +
        timeStamp + "', '" + command + "', '" + movement + "', '" + bits + "', '" +
        commandEncoded + "', '" + frequencies + "', '" + intConfirmationRec + "');";

    return execute(sql);
}

// Insert a record into PI table
bool Database::insertPI(const std::string& timeStamp,
                        const std::string& frequencies,
                        const std::string& commandEncoded,
                        bool crc,
                        const std::string& command,
                        const std::string& movement,
                        const std::string& intConfirmationSen)
{
    std::string sql =
        "INSERT INTO PIData (timeStamp, frequencies, commandEncoded, crc, command, movement, intConfirmationSen) VALUES ('" +
        timeStamp + "', '" + frequencies + "', '" + commandEncoded + "', " +
        std::to_string(crc ? 1 : 0) + ", '" + command + "', '" + movement + "', '" +
        intConfirmationSen + "');";

    return execute(sql);
}
