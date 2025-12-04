#include "../INCLUDE/Database.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <ctime>

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
        "responseTime INTEGER,"
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
        "confirmationType INTEGER"  // NULL = no response, 1 = positive, 2 = negative
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
                          int64_t responseTime,
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
                        bool hasConfirmation,
                        int confirmationType)
{
    sqlite3_stmt* stmt;
    std::string sql =
        "INSERT INTO SentData (startTimeStamp, endTimeStamp, responseTime, command, speed, turnSpeed, duration, "
        "commandBitDecoded, commandBitEncoded, "
        "tone1, tone2, tone3, tone4, confirmationType) "
        "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?);";;

    if (sqlite3_prepare_v2(db, sql.c_str(), -1, &stmt, nullptr) != SQLITE_OK) {
        std::cerr << "Failed to prepare SentData statement: " << sqlite3_errmsg(db) << "\n";
        return false;
    }

    // Bind parameters
    sqlite3_bind_int64(stmt, 1, startTimeStamp);
    sqlite3_bind_int64(stmt, 2, endTimeStamp);
    if (responseTime > 0) {
        sqlite3_bind_int64(stmt, 3, responseTime);
    } else {
        sqlite3_bind_null(stmt, 3);  // NULL if no response time
    }
    sqlite3_bind_text(stmt, 4, command.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_double(stmt, 5, speed);
    sqlite3_bind_double(stmt, 6, turnSpeed);
    sqlite3_bind_double(stmt, 7, duration);
    sqlite3_bind_int(stmt, 8, commandBitDecoded);
    sqlite3_bind_int(stmt, 9, commandBitEncoded);
    sqlite3_bind_double(stmt, 10, tone1);
    sqlite3_bind_double(stmt, 11, tone2);
    sqlite3_bind_double(stmt, 12, tone3);
    sqlite3_bind_double(stmt, 13, tone4);

    // Handle confirmation: NULL if no response, 1 for positive, 2 for negative
    if (hasConfirmation) {
        sqlite3_bind_int(stmt, 14, confirmationType);
    } else {
        sqlite3_bind_null(stmt, 14);  // NULL for no response
    }

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

// Helper function to format millisecond timestamp as human-readable date
static std::string formatTimestamp(int64_t timestampMs) {
    if (timestampMs <= 0) {
        return "N/A";
    }
    // Convert milliseconds to seconds
    std::time_t seconds = static_cast<std::time_t>(timestampMs / 1000);
    int millis = static_cast<int>(timestampMs % 1000);

    std::tm* tm_info = std::localtime(&seconds);
    if (!tm_info) {
        return "Invalid timestamp";
    }

    std::ostringstream oss;
    oss << std::put_time(tm_info, "%Y-%m-%d %H:%M:%S") << "." << std::setfill('0') << std::setw(3) << millis;
    return oss.str();
}

// Dump database contents to a log file
bool Database::dumpToLog(const std::string& logFileName) {
    if (!db) {
        std::cerr << "Database not open, cannot dump to log\n";
        return false;
    }

    std::ofstream logFile(logFileName);
    if (!logFile.is_open()) {
        std::cerr << "Failed to open log file: " << logFileName << "\n";
        return false;
    }

    // Write header with timestamp
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    logFile << "================================================================================\n";
    logFile << "Protocol Communication Database Log\n";
    logFile << "Generated: " << std::ctime(&time_t_now);
    logFile << "Database file: " << fileName << "\n";
    logFile << "================================================================================\n\n";

    // Dump SentData table
    logFile << "=== SENT DATA ===\n";
    logFile << std::string(80, '-') << "\n";

    sqlite3_stmt* stmt;
    std::string sql = "SELECT id, startTimeStamp, endTimeStamp, responseTime, command, speed, turnSpeed, duration, "
                      "commandBitDecoded, commandBitEncoded, tone1, tone2, tone3, tone4, confirmationType "
                      "FROM SentData ORDER BY id;";

    if (sqlite3_prepare_v2(db, sql.c_str(), -1, &stmt, nullptr) == SQLITE_OK) {
        int count = 0;
        while (sqlite3_step(stmt) == SQLITE_ROW) {
            count++;
            int id = sqlite3_column_int(stmt, 0);
            int64_t startTs = sqlite3_column_int64(stmt, 1);
            int64_t endTs = sqlite3_column_int64(stmt, 2);
            int64_t respTime = sqlite3_column_int64(stmt, 3);
            bool respTimeNull = (sqlite3_column_type(stmt, 3) == SQLITE_NULL);
            const char* cmd = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 4));
            double speed = sqlite3_column_double(stmt, 5);
            double turnSpeed = sqlite3_column_double(stmt, 6);
            double duration = sqlite3_column_double(stmt, 7);
            int cmdDecoded = sqlite3_column_int(stmt, 8);
            int cmdEncoded = sqlite3_column_int(stmt, 9);
            double t1 = sqlite3_column_double(stmt, 10);
            double t2 = sqlite3_column_double(stmt, 11);
            double t3 = sqlite3_column_double(stmt, 12);
            double t4 = sqlite3_column_double(stmt, 13);

            logFile << "Entry #" << id << "\n";
            logFile << "  Command: " << (cmd ? cmd : "NULL") << "\n";
            logFile << "  Start Time: " << formatTimestamp(startTs) << "\n";
            logFile << "  End Time: " << formatTimestamp(endTs) << "\n";
            if (!respTimeNull && respTime > 0) {
                logFile << "  Response Time: " << respTime << " ms\n";
            } else {
                logFile << "  Response Time: N/A\n";
            }
            logFile << "  Speed: " << speed << ", Turn Speed: " << turnSpeed << ", Duration: " << duration << "\n";
            logFile << "  Command Bits - Decoded: 0x" << std::hex << std::setfill('0') << std::setw(3) << cmdDecoded
                    << ", Encoded: 0x" << std::setw(4) << cmdEncoded << std::dec << "\n";
            logFile << "  Tones: " << std::fixed << std::setprecision(1)
                    << t1 << " Hz, " << t2 << " Hz, " << t3 << " Hz, " << t4 << " Hz\n";

            if (sqlite3_column_type(stmt, 14) == SQLITE_NULL) {
                logFile << "  Confirmation: No response\n";
            } else {
                int confType = sqlite3_column_int(stmt, 14);
                logFile << "  Confirmation: " << (confType == 1 ? "Positive" : (confType == 2 ? "Negative" : "Unknown")) << "\n";
            }
            logFile << std::string(80, '-') << "\n";
        }
        logFile << "Total sent entries: " << count << "\n\n";
        sqlite3_finalize(stmt);
    } else {
        logFile << "Error reading SentData: " << sqlite3_errmsg(db) << "\n\n";
    }

    // Dump ReceivedData table
    logFile << "=== RECEIVED DATA ===\n";
    logFile << std::string(80, '-') << "\n";

    sql = "SELECT id, timeStamp, tone1, tone2, tone3, tone4, commandBitEncoded, crc, "
          "commandBitDecoded, command, speed, turnSpeed, duration, intConfirmationSen "
          "FROM ReceivedData ORDER BY id;";

    if (sqlite3_prepare_v2(db, sql.c_str(), -1, &stmt, nullptr) == SQLITE_OK) {
        int count = 0;
        while (sqlite3_step(stmt) == SQLITE_ROW) {
            count++;
            int id = sqlite3_column_int(stmt, 0);
            int64_t ts = sqlite3_column_int64(stmt, 1);
            double t1 = sqlite3_column_double(stmt, 2);
            double t2 = sqlite3_column_double(stmt, 3);
            double t3 = sqlite3_column_double(stmt, 4);
            double t4 = sqlite3_column_double(stmt, 5);
            int cmdEncoded = sqlite3_column_int(stmt, 6);
            int crc = sqlite3_column_int(stmt, 7);
            int cmdDecoded = sqlite3_column_int(stmt, 8);
            const char* cmd = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 9));
            double speed = sqlite3_column_double(stmt, 10);
            double turnSpeed = sqlite3_column_double(stmt, 11);
            double duration = sqlite3_column_double(stmt, 12);
            int confSent = sqlite3_column_int(stmt, 13);

            logFile << "Entry #" << id << "\n";
            logFile << "  Timestamp: " << formatTimestamp(ts) << "\n";
            logFile << "  Tones: " << std::fixed << std::setprecision(1)
                    << t1 << " Hz, " << t2 << " Hz, " << t3 << " Hz, " << t4 << " Hz\n";
            logFile << "  Command Bits - Encoded: 0x" << std::hex << std::setfill('0') << std::setw(4) << cmdEncoded
                    << ", Decoded: 0x" << std::setw(3) << cmdDecoded << std::dec << "\n";
            logFile << "  CRC: " << (crc ? "Valid" : "Invalid") << "\n";
            logFile << "  Command: " << (cmd ? cmd : "NULL") << "\n";
            logFile << "  Speed: " << speed << ", Turn Speed: " << turnSpeed << ", Duration: " << duration << "\n";
            logFile << "  Confirmation Sent: " << confSent << "\n";
            logFile << std::string(80, '-') << "\n";
        }
        logFile << "Total received entries: " << count << "\n\n";
        sqlite3_finalize(stmt);
    } else {
        logFile << "Error reading ReceivedData: " << sqlite3_errmsg(db) << "\n\n";
    }

    logFile << "================================================================================\n";
    logFile << "End of log\n";
    logFile << "================================================================================\n";

    logFile.close();
    return true;
}
