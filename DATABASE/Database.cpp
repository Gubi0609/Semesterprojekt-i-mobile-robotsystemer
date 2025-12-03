#include "Database.h"
#include <sqlite3.h>
#include <iostream>

// Constructor: store the database filename
Database::Database(const std::string& filename) : db(nullptr), filename(filename) {}

// Destructor: ensure the database is closed
Database::~Database() { close(); }

// Open a connection to the SQLite database
bool Database::open() {
    if (sqlite3_open(filename.c_str(), reinterpret_cast<sqlite3**>(&db)) == SQLITE_OK) {
        return true; // success
    } else {
        std::cerr << "Failed to open database: " 
                  << sqlite3_errmsg(reinterpret_cast<sqlite3*>(db)) << std::endl;
        return false;
    }
}

// Close the database connection if open
void Database::close() {
    if (db) {
        sqlite3_close(reinterpret_cast<sqlite3*>(db));
        db = nullptr;
    }
}

// Execute a SQL command (e.g., CREATE TABLE, INSERT)
bool Database::execute(const std::string& sql) {
    char* errMsg = nullptr;
    if (sqlite3_exec(reinterpret_cast<sqlite3*>(db), sql.c_str(), nullptr, nullptr, &errMsg) != SQLITE_OK) {
        std::cerr << "SQL error: " << errMsg << std::endl;
        sqlite3_free(errMsg);
        return false;
    }
    return true;
}

// Execute a SQL query that returns rows (SELECT)
std::vector<std::vector<std::string>> Database::query(const std::string& sql) {
    std::vector<std::vector<std::string>> results;
    sqlite3_stmt* stmt;

    // Prepare the SQL statement
    if (sqlite3_prepare_v2(reinterpret_cast<sqlite3*>(db), sql.c_str(), -1, &stmt, nullptr) == SQLITE_OK) {
        int cols = sqlite3_column_count(stmt);

        // Iterate over each row
        while (sqlite3_step(stmt) == SQLITE_ROW) {
            std::vector<std::string> row;
            for (int i = 0; i < cols; ++i) {
                const char* val = reinterpret_cast<const char*>(sqlite3_column_text(stmt, i));
                row.push_back(val ? val : ""); // convert NULL to empty string
            }
            results.push_back(row);
        }
    } else {
        std::cerr << "Failed to execute query: "
                  << sqlite3_errmsg(reinterpret_cast<sqlite3*>(db)) << std::endl;
    }

    sqlite3_finalize(stmt); // clean up
    return results;
}

// Create the PC and PI tables if they do not exist
bool Database::createTables() {
    std::string sql_pc = R"(
        CREATE TABLE IF NOT EXISTS pc_data (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            command TEXT,
            bits_raw TEXT,
            bits_encoded TEXT,
            speed REAL,
            duration REAL
        );
    )";

    std::string sql_pi = R"(
        CREATE TABLE IF NOT EXISTS pi_data (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            bits_encoded TEXT,
            bits_decoded TEXT,
            command TEXT,
            speed REAL,
            duration REAL
        );
    )";

    return execute(sql_pc) && execute(sql_pi);
}

// Insert a new record into the PC table
bool Database::insertPC(const std::string& command,
                        const std::string& bits_raw,
                        const std::string& bits_encoded,
                        double speed,
                        double duration) {
    std::string sql = "INSERT INTO pc_data (command, bits_raw, bits_encoded, speed, duration) VALUES ('" +
                      command + "', '" + bits_raw + "', '" + bits_encoded + "', " +
                      std::to_string(speed) + ", " + std::to_string(duration) + ");";
    return execute(sql);
}

// Insert a new record into the PI table
bool Database::insertPI(const std::string& bits_encoded,
                        const std::string& bits_decoded,
                        const std::string& command,
                        double speed,
                        double duration) {
    std::string sql = "INSERT INTO pi_data (bits_encoded, bits_decoded, command, speed, duration) VALUES ('" +
                      bits_encoded + "', '" + bits_decoded + "', '" + command + "', " +
                      std::to_string(speed) + ", " + std::to_string(duration) + ");";
    return execute(sql);
}

// Fetch all records from the PC table
std::vector<std::vector<std::string>> Database::getAllPC() {
    return query("SELECT * FROM pc_data;");
}

// Fetch all records from the PI table
std::vector<std::vector<std::string>> Database::getAllPI() {
    return query("SELECT * FROM pi_data;");
}
