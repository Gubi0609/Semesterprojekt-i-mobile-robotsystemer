#pragma once
#include <string>
#include <vector>

// Database class handles all interactions with the SQLite database
class Database {
private:
    void* db;               // Pointer to the SQLite database connection
    std::string filename;   // Name of the database file

public:
    // Constructor: initialize database with filename
    Database(const std::string& filename);

    // Destructor: close the database connection if open
    ~Database();

    // Open a connection to the database
    bool open();

    // Close the database connection
    void close();

    // Execute a general SQL command (e.g., CREATE TABLE, INSERT)
    bool execute(const std::string& sql);

    // Execute a SQL query that returns rows (e.g., SELECT)
    std::vector<std::vector<std::string>> query(const std::string& sql);

    // Create the PC and PI tables if they do not exist
    bool createTables();

    // Insert a record into the PC table
    bool insertPC(const std::string& command,
                  const std::string& bits_raw,
                  const std::string& bits_encoded,
                  double speed,
                  double duration);

    // Insert a record into the PI table
    bool insertPI(const std::string& bits_encoded,
                  const std::string& bits_decoded,
                  const std::string& command,
                  double speed,
                  double duration);

    // Fetch all records from the PC table
    std::vector<std::vector<std::string>> getAllPC();

    // Fetch all records from the PI table
    std::vector<std::vector<std::string>> getAllPI();
};
