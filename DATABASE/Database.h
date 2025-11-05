#pragma once
#include <string>
#include <vector>

using namespace std;

// Database class handles all interactions with the SQLite database
class Database {
private:
    void* db;               // Pointer to the SQLite database connection
    string filename;   // Name of the database file

public:
    // Constructor: initialize database with filename
    Database(const string& filename);

    // Destructor: close the database connection if open
    ~Database();

    // Open a connection to the database
    bool open();

    // Close the database connection
    void close();

    // Execute a general SQL command (e.g., CREATE TABLE, INSERT)
    bool execute(const string& sql);

    // Execute a SQL query that returns rows (e.g., SELECT)
    vector<vector<string>> query(const string& sql);

    // Create the PC and PI tables if they do not exist
    bool createTables();

    // Insert a record into the PC table
    bool insertPC(const string& command,
                  const string& bits_raw,
                  const string& bits_encoded,
                  double speed,
                  double duration);

    // Insert a record into the PI table
    bool insertPI(const string& bits_encoded,
                  const string& bits_decoded,
                  const string& command,
                  double speed,
                  double duration);

    // Fetch all records from the PC table
    vector<vector<string>> getAllPC();

    // Fetch all records from the PI table
    vector<vector<string>> getAllPI();
};
