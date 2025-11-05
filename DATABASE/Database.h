#pragma once
#include <string>
#include <vector>

using namespace std;

class Database {
private:
    void* db;
    string filename;

public:
    Database(const string& filename);
    ~Database();

    bool open();
    void close();

    // General SQL execution
    bool execute(const string& sql);
    vector<vector<string>> query(const string& sql);

    bool createTables();

    // PC table operations
    bool insertPC(const string& command,
                  const string& bits_raw,
                  const string& bits_encoded,
                  double speed);

    // PI table operations
    bool insertPI(const string& bits_encoded,
                  const string& bits_decoded,
                  const string& command,
                  double speed);

    // Fetch data
    vector<vector<string>> getAllPC();
    vector<vector<string>> getAllPI();
};
