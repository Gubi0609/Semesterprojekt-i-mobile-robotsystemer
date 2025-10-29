#include <iostream>
#include <sqlite3.h>

using namespace std;

int main() {
    sqlite3* db;
    char* errMsg = nullptr;

    // Open or create database
    if (sqlite3_open("inputs.db", &db)) {
        cerr << "Can't open database: " << sqlite3_errmsg(db) << endl;
        return 1;
    }
    cout << "Database 'inputs.db' opened successfully." << endl;

    // Create table if it doesn't exist
    const char* sql_create =
        "CREATE TABLE IF NOT EXISTS commands ("
        "id INTEGER PRIMARY KEY AUTOINCREMENT, "
        "command TEXT NOT NULL, "
        "bits_raw TEXT NOT NULL, "
        "bits_encoded TEXT NOT NULL);";

    if (sqlite3_exec(db, sql_create, nullptr, nullptr, &errMsg) != SQLITE_OK) {
        cerr << "SQL error (table creation): " << errMsg << endl;
        sqlite3_free(errMsg);
        sqlite3_close(db);
        return 1;
    }
    cout << "Table 'commands' is ready." << endl;

    // Insert sample commands only if they don't already exist
    const char* sql_insert =
        "INSERT INTO commands (command, bits_raw, bits_encoded) "
        "SELECT 'MOVE_FORWARD', '0001', '1010' "
        "WHERE NOT EXISTS (SELECT 1 FROM commands WHERE command='MOVE_FORWARD');"
        "INSERT INTO commands (command, bits_raw, bits_encoded) "
        "SELECT 'TURN_LEFT', '0010', '0101' "
        "WHERE NOT EXISTS (SELECT 1 FROM commands WHERE command='TURN_LEFT');";

    if (sqlite3_exec(db, sql_insert, nullptr, nullptr, &errMsg) != SQLITE_OK) {
        cerr << "SQL error (insert sample data): " << errMsg << endl;
        sqlite3_free(errMsg);
        sqlite3_close(db);
        return 1;
    }
    cout << "Sample commands inserted (if not already present)." << endl;

    // Query and display all commands
    sqlite3_stmt* stmt;
    const char* sql_select = "SELECT command, bits_raw, bits_encoded FROM commands;";

    if (sqlite3_prepare_v2(db, sql_select, -1, &stmt, nullptr) != SQLITE_OK) {
        cerr << "Failed to prepare SELECT statement: " << sqlite3_errmsg(db) << endl;
        sqlite3_close(db);
        return 1;
    }

    cout << "\nCommands in database:\n";
    cout << "Command       | Bits (raw) | Bits (encoded)\n";
    cout << "------------------------------------------\n";

    while (sqlite3_step(stmt) == SQLITE_ROW) {
        cout << sqlite3_column_text(stmt, 0) << " | "
             << sqlite3_column_text(stmt, 1) << " | "
             << sqlite3_column_text(stmt, 2) << endl;
    }

    sqlite3_finalize(stmt);
    sqlite3_close(db);

    return 0;
}
