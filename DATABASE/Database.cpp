#include "Database.h"
#include <iostream>
#include <sqlite3.h>

Database::Database(const std::string& filename) : db(nullptr), filename(filename) {}

Database::~Database() {
    close();
}

bool Database::open() {
    if (sqlite3_open(filename.c_str(), reinterpret_cast<sqlite3**>(&db)) == SQLITE_OK) {
        return true;
    } else {
        std::cerr << "Failed to open database: " << sqlite3_errmsg(reinterpret_cast<sqlite3*>(db)) << std::endl;
        return false;
    }
}

void Database::close() {
    if (db) {
        sqlite3_close(reinterpret_cast<sqlite3*>(db));
        db = nullptr;
    }
}

bool Database::execute(const std::string& sql) {
    char* errMsg = nullptr;
    if (sqlite3_exec(reinterpret_cast<sqlite3*>(db), sql.c_str(), nullptr, nullptr, &errMsg) != SQLITE_OK) {
        std::cerr << "SQL error: " << errMsg << std::endl;
        sqlite3_free(errMsg);
        return false;
    }
    return true;
}

std::vector<std::vector<std::string>> Database::query(const std::string& sql) {
    std::vector<std::vector<std::string>> results;
    sqlite3_stmt* stmt;
    if (sqlite3_prepare_v2(reinterpret_cast<sqlite3*>(db), sql.c_str(), -1, &stmt, nullptr) == SQLITE_OK) {
        int cols = sqlite3_column_count(stmt);
        while (sqlite3_step(stmt) == SQLITE_ROW) {
            std::vector<std::string> row;
            for (int i = 0; i < cols; ++i) {
                const char* val = reinterpret_cast<const char*>(sqlite3_column_text(stmt, i));
                row.push_back(val ? val : "");
            }
            results.push_back(row);
        }
    } else {
        std::cerr << "Failed to execute query: " << sqlite3_errmsg(reinterpret_cast<sqlite3*>(db)) << std::endl;
    }
    sqlite3_finalize(stmt);
    return results;
}
