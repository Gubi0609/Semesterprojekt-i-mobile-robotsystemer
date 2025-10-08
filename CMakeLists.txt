cmake_minimum_required(VERSION 3.10)

# Project name and language
project(main VERSION 1.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add executable (include all cpp files here)

# Automatically find all .cpp and .h files
file(GLOB_RECURSE SOURCES "*.cpp" "*.h")

# Exclude files in the CMakeFiles directory
foreach(FILE ${SOURCES})
    if(${FILE} MATCHES ".*/CMakeFiles/.*")
        list(REMOVE_ITEM SOURCES ${FILE})
    endif()
endforeach()

# Add executable
add_executable(main ${SOURCES})

# Find libraries that are needed
# Find and link SQLite3
find_package(SQLite3 REQUIRED)
target_link_libraries(main PRIVATE SQLite::SQLite3)

# Manual fallback if find_package fails
target_link_libraries(main PRIVATE sqlite3)
