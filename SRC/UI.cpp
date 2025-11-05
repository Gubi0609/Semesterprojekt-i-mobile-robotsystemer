#include "../INCLUDE/UI.h"

#include <iostream>
#include <sstream>
#include <iomanip>
#include <algorithm>

using namespace std;

UI::UI() {}

void UI::printGeneralHelp() {
    cout << "Accepted commands:\n"
              << "  DRIVE_FORWARD           Drives forward with set linear speed.\n"
              << "  DRIVE_FOR_DURATION      Drives forward with set linear speed for set duration.\n"
              << "  TURN_IN_PLACE           Turns in place with set rotational speed.\n"
              << "  TURN_WITH_ARC           Turns while driving forward creating an arc.\n"
              << "  SEND                    Sends your command stack.\n"
              << "  HELP                    Shows this help page.\n"
              << "\nFor help on specific functions, use: <COMMAND> HELP\n"
              << "-------------------------------------------\n";
}

void UI::printHelp(const string& cmd) {
    if (cmd == "DRIVE_FORWARD") {
        cout << "DRIVE_FORWARD HELP\n"
                  << "  Input parameters:\n"
                  << "    <linear_speed>    Speed at which to drive forward.\n"
                  << "      Type: float, Range: 0 to 100\n";
    } 
    else if (cmd == "DRIVE_FOR_DURATION") {
        cout << "DRIVE_FOR_DURATION HELP\n"
                  << "  Input parameters:\n"
                  << "    <linear_speed>    Type: float, Range: 0 to 100\n"
                  << "    <duration>        Type: float (seconds)\n";
    } 
    else if (cmd == "TURN_IN_PLACE") {
        cout << "TURN_IN_PLACE HELP\n"
                  << "  Input parameters:\n"
                  << "    <rotation_speed>  Type: float, Range: -100 to 100 (Left/Right)\n";
    } 
    else if (cmd == "TURN_WITH_ARC") {
        cout << "TURN_WITH_ARC HELP\n"
                  << "  Input parameters:\n"
                  << "    <linear_speed>    Type: float, Range: 0 to 100\n"
                  << "    <rotation_speed>  Type: float, Range: -100 to 100 (Left/Right)\n";
    } 
    else {
        cout << "No help available for '" << cmd << "'. Try HELP.\n";
    }
}

bool UI::isFloat(const string& s) {
    istringstream iss(s);
    float f;
    return (iss >> f) && iss.eof();
}

void UI::printCommandStack(const vector<Command>& stack) {
    cout << "Command stack sent:\n";
    for (const auto& cmd : stack) {
        cout << "  " << cmd.name;
        for (float p : cmd.params)
            cout << " " << p;
        cout << "\n";
    }
}
vector<Command> UI::writeCommandStack() {
    string input;

    cout << "Robot Command Interface\nType HELP for available commands.\n";

    while (true) {
        cout << "> ";
        getline(cin, input);
        if (input.empty()) continue;

        istringstream iss(input);
        string cmd;
        iss >> cmd;

        transform(cmd.begin(), cmd.end(), cmd.begin(), ::toupper);

        if (cmd == "HELP") {
            printGeneralHelp();
        } 
        else if (cmd == "DRIVE_FORWARD") {
            string arg;
            if (iss >> arg && arg == "HELP") { printHelp(cmd); continue; }

            float linear_speed;
            if (!(iss >> linear_speed)) {
                cout << "Invalid or missing parameter. Type 'DRIVE_FORWARD HELP'.\n";
                continue;
            }
            if (linear_speed < 0 || linear_speed > 100) {
                cout << "Error: linear_speed out of range (0–100).\n";
                continue;
            }
            commandStack.push_back({cmd, {linear_speed}});
            
        } 
        else if (cmd == "DRIVE_FOR_DURATION") {
            string arg;
            if (iss >> arg && arg == "HELP") { printHelp(cmd); continue; }

            iss.clear(); iss.seekg(0); iss >> cmd; // reset parsing
            float linear_speed, duration;
            if (!(iss >> linear_speed >> duration)) {
                cout << "Invalid parameters. Type 'DRIVE_FOR_DURATION HELP'.\n";
                continue;
            }
            if (linear_speed < 0 || linear_speed > 100) {
                cout << "Error: linear_speed out of range (0–100).\n";
                continue;
            }
            commandStack.push_back({cmd, {linear_speed, duration}});
            
        } 
        else if (cmd == "TURN_IN_PLACE") {
            string arg;
            if (iss >> arg && arg == "HELP") { printHelp(cmd); continue; }

            iss.clear(); iss.seekg(0); iss >> cmd;
            float rotation_speed;
            if (!(iss >> rotation_speed)) {
                cout << "Invalid parameter. Type 'TURN_IN_PLACE HELP'.\n";
                continue;
            }
            if (rotation_speed < -100 || rotation_speed > 100) {
                cout << "Error: rotation_speed out of range (-100–100).\n";
                continue;
            }
            commandStack.push_back({cmd, {rotation_speed}});
            
        } 
        else if (cmd == "TURN_WITH_ARC") {
            string arg;
            if (iss >> arg && arg == "HELP") { printHelp(cmd); continue; }

            iss.clear(); iss.seekg(0); iss >> cmd;
            float linear_speed, rotation_speed;
            if (!(iss >> linear_speed >> rotation_speed)) {
                cout << "Invalid parameters. Type 'TURN_WITH_ARC HELP'.\n";
                continue;
            }
            if (linear_speed < 0 || linear_speed > 100) {
                cout << "Error: linear_speed out of range (0–100).\n";
                continue;
            }
            if (rotation_speed < -100 || rotation_speed > 100) {
                cout << "Error: rotation_speed out of range (-100–100).\n";
                continue;
            }
            commandStack.push_back({cmd, {linear_speed, rotation_speed}});
            
        } 
        else if (cmd == "SEND") {
            printCommandStack(commandStack);
            commandStack.clear();
            cout << "(Stack cleared after send.)\n";
            return commandStack;
        } 
        else if (cmd == "EXIT" || cmd == "QUIT") {
            cout << "Exiting...\n";
            break;
        } 
        else {
            cout << "Unknown command. Type HELP.\n";
        }
    }

}

UI::~UI() {}

