#ifndef UI_H
#define UI_H

#include <iostream>
#include <vector>
#include <string>

using namespace std;

struct Command {
    std::string name;
    std::vector<float> params;
};

class UI {

    public:
        UI();
        vector<Command> writeCommandStack();
        void printGeneralHelp();
        void printHelp(const std::string& cmd);
        bool isFloat(const std::string& s);
        void printCommandStack(const std::vector<Command>& stack);
        ~UI();

    protected:
        vector<Command> commandStack;
        string helpFile = "../ASSETS/UIHelp.txt";

};

#endif