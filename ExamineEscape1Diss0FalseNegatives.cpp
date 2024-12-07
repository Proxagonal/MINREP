#include "Solver.h"
#include <chrono>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace std;
using namespace Eigen;

int main() {

    std::ifstream file("/mnt/c/Users/eitan/Desktop/DATA/yaf/EOnly.txt");

    std::string line, systemData;
    int systemIndex = 0;
    std::vector<std::pair<int, std::string>> systems;

    while (std::getline(file, line)) {
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        if (line.find("SYSTEM") != std::string::npos) {
            if (!systemData.empty()) systems.emplace_back(systemIndex, systemData), systemData.clear();
            systemIndex = std::stoi(line.substr(7)); // Extract system index
        } else if (!line.empty()) {
            systemData += line + "\n";
        }
    }
    if (!systemData.empty()) systems.emplace_back(systemIndex, systemData); // Add last system

    for (const auto& [index, str] : systems) {

        string str2 = str;

        //cout << "||" << str2 << "||" << endl;;

        initialData init = Bodyfold::stringToInitialData(str2);

        Solver solver(Solver::GFNOC_HideAndSeekTime, pow(10, -3), init);

        solver.run_dry();

        if (!solver.confirmEscapeHeuristicWrongInThisWay())
            cout << "WOW SYSTEM " << index << endl;
        else
            cout << "Okay System " << index << endl;
    }

    return 0;


}






