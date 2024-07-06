#include "Solver.h"
#include <chrono>
#include <unistd.h>
#include <iostream>

using namespace std;
using namespace Eigen;

void printMils(std::chrono::steady_clock::time_point start, std::chrono::steady_clock::time_point end) {
    cout << (duration_cast<chrono::milliseconds>(end - start)).count() << endl;
}

std::chrono::steady_clock::time_point now() {
    return std::chrono::steady_clock::now();
}

int main() {


    string str = "Body #0: \n"
                 "Mass: 0.5008354695439635\n"
                 "Position: -6.237708960584566  3.205002857458538\n"
                 "Velocity:  5.209721563384095 -2.542971086141526\n"
                 "Body #1: \n"
                 "Mass: 1.855557267872371\n"
                 "Position: -4.041242930423049 -3.511262951907347\n"
                 "Velocity: 0.7199298404618073  2.103058676442367\n"
                 "Body #2: \n"
                 "Mass: 1.366899307653894\n"
                 "Position: 7.771474846358647 3.592196112269125\n"
                 "Velocity: -2.886155820789106 -1.923137775511901";
    initialData init = Bodyfold::stringToInitialData(str);

    //Solver solver(100, pow(10, -6)/2);

    Solver solver(150, pow(10, -5), init);

    auto start = now();

    solver.run();

    auto end = now();

    solver.dumpSystemStateString();

    printMils(start, end);

    return 0;


}






