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


    string str = "Body #0:\n"
                 "Mass: 1.292080099799268\n"
                 "Position: 1.599318694671322 5.553392826262194\n"
                 "Velocity:  -2.44163630205618 -3.606830431321515\n"
                 "Body #1:\n"
                 "Mass: 1.470712956397409\n"
                 "Position:  3.952350623389272 -3.870711661021517\n"
                 "Velocity: -0.756039528894016 0.8534780761757288\n"
                 "Body #2:\n"
                 "Mass: 1.182641327572333\n"
                 "Position: -6.662392853156769 -1.253738164116226\n"
                 "Velocity: 3.607777529883693 2.879226761014655\n";
    initialData init = Bodyfold::stringToInitialData(str);

    Solver solver(10000, pow(10, -5), init);

    //Solver solver(10000, pow(10, -5));

    auto start = now();

    solver.run();

    auto end = now();

    solver.dumpSystemStateString();

    printMils(start, end);

    return 0;


}






