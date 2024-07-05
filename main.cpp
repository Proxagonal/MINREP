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
                 "Mass: 1.223289132850786\n"
                 "Position:  11.19485510659843 0.8823399913395003\n"
                 "Velocity:   1.327491688493547 -0.3702353160255317\n"
                 "Body #1: \n"
                 "Mass: 1.579760029157488\n"
                 "Position: -9.465333941075546 -3.268810201054937\n"
                 "Velocity: -1.727204658308442 0.1976714660870507\n"
                 "Body #2: \n"
                 "Mass: 0.5859742323103038\n"
                 "Position: 2.147554546957428 6.970577459591397\n"
                 "Velocity:   1.88517286934457 0.2399958051190474";
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






