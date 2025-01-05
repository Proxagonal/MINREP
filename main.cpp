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

    initialData init = Bodyfold::generateRandomCOM_no3();
    Solver solver(100000, pow(10, -4), init);

    auto start = now();

    auto [T, lst] = solver.run_vdt_TBCTPOSS(pow(10, -5));
    cout << "stat " << T << endl;

    auto end = now();

    //solver.dumpSystemStateString();

    printMils(start, end);

    return 0;


}






