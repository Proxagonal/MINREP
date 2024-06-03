#include "Solver.h"
#include <chrono>

using namespace std;
using namespace Eigen;

void printMils(std::chrono::steady_clock::time_point start, std::chrono::steady_clock::time_point end) {
    cout << (duration_cast<chrono::milliseconds>(end - start)).count() << endl;
}

std::chrono::steady_clock::time_point now() {
    return std::chrono::steady_clock::now();
}

int main() {

    Solver solver(50, pow(10, -6)/2);

    auto start = now();

    solver.run();

    auto end = now();

    solver.dumpSystemState();

    printMils(start, end);

    return 0;


}






