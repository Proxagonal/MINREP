#include "Runner.h"
#include <chrono>


using namespace std;
using namespace Eigen;

void printMils(std::chrono::steady_clock::time_point start, std::chrono::steady_clock::time_point end) {
    cout << (duration_cast<chrono::milliseconds>(end - start)).count() << endl;
}

std::chrono::steady_clock::time_point now() {
    return std::chrono::steady_clock::now();
}

#define LOOP 100000000

int main() {

    auto start = now();

    Runner runner;

    runner.run();

    auto end = now();

    printMils(start, end);

    return 0;


}






