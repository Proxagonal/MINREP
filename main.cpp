#include "Runner.h"
#include <chrono>
#include "Solver.h"


using namespace std;
using namespace Eigen;


int main() {
    Solver solver{0.001};

    auto start = std::chrono::steady_clock::now();

    solver.strange1();

    auto end = std::chrono::steady_clock::now();

    cout << (duration_cast<chrono::milliseconds>(end - start)).count() << endl;

    start = std::chrono::steady_clock::now();

    solver.strange2();

    end = std::chrono::steady_clock::now();

    cout << "AAA" << (duration_cast<chrono::milliseconds>(end - start)).count() << "AAA" << endl;

    return 0;

    //auto start = std::chrono::steady_clock::now();
//
    //Runner runner;
//
    //runner.run();
//
    //auto end = std::chrono::steady_clock::now();
//
    //cout << (duration_cast<chrono::milliseconds>(end - start)).count() << endl;
//
    //return 0;
}





