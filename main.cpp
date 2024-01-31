#include "Runner.h"
#include <chrono>


using namespace std;
using namespace Eigen;


int main() {

    auto start = std::chrono::steady_clock::now();

    Runner runner;

    runner.run();

    auto end = std::chrono::steady_clock::now();

    cout << (duration_cast<chrono::milliseconds>(end - start)).count() << endl;

    return 0;
}





