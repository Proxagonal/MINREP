#include "Solver.h"
#include "Constructors.h"
#include <chrono>
#include <unistd.h>
#include <iostream>

using namespace std;
using namespace Eigen;

string mils(std::chrono::steady_clock::time_point start, std::chrono::steady_clock::time_point end) {
    return to_string((duration_cast<chrono::milliseconds>(end - start)).count());
}

std::chrono::steady_clock::time_point now() {
    return std::chrono::steady_clock::now();
}

initialData randomsys() {

    return Solver::generateRandomNONCOM();

}

int main() {

    int N = 10000;
    double dt = pow(10, -4);
    vector<double> EAs;

    double ESUM = 0;

    for (int i = 0; i < N; i++) {

        //Solver solver();


    }

}