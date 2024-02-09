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

    /*Vector2d frust;

    Vector2d sum;
    double dev;

    sum.setZero();
    dev = 0;

    auto start = now();

    for (int i = 0; i < LOOP; i++) {
        frust.setRandom();
        sum += frust.newtonianNormalized();
    }

    auto end = now();
    printMils(start, end);
    cout << frust << endl;

    sum.setZero();

    start = now();

    for (int i = 0; i < LOOP; i++) {
        frust.setRandom();
        sum += frust/frust.newtonianNorm();
    }

    end = now();
    printMils(start, end);
    cout << frust << endl;

    sum.setZero();

    start = now();

    for (int i = 0; i < LOOP; i++) {
        frust.setRandom();
        sum += frust/(frust.norm()*frust.squaredNorm());
    }

    end = now();
    printMils(start, end);
    cout << frust << endl;


    start = now();


    for (int i = 0; i < LOOP; i++) {
        frust.setRandom();
        dev += frust.newtonianNorm();
    }

    end = now();
    printMils(start, end);
    cout << dev << endl;

    dev = 0;

    start = now();

    for (int i = 0; i < LOOP; i++) {
        frust.setRandom();
        dev += frust.norm()*frust.squaredNorm();
    }

    end = now();
    printMils(start, end);
    cout << dev << endl;*/



    auto start = std::chrono::steady_clock::now();

    Runner runner;

    runner.run();

    auto end = std::chrono::steady_clock::now();

    cout << (duration_cast<chrono::milliseconds>(end - start)).count() << endl;

    return 0;


}






