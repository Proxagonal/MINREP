#include "Solver.h"
#include "Bodyfold.h"
#include <chrono>
#include <fstream>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/wait.h>

#define SAMPLE 500
#define COMPS 12
#define POWMIN 6
#define POWMAX 6

using namespace std;
using namespace Eigen;

constexpr static int ALLDIGITS = std::numeric_limits<double>::max_digits10 - 1;

void printMils(std::chrono::steady_clock::time_point start, std::chrono::steady_clock::time_point end) {
    cout << (duration_cast<chrono::milliseconds>(end - start)).count() << endl;
}

int getMils(std::chrono::steady_clock::time_point start, std::chrono::steady_clock::time_point end) {
    return (duration_cast<chrono::milliseconds>(end - start)).count();
}

void streamMils(ofstream &stream, std::chrono::steady_clock::time_point start, std::chrono::steady_clock::time_point end) {
    stream << (duration_cast<chrono::milliseconds>(end - start)).count() << endl;
}

std::chrono::steady_clock::time_point now() {
    return std::chrono::steady_clock::now();
}

int main() {

    pid_t pid = 0;
    int index;

    for (int i = 0; i < COMPS - 1; i++) {
        pid = fork();
        if (pid == 0) {
            index = i;
            break;
        }
    }

    if (pid != 0)
        index = COMPS - 1;

    vector<initialData> randoms;

    int i;
    for (i = 0; i < SAMPLE/COMPS; i++)
        randoms.emplace_back(Bodyfold::generateRandomCOM());

    int T = 50;
    double checkTPeriod = T/50;
    double dt = pow(10, -POWMIN);
    int timeWithHalt;
    int timeNoHalt;
    auto measure = now();
    int amountOfSystems;
    double sumRatios;
    double sumDifferences;


    //pow, time ratio, time diff
    vector<tuple<int, double, double>> timeStuff;

    auto total = now();
    for (int pow = POWMIN; pow < POWMAX + 1; pow++) {

        if (pid != 0)
            cout << dt << endl;

        sumRatios = 0;
        sumDifferences = 0;
        amountOfSystems = 0;
        for (auto &rando : randoms) {

            Solver solverHalt(T, dt, checkTPeriod, rando);

            measure = now();
            solverHalt.runDry();
            timeWithHalt = getMils(measure, now());


            Solver solverNoHalt(T, dt, checkTPeriod, rando);

            measure = now();
            solverNoHalt.runDryNoHalt();
            timeNoHalt = getMils(measure, now());

            sumRatios += ((double)timeWithHalt)/timeNoHalt;
            sumDifferences += timeWithHalt - timeNoHalt;
            amountOfSystems++;
        }

        cout << "Pow: " << pow << endl;
        cout << "Amount of systems: " << amountOfSystems << endl;
        cout << "Average time ratio: " << sumRatios/amountOfSystems << endl;
        cout << "Average time difference: " << sumDifferences/amountOfSystems << endl;


        timeStuff.emplace_back(pow, sumRatios/amountOfSystems,
                               sumDifferences/amountOfSystems);

        dt = dt/10;
    }

    if (pid == 0)
        exit(0);

    //printMils(total, now());

    int status;
    for (i = 0; i < COMPS - 1; i++) {
        wait(&status);
    }

    return 0;

}
