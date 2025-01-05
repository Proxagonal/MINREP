#include "Solver.h"
#include "Bodyfold.h"
#include <chrono>
#include <fstream>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <semaphore.h>
#include <sys/mman.h>

#define SAMPLE 1200
#define COMPS 12
#define PATHSTART "/mnt/c/Users/eitan/Desktop/DATA/DATAOUT"

using namespace std;
using namespace Eigen;

constexpr static int ALLDIGITS = std::numeric_limits<double>::max_digits10;

struct InterProcessCounter {
    int counter;
    pthread_mutex_t mutex;
};

void printMils(std::chrono::steady_clock::time_point start, std::chrono::steady_clock::time_point end) {
    cout << (duration_cast<chrono::milliseconds>(end - start)).count() << endl;
}

void streamMils(ofstream &stream, std::chrono::steady_clock::time_point start, std::chrono::steady_clock::time_point end) {
    stream << (duration_cast<chrono::milliseconds>(end - start)).count() << endl;
}

std::chrono::steady_clock::time_point now() {
    return std::chrono::steady_clock::now();
}

string intintTupleToString(tuple<int, int> tup) {
    int x, y;
    tie(x, y) = tup;
    return "(" + to_string(x) + ", " + to_string(y) + ")";
}

string getStatuses(vector<tuple<tuple<int, int>, double>> &vtup) {
    stringstream ss;
    for (auto &tup: vtup)
        ss << intintTupleToString(get<0>(tup)) << " | ";
    return ss.str();
}

string getTimes(vector<tuple<tuple<int, int>, double>> &vtup) {
    stringstream ss;
    ss.precision(ALLDIGITS);
    for (auto &x: vtup)
        ss << get<1>(x) << " | ";
    return ss.str();
}

string getPowEnergyInfos(vector<tuple<int, double>> &infos) {
    stringstream ss;
    ss.precision(ALLDIGITS);
    for (auto &x: infos)
        ss << "(" << get<0>(x) << ", " << get<1>(x) << ")" << " /\\ ";
    return ss.str();
}

string getTimeInfos(vector<tuple<double>> &infos) {
    stringstream ss;
    ss.precision(ALLDIGITS);
    for (auto &x: infos)
        ss << get<0>(x) << " /\\ ";
    return ss.str();
}

double deviation(double x, double xAfter) {
    return abs((x-xAfter)/x);
}

mode_t mode = 0666 | S_IRWXU | S_IRWXG | S_IRWXO;

const double MAX_ENERGY_DEVIATION = pow(10, -5); //6?-----------------------------

int main() {

    int N = 2000;
    int T = 20;

    int powSim = -4;
    double dt = pow(10, powSim);


    for (int i = 0; i < N; i++) {

        initialData rando = Bodyfold::generateRandomCOM_no3();


        Solver solver(T, dt, rando);
        auto [sowInfo, b1pos_time] = solver.run_vdt_TBCTPOSS(MAX_ENERGY_DEVIATION);

        auto [sowTime, p1sow, v1sow, p2sow, v2sow] = sowInfo;

        if (sowTime == -1 || sowTime == -2 || b1pos_time.empty()) {
            //cout << "EA or no skip" << endl;
            continue;
        }

        double minangle = abs(get<0>(b1pos_time.at(0)));
        double angle;
        int winner = -1;
        for (int i = 0; i < b1pos_time.size(); i++)
            if ((angle = abs(get<0>(b1pos_time.at(i)))) <= minangle) {
                minangle = abs(angle);
                winner = i;
            }

        auto [_, simMeasure, p1, v1, p2, v2] = b1pos_time.at(winner);

        //cout.precision(std::numeric_limits<double>::max_digits10 - 1);
        //cout << minangle << ", " << simMeasure << endl;
        //cout << sowTime << endl;
        if (abs(simMeasure - sowTime) >= dt
            || (p1sow - p1).norm() >= 100*dt
            || (v1sow - v1).norm() >= 100*dt
            || (p2sow - p2).norm() >= 100*dt
            || (v2sow - v2).norm() >= 100*dt) {
            cout << "--- invalid " << i << endl;
            solver.dumpSystemStateString();
            cout << simMeasure << ", " << sowTime << endl;
            cout << p1.transpose() << ", " << p1sow.transpose() << endl;
            cout << v1.transpose() << ", " << v1sow.transpose() << endl;
            cout << p2.transpose() << ", " << p2sow.transpose() << endl;
            cout << v2.transpose() << ", " << v2sow.transpose() << endl;
        }
        //else
        //    cout << "valid " << i << endl;
    }

    return 0;

}