#include "Solver.h"
#include "Bodyfold.h"
#include <chrono>
#include <fstream>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <semaphore.h>
#include <sys/mman.h>

#define PATHSTART "/mnt/c/Users/eitan/Desktop/INSANE/out"
#define POWSTART "/mnt/c/Users/eitan/Desktop/INSANE/pow"


using namespace std;
using namespace Eigen;

constexpr static int ALLDIGITS = std::numeric_limits<long double>::max_digits10;

struct InterProcessDoubleCounter {
    int counter;
    int counter2;
    pthread_mutex_t mutex;
};

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

string getStatuses(vector<tuple<tuple<int, int>, long double>> &vtup) {
    stringstream ss;
    for (auto &tup: vtup)
        ss << intintTupleToString(get<0>(tup)) << " | ";
    return ss.str();
}

string getTimes(vector<tuple<tuple<int, int>, long double>> &vtup) {
    stringstream ss;
    ss.precision(ALLDIGITS);
    for (auto &x: vtup)
        ss << get<1>(x) << " | ";
    return ss.str();
}

string getPowEnergyInfos(vector<tuple<int, long double>> &infos) {
    stringstream ss;
    ss.precision(ALLDIGITS);
    for (auto &x: infos)
        ss << "(" << get<0>(x) << ", " << get<1>(x) << ")" << " /\\ ";
    return ss.str();
}

string getTimeInfos(vector<tuple<long double>> &infos) {
    stringstream ss;
    ss.precision(ALLDIGITS);
    for (auto &x: infos)
        ss << get<0>(x) << " /\\ ";
    return ss.str();
}

long double deviation(long double x, long double xAfter) {
    return abs((x-xAfter)/x);
}

mode_t mode = 0666 | S_IRWXU | S_IRWXG | S_IRWXO;

const long double MAX_ENERGY_DEVIATION = pow(10, -5); //6?-----------------------------

int main() {

    string SYSTEMTEXT = R"(Body #0:
Mass: 1.1011802335942111
Position:  5.5008317108382769 -3.6735804046871552
Velocity:  -1.9915882840100458 -0.77857246020491244
Body #1:
Mass: 0.86718424944861017
Position: 1.8757830959650619 8.5714404076736361
Velocity: 0.066168068319390372  0.18052253200846247
Body #2:
Mass: 0.65498813423986568
Position: -11.731596807358638 -5.1722219248948917
Velocity: 3.2606968485394372 1.0699465693122017)";

    ofstream sysFile(PATHSTART + string("_") +  SYSTEMTEXT.substr(17, 5) + string(".txt"));

    initialData SYSTEM = Bodyfold::stringToInitialData(SYSTEMTEXT);

    sysFile << "__SYSTEM__" << endl;
    sysFile << Bodyfold::toString(SYSTEM);
    sysFile << Solver::calcQuantities(SYSTEM).toString() << endl;

    int T = 4000;

    int powStart = -3;
    int powEnd = -4;
    int powNow = powStart;

    while (powNow > powEnd) {

        long double dt = pow(10, powNow);

        auto start = now();
        Solver solver(T, dt, SYSTEM);
        solver.run_ISI(MAX_ENERGY_DEVIATION);
        auto end = now();

        // If good energy conservation
        if (true) {

            sysFile << "__POW" << powNow-- << "__" << endl;

            sysFile << "END SYSTEM:" << endl;
            sysFile << solver.bodyfold.toString();
            sysFile << solver.quantities().toString();

            sysFile << "REAL TIME: ";
            streamMils(sysFile, start, end);
            sysFile << "SIM TIME: " << solver.time << endl;

            sysFile << "HALTING STATUSES: " << getStatuses(solver.statuses) << endl;
            sysFile << "HALTING TIMES: " << getTimes(solver.statuses) << endl;
            sysFile << "DISS HEURISTIC TIME: " << solver.ISI_hDissTime << endl;
            sysFile << "ESCAPE HEURISTIC TIME: " << solver.ISI_hEscapeTime << endl;

        }

        ofstream powFile(string(POWSTART) + "_" + to_string(-powNow) + "_" + SYSTEMTEXT.substr(17, 5) + string(".txt"));

        for (int i = 0; i < solver.ISI_times.size(); i++) {
            powFile << solver.ISI_times[i] << endl;
            powFile << endl;

            powFile << get<0>(solver.ISI_positions[i]).transpose() << endl;
            powFile << get<0>(solver.ISI_velocities[i]).transpose() << endl;
            powFile << endl;

            powFile << get<1>(solver.ISI_positions[i]).transpose() << endl;
            powFile << get<1>(solver.ISI_velocities[i]).transpose() << endl;
            powFile << endl;

            powFile << get<2>(solver.ISI_positions[i]).transpose() << endl;
            powFile << get<2>(solver.ISI_velocities[i]).transpose() << endl;
            powFile << endl;

            powFile << endl;
        }
        powFile.flush();
    }

    sysFile.flush();

    return 0;

}