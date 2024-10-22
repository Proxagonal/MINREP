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

constexpr static int ALLDIGITS = std::numeric_limits<double>::max_digits10 - 1;

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

    string unixTime = to_string(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count());
    string path = PATHSTART + ("_" + unixTime + "/");
    mkdir(path.c_str(), mode);

    InterProcessCounter* systemsLeft = static_cast<InterProcessCounter*>(mmap(nullptr, sizeof(InterProcessCounter), PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0));

    // Initialize the counter to 0
    systemsLeft->counter = SAMPLE;

    // Initialize the mutex with the attribute to allow process sharing
    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);  // Set the mutex as process-shared
    pthread_mutex_init(&systemsLeft->mutex, &attr);


    pid_t pid;
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

    string ID = to_string(index);
    ofstream goodSystemResults(path + ID + string("_EndResults.txt"));
    ofstream goodSystemInfos(path + ID + string("_initialConditions.txt"));
    ofstream badSystemInfos(path + ID + string("_initialConditions_BAD.txt"));

    int iGood = 0, iBad = 0;

    int T = 400;

    int powStart = -3;
    int powJump = -1;
    int powOver = -8;

    while (true) {

        int left;
        pthread_mutex_lock(&systemsLeft->mutex);
        left = systemsLeft->counter;
        if (left > 0)
            systemsLeft->counter--;
        pthread_mutex_unlock(&systemsLeft->mutex);

        if (left == 0)
            break;
        if (left % 100 == 0)
            cout << left << endl;

        initialData rando = Bodyfold::generateRandomCOM();

        double energyBefore = Solver::calcQuantities(rando).E();
        vector<tuple<int, double>> energyInfo;
        vector<tuple<double>> timeStopInfo;

        int powNow = powStart;
        while (powNow > powOver) {

            double dt = pow(10, powNow);

            auto start = now();
            Solver solver(T, dt, rando);
            bool finishedCorrectly = solver.run_EANDT_TCYCLE(energyBefore, MAX_ENERGY_DEVIATION);
            auto end = now();

            timeStopInfo.emplace_back(solver.time);

            double energyAfter = solver.quantities().E();
            double energyDeviation = deviation(energyBefore, energyAfter);

            energyInfo.emplace_back(powNow, energyDeviation);

            // If good energy conservation
            if (energyDeviation <= MAX_ENERGY_DEVIATION and finishedCorrectly) {

                goodSystemInfos << "SYSTEM " << iGood << ": \n";
                goodSystemInfos << Bodyfold::toString(rando);
                goodSystemInfos << "-------------" << endl;
                goodSystemInfos << Solver::calcQuantities(rando).toString();

                goodSystemResults << "SYSTEM " << iGood++ << ": \n";
                goodSystemResults << "END SYSTEM:" << endl;
                goodSystemResults << solver.bodyfold.toString();
                goodSystemResults << "POW ENERGY DIV: " << getPowEnergyInfos(energyInfo) << endl;
                goodSystemResults << "POW END TIME: " << getTimeInfos(timeStopInfo) << endl;

                goodSystemResults << "REAL TIME: ";
                streamMils(goodSystemResults, start, end);
                goodSystemResults << "SIM TIME: " << solver.time << endl;
                goodSystemResults << "HALTING STATUSES: " << getStatuses(solver.statuses) << endl;
                goodSystemResults << "HALTING TIMES: " << getTimes(solver.statuses) << endl;

                break;
            }

            powNow += powJump;

        }

        if (powNow == powOver) {
            badSystemInfos << "SYSTEM " << iBad++ << ": \n";
            badSystemInfos << Bodyfold::toString(rando);
            badSystemInfos << Solver::calcQuantities(rando).toString();
            badSystemInfos << "POW ENERGY DIV: " << getPowEnergyInfos(energyInfo) << endl;
            badSystemInfos << "POW END TIME: " << getTimeInfos(timeStopInfo) << endl;

            continue;
        }
    }

    goodSystemInfos.flush();
    badSystemInfos.flush();
    goodSystemResults.flush();


    if (pid == 0)
        exit(0);

    int status;
    for (int i = 0; i < COMPS - 1; i++) {
        wait(&status);
    }

    return 0;

}






