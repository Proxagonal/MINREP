#include "Solver.h"
#include "Bodyfold.h"
#include <chrono>
#include <fstream>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <semaphore.h>
#include <sys/mman.h>
#include <map>

#define SAMPLE 100000
#define COMPS 12
#define PATHSTART "/mnt/c/Users/eitan/Desktop/DATA/DATAOUT"

using namespace std;
using namespace Eigen;

constexpr static int ALLDIGITS = std::numeric_limits<double>::max_digits10;

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

int toMils(std::chrono::steady_clock::time_point start, std::chrono::steady_clock::time_point end) {
    return (duration_cast<chrono::milliseconds>(end - start)).count();
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

vector<double> expArray(double base, double startPow, double endPow, double jumpPow) {

    vector<double> vals;
    double nowpow = startPow;

    while (nowpow <= endPow) {
        vals.emplace_back(pow(base, nowpow));
        nowpow += jumpPow;
    }

    return vals;

}

bool inRange(double c, double R) {
    if (c == -1 || R == -1)
        return (c == -1 && R == -1);

    return !((c == 10 && R <= 99) || ((c >= 30 && R <= 300)));
}

mode_t mode = 0666 | S_IRWXU | S_IRWXG | S_IRWXO;

const double MAX_ENERGY_DEVIATION = pow(10, -5); //6?-----------------------------

int main() {

    string unixTime = to_string(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count());
    string path = PATHSTART + ("_" + unixTime + "/");
    mkdir(path.c_str(), mode);

    auto* systemsLeft = static_cast<InterProcessDoubleCounter*>(mmap(nullptr, sizeof(InterProcessDoubleCounter), PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0));

    // Initialize the counter to 0
    systemsLeft->counter = SAMPLE;
    systemsLeft->counter2 = 0;

    // Initialize the mutex with the attribute to allow process sharing
    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);  // Set the mutex as process-shared
    pthread_mutex_init(&systemsLeft->mutex, &attr);


    vector<double> cArr;// = expArray(10, 0.5, 0.5, 0.5);
    vector<double> rArr;// = expArray(10, 0.5, 2.5, 0.5);
    //cArr.emplace_back(-1);
    //rArr.emplace_back(-1);
    cArr.emplace_back(5.5);
    cArr.emplace_back(8);

    vector<double> tempexp = expArray(10, 0.5, 2.5, 0.5);
    for (int i = 0; i < tempexp.size() - 1; i++) {
        rArr.emplace_back((tempexp.at(i) + tempexp.at(i+1))/2);
    }

    for (double c : cArr)
        for (double R : rArr) {

            if (!inRange(c, R))
                continue;

            string RcPath = path + to_string((int)c) + "_" + to_string((int)R) + "/";
            mkdir(RcPath.c_str(), mode);
        }


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

    int T = 40;

    int powStart = -3;
    int powJump = -1;
    int powOver = -8;


    std::map<tuple<double, double>, int> counters;
    for (double c : cArr)
        for (double R : rArr) {

            if (!inRange(c, R))
                continue;

            counters[tuple<double, double>(c, R)] = 0;
        }


    while (true) {

        int left;
        bool isSuccess;

        pthread_mutex_lock(&systemsLeft->mutex);

        left = systemsLeft->counter;

        if (left <= 0) {
            pthread_mutex_unlock(&systemsLeft->mutex);
            break;
        }
        systemsLeft->counter--;

        if (left % 10 == 0)
            cout << left << ", " << systemsLeft->counter2 << endl;

        pthread_mutex_unlock(&systemsLeft->mutex);

        initialData rando = Bodyfold::generateRandomCOM();

        double energyBefore = Solver::calcQuantities(rando).E();

        for (double c : cArr)
            for (double R : rArr) {

                if (!inRange(c, R))
                    continue;

                string RcPath = path + to_string((int)c) + "_" + to_string((int)R) + "/";

                ofstream systemResults(RcPath + ID + string("_Results.txt"), std::ios::app);

                vector<tuple<int, double>> energyInfo;
                vector<tuple<double>> timeStopInfo;
                vector<tuple<double>> realTimeInfo;

                //cout << index << ", " << to_string((int)c) + ", " + to_string((int)R) + ", " << counters[tuple<double, double>(c, R)] << endl;

                int powNow = powStart;
                while (powNow > powOver) {

                    double dt = pow(10, powNow);

                    auto start = now();
                    Solver solver(T, dt, R, c, rando);

                    if (c == -1 && R == -1)
                        isSuccess = solver.run_clean_butcheckEA(energyBefore, MAX_ENERGY_DEVIATION);
                    else
                        isSuccess = solver.run_TSPDT(energyBefore, MAX_ENERGY_DEVIATION);

                    auto end = now();

                    energyInfo.emplace_back(powNow, solver.EAMax);
                    timeStopInfo.emplace_back(solver.time);
                    realTimeInfo.emplace_back(toMils(start, end));

                    powNow += powJump;
                    if (isSuccess)
                        break;
                }

                systemResults << "SYSTEM " << counters[tuple<double, double>(c, R)] << ": \n";
                systemResults << Bodyfold::toString(rando);
                systemResults << "-------------" << endl;
                systemResults << Solver::calcQuantities(rando).toString();

                systemResults << "POW MAX ENERGY DIV: " << getPowEnergyInfos(energyInfo) << endl;
                systemResults << "POW END TIME: " << getTimeInfos(timeStopInfo) << endl;

                systemResults << "POW REAL TIME: " << getTimeInfos(realTimeInfo) << endl;
                systemResults << "DID RESOLVE: " << to_string(isSuccess) << endl;

                counters[tuple<double, double>(c, R)]++;

                systemResults.flush();
            }
    }


    if (pid == 0)
        exit(0);

    int status;
    for (int i = 0; i < COMPS - 1; i++) {
        wait(&status);
    }

    return 0;

}