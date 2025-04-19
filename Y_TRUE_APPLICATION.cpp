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
#include <algorithm>

#define COMPS 12
#define PATHSTART "/mnt/c/Users/eitan/Desktop/DATA/DATAOUT"

#define M1 17.5
#define M2 15
#define M3 12.5
#define R12 5
#define R12_3 100
#define V3 5

#define PHASESAMPLE 150
#define SHOOTSAMPLE 150

#define SHOOTMIN (M_PI - atan(0.5*R12 / R12_3))
#define SHOOTMAX (M_PI + atan(0.5*R12 / R12_3))

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

string doubledoubleToString(double x, double y) {
    stringstream ss;
    ss.precision(ALLDIGITS);

    ss << x << " " << y;

    return ss.str();
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

bool inRange(double c, double R, vector<tuple<double, double>> exc = {}) {
    if (c == -1 || R == -1)
        return (c == -1 && R == -1);

    if (find(exc.begin(), exc.end(), tuple<double, double>(c, R)) != exc.end())
        return false;

    return true;
}
string dToS(double x, int n) {
    string full = to_string(x);
    return full.substr(0, full.find(".")+n+1);
}

string toString(double c, double R) {
    return dToS(c, 1) + "_" + dToS(R, 1);
}

bool isIntIn(int x, vector<int> &lst) {
    return (find(lst.begin(), lst.end(), x) != lst.end());
}

void printvec(vector<tuple<double, double>> &vec) {

    if (vec.size() == 0)
        return;

    for (auto &[x, y] : vec)
        cout << std::fixed << "(" << x << ", " << y << ")" << ", ";
    cout << endl;
}

static Vector2d toCartesian(double rad, double theta) {
    return {rad*cos(theta), rad*sin(theta)};
}

inline static Vector2d rotate(Vector2d v, double angle) {
    double cosA = cos(angle);
    double sinA = sin(angle);

    return {v.x() * cosA - v.y() * sinA, v.x() * sinA + v.y() * cosA};
}

static initialData ergodicScatterRing(double innerPhase, double shootAngle) {
    double v12 = sqrt(G * (M1 + M2) / R12);

    initialData circularInit{{M1, {0, 0}, {0, 0}}, {M2, rotate({0, R12}, innerPhase), rotate({-v12, 0}, innerPhase)}};
    auto centered = Bodyfold::transformToCOMSystem(circularInit);
    auto &b1 = centered[0];
    auto &b2 = centered[1];

    initialData init{b1, b2, {M3, {R12_3, 0}, toCartesian(V3, shootAngle)}};
    init = Bodyfold::transformToCOMSystem(init);

    return init;
}




mode_t mode = 0666 | S_IRWXU | S_IRWXG | S_IRWXO;

const double MAX_ENERGY_DEVIATION = pow(10, -5); //6?-----------------------------

int main() {

    string unixTime = to_string(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count());
    string path = PATHSTART + ("_" + unixTime + "/");
    mkdir(path.c_str(), mode);

    auto* systemsLeft = static_cast<InterProcessCounter*>(mmap(nullptr, sizeof(InterProcessCounter), PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0));

    // Initialize the counter to 0
    systemsLeft->counter = PHASESAMPLE;

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

    ofstream systemResults(path + ID + string("_Results.txt"), std::ios::app);


    int threadCounter = 0;

    int T = pow(10, 5);

    int powStart = -3;
    int powJump = -1;
    int powOver = -8;

    while (true) {
        int left;

        pthread_mutex_lock(&systemsLeft->mutex);

        left = systemsLeft->counter;

        if (left <= 0) {
            pthread_mutex_unlock(&systemsLeft->mutex);
            break;
        }

        systemsLeft->counter--;

        if (left % 10 == 0)
            cout << left << endl;

        pthread_mutex_unlock(&systemsLeft->mutex);


        double phase = ((double)(left - 1))/PHASESAMPLE * 2 * M_PI;

        for (int i_shoot = 0; i_shoot < SHOOTSAMPLE; i_shoot++) {

            double shoot = SHOOTMIN + (SHOOTMAX - SHOOTMIN)*(((double)i_shoot)/SHOOTSAMPLE);

            initialData sys = ergodicScatterRing(phase, shoot);

            double energyBefore = Solver::calcQuantities(sys).E();

            vector<tuple<int, double>> energyInfo;
            vector<tuple<double>> timeStopInfo;
            vector<tuple<double>> realTimeInfo;

            tuple<int, tuple<int, int>> status;

            int powNow = powStart;
            while (powNow > powOver) {

                double dt = pow(10, powNow);

                auto start = now();
                Solver solver(T, dt, sys);

                status = solver.run_TSPDT(energyBefore, MAX_ENERGY_DEVIATION);

                auto end = now();

                energyInfo.emplace_back(powNow, solver.EAMax);
                timeStopInfo.emplace_back(solver.time);
                realTimeInfo.emplace_back(toMils(start, end));

                powNow += powJump;
                if (get<0>(status) != -1)
                    break;
            }

            systemResults << "SYSTEM " << threadCounter++ << ": \n";
            systemResults << Bodyfold::toString(sys);
            systemResults << "-------------" << endl;
            systemResults << Solver::calcQuantities(sys).toString();

            systemResults << "PHASE SHOOT: " << doubledoubleToString(phase, shoot) << endl;

            systemResults << "POW MAX ENERGY DIV: " << getPowEnergyInfos(energyInfo) << endl;
            systemResults << "POW END TIME: " << getTimeInfos(timeStopInfo) << endl;

            systemResults << "POW REAL TIME: " << getTimeInfos(realTimeInfo) << endl;
            systemResults << "GOT EA: " << (get<0>(status) != -1) << endl;
            systemResults << "HALTED: " << (get<0>(status) == 1) << endl;
            systemResults << "HALT STATUS: " << intintTupleToString(get<1>(status)) << endl;

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