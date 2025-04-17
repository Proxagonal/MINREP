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


#define SAMPLE 80000
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

struct InterProcessCounterCRList {
    int counter;
    tuple<double, double> exclude[3000];
    int excCount;
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

vector<tuple<double, double>> inputExcludes(vector<tuple<double, double>> &exampleList) {
    vector<tuple<double, double>> data;
    string line;
    double temp;

    cout << "EXCLUDE?" << endl;

    // Part 1: Input multiple list pairs
    while (true) {
        std::vector<double> list1, list2;

        // First list
        while (true) {
            std::cout << "Enter first list (or blank to move on to (a, b) pairs):\n";
            std::getline(std::cin, line);
            if (line.empty()) break;

            std::istringstream iss1(line);
            bool valid = true;
            while (iss1 >> temp) {
                list1.push_back(temp);
            }

            if (!iss1.eof()) {
                std::cerr << "Invalid input. Please enter only numbers separated by spaces.\n";
                list1.clear();
            }

            if (!list1.empty()) break;
        }

        if (list1.empty()) break;  // move on to individual pairs

        // Second list
        while (true) {
            std::cout << "Enter second list:\n";
            std::getline(std::cin, line);
            std::istringstream iss2(line);
            bool valid = true;
            while (iss2 >> temp) {
                list2.push_back(temp);
            }

            if (!iss2.eof()) {
                std::cerr << "Invalid input. Please enter only numbers separated by spaces.\n";
                list2.clear();
                continue;
            }

            if (list2.empty()) {
                std::cerr << "Second list cannot be empty.\n";
                continue;
            }

            break;
        }

        // Add all pairs from cross product

        if (list1[0] == -1 && list2[0] == -1) {

            if (!(list1.size() == 3 && list2.size() == 3)) {
                cerr << "Invalid ranges.\n";
                continue;
            }


            for (auto &[x, y] : exampleList) {
                if (list1[1] <= x && x <= list1[2]+0.1 && list2[1] <= y && y <= list2[2]+0.1)
                    data.emplace_back(x, y);
            }
            continue;
        }


        for (double x : list1) {
            for (double y : list2) {
                data.emplace_back(x, y);
            }
        }
    }

    // Part 2: Input individual (a, b) pairs
    std::cout << "Enter (a b) pairs one per line. Empty line to finish:\n";
    while (true) {
        std::getline(std::cin, line);
        if (line.empty()) break;

        std::istringstream iss(line);
        double a, b;
        if (iss >> a >> b && iss.eof()) {
            data.emplace_back(a, b);
        } else {
            std::cerr << "Invalid input. Please enter exactly two numbers separated by a space.\n";
        }
    }

    return data;
}

vector<tuple<double, double>> closeEnough(vector<tuple<double, double>> close, vector<tuple<double, double>> &real) {

    vector<tuple<double, double>> actuals;

    for (auto &[c, R] : close)
        for (auto &[cr, Rr] : real)
            if (abs(c - cr) < 0.1 && abs(R - Rr) < 0.1) {
                actuals.emplace_back(cr, Rr);
                cout << "(" << cr << ", " << Rr << ")" << ", ";
            }

    if (actuals.size() !=0)
        cout << endl;

    return actuals;
}



mode_t mode = 0666 | S_IRWXU | S_IRWXG | S_IRWXO;

const double MAX_ENERGY_DEVIATION = pow(10, -5); //6?-----------------------------

int main() {

    vector<int> STOPS = {};//{0, 5, 20, 100, 1000, 10000, 40000, 80000};



    string unixTime = to_string(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count());
    string path = PATHSTART + ("_" + unixTime + "/");
    mkdir(path.c_str(), mode);

    auto* systemsLeft = static_cast<InterProcessCounterCRList*>(mmap(nullptr, sizeof(InterProcessCounterCRList), PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0));

    // Initialize the counter to 0
    systemsLeft->counter = SAMPLE;
    systemsLeft->exclude;
    systemsLeft->excCount = 0;

    // Initialize the mutex with the attribute to allow process sharing
    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);  // Set the mutex as process-shared
    pthread_mutex_init(&systemsLeft->mutex, &attr);


    auto* workers = static_cast<InterProcessCounter*>(mmap(nullptr, sizeof(InterProcessCounter), PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0));

    // Initialize the counter to 0
    workers->counter = 0;

    // Initialize the mutex with the attribute to allow process sharing
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);  // Set the mutex as process-shared
    pthread_mutex_init(&workers->mutex, &attr);


    //vector<double> cv = {2.3, 2.5, 2.8, sqrt(10)};
//
    //vector<tuple<double, double>> CRList;
//
    //for (double c : cv)
    //    for (double R : expArray(10, 0, 2.6, 0.2))
    //        CRList.emplace_back(c, R);
    //printvec(CRList);

    vector<tuple<double, double>> CRList = {{-1.0, -1.0}, {2.8, pow(10, 2.2)}};

    for (auto &[c, R] : CRList) {

        if (!inRange(c, R))
            continue;

        string RcPath = path + toString(c, R) + "/";
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


    std::map<tuple<double, double>, int> counters;
    for (auto &[c, R] : CRList) {

        if (!inRange(c, R))
            continue;

        counters[tuple<double, double>(c, R)] = 0;
    }

    string ID = to_string(index);

    int T = 40;

    int powStart = -3;
    int powJump = -1;
    int powOver = -8;//-8;

    while (true) {
        int left;
        bool isSuccess;
        vector<tuple<double, double>> exclude;

        pthread_mutex_lock(&systemsLeft->mutex);

        left = systemsLeft->counter;

        if (left <= 0) {
            pthread_mutex_unlock(&systemsLeft->mutex);
            break;
        }

        if (isIntIn(SAMPLE - left, STOPS)) {
            cout << "LEFT IS " << left << endl;
            cout << "Waiting for all to finish\n";

            while (true) {
                pthread_mutex_lock(&workers->mutex);
                if (workers->counter == 0) {
                    pthread_mutex_unlock(&workers->mutex);
                    break;
                }
                pthread_mutex_unlock(&workers->mutex);
                usleep(1000000);
            }

            vector<tuple<double, double>> add = closeEnough(inputExcludes(CRList), CRList);

            for (auto &[c, R] : add)
                systemsLeft->exclude[systemsLeft->excCount++] = make_tuple(c, R);

        }

        for (int i = 0; i < systemsLeft->excCount; i++)
            exclude.emplace_back(systemsLeft->exclude[i]);

        systemsLeft->counter--;

        if (left % 10 == 0)
            cout << left << endl;

        pthread_mutex_unlock(&systemsLeft->mutex);

        pthread_mutex_lock(&workers->mutex);
        workers->counter++;
        pthread_mutex_unlock(&workers->mutex);


        initialData rando = Bodyfold::generateRandomCOM();

        double energyBefore = Solver::calcQuantities(rando).E();

        for (auto &[c, R] : CRList) {

            if (!inRange(c, R, exclude))
                continue;

            string RcPath = path + toString(c, R) + "/";

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

        pthread_mutex_lock(&workers->mutex);
        workers->counter--;
        pthread_mutex_unlock(&workers->mutex);
    }


    if (pid == 0)
        exit(0);

    int status;
    for (int i = 0; i < COMPS - 1; i++) {
        wait(&status);
    }

    return 0;

}