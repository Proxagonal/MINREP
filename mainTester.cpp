#include "Solver.h"
#include "Bodyfold.h"
#include <chrono>
#include <fstream>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/wait.h>

#define SAMPLE 300
#define COMPS 12
#define PATHSTART "/mnt/c/Users/eitan/Desktop/DATA/DATAOUT"

using namespace std;
using namespace Eigen;

constexpr static int ALLDIGITS = std::numeric_limits<double>::max_digits10 - 1;


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

string vectorIntIntTupleToString(vector<tuple<int, int>> &vtup) {
    stringstream ss;
    for (tuple<int, int> &tup: vtup)
        ss << intintTupleToString(tup) << " | ";
    return ss.str();
}
string vectorDoubleToString(vector<double> &vdouble) {
    stringstream ss;
    ss.precision(ALLDIGITS);
    for (double &x: vdouble)
        ss << x << " | ";
    return ss.str();
}

mode_t mode = 0666 | S_IRWXU | S_IRWXG | S_IRWXO;

int main() {

    string unixTime = to_string(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count());
    string path = PATHSTART + ("_" + unixTime + "/");
    mkdir(path.c_str(), mode);


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


    ofstream systemInfos(path + ID + string("_initialConditions.txt"));


    vector<initialData> randoms;

    int i;
    for (i = 0; i < SAMPLE/COMPS; i++)
        randoms.emplace_back(Bodyfold::generateRandomCOM());

    i = 0;
    for (auto &rando : randoms) {
        systemInfos << "SYSTEM " << i++ << ": \n";
        systemInfos << Bodyfold::toString(rando);
    }

    int powLim = 8;
    int powStart = 7;
    int T = 50;
    double checkTPeriod = 1;
    double dt = pow(10, -powStart);

    auto start = now();

    for (int pow = powStart; pow < powLim; pow++) {

        if (pid != 0)
            cout << dt << endl;
        ofstream beforeAndAfter(path + ID + "_POW_" + to_string(pow) + ".txt");
        beforeAndAfter.precision(ALLDIGITS);

        i = 0;
        for (auto &rando : randoms) {

            Solver solver(T, dt, checkTPeriod, rando);

            beforeAndAfter << "SYSTEM " << i << ": \n";
            beforeAndAfter << "BEFORE:" << endl;
            beforeAndAfter << solver.quantities().toString();
            auto measure = now();
            solver.run();
            beforeAndAfter << "AFTER:" << endl;
            beforeAndAfter << solver.quantities().toString();
            beforeAndAfter << "^TOTAL TIME: ";
            streamMils(beforeAndAfter, measure, now());
            beforeAndAfter << solver.bodyfold.toString();
            //changingTimes, haltStatuses, unresolved, systemTime
            beforeAndAfter << "^CHANGING TIMES: " << vectorDoubleToString(solver.changingTimes) << endl;
            beforeAndAfter << "^HALTING STATUSES: " << vectorIntIntTupleToString(solver.haltStatuses) << endl;
            beforeAndAfter << "^IS UNRESOLVED: " << solver.unresolved << endl;
            beforeAndAfter << "^IN_UNIVERSE TIME: " << solver.systemTime << endl;

            i++;
        }

        dt = dt/10;
    }

    if (pid == 0)
        exit(0);

    int status;
    for (i = 0; i < COMPS - 1; i++) {
        wait(&status);
    }

    printMils(start, now());

    return 0;

}






