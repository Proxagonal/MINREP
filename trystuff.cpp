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

#define M1 20
#define M2 15
#define M3 10
#define R12 10
#define R12_3 100


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


static double rand01() {

    static std::random_device rd;
    static std::mt19937_64 gen(rd());
    static std::uniform_real_distribution<double> dist{0, 1};

    return dist(gen);
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

string printOneDoubleVectorPyramid(vector<double> &infos) {
    stringstream ss;
    ss.precision(ALLDIGITS);

    for (double &x: infos)
        ss << x << " /\\ ";

    return ss.str();
}

template <typename... Args>
string printTupleVectorPyramid(const vector<tuple<Args...>>& vec) {

    stringstream ss;
    ss.precision(ALLDIGITS);

    for (const auto& t : vec) {
        ss << "(";
        size_t count = 0;
        apply([&](const auto&... elems) {
            ((ss << (count++ ? ", " : "") << elems), ...);
        }, t);
        ss << ") /\\ ";
    }

    return ss.str();
}

double deviation(double x, double xAfter) {
    return abs((x-xAfter)/x);
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

//static initialData ergodicScatterRing3D(double innerPhase, double incline) {
//    double v12 = sqrt(G * (M1 + M2) / R12);
//
//    initialData circularInit{{M1, {0, 0, 0}, {0, 0, 0}},
//        {M2, rotateAroundAxis({0, 0, 1}, {0, R12, 0}, innerPhase), rotateAroundAxis({0, 0, 1}, {-v12, 0, 0}, innerPhase)}};
//    auto centered = Bodyfold::transformToCOMSystem(circularInit);
//    auto &b1 = centered[0];
//    auto &b2 = centered[1];
//
//    initialData init{b1, b2, {M3, rotateAroundAxis({0, 1, 0}, {R12_3, 0, 0}, incline - M_PI/2), {0, 0, 0}}};
//    init = Bodyfold::transformToCOMSystem(init);
//
//    return init;
//}

static initialData ergodicScatterRing(double innerPhase) {
    double v12 = sqrt(G * (M1 + M2) / R12);

    initialData circularInit{{M1, {0, 0}, {0, 0}},
        {M2, rotate({0, R12}, innerPhase), rotate({-v12, 0}, innerPhase)}};
    auto centered = Bodyfold::transformToCOMSystem(circularInit);
    auto &b1 = centered[0];
    auto &b2 = centered[1];

    initialData init{b1, b2, {M3, {R12_3, 0}, {0, 0}}};
    init = Bodyfold::transformToCOMSystem(init);

    return init;
}


int main() {

    initialData sys = ergodicScatterRing(0);

    VectorAngd ANG0 = Solver::calcQuantities(sys).angMom;

    double ANGMAX = ANG0.norm();
    double ANGMIN = ANG0.norm();

    double ENE0 = Solver::calcQuantities(sys).E();

    double ENEMAX = ENE0;
    double ENEMIN = ENE0;


    for (int i = 0; i < 1000; i++) {

        initialData sys2 = ergodicScatterRing(2*M_PI*i/1000);

        VectorAngd ANG = Solver::calcQuantities(sys2).angMom;
        double ENE = Solver::calcQuantities(sys2).E();


        ANGMAX = max(ANGMAX, ANG.norm());
        ANGMIN = min(ANGMIN, ANG.norm());

        ENEMAX = max(ENEMAX, ENE);
        ENEMIN = min(ENEMIN, ENE);

    }

    cout << ANGMAX - ANGMIN << endl;
    cout << (ENEMAX - ENEMIN)/ENEMAX << endl;





    return 0;

}