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
#define PATHSTART "/home/ethan/Desktop/DATA/DATAOUT"

#define M1 20
#define M2 20
#define M3 20
#define R12 10
#define R12_3 100

#define SAMPLE 10000

#define POWNUM 5

enum class FieldType : uint8_t {
    INT32   = 1,
    INT64   = 2,
    FLOAT32 = 3,
    FLOAT64 = 4
};

// --- Field descriptor ---
struct FieldDescriptor {
    char name[32];       // null-terminated
    FieldType type;      // type of the field
    uint8_t ndim;        // number of dimensions
    uint32_t shape[4];   // sizes of each dimension
    uint8_t _pad[3];     // padding for alignment
};

// --- Helper to create a field descriptor ---
FieldDescriptor make_field(const char* name, FieldType type, uint8_t ndim,
                           uint32_t s0 = 0, uint32_t s1 = 0,
                           uint32_t s2 = 0, uint32_t s3 = 0) {
    FieldDescriptor fd{};
    std::strncpy(fd.name, name, 31);
    fd.type = type;
    fd.ndim = ndim;
    fd.shape[0] = s0;
    fd.shape[1] = s1;
    fd.shape[2] = s2;
    fd.shape[3] = s3;
    return fd;
}

    // --- List of fields for your Record ---
FieldDescriptor fields[] = {
    make_field("massarr",      FieldType::FLOAT64, 1, NUM),
    make_field("init_parr",    FieldType::FLOAT64, 2, NUM, DIM),
    make_field("init_varr",    FieldType::FLOAT64, 2, NUM, DIM),
    make_field("end_parr",     FieldType::FLOAT64, 2, NUM, DIM),
    make_field("end_varr",     FieldType::FLOAT64, 2, NUM, DIM),

    make_field("inclination",  FieldType::FLOAT64, 0),
    make_field("phase",        FieldType::FLOAT64, 0),

    make_field("dts",    FieldType::FLOAT64, 1, POWNUM),
    make_field("EAMax_POW",    FieldType::FLOAT64, 1, POWNUM),
    make_field("endTime_POW",  FieldType::FLOAT64, 1, POWNUM),
    make_field("realTime_POW", FieldType::FLOAT64, 1, POWNUM)
};



struct Record {

    double massarr[NUM];
    double init_parr[NUM][DIM];
    double init_varr[NUM][DIM];
    double end_parr[NUM][DIM];
    double end_varr[NUM][DIM];

    double inclination;
    double phase;

    double dts[POWNUM];
    double EAMax_POW[POWNUM];
    double endTime_POW[POWNUM];
    double realTime_POW[POWNUM];

    int status[3];
};


using namespace std;
using namespace Eigen;


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


mode_t mode = 0666 | S_IRWXU | S_IRWXG | S_IRWXO;

const double MAX_ENERGY_DEVIATION = pow(10, -5); //6?-----------------------------

int main() {

    string unixTime = to_string(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count());
    string path = PATHSTART + ("_" + unixTime + "/");
    mkdir(path.c_str(), mode);

    auto* systemsLeft = static_cast<InterProcessCounter*>(mmap(nullptr, sizeof(InterProcessCounter), PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0));

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

    ofstream systemResults(path + ID + string("_Results.txt"), std::ios::app);

    int threadCounter = 0;


    Record rec{};

    int T = pow(10, 7);

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


        double phase = rand01() * 2 * M_PI;
        double inc = rand01() * M_PI;

        double b = 0.05;
        initialData sys = Solver::ergodicScatterRing3D_eccentric({M1, M2, M3}, R12, b*R12, R12_3, phase, 0, inc);

        Vector3d nhat = (get<1>(sys.at(0)).cross(get<1>(sys.at(2)))).normalized();


        double energyBefore = Bodyfold{sys}.sumEnergy();

        vector<double> dts;
        vector<double> energyInfo;
        vector<double> timeStopInfo;
        vector<double> realTimeInfo;

        //vector<tuple<int, double, double, double>> SEEInfo;
        //vector<tuple<int, int, double, double, double, double, double, double>> SOWInfo;
        //vector<tuple<int, double, double, double, double>> SKIPInfo;

        tuple<int, tuple<int, int>> status;

        int powNow = powStart;
        while (powNow > powOver) {

            double dt = pow(10, powNow);

            auto start = now();
            Solver solver(T, dt, sys);

            status = solver.run_STATS(energyBefore, MAX_ENERGY_DEVIATION);

            auto end = now();

            energyInfo.emplace_back(solver.EAMax);
            timeStopInfo.emplace_back(solver.time);
            realTimeInfo.emplace_back(toMils(start, end));

            //SEEInfo = solver.SEEInfo;
            //SOWInfo = solver.SOWInfo;
            //SKIPInfo = solver.SKIPInfo;

            powNow += powJump;
            if (get<0>(status) != -1) {
                break;
            }
        }

        //ALSO ORBITAL ELEMENTS

        systemResults << "SYSTEM " << threadCounter++ << ": \n";
        systemResults << Bodyfold::toString(sys);
        systemResults << "-------------" << endl;
        systemResults << Solver::calcQuantities(sys).toString();

        systemResults << "PHASE: " << setprecision(ALLDIGITS) << phase << endl;
        systemResults << "INCLINE: " << inc << endl;
        systemResults << "PLANAR ANGLE: " << finalAngle << endl;

        systemResults << setprecision(DEFAULTDIGITS);

        systemResults << "POW MAX ENERGY DIV: " << printTupleVectorPyramid(energyInfo) << endl;
        systemResults << "POW END TIME: " << printOneDoubleVectorPyramid(timeStopInfo) << endl;

        systemResults << "POW REAL TIME: " << printOneDoubleVectorPyramid(realTimeInfo) << endl;
        systemResults << "GOT EA: " << (get<0>(status) != -1) << endl;
        systemResults << "HALTED: " << (get<0>(status) == 1) << endl;
        systemResults << "INTIME: " << (get<0>(status) != -2) << endl;
        systemResults << "HALT STATUS: " << intintTupleToString(get<1>(status)) << endl;

        systemResults << "SEE INFO: " << printTupleVectorPyramid(SEEInfo) << endl;
        systemResults << "SOW INFO: " << printTupleVectorPyramid(SOWInfo) << endl;
        systemResults << "SKIP INFO: " << printTupleVectorPyramid(SKIPInfo) << endl;


        systemResults.flush();

    }


    if (pid == 0)
        exit(0);

    int status;
    for (int i = 0; i < COMPS - 1; i++) {
        wait(&status);
    }

    return 0;

}