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
#include "OrbitalElements.h"
#include <array>


#define COMPS 12
#define PATHSTART "/mnt/c/Users/eitan/Desktop/DATA/DATAOUT"
//"/home/ethan/Desktop/DATA/DATAOUT"

#define M1 20
#define M2 20
#define M3 20
#define R12 10
#define R12_3 100

#define SAMPLE 10000

#define POWNUM 5

const double MAX_ENERGY_DEVIATION = pow(10, -5);


struct BinHeader {
    char DESCRIPTION[100];
    char date[16];
    int fieldCount;
};

enum class FieldType : uint8_t {
    INT32   = 1,
    INT64   = 2,
    FLOAT32 = 3,
    FLOAT64 = 4,
    FLOAT128 = 5
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
    strncpy(fd.name, name, 31);
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
    make_field("end_inner_OE", FieldType::FLOAT128, 1, ORB_ELEMENT_NUM),
    make_field("end_outer_OE", FieldType::FLOAT128, 1, ORB_ELEMENT_NUM),

    make_field("inclination",  FieldType::FLOAT64, 0),
    make_field("phase",        FieldType::FLOAT64, 0),

    make_field("dts",          FieldType::FLOAT64, 1, POWNUM),
    make_field("EAMax_POW",    FieldType::FLOAT64, 1, POWNUM),
    make_field("endTime_POW",  FieldType::FLOAT64, 1, POWNUM),
    make_field("realTime_POW", FieldType::FLOAT64, 1, POWNUM),
    make_field("simStatus",    FieldType::INT32, 0),
    make_field("haltStatus",   FieldType::INT32, 0)
};



struct Record {

    double massarr[NUM];
    double init_parr[NUM][DIM];
    double init_varr[NUM][DIM];
    long double end_inner_OE[ORB_ELEMENT_NUM];
    long double end_outer_OE[ORB_ELEMENT_NUM];

    double inclination;
    double phase;

    double dts[POWNUM];
    double EAMax_POW[POWNUM];
    double endTime_POW[POWNUM];
    double realTime_POW[POWNUM];

    int simStatus;
    int haltStatus;
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

static tuple<OrbitalElements, OrbitalElements> innerOuterOE(Bodyfold &bodyfold, nat i) {

    nat uno = (i+1) % NUM;
    nat dos = (i+2) % NUM;

    long double m_uno = bodyfold.massList[uno];
    long double m_dos = bodyfold.massList[dos];
    VectorDld innerPosRel = (bodyfold.posList[dos] - bodyfold.posList[uno]).cast<long double>();
    VectorDld innerVelRel = (bodyfold.velList[dos] - bodyfold.velList[uno]).cast<long double>();

    auto innerOE = OrbitalElements::calcOrbitalElements(m_uno, m_dos, innerPosRel, innerVelRel);

    long double innerM = m_uno + m_dos;

    VectorDld innerCOM = (m_uno*bodyfold.posList[uno] + m_dos*bodyfold.posList[dos]).cast<long double>() / innerM;
    VectorDld innerCOMvel = (m_uno*bodyfold.velList[uno] + m_dos*bodyfold.velList[dos]).cast<long double>() / innerM;

    VectorDld outerPosRel = bodyfold.posList[i].cast<long double>() - innerCOM;
    VectorDld outerVelRel = bodyfold.velList[i].cast<long double>() - innerCOMvel;

    auto outerOE = OrbitalElements::calcOrbitalElements(innerM, bodyfold.massList[i], outerPosRel, outerVelRel);

    return {innerOE, outerOE};
}


mode_t mode = 0666 | S_IRWXU | S_IRWXG | S_IRWXO;

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

    ofstream binfile(path + ID + string("_Records.bin"), ios::binary | ios::out);

    if (!binfile) {
        cerr << "Failed to open binary output file\n";
        return 1;
    }

    auto today = floor<chrono::days>(std::chrono::system_clock::now());

    std::chrono::year_month_day ymd{today};

    BinHeader header;
    strncpy(header.DESCRIPTION, "name", 99);

    ostringstream ss;
    ss << unsigned(ymd.day()) << "/" << unsigned(ymd.month()) << "/" << int(ymd.year());

    strncpy(header.date, ss.str().c_str(), 15);
    header.fieldCount = sizeof(fields)/sizeof(FieldDescriptor);

    binfile.write(reinterpret_cast<char*>(&header), sizeof(header));
    binfile.write(reinterpret_cast<char*>(fields), sizeof(fields));



    array<double, POWNUM> dtsNAN;
    for (int i = 0; i < POWNUM; i++)
        dtsNAN[i] = DNAN;
    array<long double, ORB_ELEMENT_NUM> OENAN;
    for (int i = 0; i < ORB_ELEMENT_NUM; i++)
        OENAN[i] = DNAN;


    Record rec{};

    int T = pow(10, 7);

    int powStart = -3;
    int powJump = -1;
    int powOver = powStart - POWNUM;

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

        Bodyfold initFold = Bodyfold{sys};

        copy(initFold.massList.begin(), initFold.massList.end(), rec.massarr);

        for (int i  = 0; i < NUM; i++) {
            double* posdata = initFold.posList[i].data();
            double* veldata = initFold.velList[i].data();

            copy(posdata, posdata + DIM, rec.init_parr[i]);
            copy(veldata, veldata + DIM, rec.init_varr[i]);
        }

        rec.inclination = inc;
        rec.phase = phase;
        copy(dtsNAN.begin(), dtsNAN.end(), rec.dts);
        copy(OENAN.begin(), OENAN.end(), rec.end_inner_OE);
        copy(OENAN.begin(), OENAN.end(), rec.end_outer_OE);


        double initialEnergy = Bodyfold{sys}.sumEnergy();

        vector<double> dts;
        vector<double> energyInfo;
        vector<double> timeStopInfo;
        vector<double> realTimeInfo;

        //vector<tuple<int, double, double, double>> SEEInfo;
        //vector<tuple<int, int, double, double, double, double, double, double>> SOWInfo;
        //vector<tuple<int, double, double, double, double>> SKIPInfo;

        int simStatus = -1;
        int haltStatus = Solver::haltStatus::UNDETERMINED;

        int powNow = powStart;
        while (powNow > powOver) {

            double dt = pow(10, powNow);

            Solver solver(dt, sys);

            Solver::excursionStatus exc_status = Solver::excursionStatus::NONE;
            double EAMax = 0;
            auto start = now();
            while (true) {

                solver.runIteration();

                /*if (solver.pass % solver.see_checkPer && exc_status != solver.exc_status) {

                    if (solver.exc_status == Solver::excursionStatus::SUSPECTED)
                        cout << "Exc Potential Time: " << solver.time() << endl;
                    if (solver.exc_status == Solver::excursionStatus::RETURNING)
                        cout << "AARatio: " << solver.AARatio(solver.exc_body) << endl;
                    if (solver.exc_status == Solver::excursionStatus::NONE && exc_status == Solver::excursionStatus::RETURNING)
                        cout << "EXC Done at: " << solver.time() << endl;

                    exc_status = (Solver::excursionStatus)solver.exc_status;
                }*/

                if (solver.pass % solver.crossingTimePasses == 0) {
                    if (solver.haltCheck() != Solver::haltStatus::UNDETERMINED) {
                        simStatus = 1;
                        haltStatus = solver.haltCheck();
                    }
                    if (solver.dt * solver.pass > T)
                        simStatus = 2;
                }

                if (solver.pass % (solver.crossingTimePasses/100) == 0) {
                    double EA = abs((solver.bodyfold.sumEnergy() - initialEnergy)/initialEnergy);
                    EAMax = max(EA, EAMax);
                    if (EAMax > MAX_ENERGY_DEVIATION)
                        simStatus = 0;
                }

                if (simStatus != -1) {
                    energyInfo.emplace_back(EAMax);
                    break;
                }

            }
            //SEEInfo = solver.SEEInfo;
            //SOWInfo = solver.SOWInfo;
            //SKIPInfo = solver.SKIPInfo;

            dts.emplace_back(solver.dt);
            realTimeInfo.emplace_back(toMils(start, now()));
            energyInfo.emplace_back(EAMax);
            timeStopInfo.emplace_back(solver.time());


            powNow += powJump;
            if (simStatus == 1) {
                if (haltStatus != Solver::haltStatus::DISSOLUTION) {

                    auto [innerOE, outerOE] = innerOuterOE(solver.bodyfold, haltStatus);
                    auto innerArr = innerOE.asArray();
                    auto outerArr = outerOE.asArray();

                    copy(innerArr.begin(), innerArr.end(), rec.end_inner_OE);
                    copy(outerArr.begin(), outerArr.end(), rec.end_outer_OE);
                }

                break;

            }
        }

        copy(dts.begin(), dts.end(), rec.dts);
        copy(energyInfo.begin(), energyInfo.end(), rec.EAMax_POW);
        copy(timeStopInfo.begin(), timeStopInfo.end(), rec.endTime_POW);
        copy(realTimeInfo.begin(), realTimeInfo.end(), rec.realTime_POW);

        rec.simStatus = simStatus;
        rec.haltStatus = haltStatus;

        binfile.write(reinterpret_cast<char*>(&rec), sizeof(rec));
        binfile.flush();
    }


    if (pid == 0)
        exit(0);

    int status;
    for (int i = 0; i < COMPS - 1; i++) {
        wait(&status);
    }

    return 0;

}