#include <iostream>
#include <vector>
#include <thread>
#include <mutex>
#include <queue>
#include <string>
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
#include "H5Cpp.h"

#define COMPS 12
#define PATHSTART "/home/ethan/Desktop/DATA/DATAOUT"
//"/mnt/c/Users/eitan/Desktop/DATA/DATAOUT"
//"/home/ethan/Desktop/DATA/DATAOUT"
mode_t mode = 0666 | S_IRWXU | S_IRWXG | S_IRWXO;

#define M1 12.5
#define M2 15
#define M3 17.5
#define R12 10
#define R12_3 100

#define SAMPLE 100

#define POWNUM 5

#define DATANAME "name"

const double MAX_ENERGY_DEVIATION = pow(10, -5);


using namespace std;
using namespace Eigen;
using namespace H5;


#pragma pack(push, 1)
struct Record {

    double mass_arr[NUM];
    double init_pos_arr[NUM][DIM];
    double init_vel_arr[NUM][DIM];
    double end_inner_OE[ORB_ELEMENT_NUM] = {DNAN, DNAN, DNAN, DNAN, DNAN, DNAN};
    double end_outer_OE[ORB_ELEMENT_NUM] = {DNAN, DNAN, DNAN, DNAN, DNAN, DNAN};

    double phase;

    double dts[POWNUM] = {DNAN, DNAN, DNAN, DNAN, DNAN};
    double EAMax_POW[POWNUM];
    double endTime_POW[POWNUM];
    double realTime_POW[POWNUM];

    int simStatus;
    int haltStatus;
};
#pragma pack(pop)

struct Feature {
    string name;
    int numOfDoubles;
    size_t offset; // We'll calculate this automatically
};

// This list is now your "Single Source of Truth"
std::vector<Feature> Schema = {
    {"phase",         1, 0},
    {"simStatus",     1, 0},
    {"mass_arr",      NUM, 0},
    {"end_inner_OE",  FeedType::ARRAY_OE, 0}
};


int toMils(std::chrono::steady_clock::time_point start, std::chrono::steady_clock::time_point end) {
    return (duration_cast<chrono::milliseconds>(end - start)).count();
}
std::chrono::steady_clock::time_point now() {
    return std::chrono::steady_clock::now();
}

Record runSystem();

class ThreadPool {
public:
    ThreadPool(int threads, int total_tasks, H5File &file, std::mutex& mtx) {
        // Fill the queue with task indices (0 to 999)
        for (int i = 0; i < total_tasks; ++i) {
            tasks.push(i);
        }

        // Launch 12 worker threads
        for (int i = 0; i < threads; ++i) {
            workers.emplace_back([this, &file, &mtx]() {

                DataType datatype;
                DataSet dataset;
                {
                    lock_guard<mutex> lock(queue_mutex);
                    dataset = file.openDataSet(DATANAME);
                    datatype = dataset.getDataType();
                }



                while (true) {
                    int current_task, left;
                    {
                        // Lock the queue to grab the next task safely
                        lock_guard<mutex> lock(queue_mutex);
                        left = tasks.size();
                        cout << left << endl;
                        if (tasks.empty()) return; // No more work
                        current_task = tasks.front();
                        tasks.pop();
                    }

                    // Perform the calculation
                    Record rec = runSystem();


                    // Safely append to the shared results list
                    {
                        lock_guard<mutex> lock(mtx);
                        // offset: which row to write to
                        // count: how many rows we are writing (just 1)
                        hsize_t offset[1] = { (hsize_t)(SAMPLE - left) };
                        hsize_t count[1] = { 1 };

                        DataSpace fspace = dataset.getSpace();
                        fspace.selectHyperslab(H5S_SELECT_SET, count, offset);
                        dataset.write(&rec, datatype, DataSpace(H5S_SCALAR), fspace);
                        file.flush(H5F_SCOPE_GLOBAL);
                    }
                }
            });
        }
    }

    ~ThreadPool() {
        for (auto& worker : workers) worker.join();
    }

private:
    std::queue<int> tasks;
    std::vector<std::thread> workers;
    std::mutex queue_mutex;
};

int main() {

    // Define dimensions for the arrays
    hsize_t dims_num[1] = {NUM};
    hsize_t dims_num_dim[2] = {NUM, DIM};
    hsize_t dims_orb[1] = {ORB_ELEMENT_NUM};
    hsize_t dims_pow[1] = {POWNUM};

    // Create the HDF5 ArrayTypes
    ArrayType massArrayT(PredType::NATIVE_DOUBLE, 1, dims_num);
    ArrayType posVelArrayT(PredType::NATIVE_DOUBLE, 2, dims_num_dim);
    ArrayType oeArrayT(PredType::NATIVE_DOUBLE, 1, dims_orb);
    ArrayType powArrayT(PredType::NATIVE_DOUBLE, 1, dims_pow);


    CompType rectype(sizeof(Record));

    // 1D Array
    rectype.insertMember("mass_arr", HOFFSET(Record, mass_arr), massArrayT);

    // 2D Arrays
    rectype.insertMember("init_pos_arr", HOFFSET(Record, init_pos_arr), posVelArrayT);
    rectype.insertMember("init_vel_arr", HOFFSET(Record, init_vel_arr), posVelArrayT);

    // Long Double Arrays
    rectype.insertMember("end_inner_OE", HOFFSET(Record, end_inner_OE), oeArrayT);
    rectype.insertMember("end_outer_OE", HOFFSET(Record, end_outer_OE), oeArrayT);

    // Simple Scalars
    rectype.insertMember("phase", HOFFSET(Record, phase), PredType::NATIVE_DOUBLE);

    // POWNUM Arrays
    rectype.insertMember("dts", HOFFSET(Record, dts), powArrayT);
    rectype.insertMember("EAMax_POW", HOFFSET(Record, EAMax_POW), powArrayT);
    rectype.insertMember("endTime_POW", HOFFSET(Record, endTime_POW), powArrayT);
    rectype.insertMember("realTime_POW", HOFFSET(Record, realTime_POW), powArrayT);

    // Integers
    rectype.insertMember("simStatus", HOFFSET(Record, simStatus), PredType::NATIVE_INT);
    rectype.insertMember("haltStatus", HOFFSET(Record, haltStatus), PredType::NATIVE_INT);

    string unixTime = to_string(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count());
    string path = PATHSTART + ("_" + unixTime + "/");
    mkdir(path.c_str(), mode);

    hsize_t datasize[1] = {SAMPLE};

    H5File file(path + string("DATA.h5"), H5F_ACC_TRUNC);
    DataSpace dataspace(1, datasize);
    file.createDataSet(DATANAME, rectype, dataspace);



    std::mutex results_mutex;
    const int num_threads = 12;

    {
        ThreadPool pool(num_threads, SAMPLE, file, results_mutex);
        // Pool destructor joins threads here, ensuring all 1000 are done
    }

    return 0;
}

static tuple<OrbitalElements, OrbitalElements> innerOuterOE(const Bodyfold &bodyfold, nat i) {

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


int b = 0.05;
initialData generateSystem(double phase) {
    return Solver::ergodicScatterRing2D({M1, M2, M3}, R12, R12_3, phase);
}



int T = pow(10, 7);
int powStart = -3;
int powJump = -1;
int powOver = powStart - POWNUM;

Record runSystem() {

    Record rec{};

    double phase = rand01() * 2 * M_PI;

    initialData sys = generateSystem(phase);

    Bodyfold initFold = Bodyfold{sys};

    copy(initFold.massList.begin(), initFold.massList.end(), rec.mass_arr);

    for (int i  = 0; i < NUM; i++) {
        double* posdata = initFold.posList[i].data();
        double* veldata = initFold.velList[i].data();

        copy(posdata, posdata + DIM, rec.init_pos_arr[i]);
        copy(veldata, veldata + DIM, rec.init_vel_arr[i]);
    }

    //std::copy(initFold.posList.data()->data(), initFold.posList.data()->data() + 9, &posdata[0][0]);
    //std::copy(initFold.velList.data()->data(), initFold.velList.data()->data() + 9, &veldata[0][0]);

    rec.phase = phase;


    double initialEnergy = Bodyfold{sys}.sumEnergy();

    vector<double> dts;
    vector<double> energyInfo;
    vector<double> timeStopInfo;
    vector<double> realTimeInfo;

    //vector<tuple<int, double, double, double>> SEEInfo;
    //vector<tuple<int, int, double, double, double, double, double, double>> SOWInfo;
    //vector<tuple<int, double, double, double, double>> SKIPInfo;

    int simStatus;
    int haltStatus;
    unique_ptr<Bodyfold> finalBF;

    int powNow = powStart;
    while (powNow > powOver) {

        double dt = pow(10, powNow);

        Solver solver(dt, sys);

        Solver::excursionStatus exc_status = Solver::excursionStatus::NONE;
        simStatus = -1;
        haltStatus = Solver::haltStatus::UNDETERMINED;
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
            finalBF = make_unique<Bodyfold>(solver.bodyfold);
            break;
        }
    }

    if (simStatus == 1 && haltStatus != Solver::haltStatus::DISSOLUTION) {

        auto [innerOE, outerOE] = innerOuterOE(*finalBF, haltStatus);
        auto innerArr = innerOE.asArray();
        auto outerArr = outerOE.asArray();

        copy(innerArr.begin(), innerArr.end(), rec.end_inner_OE);
        copy(outerArr.begin(), outerArr.end(), rec.end_outer_OE);
    }

    copy(dts.begin(), dts.end(), rec.dts);
    copy(energyInfo.begin(), energyInfo.end(), rec.EAMax_POW);
    copy(timeStopInfo.begin(), timeStopInfo.end(), rec.endTime_POW);
    copy(realTimeInfo.begin(), realTimeInfo.end(), rec.realTime_POW);

    rec.simStatus = simStatus;
    rec.haltStatus = haltStatus;

    return rec;
}