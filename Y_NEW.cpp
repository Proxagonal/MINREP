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
#include <sys/mman.h>
#include <map>
#include <algorithm>
#include "OrbitalElements.h"
#include <array>
#include "H5Cpp.h"
#include <condition_variable>

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


struct Feature {
    string name;
    hsize_t numOfDoubles;
    size_t offset;
};

vector<Feature> Schema = {
    {"mass_arr",      NUM, 0},
    {"init_pos_arr",  NUM * DIM, 0},
    {"init_vel_arr",  NUM * DIM, 0},
    {"end_inner_OE",  ORB_ELEMENT_NUM, 0},
    {"end_outer_OE",  ORB_ELEMENT_NUM, 0},
    {"phase",         1, 0},
    //{"dts",           POWNUM, 0},
    //{"EAMax_POW",     POWNUM, 0},
    //{"endTime_POW",   POWNUM, 0},
    //{"realTime_POW",  POWNUM, 0},
    {"endTime",         1, 0},
    {"lastExcursionTime",         1, 0},
    {"scrambleNumber", 1, 0},
    {"simStatus",     1, 0},
    {"haltStatus",    1, 0}
};

CompType buildCompType() {
    size_t current_offset = 0;
    // Calculate total size first
    for (auto& f : Schema) {
        f.offset = current_offset;
        current_offset += f.numOfDoubles * sizeof(double);
    }

    CompType mtype(current_offset);
    for (auto& f : Schema) {
        if (f.numOfDoubles == 1) {
            mtype.insertMember(f.name, f.offset, PredType::NATIVE_DOUBLE);
        } else {
            hsize_t d[1] = { f.numOfDoubles };
            ArrayType atype(PredType::NATIVE_DOUBLE, 1, d);
            mtype.insertMember(f.name, f.offset, atype);
        }
    }
    mtype.pack();
    return mtype;
}

class DynamicRecord {
    public:
    vector<double> buffer;

    // Use a static method or global to get the total size once
    DynamicRecord() {
        size_t totalDoubles = 0;
        for (const auto& f : Schema) totalDoubles += f.numOfDoubles;

        // Initialize the whole buffer with NAN immediately
        buffer.assign(totalDoubles, std::numeric_limits<double>::quiet_NaN());
    }

    // Crucial: HDF5 needs the raw pointer to the double array
    const void* data() const { return buffer.data(); }

    void set(const string &var, double value) {
        auto [offset, size] = getFieldInfo(var);
        if (size != 1) throw std::logic_error(var + " is an array, use vector set.");
        buffer[offset] = value;
    }

    void set(const string &var, const vector<double> &values) {
        set(var, values.data(), values.size());
    }

    template <size_t N>
    void set(const string &var, const double (&arr)[N]) {
        set(var, arr, N);
    }

    template <size_t N>
    void set(const string &var, array<double, N> arr) {
        set(var, arr.data(), N);
    }

    void set(const string &var, const double* arr, size_t N) {
        auto [offset, size] = getFieldInfo(var);
        if (N > size) throw std::logic_error("Size mismatch for " + var);

        for (size_t i = 0; i < N; ++i)
            buffer[offset + i] = arr[i];
    }

    private:
    // Helper to find where a name lives in the flat buffer
    pair<size_t, size_t> getFieldInfo(const string& name) {
        size_t current_offset = 0;
        for (const auto& f : Schema) {
            if (f.name == name) return {current_offset, f.numOfDoubles};
            current_offset += f.numOfDoubles;
        }
        throw std::logic_error("Field " + name + " not in Schema");
    }
};

int toMils(std::chrono::steady_clock::time_point start, std::chrono::steady_clock::time_point end) {
    return (duration_cast<chrono::milliseconds>(end - start)).count();
}
std::chrono::steady_clock::time_point now() {
    return std::chrono::steady_clock::now();
}

DynamicRecord runSystem();


queue<DynamicRecord> writeQueue;
mutex queueMtx;
condition_variable cv;
bool allTasksSubmitted = false;

void hdf5Writer(string filePath) {

    H5File file(filePath, H5F_ACC_TRUNC);
    CompType mtype = buildCompType();
    hsize_t dims[1] = {(hsize_t)SAMPLE};
    DataSpace fspace(1, dims);
    DataSet dataset = file.createDataSet(DATANAME, mtype, fspace);

    hsize_t one[1] = {1};
    DataSpace onespace(1, one);

    hsize_t count = 0;
    while (true) {
        DynamicRecord rec;
        {
            unique_lock<mutex> lock(queueMtx);
            cv.wait(lock, [] { return !writeQueue.empty() || allTasksSubmitted; });

            if (writeQueue.empty() && allTasksSubmitted) break;

            rec = std::move(writeQueue.front());
            writeQueue.pop();
        }

        // Target the specific row based on the simulation task index
        hsize_t offset[1] = {count};
        DataSpace currentFSpace = dataset.getSpace();
        currentFSpace.selectHyperslab(H5S_SELECT_SET, one, offset);

        dataset.write(rec.data(), mtype, onespace, currentFSpace);
        count++;
        cout << count << endl;
    }
    file.close();
}

void worker(queue<int>& tasks, mutex& taskMtx) {
    while (true) {
        int taskIndex;
        {
            lock_guard<mutex> lock(taskMtx);
            if (tasks.empty()) return;
            taskIndex = tasks.front();
            tasks.pop();
        }

        // Long-running simulation
        DynamicRecord rec = runSystem();

        // Queue the result for the writer
        {
            lock_guard<mutex> lock(queueMtx);
            writeQueue.push(std::move(rec));
            cv.notify_one();
        }
    }
}

int main() {

    string unixTime = to_string(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count());
    string path = PATHSTART + ("_" + unixTime + "/");
    mkdir(path.c_str(), mode);

    string fullpath = path + string("DATA.h5");

    queue<int> tasks;
    for (int i = 0; i < SAMPLE; ++i) tasks.push(i);
    mutex taskMtx;

    // 3. Start the Single Writer Thread
    thread writer(hdf5Writer, fullpath);

    // 4. Start Worker Threads
    vector<thread> workers;
    for (int i = 0; i < 12; ++i) {
        workers.emplace_back(worker, ref(tasks), ref(taskMtx));
    }

    // 5. Wait for Workers to finish simulations
    for (auto& w : workers) w.join();

    // 6. Signal Writer to wrap up and join
    {
        lock_guard<mutex> lock(queueMtx);
        allTasksSubmitted = true;
        cv.notify_one();
    }
    writer.join();

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

DynamicRecord runSystem() {

    DynamicRecord record;

    double phase = rand01() * 2 * M_PI;

    initialData initConditions = generateSystem(phase);
    Bodyfold initialSystem(initConditions);

    double initialEnergy = initialSystem.sumEnergy();

    vector<double> dts;
    vector<double> energyInfo;
    vector<double> timeStopInfo;
    vector<double> realTimeInfo;

    //vector<tuple<int, double, double, double>> SEEInfo;
    //vector<tuple<int, int, double, double, double, double, double, double>> SOWInfo;
    //vector<tuple<int, double, double, double, double>> SKIPInfo;

    int simStatus;
    int haltStatus;
    double excursionTime = -1;
    double simTime = -1;
    unique_ptr<Bodyfold> finalBF;

    int powNow = powStart;
    while (powNow > powOver) {

        double dt = pow(10, powNow);

        Solver solver(dt, initialSystem);

        Solver::excursionStatus exc_status = Solver::excursionStatus::NONE;
        simStatus = -1;
        haltStatus = Solver::haltStatus::UNDETERMINED;
        double EAMax = 0;
        auto start = now();

        while (true) {

            solver.runIteration();

            if (solver.pass % solver.see_checkPer && exc_status != solver.exc_status) {

                //if (solver.exc_status == Solver::excursionStatus::SUSPECTED)
                //    //cout << "Exc Potential Time: " << solver.time() << endl;
                //if (solver.exc_status == Solver::excursionStatus::RETURNING)
                //    cout << "AARatio: " << solver.AARatio(solver.exc_body) << endl;
                if (exc_status == Solver::excursionStatus::RETURNING && solver.exc_status == Solver::excursionStatus::NONE)
                    excursionTime = solver.time();

                exc_status = (Solver::excursionStatus)solver.exc_status;
            }

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

            if (simStatus != -1)
                break;


        }
        //SEEInfo = solver.SEEInfo;
        //SOWInfo = solver.SOWInfo;
        //SKIPInfo = solver.SKIPInfo;

        //dts.emplace_back(solver.dt);
        //realTimeInfo.emplace_back(toMils(start, now()));
        //energyInfo.emplace_back(EAMax);
        //timeStopInfo.emplace_back(solver.time());


        powNow += powJump;
        if (simStatus == 1) {
            finalBF = make_unique<Bodyfold>(solver.bodyfold);
            simTime = solver.time();
            break;
        }
    }

    if (simStatus == 1 && haltStatus != Solver::haltStatus::DISSOLUTION) {

        auto [innerOE, outerOE] = innerOuterOE(*finalBF, haltStatus);

        record.set("end_inner_OE", innerOE.asDoubleArray());
        record.set("end_outer_OE", outerOE.asDoubleArray());
    }

    record.set("mass_arr", initialSystem.massList);
    record.set("init_pos_arr", (double*) initialSystem.posList.data(), NUM*DIM);
    record.set("init_vel_arr", (double*) initialSystem.velList.data(), NUM*DIM);
    record.set("phase", phase);

    record.set("phase", phase); SIM TIME EXC TIME SCRAMBLE NUM


    //record.set("dts", dts);
    //record.set("EAMax_POW", energyInfo);
    //record.set("endTime_POW", timeStopInfo);
    //record.set("realTime_POW", realTimeInfo);

    record.set("simStatus", simStatus);
    record.set("haltStatus", haltStatus);

    return record;
}