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
#include <ctime>

/*

Press the hammer icon         -                  -                   -                               -                   ^^^

then do CTRL + ALT + T, and paste:

sudo nice -n -20 /home/ethan/CLionProjects/MINREP/cmake-build-release/MINREP

then input password: 1248

*/

#define FOLDERPATH "/home/ethan/Desktop/DATA/"
#define SAMPLE 10000
#define COMPS 100
#define DATANAME "SIMULATION"
#define POWSTART -3
#define POWOVER -8

const double MAX_ENERGY_DEVIATION = pow(10, -5);

using namespace std;
using namespace Eigen;
using namespace H5;

mode_t mode = 0666 | S_IRWXU | S_IRWXG | S_IRWXO;

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

    {"end_inner_AngMom", 1, 0},
    {"end_outer_AngMom", 1, 0},

    {"endTime",         1, 0},
    {"lastExcursionEnd",         1, 0},
    {"totalExcursionTime",         1, 0},
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

using excStatus = Solver::excursionStatus;
using haltStatus = Solver::haltStatus;

enum simStatus {
    RUNNING = 0,
    WELL_ENDED = 1,
    NOT_ACCURATE = 2,
    TIMED_OUT = 3
};

int toMils(std::chrono::steady_clock::time_point start, std::chrono::steady_clock::time_point end) {
    return (duration_cast<chrono::milliseconds>(end - start)).count();
}
std::chrono::steady_clock::time_point now() {
    return std::chrono::steady_clock::now();
}

DynamicRecord runSystem(int index);


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
        if (count % 100 == 0)
            cout << count << endl;
    }
    file.close();
    chmod(filePath.c_str(), 0666);
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
        DynamicRecord rec = runSystem(taskIndex);

        // Queue the result for the writer
        {
            lock_guard<mutex> lock(queueMtx);
            writeQueue.push(std::move(rec));
            cv.notify_one();
        }
    }
}

int main() {

    auto time = std::chrono::system_clock::now();

    std::time_t time_c = std::chrono::system_clock::to_time_t(time);

    std::tm now_tm = *std::localtime(&time_c);

    stringstream ss;
    ss << put_time(&now_tm, "%H;%M;%S@%d-%m-%Y");
    string path = FOLDERPATH + ("OUT_" + ss.str() + "/");
    mkdir(path.c_str(), 0777);
    chmod(path.c_str(), 0777);

    string fullpath = path + string("DATA.h5");

    queue<int> tasks;
    for (int i = 0; i < SAMPLE; ++i) tasks.push(i);
    mutex taskMtx;

    // 3. Start the Single Writer Thread
    thread writer(hdf5Writer, fullpath);

    // 4. Start Worker Threads
    vector<thread> workers;
    for (int i = 0; i < COMPS; ++i) {
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

static tuple<VectorAngd, VectorAngd> innerOuterAngularMomentum(const Bodyfold &bodyfold, nat i) {

    VectorDd zero = VectorDd::Zero();

    nat uno = (i+1)%NUM;
    nat dos = (i+2)%NUM;

    double m_uno = bodyfold.massList[uno];
    double m_dos = bodyfold.massList[dos];

    Bodyfold tempbf({{m_uno, bodyfold.posList[uno], bodyfold.velList[uno]},
                            {m_dos, bodyfold.posList[dos], bodyfold.velList[dos]},
                            {0, zero, zero}});

    VectorAngd innerAng = tempbf.sumAngularMomentum();

    VectorDd innerCOM = (m_uno * bodyfold.posList[uno] + m_dos * bodyfold.posList[dos]) / (m_uno + m_dos);
    VectorDd innerCOMVel = (m_uno * bodyfold.velList[uno] + m_dos * bodyfold.velList[dos]) / (m_uno + m_dos);

    Bodyfold tempbf2({{bodyfold.massList[i], bodyfold.posList[i], bodyfold.velList[i]},
                        {m_uno + m_dos, innerCOM, innerCOMVel},
                        {0, zero, zero}});

    VectorAngd outerAng = tempbf2.sumAngularMomentum();

    return {innerAng, outerAng};
}



double scrambleRatioSquared = pow(0.33, 2);
int T = pow(10, 7);

initialData generateSystem(double phase) { //(double phase, double inclination)

    return Solver::ergodicScatterRing2D({17.5, 15, 12.5}, 10, 100, phase); //LAST MASS IS BULLET
    //return Solver::ergodicScatterRing3D({17.5, 15, 12.5}, 10, 100, phase, inclination); //LAST MASS IS BULLET

}


tuple<simStatus, haltStatus> calculateStatuses(Solver& solver, double EA) {

    if (EA > MAX_ENERGY_DEVIATION)
        return {simStatus::NOT_ACCURATE, haltStatus::UNDETERMINED};

    if (solver.dt * solver.pass > T)
        return {simStatus::TIMED_OUT, haltStatus::UNDETERMINED};

    haltStatus stat = (haltStatus)solver.haltCheck();
    if (stat != haltStatus::UNDETERMINED)
        return {simStatus::WELL_ENDED, stat};

    return {simStatus::RUNNING, haltStatus::UNDETERMINED};
}


DynamicRecord runSystem(int index) {

    DynamicRecord record;

    double phase = rand01() * 2 * M_PI;
    initialData initConditions = generateSystem(phase);


    //double inclination = rand01()*M_PI; //3D
    //initialData initConditions = generateSystem(phase, inclination); //3D
    Bodyfold initialSystem(initConditions);

    double initialEnergy = initialSystem.sumEnergy();


    simStatus simStat;
    haltStatus haltStat; // see Solver::haltStatus:: for all statuses
    double lastExcursionEnd;
    double totalExcursionTime;
    double simTime;
    int scrambleNumber;
    unique_ptr<Bodyfold> finalBF;

    for (int powNow = POWSTART; powNow > POWOVER; powNow--) {

        double dt = pow(10, powNow);

        Solver solver(dt, initialSystem);

        simStat = simStatus::RUNNING;
        haltStat = haltStatus::UNDETERMINED;
        lastExcursionEnd = -1;
        totalExcursionTime = 0;
        scrambleNumber = 0;


        double EAMax = 0;
        double excStartTime = -1;
        Solver::excursionStatus exc_status = Solver::excursionStatus::NONE;
        bool scrambleStatus = false;

        while (true) {

            solver.runIteration();

            if (solver.pass % solver.see_checkPer) {

                if (exc_status != solver.exc_status) {
                    if (solver.exc_status == Solver::excursionStatus::SUSPECTED)
                        excStartTime = solver.time();

                    if (exc_status == Solver::excursionStatus::RETURNING && solver.exc_status == Solver::excursionStatus::NONE) {
                        lastExcursionEnd = solver.time();
                        totalExcursionTime += lastExcursionEnd - excStartTime;
                    }
                    exc_status = (Solver::excursionStatus)solver.exc_status;
                }

                double minDistSquared = numeric_limits<double>::infinity();
                double maxDistSquared = 0;
                for (nat i = 0; i < NUM; i++) {
                    double distsquare = (solver.bodyfold.posList[i] - solver.bodyfold.posList[(i+1)%NUM]).squaredNorm();
                    minDistSquared = min(minDistSquared, distsquare);
                    maxDistSquared = max(maxDistSquared, distsquare);
                }

                bool testIfScramble = (minDistSquared > scrambleRatioSquared * maxDistSquared);
                if (testIfScramble != scrambleStatus) {
                    if (testIfScramble)
                        scrambleNumber += 1;
                    scrambleStatus = testIfScramble;
                }

            }

            if (solver.pass % solver.crossingTimePasses == 0) {

                double EA = abs((solver.bodyfold.sumEnergy() - initialEnergy)/initialEnergy);
                EAMax = max(EA, EAMax);

                tie(simStat, haltStat) = calculateStatuses(solver, EAMax);

                if (simStat != simStatus::RUNNING)
                    break;
            }
        }

        if (simStat == simStatus::WELL_ENDED) {
            finalBF = make_unique<Bodyfold>(solver.bodyfold);
            simTime = (excStartTime != -1 ? excStartTime : solver.time());
            break;
        }
    }

    if (simStat == simStatus::WELL_ENDED && haltStat != haltStatus::DISSOLUTION) {

        auto [innerOE, outerOE] = innerOuterOE(*finalBF, haltStat);

        record.set("end_inner_OE", innerOE.asDoubleArray());
        record.set("end_outer_OE", outerOE.asDoubleArray());

        auto [innerAngMom, outerAngMom] = innerOuterAngularMomentum(*finalBF, haltStat);

        record.set("end_inner_AngMom", innerAngMom.data(), 1);
        record.set("end_outer_AngMom", outerAngMom.data(), 1);
    }

    record.set("mass_arr", initialSystem.massList);
    record.set("init_pos_arr", (double*) initialSystem.posList.data(), NUM*DIM);
    record.set("init_vel_arr", (double*) initialSystem.velList.data(), NUM*DIM);
    record.set("phase", phase);

    record.set("phase", phase);
    record.set("endTime", simTime);
    record.set("lastExcursionEnd", lastExcursionEnd);
    record.set("totalExcursionTime", totalExcursionTime);
    record.set("scrambleNumber", scrambleNumber);

    record.set("simStatus", simStat);
    record.set("haltStatus", haltStat);

    return record;
}