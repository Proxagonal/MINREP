    #include "Solver.h"
    #include "Bodyfold.h"
    #include <chrono>
    #include <fstream>
    #include <unistd.h>
    #include <sys/stat.h>
    #include <sys/wait.h>
    #include <semaphore.h>
    #include <sys/mman.h>

    #define SAMPLE 50000
    #define COMPS 12
    #define PATHSTART "/mnt/c/Users/eitan/Desktop/DATA/DATAOUT"

    using namespace std;
    using namespace Eigen;

    constexpr static int ALLDIGITS = std::numeric_limits<long double>::max_digits10;

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

    std::chrono::steady_clock::time_point now() {
        return std::chrono::steady_clock::now();
    }

    string intintTupleToString(tuple<int, int> tup) {
        int x, y;
        tie(x, y) = tup;
        return "(" + to_string(x) + ", " + to_string(y) + ")";
    }

    string getStatuses(vector<tuple<tuple<int, int>, long double>> &vtup) {
        stringstream ss;
        for (auto &tup: vtup)
            ss << intintTupleToString(get<0>(tup)) << " | ";
        return ss.str();
    }

    string getTimes(vector<tuple<tuple<int, int>, long double>> &vtup) {
        stringstream ss;
        ss.precision(ALLDIGITS);
        for (auto &x: vtup)
            ss << get<1>(x) << " | ";
        return ss.str();
    }

    string getPowEnergyInfos(vector<tuple<int, long double>> &infos) {
        stringstream ss;
        ss.precision(ALLDIGITS);
        for (auto &x: infos)
            ss << "(" << get<0>(x) << ", " << get<1>(x) << ")" << " /\\ ";
        return ss.str();
    }

    string getTimeInfos(vector<tuple<long double>> &infos) {
        stringstream ss;
        ss.precision(ALLDIGITS);
        for (auto &x: infos)
            ss << get<0>(x) << " /\\ ";
        return ss.str();
    }

    long double deviation(long double x, long double xAfter) {
        return abs((x-xAfter)/x);
    }

    mode_t mode = 0666 | S_IRWXU | S_IRWXG | S_IRWXO;

    const long double MAX_ENERGY_DEVIATION = pow(10, -5); //6?-----------------------------

    int main() {

        string unixTime = to_string(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count());
        string path = PATHSTART + ("_" + unixTime + "/");
        mkdir(path.c_str(), mode);

        auto* systemsLeft = static_cast<InterProcessDoubleCounter*>(mmap(nullptr, sizeof(InterProcessDoubleCounter), PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0));

        // Initialize the counter to 0
        systemsLeft->counter = SAMPLE;
        systemsLeft->counter2 = 0;

        vector<InterProcessCounter*> statusCounters;

        for (int i = 0; i < Solver::GNFOC_StatusesAmount; i++) {

            auto* counter = static_cast<InterProcessCounter*>(mmap(nullptr, sizeof(InterProcessCounter), PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0));
            counter->counter = 0;

            statusCounters.emplace_back(counter);
        }

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
        ofstream goodSystemResults(path + ID + string("_EndResults.txt"));

        int iGood = 0;
        bool isLastSuccess = false;
        int finishStatNum = -1;

        int T = 400000;

        int POW = -3;

        while (true) {

            int left, succ;

            pthread_mutex_lock(&systemsLeft->mutex);
            left = systemsLeft->counter;
            succ = systemsLeft->counter2;
            if (left > 0)
                systemsLeft->counter--;
            if (isLastSuccess)
                systemsLeft->counter2++;
            pthread_mutex_unlock(&systemsLeft->mutex);

            if (finishStatNum != -1) {
                pthread_mutex_lock(&statusCounters[finishStatNum]->mutex);
                statusCounters[finishStatNum]->counter++;
                pthread_mutex_unlock(&statusCounters[finishStatNum]->mutex);
            }

            if (left == 0)
                break;

            if (left % 100 == 0) {
                cout << left << ", " << succ << endl;
                int count;
                for (int i = 0; i < Solver::GNFOC_StatusesAmount; i++) {
                    pthread_mutex_lock(&statusCounters[i]->mutex);
                    count = statusCounters[i]->counter;
                    pthread_mutex_unlock(&statusCounters[i]->mutex);

                    cout << Solver::GNFOC_Statuses[i] << ": " << count << endl;
                }
            }


            initialData rando = Bodyfold::generateRandomCOM();

            long double energyBefore = Solver::calcQuantities(rando).E();

            long double dt = pow(10, POW);

            auto start = now();
            Solver solver(T, dt, rando);
            finishStatNum = solver.run_GFNOC(MAX_ENERGY_DEVIATION);
            auto end = now();

            // If good energy conservation
            if (isLastSuccess = (finishStatNum == -1)) {

                goodSystemResults << "SYSTEM " << iGood++ << ": \n";
                goodSystemResults << Bodyfold::toString(rando);
                goodSystemResults << Solver::calcQuantities(rando).toString();

                goodSystemResults << "END SYSTEM:" << endl;
                goodSystemResults << solver.bodyfold.toString();
                goodSystemResults << solver.quantities().toString();

                goodSystemResults << "REAL TIME: ";
                streamMils(goodSystemResults, start, end);
                goodSystemResults << "SIM TIME: " << solver.time << endl;
            }

        }

        goodSystemResults.flush();


        if (pid == 0)
            exit(0);

        int status;
        for (int i = 0; i < COMPS - 1; i++) {
            wait(&status);
        }

        return 0;

    }