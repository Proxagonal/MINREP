#include "Solver.h"
#include <chrono>
#include <unistd.h>
#include <iostream>

using namespace std;
using namespace Eigen;

void printMils(std::chrono::steady_clock::time_point start, std::chrono::steady_clock::time_point end) {
    cout << (duration_cast<chrono::milliseconds>(end - start)).count() << endl;
}

std::chrono::steady_clock::time_point now() {
    return std::chrono::steady_clock::now();
}

int main() {


    //string str = "Body #0: \n"
    //             "Mass: 0.7588357702995112\n"
    //             "Position:   12.45607208072307 -0.4953535113882213\n"
    //             "Velocity:  2.022242290467707 0.9581873143045239\n"
    //             "Body #1: \n"
    //             "Mass: 1.020780501132239\n"
    //             "Position:  2.451299558843896 -0.963779784221944\n"
    //             "Velocity: -3.873933021570023 -2.228719653525653\n"
    //             "Body #2: \n"
    //             "Mass: 0.9552889889103799\n"
    //             "Position: -12.51385913912709  1.423338476930359\n"
    //             "Velocity:  2.53314497809907 1.620375377542334";
    //initialData init = Bodyfold::stringToInitialData(str);

    Solver solver(100, pow(10, -6));

    //Solver solver(150, pow(10, -6), init);

    auto start = now();

    solver.run();

    auto end = now();

    solver.dumpSystemStateString();

    printMils(start, end);

    return 0;


}






