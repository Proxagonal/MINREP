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

    string str = R"(
Body #0:
Mass: 1
Position: 13.500856459822167 -7.6159980199110251
Velocity:   -1.091455422073466 0.28662177149205276
Body #1:
Mass: 1
Position: -13.500856459822167 7.6159980199110251
Velocity:   1.091455422073466 -0.28662177149205276
Body #2:
Mass: 0
Position: 100 100
Velocity: 0 0
)";
    initialData init = Bodyfold::stringToInitialData(str);
    init = Bodyfold::transformToCOMSystem(init);

    Solver solver(4000, pow(10, -4), init);

    auto start = now();

    solver.run();

    auto end = now();

    solver.dumpSystemStateString();

    printMils(start, end);

    return 0;


}

/*
unit parabolic:
Body #0:
Mass: 1
Position: -1 0
Velocity: 0 4.44288293815836624701588
Body #1:s
Mass: 1
Position: 1 0
Velocity:  0 -4.44288293815836624701588
Body #2:
Mass: 0
Position: 100 100
Velocity:   0 0
*/

/*

Running unit parabolic for 10 sec:

Body #0:
Mass: 1
Position: 13.500856459822167 7.6159980199110251
Velocity:   1.091455422073466 0.28662177149205276
Acceleration: -0.035776181185701708 -0.020181780754513053
Body #1:
Mass: 1
Position: -13.500856459822167 -7.6159980199110251
Velocity:   -1.091455422073466 -0.28662177149205276
Acceleration: 0.035776181185701708 0.020181780754513053
Body #2:
Mass: 0
Position: 99.859336576578798 99.859071048498009
Velocity: -0.028258843399662576 -0.028408575010045008
Acceleration: -0.0028639259980317838 -0.0029181219440925533

*/


