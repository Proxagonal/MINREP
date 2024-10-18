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

    string str = "Body #0: \n"
"Mass: 1.024875692030475\n"
"Position: 6.396657798148969  1.47812468848513\n"
"Velocity:    2.518647777521959 -0.06233911051852381\n"
"Body #1: \n"
"Mass: 1.177690985156421\n"
"Position: -5.714578597157519 -6.235450107682265\n"
"Velocity: 0.3127158606677405   1.66576510594448\n"
"Body #2: \n"
"Mass: 0.6564131584337223\n"
"Position: 0.2654252250151536  8.879376109855995\n"
"Velocity: -4.493486299133179 -2.891268532998696\n";
    initialData init = Bodyfold::stringToInitialData(str);

    //Solver solver(100000, pow(10, -6), init);

    Solver solver(100000, pow(10, -5));

    auto start = now();

    solver.run();

    auto end = now();

    solver.dumpSystemStateString();

    printMils(start, end);

    return 0;


}

/*SYSTEM STATE:
----------------------------
Body #0:
Mass: 1.242946611213134
Position: -0.002053585436350147     1.030396127310835
Velocity:  2.504552704961396 -3.809217635797198
Acceleration:  1.697952297765992 0.6611026250358759
Body #1:
Mass: 1.557730835697107
Position: -4.037243659383559 -4.719408180617826
Velocity: -2.363441768609585  2.466642679997209
Acceleration: 0.9027176656933865  1.168476996467104
Body #2:
Mass: 1.636771220589853
Position: 3.843842900803226 3.709034102123111
Velocity: 0.3473795341393515 0.5451518069749963
Acceleration: -2.148531910905408 -1.614085024478817
----------------------------
Momentum X: -2.08957908127251e-15
Momentum Y: -9.625771029890929e-17
Total Energy: -14.39373110055243
Kinetic Energy: 22.34750714516762
Potential Energy: -36.74123824572006



insane stabillity
*/






