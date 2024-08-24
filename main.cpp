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


    string str = "Body #0:\n"
                 "Mass: 1.292080099799268\n"
                 "Position: 1.599318694671322 5.553392826262194\n"
                 "Velocity:  -2.44163630205618 -3.606830431321515\n"
                 "Body #1:\n"
                 "Mass: 1.470712956397409\n"
                 "Position:  3.952350623389272 -3.870711661021517\n"
                 "Velocity: -0.756039528894016 0.8534780761757288\n"
                 "Body #2:\n"
                 "Mass: 1.182641327572333\n"
                 "Position: -6.662392853156769 -1.253738164116226\n"
                 "Velocity: 3.607777529883693 2.879226761014655\n";
    initialData init = Bodyfold::stringToInitialData(str);

    Solver solver(10000, pow(10, -5), init);

    //Solver solver(10000, pow(10, -5));

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






