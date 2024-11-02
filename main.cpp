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

    string str = R"(Body #0:
Mass: 0.7694637043255976
Position:  14.8128184998246 9.645406071895257
Velocity: -0.7639044901223144   1.858317864613163
Body #1:
Mass: 0.9284925344733082
Position: -0.4409369359841637 -0.7505411273355911
Velocity:   2.35534821712638 -1.082280055878264
Body #2:
Mass: 1.369495421772004
Position: -8.023772381016652 -4.910507874175599
Velocity:  -1.167675650129055 -0.3103472922000856)";
    initialData init = Bodyfold::stringToInitialData(str);

    Solver solver(100000, pow(10, -3), init); //000

    //Solver solver(100000, pow(10, -5)/2);

    auto start = now();

    solver.run();

    auto end = now();

    solver.dumpSystemStateString();

    printMils(start, end);

    return 0;


}

/*
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

insane stabillity
*/

/*
string str = "Body #0\n"
"Mass: 1.538867966551895\n"
"Position: -11.66812151143027 0.8299329097457022\n"
"Velocity: 2.321785206421383 -0.5456961395856064\n"
"Body #1: \n"
"Mass: 1.777497022581172\n"
"Position: 7.479134929692112 -0.272405310121516\n"
"Velocity: 1.523257089996259 -0.3901672367415596\n"
"Body #2: \n"
"Mass: 0.766268940166189\n"
"Position: 6.083449439766614 -1.034829287673587\n"
"Velocity: -8.196216096280557 2.00096249492199\n";

breaks escape check
*/

/*
string str = "Body #0:\n"
"Mass: 1.390725509697576\n"
"Position: 6.609606267054577 -2.982757036320367\n"
"Velocity: -2.898762591259914 -3.721404424492462\n"
"Body #1:\n"
"Mass: 1.527654750445846\n"
"Position: 1.449709406287688 0.4392421898296379\n"
"Velocity: 2.592529431038656 0.7475692027890037\n"
"Body #2:\n"
"Mass: 1.52205015532158\n"
"Position: -7.494367689559578 2.284540932894901\n"
"Velocity: 0.04657742780516827 2.649994421677434\n";

breaks escape check a lot (at pow -3)
 */





