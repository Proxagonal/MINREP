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
"Mass: 0.8291072024980477\n"
"Position: -6.964788558943937 -1.411692615041455\n"
"Velocity: -0.7570953006648793 -0.1442201748337979\n"
"Body #1:\n"
"Mass: 0.8106437686511564\n"
"Position: 2.570372625685991  2.49102479623183\n"
"Velocity:   1.683908288518521 -0.4863667489894588\n"
"Body #2:\n"
"Mass: 0.5076444558375927\n"
"Position:  7.270639447608492 -1.672212124086027\n"
"Velocity: -1.452466555735988  1.012212689686195\n";
    initialData init = Bodyfold::stringToInitialData(str);

    Solver solver(100000000, pow(10, -6), init);

    //Solver solver(100000, pow(10, -6));

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
insane stabillity
*/
/*SYSTEM STATE:
----------------------------
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
----------------------------
breaks escape check
*/





