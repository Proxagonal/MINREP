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

inline static Vector2d rotate(Vector2d &v, double angle) {
    double cosA = cos(angle);
    double sinA = sin(angle);

    return {v.x() * cosA - v.y() * sinA, v.x() * sinA + v.y() * cosA};
}

int main() {
    string str = R"(
Body #0:
Mass: 4.7
Position: 14.2 6.7
Velocity: 8.7 1.3
Body #1:
Mass: 5.1
Position: 0 -7
Velocity:  7.4343651138547404722912596830064889 0
Body #2:
Mass: 0
Position: -1000 -1000
Velocity:   0 0
)";
    initialData init = Bodyfold::stringToInitialData(str);

    initialData initalDataRotated;
    Vector2d rp, rv;
    int i = 0;
    for (auto &body : init) {

        if (i == 0) {
            rp = 5*rotate(get<1>(body), M_PI*M_PI/4 + 1);
            rv = rotate(get<2>(body), M_PI*M_PI/4 + exp(M_PI));
        }

        if (i == 1) {
            rp = rp + rotate(get<1>(body), M_PI*M_PI*M_PI*M_PI*101010/4);
            rv = rv + rotate(get<2>(body), M_PI*M_PI*M_PI*M_PI*101010/4);
        }
        if (i == 2) {
            initalDataRotated.emplace_back(body);
            continue;
        }
        i++;
        initalDataRotated.emplace_back(get<0>(body), rp, rv);
    }
    init = initalDataRotated;

    //init = Bodyfold::transformToCOMSystem(init);

    Solver solver(1000, pow(10, -4), init);

    auto start = now();

    //solver.run();
    solver.run_paraoutertest();

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

Para test:

Body #0:
Mass: 1
Position: 13.500856459822167 7.6159980199110251
Velocity:   -1.091455422073466 -0.28662177149205276
Body #1:
Mass: 1
Position: -13.500856459822167 -7.6159980199110251
Velocity:   1.091455422073466 0.28662177149205276
Body #2:
Mass: 0
Position: 100 100
Velocity: 0 0

*/


