#include "Solver.h"
#include <chrono>
#include <unistd.h>
#include <iostream>

using namespace std;
using namespace Eigen;

#define M1 17.5
#define M2 15
#define M3 12.5
#define R12 10
#define R12_3 100

void printMils(std::chrono::steady_clock::time_point start, std::chrono::steady_clock::time_point end) {
    cout << (duration_cast<chrono::milliseconds>(end - start)).count() << endl;
}

std::chrono::steady_clock::time_point now() {
    return std::chrono::steady_clock::now();
}

Vector3d rotateAroundAxis(const Vector3d& U, const Vector3d& u, double theta) {
    // Normalize the axis of rotation
    Eigen::Vector3d axis = U.normalized();

    // Rodrigues' rotation formula:
    // u_rot = u * cosθ + (axis × u) * sinθ + axis * (axis • u) * (1 - cosθ)
    double cos_theta = cos(theta);
    double sin_theta = sin(theta);

    return u * cos_theta
         + axis.cross(u) * sin_theta
         + axis * (axis.dot(u)) * (1 - cos_theta);
}

static initialData ergodicScatterRing3D(double innerPhase, double incline) {
    double v12 = sqrt(G * (M1 + M2) / R12);

    initialData circularInit{{M1, {0, 0, 0}, {0, 0, 0}},
        {M2, rotateAroundAxis({0, 0, 1}, {0, R12, 0}, innerPhase), rotateAroundAxis({0, 0, 1}, {-v12, 0, 0}, innerPhase)}};
    auto centered = Bodyfold::transformToCOMSystem(circularInit);
    auto &b1 = centered[0];
    auto &b2 = centered[1];

    initialData init{b1, b2, {M3, rotateAroundAxis({0, 1, 0}, {R12_3, 0, 0}, incline - M_PI/2), {0, 0, 0}}};
    init = Bodyfold::transformToCOMSystem(init);

    return init;
}

int main() {

    string str = R"(
Body #0:
Mass: 1.636771220589853
Position: 40 -60 0
Velocity: 2.3 1 0
Body #1:
Mass: 1.557730835697107
Position: 20 -55 0
Velocity: -3 0 0
Body #2:
Mass: 1.242946611213134
Position: 20 -60 0
Velocity:  0 0 0
)";
    initialData init = Bodyfold::stringToInitialData(str);
    init = Bodyfold::transformToCOMSystem(init);

    init = ergodicScatterRing3D(Bodyfold::rand01()*2*M_PI, Bodyfold::rand01()*M_PI);

    bool RAND = false;
    init = RAND ? Bodyfold::generateRandomCOM() : init;
    Solver solver(1000000, pow(10, -3), init);

    auto start = now();

    auto t = solver.run_TSPDT(Solver::calcQuantities(init).E(), pow(10, -5));
    cout << get<0>(t) << endl;
    cout << get<0>(get<1>(t)) << ", " << get<1>(get<1>(t)) << endl;

    auto end = now();

    //solver.dumpSystemStateString();

    printMils(start, end);

    return 0;


}

/*

Y_far_orbit


Body #0:
Mass: 1.2823666392530861
Position:    8.1756459213489752    9.5129998952328556 -0.072205479601853639
Velocity: -0.22479719603701631  0.42211050370936054   1.2139649847996574
Body #1:
Mass: 0.84446192560253286
Position:   4.0741324711416214 -0.79127064400079306  -3.8562162146892751
Velocity:  -1.0345598340261668 -0.96481564289177013  0.97751484930988519
Body #2:
Mass: 1.6562314736220602
Position: -8.4074150004959058 -6.9621643814929284  2.0220734370449018
Velocity: 0.70154373521669988 0.16510352083493207 -1.4383389686901706
*/




