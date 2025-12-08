#include "Solver.h"
#include <chrono>
#include <unistd.h>
#include <iostream>

using namespace std;
using namespace Eigen;

#define M1 20
#define M2 15
#define M3 10
#define R12 10
#define R12_3 100

void printMils(std::chrono::steady_clock::time_point start, std::chrono::steady_clock::time_point end) {
    cout << (duration_cast<chrono::milliseconds>(end - start)).count() << endl;
}

std::chrono::steady_clock::time_point now() {
    return std::chrono::steady_clock::now();
}

static double rand01() {

    static std::random_device rd;
    static std::mt19937_64 gen(rd());
    static std::uniform_real_distribution<double> dist{0, 1};

    return dist(gen);
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


#if DIM == 3
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
#endif

inline static Vector2d rotate(Vector2d v, double angle) {
    double cosA = cos(angle);
    double sinA = sin(angle);

    return {v.x() * cosA - v.y() * sinA, v.x() * sinA + v.y() * cosA};
}

static initialData ergodicScatterRing(double innerPhase) {
    double v12 = sqrt(G * (M1 + M2) / R12);

    initialData circularInit{{M1, {0, 0}, {0, 0}},
        {M2, rotate({0, R12}, innerPhase), rotate({-v12, 0}, innerPhase)}};
    auto centered = Bodyfold::transformToCOMSystem(circularInit);
    auto &b1 = centered[0];
    auto &b2 = centered[1];

    initialData init{b1, b2, {M3, {R12_3, 0}, {-1, -1}}};
    init = Bodyfold::transformToCOMSystem(init);

    return init;
}

int main() {

    string str = R"(
Body #0:
Mass: 17.5
Position: -0.14931119672550497  -32.367669049057803
Velocity:   1.6044167284905331 -0.55078791773074187
Body #1:
Mass: 15
Position:  0.1741963961797558 -22.422904627951084
Velocity: -1.8718195165722886 0.64258590401919891
Body #2:
Mass: 12.5
Position: -2.4671622769447924e-18      72.222222222222229
Velocity: -2.4671622769447922e-17 -1.2335811384723961e-17

)";

    initialData init = Bodyfold::stringToInitialData(str);
    init = Bodyfold::transformToCOMSystem(init);

    //init = ergodicScatterRing(2*M_PI*rand01());

    cout << "Energy: " << Solver::calcQuantities(init).E() << endl;
    cout << "Ang: " << Solver::calcQuantities(init).angMom.transpose() << endl;


    bool RAND = false;
    init = RAND ? Bodyfold::generateRandomCOM() : init;
    Solver solver(1000000, pow(10, -4), init);

    auto start = now();

    solver.run();

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




