#ifndef MINREP_CONSTRUCTORS_H
#define MINREP_CONSTRUCTORS_H

#include <iostream>
#include <Eigen/Eigen>
#include "Solver.h"


using namespace std;
using namespace Eigen;

constexpr static double massMin = 0.5;
constexpr static double massMax = 2;
constexpr static double systemRadius = 5;
constexpr static double velocityMax = 2.5;

static double rand01() {

    static std::random_device rd;
    static std::mt19937_64 gen(rd());
    static std::uniform_real_distribution<double> dist{0, 1};

    return dist(gen);
}

static double randNormalStandard() {

    static std::random_device rd;
    static std::mt19937_64 gen(rd());
    static std::normal_distribution<double> dist{0, 1};

    return dist(gen);
}

static VectorDd randomOnRadiusD(double r) {

    // According to Box-Mueller. Needs testing.

    VectorDd Z = VectorDd::NullaryExpr([&]() { return randNormalStandard();});

    return r * pow(rand01(), 1.0/DIM) * Z.normalized();
}

// Symmetries: rotation, scale, mass sum, COM, p_COM
initialData Solver::generateRandomNONCOM() {

    initialData list;

    VectorDd pos;
    VectorDd vel;
    double mass;

    for (int i = 0; i < NUM; i++) {
        pos = randomOnRadiusD(systemRadius);
        vel = randomOnRadiusD(velocityMax);
        mass = massMin + (massMax - massMin) * rand01();
        list.emplace_back(mass, pos, vel);
    }

    return list;
}

// Symmetries: rotation, scale, mass sum, COM, p_COM
initialData Solver::generateRandomCOM() {

    initialData list;

    VectorDd pos;
    VectorDd vel;
    double mass;

    for (int i = 0; i < NUM; i++) {
        pos = randomOnRadiusD(systemRadius);
        vel = randomOnRadiusD(velocityMax);
        mass = massMin + (massMax - massMin) * rand01();
        list.emplace_back(mass, pos, vel);
    }

    return Bodyfold::transformToCOMSystem(list);
}


#if DIM == 3

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

initialData Solver::ergodicScatterRing3D(array<double, 3> m, double innerDist, double outerDist, double innerPhase, double incline) {

    auto &[M1, M2, M3] = m;

    double v12 = sqrt(G * (M1 + M2) / innerDist);

    initialData circularInit{{M1, {0, 0, 0}, {0, 0, 0}},
        {M2, rotateAroundAxis({0, 0, 1}, {0, innerDist, 0}, innerPhase), rotateAroundAxis({0, 0, 1}, {-v12, 0, 0}, innerPhase)}};
    auto centered = Bodyfold::transformToCOMSystem(circularInit);
    auto &b1 = centered[0];
    auto &b2 = centered[1];

    initialData init{b1, b2, {M3, rotateAroundAxis({0, 1, 0}, {outerDist, 0, 0}, incline - M_PI/2), {0, 0, 0}}};
    init = Bodyfold::transformToCOMSystem(init);

    return init;
}

initialData Solver::ergodicScatterRing3D_eccentric(array<double, 3> m, double r_max, double r_min, double outerDist, double innerPhase, double phi, double incline) {

    auto &[M1, M2, M3] = m;

    Vector2ld r12(0, r_max);
    Vector2ld v12(-sqrt(G * (M1 + M2) * 2 * (1/r_max - 1/(r_max + r_min))), 0);

    double a = (r_max + r_min)/2;
    long double T = innerPhase * sqrt(a*a*a/(G*(M1+M2)));

    auto [r_post, v_post] = orbitForTime(T, (long double)(M1+M2), r12, v12);

    Vector3d r_post3(r_post.x(), r_post.y(), 0);
    Vector3d v_post3(v_post.x(), v_post.y(), 0);


    initialData innerInit{{M1, {0, 0, 0}, {0, 0, 0}},
    {M2, r_post3 , v_post3}};
    auto centered = Bodyfold::transformToCOMSystem(innerInit);
    auto &b1 = centered[0];
    auto &b2 = centered[1];

    initialData init{b1, b2, {M3, rotateAroundAxis({0, 0, 1}, rotateAroundAxis({0, 1, 0}, {outerDist, 0, 0}, incline - M_PI/2), phi), {0, 0, 0}}};
    init = Bodyfold::transformToCOMSystem(init);

    return init;
}
#endif


#if DIM == 2

inline Vector2d rotateHelper(Vector2d v, double angle) {
    double cosA = cos(angle);
    double sinA = sin(angle);

    return {v.x() * cosA - v.y() * sinA, v.x() * sinA + v.y() * cosA};
}

initialData Solver::ergodicScatterRing2D(array<double, 3> m, double innerDist, double outerDist, double innerPhase) {

    auto &[M1, M2, M3] = m;

    double v12 = sqrt(G * (M1 + M2) / innerDist);

    initialData circularInit{{M1, {0, 0}, {0, 0}},
        {M2, rotateHelper({0, innerDist}, innerPhase), rotateHelper({-v12, 0}, innerPhase)}};
    auto centered = Bodyfold::transformToCOMSystem(circularInit);
    auto &b1 = centered[0];
    auto &b2 = centered[1];

    initialData init{b1, b2, {M3, {outerDist, 0}, {0, 0}}};
    init = Bodyfold::transformToCOMSystem(init);

    return init;
}

initialData Solver::ergodicScatterRing2D_eccentric_90deg(array<double, 3> m, double r_max, double r_min, double outerDist, double innerPhase) {

    auto &[M1, M2, M3] = m;

    Vector2ld r12(0, r_max);
    Vector2ld v12(-sqrt(G * (M1 + M2) * 2 * (1/r_max - 1/(r_max + r_min))), 0);

    double a = (r_max + r_min)/2;
    long double T = innerPhase * sqrt(a*a*a/(G*(M1+M2)));

    auto [r_post, v_post] = orbitForTime(T, (long double)(M1+M2), r12, v12);


    initialData innerInit{{M1, {0, 0}, {0, 0}},
        {M2, r_post.cast<double>() , v_post.cast<double>()}};
    auto centered = Bodyfold::transformToCOMSystem(innerInit);
    auto &b1 = centered[0];
    auto &b2 = centered[1];

    initialData init{b1, b2, {M3, {0, outerDist}, {0, 0}}};
    init = Bodyfold::transformToCOMSystem(init);

    return init;

}






#elif DIM == 3





#endif



#endif //MINREP_CONSTRUCTORS_H