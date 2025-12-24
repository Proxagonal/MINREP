#include "Bodyfold.h"
#include <chrono>
#include <fstream>
#include <unistd.h>
#include <iostream>

#include "OrbitalElements.h"

using namespace std;
using namespace Eigen;

#include <Eigen/Dense>
#include <iostream>
#include <cmath>
#include <tuple>
#include <random>

using Vector3ld = Eigen::Matrix<long double,3,1>;

constexpr long double TINY = 1e-3;

bool finiteVec(const Vector3ld& v) {
    return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}

bool nearVec(const Vector3ld& a, const Vector3ld& b, long double tol = TINY) {
    return (a - b).norm() < tol || (a + b).norm() < tol;
}

Vector3ld randomPerp(const Vector3ld& v) {
    Vector3ld r = Vector3ld::Random();
    Vector3ld p = v.cross(r);
    if (p.norm() == 0) return Vector3ld::UnitX();
    return p.normalized();
}

/* ========================= TESTS ========================= */

void test_radial_exact(long N) {

    cout << "WOW" << endl;

    for (long i = 0; i < N; ++i) {

        Vector3ld dp = Vector3ld::Random();
        Vector3ld dv = ((rand01() > 0.5) ? 1 : -1) * (0.0+rand01()) * dp / 100;

        cout << dp.transpose() << endl;
        cout << dv.transpose() << endl;

        auto OE = OrbitalElements::calcOrbitalElements(1,1,dp,dv);

        cout << OE.toString();

        auto [dp2,dv2] = OrbitalElements::calcStateVectors(1,1,OE);

        cout << (dp2 - dp).transpose() << endl;
        cout << (dv2 - dv).transpose() << endl;

        cout << endl;

        if (!nearVec(dp,dp2) || !nearVec(dv,dv2)) {
            std::cout << "FAIL radial exact\n";
            return;
        }
    }
}

void test_radial_near(long N) {
    cout << "HEYO" << endl;
    for (long i = 0; i < N; ++i) {


        Vector3ld dp = Vector3ld::Random();
        Vector3ld dv = dp + 1e-14 * dp.norm() * randomPerp(dp);



        auto OE = OrbitalElements::calcOrbitalElements(1,1,dp,dv);
        auto [dp2,dv2] = OrbitalElements::calcStateVectors(1,1,OE);

        if (!finiteVec(dp2) || !finiteVec(dv2)) {
            std::cout << "FAIL near-radial NaN\n";
            return;
        }
    }
}

void test_circular(long N) {
    for (long i = 0; i < N; ++i) {
        Vector3ld dp = Vector3ld::Random();
        long double r = dp.norm();
        Vector3ld dv = std::sqrt(1.0L / r) * randomPerp(dp);

        auto OE = OrbitalElements::calcOrbitalElements(1,0,dp,dv);
        auto [dp2,dv2] = OrbitalElements::calcStateVectors(1,0,OE);

        if (!nearVec(dp,dp2) || !nearVec(dv,dv2)) {
            std::cout << "FAIL circular\n";
            return;
        }
    }
}

void test_equatorial(long N) {
    for (long i = 0; i < N; ++i) {
        Vector3ld dp = 100*Vector3ld::Random(); dp.z() = 0;
        Vector3ld dv = 100*Vector3ld::Random(); dv.z() = 0;

        cout << dp.transpose() << endl;
        cout << dv.transpose() << endl;

        auto OE = OrbitalElements::calcOrbitalElements(1,1,dp,dv);
        auto [dp2,dv2] = OrbitalElements::calcStateVectors(1,1,OE);

        if (!finiteVec(dp2) || !finiteVec(dv2)) {
            std::cout << "FAIL equatorial NaN\n";
            return;
        }
    }
}

void test_near_equatorial(long N) {
    for (long i = 0; i < N; ++i) {
        Vector3ld dp = Vector3ld::Random();
        Vector3ld dv = Vector3ld::Random();
        dp.z() *= 1e-14;
        dv.z() *= 1e-14;

        auto OE = OrbitalElements::calcOrbitalElements(1,1,dp,dv);
        auto [dp2,dv2] = OrbitalElements::calcStateVectors(1,1,OE);

        if (!finiteVec(dp2) || !finiteVec(dv2)) {
            std::cout << "FAIL near-equatorial NaN\n";
            return;
        }
    }
}

void test_near_parabolic(long N) {
    for (long i = 0; i < N; ++i) {
        long double e = 1 + (rand01() - 0.5) * 1e-12;
        long double p = rand01() + 0.1;
        long double v = M_PI - 1e-6;

        OrbitalElements OE{e,p,0,0,0.3L,v};
        auto [dp,dv] = OrbitalElements::calcStateVectors(1,0,OE);
        auto OE2 = OrbitalElements::calcOrbitalElements(1,0,dp,dv);
        auto [dp2,dv2] = OrbitalElements::calcStateVectors(1,0,OE2);

        if (!finiteVec(dp2) || !finiteVec(dv2)) {
            std::cout << "FAIL parabolic NaN\n";
            return;
        }
    }
}

void test_hyperbolic(long N) {
    for (long i = 0; i < N; ++i) {
        Vector3ld dp = Vector3ld::Random();
        Vector3ld dv = 3 * Vector3ld::Random();

        auto OE = OrbitalElements::calcOrbitalElements(1,1,dp,dv);
        auto [dp2,dv2] = OrbitalElements::calcStateVectors(1,1,OE);

        if (!finiteVec(dp2) || !finiteVec(dv2)) {
            std::cout << "FAIL hyperbolic NaN\n";
            return;
        }
    }
}

void test_zero_velocity(long N) {
    for (long i = 0; i < N; ++i) {
        Vector3ld dp = Vector3ld::Random();
        Vector3ld dv = Vector3ld::Zero();

        auto OE = OrbitalElements::calcOrbitalElements(1,1,dp,dv);
        auto [dp2,dv2] = OrbitalElements::calcStateVectors(1,1,OE);

        if (!finiteVec(dp2) || !finiteVec(dv2)) {
            std::cout << "FAIL zero velocity\n";
            return;
        }
    }
}

void test_time_reversal(long N) {
    for (long i = 0; i < N; ++i) {
        Vector3ld dp = Vector3ld::Random();
        Vector3ld dv = Vector3ld::Random();

        auto OE1 = OrbitalElements::calcOrbitalElements(1,1,dp,dv);
        auto OE2 = OrbitalElements::calcOrbitalElements(1,1,dp,-dv);

        auto [dp1,dv1] = OrbitalElements::calcStateVectors(1,1,OE1);
        auto [dp2,dv2] = OrbitalElements::calcStateVectors(1,1,OE2);

        if (!nearVec(dp1,dp2) || !nearVec(dv1,-dv2)) {
            std::cout << "FAIL time reversal\n";
            return;
        }
    }
}

/* ========================= MAIN ========================= */

int main() {

    long N = 100000;

    cout << "YO" << endl;

    test_circular(N);
    test_equatorial(N);
    //test_near_equatorial(N);
    //test_near_parabolic(N);
    //test_hyperbolic(N);
    //test_zero_velocity(N);
    //test_time_reversal(N);

    std::cout << "All tests completed\n";
}


/*int main() {

    long double TINY = pow(10, -8);

    int mscale = 10;
    int pscale = 50;
    int vscale = 5;

    long N = 1000000;
    long double psum = 0;
    long double vsum = 0;

    long fails = 0;

    for (long i = 0; i < N; i++) {

        double m1 = mscale*rand01();
        double m2 = mscale*rand01();

        Vector3d dp = pscale*Vector3d::Random();
        Vector3d dv = vscale*Vector3d::Random();
        dv = dp * (rand01() - 0.5);

        OrbitalElements OE = OrbitalElements::calcOrbitalElements(m1, m2, dp.cast<long double>(), dv.cast<long double>());

        //cout << OE.toString() << endl;

        auto [ndp, ndv] = OrbitalElements::calcStateVectors(m1, m2, OE);

        long double pdiv = (ndp-dp.cast<long double>()).norm();
        long double vdiv = (ndv-dv.cast<long double>()).norm();

        if (pdiv > TINY || vdiv > TINY || pdiv != pdiv) {
            cout << "1: ";
            cout << m1 << ", " << m2 << endl;
            cout << dp.transpose() << endl;
            cout << ndp.transpose() << endl;
            cout << dv.transpose() << endl;
            cout << ndv.transpose() << endl;
            cout << (ndp-dp.cast<long double>()).norm() << ", " << (ndv-dv.cast<long double>()).norm() << endl << endl;
            fails += 1;
        }

        psum += pdiv;
        vsum += vdiv;
    }

    cout << psum/N << endl;
    cout << vsum/N << endl;
    cout << ((double)fails)/N << endl;
}*/