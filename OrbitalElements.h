#ifndef MINREP_OrbitalElements_H
#define MINREP_OrbitalElements_H

#include <csignal>
#include <iomanip>
#include <iostream>
#include <Eigen/Eigen>
#include "Consts.h"

using namespace std;
using namespace Eigen;

#define ORB_ELEMENT_NUM 6


//TODO: Nd orbital elements

#if DIM <= 3

class OrbitalElements {

    static inline const array<string, ORB_ELEMENT_NUM> names = {"Eccentricity", "Semi-Latus Rectum", "Argument of Periapsis", "Longitude of Ascending Node", "Inclination", "True Anomaly"};
    static inline const array<string, ORB_ELEMENT_NUM> symbols = {"e", "p", "ω", "Ω", "i", "v"};
    typedef Vector3<long double> Vector3ld;
    static inline const long double EPS = pow(10, -10);

public:
    const long double eccentricity;         // e
    const long double semiLatusRectum;      // p
    const long double argOfPeriapsis;       // omega
    const long double longAscendingNode;    // OMEGA
    const long double inclination;          // i
    const long double trueAnomaly;          // v

    OrbitalElements(const long double e,
                    const long double p,
                    const long double omega,
                    const long double OMEGA,
                    const long double i,
                    const long double v):
                    eccentricity{e},
                    semiLatusRectum{p},
                    argOfPeriapsis{omega},
                    longAscendingNode{OMEGA},
                    inclination{i},
                    trueAnomaly{v} {

    }

public:

    array<long double, ORB_ELEMENT_NUM> asArray() const {

        return {eccentricity,
                    semiLatusRectum,
                    argOfPeriapsis,
                    longAscendingNode,
                    inclination,
                    trueAnomaly};
    }

    static OrbitalElements calcOrbitalElements(const long double m1, const long double m2, const VectorDld &Ddp, const VectorDld &Ddv) {

        Vector3ld dp(0,0,0);
        dp.head(DIM) = Ddp;

        Vector3ld dv(0,0,0);
        dv.head(DIM) = Ddv;

        long double mu = LD_G*(m1 + m2);

        Vector3ld h = dp.cross(dv); //h2 = dp.squaredNorm() * dv.squaredNorm() - pow(dp.dot(dv), 2); doesnt rely on DIM

        long double p = h.squaredNorm()/mu;

        Vector3ld ecc = dv.cross(h)/mu - dp/dp.norm();

        long double i, OMEGA, omega, v;

        if (p < EPS) { // Degenerate linear case, every variable gets a slightly different meaning.

            return {DNAN, DNAN, DNAN, DNAN, DNAN, DNAN};

            //throw std::domain_error("Cannot use orbital parameters on linear orbit.");

            //Vector3ld hat = abs(dp.x()) > abs(dp.z()) ? Vector3ld::UnitZ() : Vector3ld::UnitX();
            //Vector3ld unit = hat.cross(dp);
            //Vector3ld dv_d = dv.norm() * unit.normalized();
//
            //if (dp.dot(dv) < 0)
            //    dv_d = -dv_d;
//
            //auto [ecc_d, p_d, omega_d, OMEGA_d, i_d, v_d] = calcOrbitalElements(m1, m2, dp, dv_d);
//
            //return {-ecc_d, p_d, omega_d, OMEGA_d, i_d, v_d};
        }

        i = acos(h.z()/h.norm());

        Vector3ld n = Vector3ld(-h.y(), h.x(), 0);
        OMEGA = (n.norm() > 0) ? atan2(n.y(), n.x()) : 0;

        if (ecc.norm() == 0)
            omega = 0;
        else if (n.norm() == 0)
            omega = atan2(ecc.x(), ecc.y());
        else {
            omega = acos(n.dot(ecc)/(ecc.norm() * n.norm()));
            if (n.cross(ecc).dot(h) < 0)
                omega = 2*M_PI - omega;
        }

        v = atan2(h.norm()/mu * dv.dot(dp), p - dp.norm());

        return {ecc.norm(), p, omega, OMEGA, i, v};
    }

    static tuple<Vector3ld, Vector3ld> calcStateVectors(const long double m1, const long double m2, const OrbitalElements &OE) {

        auto [e, p, omega, OMEGA, i, v] = OE;

        //if (e < 0) {
        //    auto [dp, dv] = calcStateVectors(m1, m2, {-e, p, omega, OMEGA, i, v});
//
        //    Vector3ld hat = abs(dp.x()) > abs(dp.z()) ? Vector3ld::UnitZ() : Vector3ld::UnitX();
        //    Vector3ld unit = hat.cross(dp);
//
        //    int sign = unit.dot(dv) > 0 ? 1 : -1;
//
        //    return {dp, sign * dv.norm() * dp.normalized()};
        //}

        long double mu = LD_G*(m1 + m2);

        long double r = p / (1 + e*cos(v));

        long double cosOMEGA = cos(OMEGA);
        long double sinOMEGA = sin(OMEGA);
        long double cosi = cos(i);
        long double sini = sin(i);
        long double cosvomega = cos(omega + v);
        long double sinvomega = sin(omega + v);

        long double h = sqrt(mu*p);

        Vector3ld dp (cosOMEGA * cosvomega - sinOMEGA * sinvomega * cosi, sinOMEGA * cosvomega + cosOMEGA * sinvomega * cosi, sini * sinvomega);
        dp = r * dp;

        Vector3ld dv (- cosOMEGA * sinvomega - sinOMEGA * cosvomega * cosi, - sinOMEGA * sinvomega + cosOMEGA * cosvomega * cosi, sini * cosvomega);
        dv = dv * h/r;
        dv = dv + dp * h*e*sin(v)/(r*p);

        return {dp, dv};
    }

    string toString(const streamsize accuracy = DEFAULTDIGITS) const {

        stringstream ss;
        ss << setprecision(accuracy);

        auto valuearr = asArray();

        for (int i = 0; i < ORB_ELEMENT_NUM; i++)
            ss << symbols[i] << ": " << valuearr[i] << endl;;

        return ss.str();
    }
};


#endif


#endif //MINREP_OrbitalElements_H