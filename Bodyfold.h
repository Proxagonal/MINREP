#ifndef MINREP_BODYFOLD_H
#define MINREP_BODYFOLD_H

#include <iostream>
#include <Eigen/Eigen>
#include <cmath>
#include <random>

#include "Consts.h"
#include "Quantities.h"

using namespace std;
using namespace Eigen;


struct Bodyfold {

private:

    constexpr static int ALLDIGITS = std::numeric_limits<double>::max_digits10;

    inline static double cross(VectorDd &a, VectorDd &b) {
        return a.x()*b.y() - a.y()*b.x();
    }

    inline static double cross(VectorDd a, VectorDd b) {
        return a.x()*b.y() - a.y()*b.x();
    }

public:

    vData posList;
    vData velList;
    vData accList;
    const Data massList;

    Bodyfold(const initialData& init): massList([&init] {
            if (init.size() != NUM)
                throw std::invalid_argument("Not Correct Body Amount");

            Data tempMassList;
            int i = 0;
            for (const auto& [mass, pos, vel] : init) {
                tempMassList[i++] = mass;
            }
            return tempMassList;
        }()) {
        int i = 0;
        for (auto const &[mass, pos, vel] : init) {
            posList[i] = pos;
            velList[i] = vel;
            accList[i] = VectorDd::Zero();
            i++;
        }
    };

    double sumKineticEnergy() {

        double kin = 0;
        for (int i = 0; i < NUM; i++)
            kin += massList[i] * velList[i].squaredNorm() / 2;

        return kin;
    };

    //calculates potential energy
    double sumPotential() {

        double total = 0;

        for (int i = 0; i < NUM; i++)
            for (int j = i + 1; j < NUM; j++)
                total += (-G * massList[i] * massList[j]) / (posList[i] - posList[j]).norm();

        return total;
    }

    VectorAngd sumAngularMomentum() {

        VectorDd COM = getCOMPosition();
        VectorDd COMvel = getCOMVelocity();

        VectorAngd total = VectorAngd::Zero();

        for (int b = 0; b < NUM; b++) {

            VectorDd posInCOM = posList[b] - COM;
            VectorDd momInCOM = massList[b]*(velList[b] - COMvel);

            for (int i = 0; i < DIM; i++)
                for (int j = i + 1; j < DIM; j++)
                    total(angularIndex(i, j)) += posInCOM(i)*momInCOM(j) - momInCOM(i)*posInCOM(j);
        }

        return total;
    }

    VectorDd sumMomentum() {

        VectorDd mom = VectorDd::Zero();
        for (int i = 0; i < NUM; i++)
            mom += massList[i] * velList[i];

        return mom;
    }

    double sumMass() {

        double mass = 0;
        for (int i = 0; i < NUM; i++)
            mass += massList[i];

        return mass;
    }

    VectorDd getCOMVelocity() {

        return sumMomentum()/sumMass();
    }

    VectorDd getWeightedPosition() {

        VectorDd weighted = VectorDd::Zero();

        for (int i = 0; i < NUM; i++)
            weighted += massList[i] * posList[i];

        return weighted;
    }

    VectorDd getCOMPosition() {

        return getWeightedPosition()/sumMass();
    }

    static initialData transformToCOMSystem(initialData &init) {

        initialData COMMED;

        double massSum = 0;
        VectorDd weightedPoses = VectorDd::Zero();
        VectorDd weightedVels = VectorDd::Zero();

        for (auto const &[mass, pos, vel] : init) {
            massSum += mass;
            weightedPoses += mass*pos;
            weightedVels += mass*vel;
        }

        VectorDd COM = weightedPoses/massSum;
        VectorDd COMVel = weightedVels/massSum;

        for (auto const &[mass, pos, vel] : init)
            COMMED.emplace_back(mass, pos - COM, vel - COMVel);

        return COMMED;
    }

    string toString() {

        stringstream ss;
        ss.precision(ALLDIGITS);

        for (int i = 0; i < NUM; i++) {

            ss << "Body #" << i << ": " << endl;
            ss << "Mass: " << massList[i] << endl;
            ss << "Position: " << posList[i].transpose() << endl;
            ss << "Velocity: " << velList[i].transpose() << endl;
            ss << "Acceleration: " << accList[i].transpose() << endl;

        }

        return ss.str();
    }

    static string toString(initialData &initialConditions) {

        stringstream ss;
        ss.precision(ALLDIGITS);

        for (int i = 0; i < NUM; i++) {

            ss << "Body #" << i << ": " << endl;
            ss << "Mass: " << get<0>(initialConditions.at(i)) << endl;
            ss << "Position: " << get<1>(initialConditions.at(i)).transpose() << endl;
            ss << "Velocity: " << get<2>(initialConditions.at(i)).transpose() << endl;
        }

        return ss.str();
    }

    static initialData stringToInitialData(string &str) {

        initialData bodies;
        std::istringstream iss(str);
        std::string line;
        double mass;
        while (std::getline(iss, line)) {
            if (line.find("Mass") != std::string::npos) {
                std::stringstream(line.substr(line.find(":") + 1)) >> mass;

                // Parse position
                VectorDd pos;
                std::getline(iss, line);
                std::stringstream posStream(line.substr(line.find(":") + 1));
                for (int i = 0; i < DIM; ++i) posStream >> pos[i];

                // Parse velocity
                VectorDd vel;
                std::getline(iss, line);
                std::stringstream velStream(line.substr(line.find(":") + 1));
                for (int i = 0; i < DIM; ++i) velStream >> vel[i];

                bodies.emplace_back(mass, pos, vel);
            }
        }

        return bodies;
    }


    //calculates important quantities
    Quantities quantities() {

        VectorDd compos = getCOMPosition();
        VectorDd mom = sumMomentum();
        VectorAngd angMom = sumAngularMomentum();

        double kin = sumKineticEnergy();
        double pot = sumPotential();

        return {compos, mom, angMom, kin, pot};
    };

    //TODO: Nd orbital elements

#if DIM <= 3

    struct orbitalElements {
        long double eccentricity;       // e
        long double semiLatusRectum;    // p
        long double argOfPeriapsis;     // omega
        long double longAscendingNode;  // OMEGA
        long double inclination;        // i
        long double trueAnomaly;        // v

        long double semiMajorAxis() {
            if (eccentricity == 1)
                return numeric_limits<double>::infinity();
            return semiLatusRectum/(1 - pow(eccentricity, 2));
        }
    };

    typedef Vector3<long double> Vector3ld;

    static orbitalElements calcOrbitalElements(long double m1, long double m2, VectorDld &p1, VectorDld &p2, VectorDld &v1, VectorDld &v2) {

        Vector3ld dp(0,0,0);
        dp.head(DIM) = p2 - p1;

        Vector3ld dv(0,0,0);
        dv.head(DIM) = v2 - v1;

        long double mu = LD_G*(m1 + m2);

        long double epsilon = dv.squaredNorm()/2 - mu/dp.norm();

        Vector3ld h = dp.cross(dv); //h2 = dp.squaredNorm() * dv.squaredNorm() - pow(dp.dot(dv), 2); doesnt rely on DIM

        long double p = h.squaredNorm()/mu;

        long double i = acos(h.z()/h.norm());

        Vector3ld n = Vector3ld(-h.y(), h.x(), 0);
        long double OMEGA = (n.norm() > 0) ? atan2(n.y(), n.x()) : 0;

        Vector3ld ecc = dv.cross(h)/mu - dp/dp.norm();

        long double omega;
        if (ecc.norm() == 0)
            omega = 0;
        else if (n.norm() == 0)
            omega = atan2(ecc.x(), ecc.y());
        else {
            omega = acos(n.dot(ecc)/(ecc.norm() * n.norm()));
            if (n.cross(ecc).dot(h) < 0)
                omega = 2*M_PI - omega;
        }


        long double v = atan2(h.norm()/mu * dv.dot(dp), p - dp.norm());

        return {ecc.norm(), p, omega, OMEGA, i, v};
    }

    static tuple<Vector3ld, Vector3ld> calcStateVectors(double m1, double m2, orbitalElements &OE) {

        auto [e, p, omega, OMEGA, i, v] = OE;

        long double r = p / (1 + e*cos(v));

        long double cosOMEGA = cos(OMEGA);
        long double sinOMEGA = sin(OMEGA);
        long double cosi = cos(i);
        long double sini = sin(i);
        long double cosvomega = cos(omega + v);
        long double sinvomega = sin(omega + v);

        long double mu = LD_G*(m1 + m2);
        long double h = sqrt(mu*p);

        Vector3ld dp (cosOMEGA * cosvomega - sinOMEGA * sinvomega * cosi, sinOMEGA * cosvomega + cosOMEGA * sinvomega * cosi, sini * sinvomega);
        dp = r * dp;

        Vector3ld dv (- cosOMEGA * sinvomega - sinOMEGA * cosvomega * cosi, - sinOMEGA * sinvomega + cosOMEGA * cosvomega * cosi, sini * cosvomega);
        dv = dv * h/r;
        dv = dv + dp * h*e*sin(v)/(r*p);

        return {dp, dv};
    }

#endif

};

#endif