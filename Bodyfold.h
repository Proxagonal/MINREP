#ifndef MINREP_BODYFOLD_H
#define MINREP_BODYFOLD_H

#include <iostream>
#include <Eigen/Eigen>
#include <cmath>
#include <random>

#include "Consts.h"

using namespace std;
using namespace Eigen;

typedef vector<tuple<double, VectorDd, VectorDd>> initialData;
typedef array<VectorDd, NUM> vData;
typedef array<double, NUM> Data;


struct Bodyfold {

private:

    constexpr static int ALLDIGITS = std::numeric_limits<double>::max_digits10;

    constexpr static double massMin = 0.5;
    constexpr static double massMax = 2;
    constexpr static double systemRadius = 10;
    constexpr static double velocityMax = 3;

    static VectorDd randomOnRadiusD(double r) {

        // According to Box-Mueller. Needs testing.

        VectorDd Z = VectorDd::NullaryExpr([&]() { return randNormalStandard();});

        return r * pow(rand01(), 1.0/DIM) * Z.normalized();
    }

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

    // Symmetries: rotation, scale, mass sum, COM, p_COM
    static initialData generateRandomCOM() {

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

        return transformToCOMSystem(list);
    }

    static initialData generateRandomCOM_no3() {

        initialData list;

        VectorDd pos;
        VectorDd vel;
        double mass;

        for (int i = 0; i < NUM; i++) {
            pos = randomOnRadiusD(systemRadius);
            vel = randomOnRadiusD(velocityMax);
            mass = (i < 2)*(massMin + (massMax - massMin) * rand01());
            list.emplace_back(mass, pos, vel);
        }

        return transformToCOMSystem(list);
    }

    // Symmetries: rotation, scale, mass sum, COM, p_COM
    static initialData generateRandomNONCOM() {

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

};

#endif