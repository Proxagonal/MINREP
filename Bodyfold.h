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

    string toString(const streamsize accuracy = DEFAULTDIGITS) {

        stringstream ss;
        ss.precision(accuracy);

        for (int i = 0; i < NUM; i++) {

            ss << "Body #" << i << ": " << endl;
            ss << "Mass: " << massList[i] << endl;
            ss << "Position: " << posList[i].transpose() << endl;
            ss << "Velocity: " << velList[i].transpose() << endl;
            ss << "Acceleration: " << accList[i].transpose() << endl;

        }

        return ss.str();
    }

    static string toString(initialData &initialConditions, const streamsize accuracy = DEFAULTDIGITS) {

        stringstream ss;
        ss.precision(accuracy);

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

};

#endif