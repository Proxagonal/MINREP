#ifndef MINREP_BODY_H
#define MINREP_BODY_H

#include <iostream>
#include <Eigen/Eigen>
#include <cmath>

using namespace std;
using namespace Eigen;

#define DIM 2
#define NUM 3

typedef Vector<double, NUM*DIM> gVector;
typedef Vector<double, DIM> VectorB;

struct Bodyfold {

    gVector gPosition;
    gVector gVelocity;
    array<double, NUM> massList;

    Bodyfold(vector<tuple<double, VectorB, VectorB>> bodies) {

        if (bodies.size() != NUM)
            throw invalid_argument("Amount of bodies doesn't match expected amount.");

        //really ugly code, but just translates ((m1, p1, v1), (m2, p2, v2), ...) to ((m1, m2, ...), (p1, p2, ...), (v1, v2, ...))
        for (int i = 0; i < NUM; i++) {
            massList[i] = get<0>(bodies[i]);
            for (int j = i*DIM; j < (i+1)*DIM; j++) {
                gPosition(j) = get<1>(bodies[i])(j - i*DIM);
                gVelocity(j) = get<2>(bodies[i])(j - i*DIM);
            }
        }

    };

    vector<VectorB> getPositions() {

        vector<VectorB> positions;

        VectorB pos;

        for (int i = 0; i < NUM; i++) {

            for (int j = i*DIM; j < (i+1)*DIM; j++)
                pos(j - i*DIM) = gPosition(j);

            positions.emplace_back(pos);
        }

        return positions;
    }

    vector<VectorB> getVelocities() {

        vector<VectorB> velocities;

        VectorB vel;

        for (int i = 0; i < NUM; i++) {

            for (int j = i*DIM; j < (i+1)*DIM; j++)
                vel(j - i*DIM) = gVelocity(j);

            velocities.emplace_back(vel);
        }

        return velocities;
    }


    double kineticEnergy() {

        double kin = 0;

        vector<VectorB> vels = getVelocities();

        for (int i = 0; i < NUM; i++)
            kin += massList[i] * vels[i].squaredNorm() / 2;

        return kin;
    };

    vector<VectorB> momenta() {

        //first set to vels, then change
        vector<VectorB> moms = getVelocities();

        for (int i = 0; i < NUM; i++)
            moms[i] = massList[i] * moms[i];

        return moms;

    }
};

#endif