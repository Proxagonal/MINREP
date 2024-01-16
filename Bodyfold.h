#ifndef MINREP_BODYFOLD_H
#define MINREP_BODYFOLD_H

#include <iostream>
#include <Eigen/Eigen>
#include <cmath>

using namespace std;
using namespace Eigen;

#define NUM 3

typedef vector<tuple<double, Vector2d, Vector2d>> initialData;
typedef array<Vector2d, NUM> vData;
typedef array<double, NUM> Data;


struct Bodyfold {

    vData posList;
    vData velList;
    vData accList;
    Data massList;

    Bodyfold(initialData init) {

        if (init.size() != NUM)
            throw invalid_argument("Not Correct Body Amount");

        int i = 0;
        for (auto const& [mass, pos, vel] : init) {
            posList[i] = pos;
            velList[i] = vel;
            accList[i] = Vector2d(0,0);
            massList[i] = mass;
            i++;
        }

    };

    double sumKineticEnergy() {

        double kin = 0;
        for (int i = 0; i < NUM; i++)
            kin += massList[i] * velList[i].squaredNorm() / 2;

        return kin;
    };

    Vector2d sumMomentum() {

        Vector2d mom(0,0);
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


};

#endif