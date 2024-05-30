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

private:

    constexpr static double massMin = 0.5;
    constexpr static double massMax = 2;
    constexpr static double systemRadius = 25;
    constexpr static double velocityMax = 15;

    static void printInitialData(const initialData &list) {

        int i = 0;
        for (auto const& [mass, pos, vel] : list) {
            cout << "Body #" << i++ << ": " << endl;
            cout << "Mass: " << mass << endl;
            cout << "init_Position: " << pos.transpose() << endl;
            cout << "init_Velocity: " << vel.transpose() << endl;
        }

    }

    static Vector2d toCartesian(double rad, double theta) {
        return {rad*cos(theta), rad*sin(theta)};
    }

    static Vector2d randomOnRadius(double r) {
        return toCartesian(r * sqrt(rand01()), M_2_PI * rand01());
    }

    static double rand01() {
        return (double) rand() / RAND_MAX;
    }

public:

    vData posList;
    vData velList;
    vData accList;
    Data massList;
    Data ratios;

    Bodyfold(const initialData& init) {

        printInitialData(init);

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
        for (i = 0; i < NUM; i++)
            ratios[i] = massList[(i+2)%NUM]/(massList[(i+1)%NUM] + massList[(i+2)%NUM]);

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

    Vector2d getCOMVelocity() {

        return sumMomentum()/sumMass();

    }

    Vector2d getCOMPosition() {

        Vector2d com(0,0);

        for (int i = 0; i < NUM; i++)
            com += massList[i] * posList[i];

        return com/sumMass();
    }


    static initialData generateRandom() {

        initialData list;

        srand(time(0));

        Vector2d pos;
        Vector2d vel;
        double mass;

        for (int i = 0; i < NUM; i++) {
            pos = randomOnRadius(systemRadius);
            vel = randomOnRadius(velocityMax);
            mass = massMin + (massMax - massMin) * rand01();
            list.emplace_back(mass, pos, vel);
        }

        return list;
    }

};

#endif