#ifndef MINREP_BODYFOLDGENERATOR_H
#define MINREP_BODYFOLDGENERATOR_H

#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;

typedef vector<tuple<double, Vector2d, Vector2d>> initialData;

const static double massMin = 0.5;
const static double massMax = 2;
const static double systemRadius = 25;
const static double velocityMax = 10;

class BodyfoldGenerator {

private:

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

#endif //MINREP_BODYFOLDGENERATOR_H
