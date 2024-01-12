#ifndef MINREP_BODY_H
#define MINREP_BODY_H

#include <iostream>
#include <Eigen/Eigen>
#include <cmath>

using namespace std;
using namespace Eigen;

static const double G = 4*M_PI*M_PI;

struct Body {

    Vector2d position;
    Vector2d velocity;
    Vector2d acceleration = Vector2d(0,0);
    double mass;
    //due to padding, the size of Body with gMass and without is the same, so no cost
    double gMass;

    Body(double mass, Vector2d pos, Vector2d vel) {
        this->mass = mass;
        position = pos;
        velocity = vel;
        gMass = G * mass;
    };

    double kineticEnergy() {
        return momentum().squaredNorm()/(2 * mass);
    };

    Vector2d momentum() {
        return mass * velocity;
    }

    Vector2d vectorTo(Body &body) {
        return body.position - position;
    };
};

#endif