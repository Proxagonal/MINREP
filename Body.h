#ifndef MINREP_BODY_H
#define MINREP_BODY_H

#include <iostream>
#include <Eigen/Eigen>
#include <cmath>

using namespace std;
using namespace Eigen;


struct Body {

    //biggest first
    //has 8 bytes padding -> 64

    Vector2d position;
    Vector2d velocity;
    Vector2d acceleration = Vector2d(0,0);
    double mass;

    Body(double mass, Vector2d pos, Vector2d vel) {
        this->mass = mass;
        position = pos;
        velocity = vel;
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