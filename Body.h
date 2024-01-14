#ifndef MINREP_BODY_H
#define MINREP_BODY_H

#include <iostream>
#include <Eigen/Eigen>
#include <cmath>

using namespace std;
using namespace Eigen;

struct Body {

    Vector2d position;
    Vector2d lastPosition;
    Vector2d acceleration = Vector2d(0,0);
    double mass;

    Body(double mass, Vector2d pos) {
        this->mass = mass;
        lastPosition = pos;
    };

    double kineticEnergy(double dt) {
        return momentum(dt).squaredNorm()/(2 * mass);
    };

    Vector2d momentum(double dt) {
        return mass * velocity(dt);
    }

    Vector2d vectorTo(Body &body) {
        return body.position - position;
    };

    Vector2d velocity(double dt) {
        return (position - lastPosition)/dt;
    }

    void pushPosition(Vector2d pos) {
        lastPosition = position;
        position = pos;
    }
};

#endif