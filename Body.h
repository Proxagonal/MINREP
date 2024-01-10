#ifndef MINREP_BODY_H
#define MINREP_BODY_H

#include <iostream>
#include <Eigen/Eigen>
#include <cmath>

using namespace std;
using namespace Eigen;

class Body {

private:
    Vector2d lastPosition;
    double lastDt;
    bool firstStepDone = false;

public:
    double mass;
    Vector2d position;
    Vector2d velocity;
    Vector2d acceleration = Vector2d(0,0);

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

    void verletNextStep(double dt) {

        if (dt == 0)
            return;

        if (!firstStepDone) {
            lastPosition = position;
            position = lastPosition
                       + velocity * dt
                       + 0.5 * acceleration * dt * dt;
            lastDt = dt;
            firstStepDone = true;
            return;
        }

        Vector2d nextPosition = position
                                + (position - lastPosition) * dt/lastDt
                                + acceleration * dt * (dt + lastDt)/2;
        lastDt = dt;

        lastPosition = position;
        position = nextPosition;

        velocity = (position - lastPosition)/dt;

    };

    Vector2d vectorTo(Body &body) {
        return body.position - position;
    };
};

#endif