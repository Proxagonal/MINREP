#ifndef MINREP_BODY_H
#define MINREP_BODY_H

#include <iostream>
#include <Eigen/Eigen>
#include <cmath>

using namespace std;
using namespace Eigen;

class Body {

private:
    Vector2<long double> lastPosition;
    double lastDt;
    bool firstStepDone = false;

public:
    double mass;
    Vector2<long double> position;
    Vector2<long double> velocity;
    Vector2<long double> acceleration = Vector2<long double>(0,0);

    Body(double mass, Vector2<long double> pos, Vector2<long double>vel) {
        this->mass = mass;
        position = pos;
        velocity = vel;
    };

    long double kineticEnergy() {
        return momentum().squaredNorm()/(2 * mass);
    };

    Vector2<long double> momentum() {
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

        Vector2<long double> nextPosition = position
                                + (position - lastPosition) * dt/lastDt
                                + acceleration * dt * (dt + lastDt)/2;
        lastDt = dt;

        lastPosition = position;
        position = nextPosition;

        velocity = (position - lastPosition)/dt;

    };

    Vector2<long double> vectorTo(Body &body) {
        return body.position - position;
    };
};

#endif