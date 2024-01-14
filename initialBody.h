#ifndef MINREP_INITIALBODY_H
#define MINREP_INITIALBODY_H

#include <Eigen/Eigen>

using namespace Eigen;

struct initialBody {
    Vector2d position;
    Vector2d velocity;
    Vector2d acceleration;
    double mass;

    initialBody(double mass, Vector2d pos, Vector2d vel) {
        position = pos;
        velocity = vel;
        this->mass = mass;
    }
};

#endif //MINREP_INITIALBODY_H
