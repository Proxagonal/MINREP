#ifndef MINREP_SOLVER_H
#define MINREP_SOLVER_H

#include <Eigen/Eigen>
#include "Body.h"
#include "Quantities.h"

using namespace std;
using namespace Eigen;

#define ORDER 4
static const array<double, ORDER> C = {1/(2*(2-cbrt(2))), (1-cbrt(2))/(2*(2-cbrt(2))), (1-cbrt(2))/(2*(2-cbrt(2))), 1/(2*(2-cbrt(2)))};
static const array<double, ORDER> D = {1/(2-cbrt(2)), -cbrt(2)/(2-cbrt(2)), 1/(2-cbrt(2)), 0};

class Solver {

private:

    const double G = 4*M_PI*M_PI;
    const int subSteps = 2000;

    vector<Body> bodyList = initialConditions();
    const double totalMass = calcMass();


public:

    Solver() {
        tranformToCOMSystem();
    }

    void tranformToCOMSystem() {

        Vector2<long double> COM = calcCOM();
        Vector2<long double> COMVel = calcCOMVelocity();

        for (Body &body : bodyList) {
            body.position -= COM;
            body.velocity -= COMVel;
        }
    }

    void passTime(double dt) {

        for (int i = 0; i < subSteps; i++) {
            doSymplecticIntegrator(dt/subSteps);
            //doVerlet(dt/subSteps);
        }

    }

    void doSymplecticIntegrator(double dt) {

        for (int i = 0; i < ORDER; i++) {
            updateAccelerations();
            for (Body &body : bodyList)
                body.velocity += C.at(i) * dt * body.acceleration;
            for (Body &body : bodyList)
                body.position += D.at(i) * dt * body.velocity;
        }

    }

    void doVerlet(double dt) {

        updateAccelerations();

        for (Body &body : bodyList)
            body.verletNextStep(dt);

    }

    void updateAccelerations() {
        for (Body &body : bodyList)
            body.acceleration = totalGravitationalAccelerationOf(body);
    }

    Vector2<long double> totalGravitationalAccelerationOf(Body &body) {

        Vector2<long double> acc(0, 0);

        for (Body &otherBody : bodyList)
            if (body.vectorTo(otherBody).norm() != 0)
                acc += gravitationalAccelerationOf(body, otherBody);

        return acc;
    };

    //acceleration exerted by body2, on body1
    Vector2<long double> gravitationalAccelerationOf(Body &body1, Body &body2) {

        Vector2<long double> rHat = body1.vectorTo(body2).normalized();
        long double rSquared = body1.vectorTo(body2).squaredNorm();

        return rHat * G * body2.mass/rSquared;
    };

    //calculates potential energy
    long double calcPotential() {

        long double total = 0;

        for (Body &body1 : bodyList) {
            for (Body &body2: bodyList) {
                if (&body1 != &body2) {
                    total += ((long double)(-G * body1.mass * body2.mass)) / (long double)body1.vectorTo(body2).norm();
                }
            }
        }

        return total/2;
    }

    Vector2<long double> calcCOM() {

        Vector2<long double> com(0,0);

        for (Body &body : bodyList)
            com += body.mass * body.position;

        return com/totalMass;
    }

    Vector2<long double> calcMomentum() {

        Vector2<long double> momentum(0, 0);

        for (Body &body : bodyList)
            momentum += body.momentum();

        return momentum;
    }

    Vector2<long double> calcCOMVelocity() {

        return calcMomentum()/totalMass;
    }

    double calcMass() {

        double mass = 0;

        for (Body body : bodyList)
            mass += body.mass;

        return mass;
    }

    //calculates important quantities
    Quantities quantities() {

        long double momx = 0;
        long double momy = 0;
        long double kin = 0;

        long double pot = calcPotential();

        for (Body body : bodyList) {
            momx += body.momentum().x();
            momy += body.momentum().y();
            kin += body.kineticEnergy();
        }

        return {momx, momy, 1, kin, pot};
    };

    //returns initial conditions of system
    vector<Body> initialConditions() {

        vector<Body> list;

        vector<long double> sine = {1, 1.5, -2.5};
        vector<long double> cosine = {1.5, 4.5, -1.5};


        double rad = 20;
        double speed = 0;

        Vector2<long double> pos;
        Vector2<long double> vel;

        for (int i = 0; i < 3; i++) {
            double theta = i*2*M_PI/3;
            pos = rad*Vector2<long double>(cosine.at(i), sine.at(i));
            vel = speed*Vector2<long double>(-sine.at(i), cosine.at(i));
            list.emplace_back(1.0,
                              pos,
                              vel);
        }

        return list;

    }

    const vector<Body> &getBodiesInfo() {
        return bodyList;
    }

};

#endif
