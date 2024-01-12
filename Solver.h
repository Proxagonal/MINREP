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

    const int subSteps = 2000;
    const double dt;

    vector<Body> bodyList = initialConditions();
    const double totalMass = calcMass();


    //returns initial conditions of system
    vector<Body> initialConditions() {
        //cout << sizeof(Body);

        vector<Body> list;

        vector<double> sine = {1, 1.5, -2.5};
        vector<double> cosine = {1.5, 4.5, -1.5};


        double rad = 20;
        double speed = 0;

        Vector2d pos;
        Vector2d vel;

        for (int i = 0; i < 3; i++) {
            pos = rad*Vector2d(cosine.at(i), sine.at(i));
            vel = speed*Vector2d(-sine.at(i), cosine.at(i));
            list.emplace_back(1.13234367832 * i,
                           pos,
                           vel);
        }

        return list;

    }


    void doSymplecticIntegrator(double dt) {

        for (int i = 0; i < ORDER; i++) {
            updateAccelerations();
            for (Body &body : bodyList) {
                body.velocity += C.at(i) * dt * body.acceleration;
                body.position += D.at(i) * dt * body.velocity;
            }
        }

    }

    void updateAccelerations() {

        Vector2d mutualVector;

        for (Body &body : bodyList)
            body.acceleration = Vector2d(0,0);


        for (auto body1 = bodyList.begin(); body1 != bodyList.end(); ++body1)
            for (auto body2 = body1 + 1; body2 != bodyList.end(); ++body2)
            {

                mutualVector = directedInverseSquare(body1->position, body2->position);
                body1->acceleration += body2->gMass * mutualVector;
                body2->acceleration += - body1->gMass * mutualVector;
            }
    }

    //acceleration exerted by body2, on body1
    Vector2d directedInverseSquare(Vector2d pos1, Vector2d pos2) {

        Vector2d diff = pos2 - pos1;
        Vector2d rHat = diff.normalized();
        double rSquared = diff.squaredNorm();

        return rHat / rSquared;
    };

    //calculates potential energy
    double calcPotential() {

        double total = 0;

        for (Body &body1 : bodyList) {
            for (Body &body2: bodyList) {
                if (&body1 != &body2) {
                    total += ((double)(-G * body1.mass * body2.mass)) / (double)body1.vectorTo(body2).norm();
                }
            }
        }

        return total/2;
    }

    Vector2d calcCOM() {

        Vector2d com(0,0);

        for (Body &body : bodyList)
            com += body.mass * body.position;

        return com/totalMass;
    }

    Vector2d calcMomentum() {

        Vector2d momentum(0, 0);

        for (Body &body : bodyList)
            momentum += body.momentum();

        return momentum;
    }

    Vector2d calcCOMVelocity() {

        return calcMomentum()/totalMass;
    }

    double calcMass() {

        double mass = 0;

        for (Body &body : bodyList)
            mass += body.mass;

        return mass;
    }

    void transformToCOMSystem() {

        Vector2d COM = calcCOM();
        Vector2d COMVel = calcCOMVelocity();

        for (Body &body : bodyList) {
            body.position -= COM;
            body.velocity -= COMVel;
        }
    }


public:

    Solver(double constDt): dt{constDt} {
        transformToCOMSystem();
    }

    void passTime() {

        for (int i = 0; i < subSteps; i++)
            doSymplecticIntegrator(dt/subSteps);

    }

    //calculates important quantities
    Quantities quantities() {

        double momx = 0;
        double momy = 0;
        double kin = 0;

        double pot = calcPotential();

        for (Body body : bodyList) {
            momx += body.momentum().x();
            momy += body.momentum().y();
            kin += body.kineticEnergy();
        }

        return {momx, momy, 1, kin, pot};
    };

    const vector<Body> &getBodiesInfo() {
        return bodyList;
    }

    double getSystemRadius() {

        double maximum = 0;
        for (Body &body: bodyList)
            maximum = max(maximum, body.position.norm());

        return maximum;
    }

};

#endif
