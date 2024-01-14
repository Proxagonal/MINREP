#ifndef MINREP_SOLVER_H
#define MINREP_SOLVER_H

#include <Eigen/Eigen>
#include "Body.h"
#include "initialBody.h"
#include "Quantities.h"

using namespace std;
using namespace Eigen;

static const double G = 4*M_PI*M_PI;

class Solver {

public:

private:

    const int subSteps = 2000;
    const double dt;

    vector<Body> bodyList;
    const double totalMass;


    //returns initial conditions of system
    vector<initialBody> initialConditions() {

        //cout << sizeof(Body);

        vector<initialBody> bods;

        vector<double> sine = {1, 1.5, -2.5};
        vector<double> cosine = {1.5, 4.5, -1.5};


        double rad = 20;
        double speed = 0;

        Vector2d pos;
        Vector2d vel;

        for (int i = 0; i < 3; i++) {
            pos = rad*Vector2d(cosine.at(i), sine.at(i));
            vel = speed*Vector2d(-sine.at(i), cosine.at(i));
            bods.emplace_back(1.13234367832 * (i+1),
                           pos,
                           vel);
        }

        return bods;

    }

    void doVerlet() {
        for (Body &body : bodyList)
            body.pushPosition(2*body.position - body.lastPosition + body.acceleration * dt*dt);
    }

    vector<Body> initiateBodies(vector<initialBody> bods) {


        transformToCOMSystem(bods);

        vector<Body> list;

        updateAccelerations(bods);

        for (initialBody &bod : bods) {
            list.emplace_back(bod.mass, bod.position);
            list.back().position = bod.position + bod.velocity * dt + bod.acceleration*dt*dt/2;
        }



        return list;
    }

    void updateAccelerations() {

        Vector2d mutualVector;

        for (Body &body : bodyList)
            body.acceleration = Vector2d(0,0);


        for (auto body1 = bodyList.begin(); body1 != bodyList.end(); ++body1)
            for (auto body2 = body1 + 1; body2 != bodyList.end(); ++body2)
            {

                mutualVector = directedInverseSquare(body1->position, body2->position);
                body1->acceleration += body2->mass * G * mutualVector;
                body2->acceleration += - body1->mass * G * mutualVector;
            }
    }

    void updateAccelerations(vector<initialBody> &bods) {

        Vector2d mutualVector;

        for (initialBody &body : bods)
            body.acceleration = Vector2d(0,0);


        for (auto body1 = bods.begin(); body1 != bods.end(); ++body1)
            for (auto body2 = body1 + 1; body2 != bods.end(); ++body2)
            {

                mutualVector = directedInverseSquare(body1->position, body2->position);
                body1->acceleration += body2->mass * G * mutualVector;
                body2->acceleration += - body1->mass * G * mutualVector;
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

    Vector2d calcCOM(vector<initialBody> bods) {

        Vector2d com(0,0);

        for (initialBody &body : bods)
            com += body.mass * body.position;

        return com/totalMass;
    }

    Vector2d calcMomentum(vector<initialBody> bods) {

        Vector2d momentum(0, 0);

        for (initialBody &body : bods)
            momentum += body.velocity * body.mass;

        return momentum;
    }

    Vector2d calcCOMVelocity(vector<initialBody> bods) {

        return calcMomentum(bods)/totalMass;
    }

    double calcMass(vector<initialBody> bods) {

        double mass = 0;

        for (initialBody &body : bods)
            mass += body.mass;

        return mass;
    }

    void transformToCOMSystem(vector<initialBody> bods) {

        Vector2d COM = calcCOM(bods);
        Vector2d COMVel = calcCOMVelocity(bods);

        for (initialBody &body : bods) {
            body.position -= COM;
            body.velocity -= COMVel;
        }
    }


public:

    Solver(double frameTime):
    dt{frameTime/subSteps},
    bodyList{initiateBodies(initialConditions())},
    totalMass{calcMass(initialConditions())}
    {
    }

    void passTime() {

        for (int i = 0; i < subSteps; i++) {
            updateAccelerations();
            doVerlet();
        }

    }

    //calculates important quantities
    Quantities quantities() {

        double momx = 0;
        double momy = 0;
        double kin = 0;

        double pot = calcPotential();

        for (Body body : bodyList) {
            momx += body.momentum(dt).x();
            momy += body.momentum(dt).y();
            kin += body.kineticEnergy(dt);
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
