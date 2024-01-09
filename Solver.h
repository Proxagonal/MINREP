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
//static const array<double, ORDER> C = {1, -2.0/3, 2.0/3};
//static const array<double, ORDER> D = {-1.0/24, 3.0/4, 7.0/24};

class Solver {

private:

    const double G = 6.674 * pow(10, -11);
    const int subSteps = 2000;
    double totalMass = 0;
    Quantities initQuants;

    vector<Body> bodyList;

public:

    Solver() {

        bodyList = initialConditions();

        for (Body &body : bodyList)
            totalMass += body.mass;

        tranformToCOMSystem();

        initQuants = Quantities(calcMomentum(), 1, calcKinetic(), calcPotential());

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

        return total;
    }

    long double calcKinetic() {

        long double total = 0;

        for (Body &body : bodyList)
            total += body.kineticEnergy();

        return total;
    }

    long double calcEnergy() {
        return calcKinetic() + calcPotential();
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

    long double calcAngularMomentum() {

        long double total = 0;
        Vector2<long double> COM = calcCOM();
        Vector2<long double> COMVel = calcCOMVelocity();

        //for (Body &body : bodyList)
        //    total += crossProd2D(body.position - COM, body.velocity - COMVel);

    }

    template<class T>
    static T crossProd2D(Vector2<T> vec1, Vector2<T> vec2) {
        return vec1.x() * vec2.y() - vec1.y() * vec2.x();
    }


    //calculates important quantities
    vector<long double> quantsInfo() {

        long double momx = 0;
        long double momy = 0;
        long double kin = 0;
        long double energy = 0;

        long double pot = calcPotential();

        for (Body body : bodyList) {
            momx += body.momentum().x();
            momy += body.momentum().y();
            kin += body.kineticEnergy();
        }

        energy = kin + pot;

        return {momx, momy, energy, kin, pot};
    };

    //returns initial conditions of system
    vector<Body> initialConditions() {

        vector<Body> list;

        vector<long double> sine = {1, 1.5, -2.5};
        vector<long double> cosine = {1.5, 4.5, -1.5};


        double rad = 10000;
        double speed = 500;

        Vector2<long double> pos;
        Vector2<long double> vel;

        for (int i = 0; i < 3; i++) {
            double theta = i*2*M_PI/3;
            pos = rad*Vector2<long double>(cosine.at(i), sine.at(i));
            vel = speed*Vector2<long double>(-sine.at(i), cosine.at(i));
            list.emplace_back(5000000000000000000000.0,
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
