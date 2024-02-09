#ifndef MINREP_SOLVER_H
#define MINREP_SOLVER_H

#include <Eigen/Eigen>
#include "Bodyfold.h"
#include "Quantities.h"

using namespace std;
using namespace Eigen;

#define ORDER 4
static const array<double, ORDER> C = {1/(2*(2-cbrt(2))), (1-cbrt(2))/(2*(2-cbrt(2))), (1-cbrt(2))/(2*(2-cbrt(2))), 1/(2*(2-cbrt(2)))};
static const array<double, ORDER> D = {1/(2-cbrt(2)), -cbrt(2)/(2-cbrt(2)), 1/(2-cbrt(2)), 0};

static const double G = 4*M_PI*M_PI;

typedef uint_fast8_t nat;

class Solver {

private:

    const int subSteps = 2000;

    const double dt;

    Bodyfold bodyfold{initialConditions()};
    const double totalMass = bodyfold.sumMass();

    //returns initial conditions of system
    initialData initialConditions() {

        initialData list;

        vector<double> sine = {1, 1.5, -2.5};
        vector<double> cosine = {1.5, 4.5, -1.5};


        double rad = 20;
        double speed = 0;

        Vector2d pos;
        Vector2d vel;

        for (int i = 0; i < 3; i++) {
            pos = rad*Vector2d(cosine.at(i), sine.at(i));
            vel = speed*Vector2d(-sine.at(i), cosine.at(i));
            list.emplace_back(1.13234367832 * (i+1),
                           pos,
                           vel);
        }

        return list;
    }

    void doSymplecticIntegrator() {

        for (nat p = 0; p < ORDER; p++) {
            updateAccelerations();
            for (nat i = 0; i < NUM; i++) {
                bodyfold.velList[i] += C[p] * dt * bodyfold.accList[i];
                bodyfold.posList[i] += D[p] * dt * bodyfold.velList[i];
            }
        }
    }

    void updateAccelerations() {

        Vector2d mutualVector;

        for (Vector2d &acc : bodyfold.accList)
            acc.setZero();

        nat j;
        for (nat i = 0; i < NUM; i++) {
            j = (i+1)%NUM;
            mutualVector = directedInverseSquare(bodyfold.posList[i], bodyfold.posList[j]);
            bodyfold.accList[i] += bodyfold.massList[j] * mutualVector;
            bodyfold.accList[j] += - bodyfold.massList[i] * mutualVector;
        }
    }

    static Vector2d directedInverseSquare(const Vector2d &pos1, const Vector2d &pos2) {

        Vector2d diff = pos2 - pos1;
        return G * diff / (diff.norm() * diff.squaredNorm());
        //return G * diff.newtonianNormalized();
        //return G * diff/diff.newtonianNorm();

    }

    //calculates potential energy
    double calcPotential() {

        double total = 0;

        //for (Body &body1 : bodyList) {
        //    for (Body &body2: bodyList) {
        //        if (&body1 != &body2) {
        //            total += ((double)(-G * body1.mass * body2.mass)) / (double)body1.vectorTo(body2).norm();
        //        }
        //    }
        //}

        for (int i = 0; i < NUM; i++)
            for (int j = 0; j < NUM; j++)
            {
                if (i != j)
                    total += ((double)(-G * bodyfold.massList[i] * bodyfold.massList[j])) / (bodyfold.posList[i] - bodyfold.posList[j]).norm();

            }

        return total/2;
    }

    Vector2d calcCOM() {

        Vector2d com(0,0);

        for (int i = 0; i < NUM; i++)
            com += bodyfold.massList[i] * bodyfold.posList[i];

        return com/totalMass;
    }

    Vector2d calcCOMVelocity() {

        return bodyfold.sumMomentum()/totalMass;
    }

    void transformToCOMSystem() {

        Vector2d COM = calcCOM();
        Vector2d COMVel = calcCOMVelocity();

        for (int i = 0; i < NUM; i++) {
            bodyfold.posList[i] -= COM;
            bodyfold.velList[i] -= COMVel;
        }
    }


public:

    Solver(double frameTime): dt{frameTime/subSteps} {
        transformToCOMSystem();
    }

    void passTime() {

        for (int i = 0; i < subSteps; i++)
            doSymplecticIntegrator();

    }

    //calculates important quantities
    Quantities quantities() {

        Vector2d mom = bodyfold.sumMomentum();
        double kin = bodyfold.sumKineticEnergy();

        double pot = calcPotential();

        return {mom.x(), mom.y(), 1, kin, pot};
    };

    const vData &getDrawInfo() {
        return bodyfold.posList;
    }

    double getSystemRadius() {

        double maximum = 0;
        for (int i = 0; i < NUM; i++)
            maximum = max(maximum, bodyfold.posList[i].norm());

        return maximum;
    }
};

#endif
