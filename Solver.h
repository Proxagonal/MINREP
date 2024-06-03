#ifndef MINREP_SOLVER_H
#define MINREP_SOLVER_H

#include <Eigen/Eigen>
#include "Bodyfold.h"
#include "Quantities.h"
#include "Visualizer.h"

using namespace std;
using namespace Eigen;

#define VISUALIZE true
#define COMPARE_QUANTS true

#define ORDER 4
static const array<double, ORDER> C = {1/(2*(2-cbrt(2))), (1-cbrt(2))/(2*(2-cbrt(2))), (1-cbrt(2))/(2*(2-cbrt(2))), 1/(2*(2-cbrt(2)))};
static const array<double, ORDER> D = {1/(2-cbrt(2)), -cbrt(2)/(2-cbrt(2)), 1/(2-cbrt(2)), 0};

static const double G = 4*M_PI*M_PI;

typedef uint_fast8_t nat;

class Solver {

private:

    const int T;
    const double dt;

    static const int checkTime = 5;
    static const int ratio = 20;
    static const int ratioSquare = ratio*ratio;


    Bodyfold bodyfold;

#if VISUALIZE
    Visualizer visuals;
    const int framePerPasses = 5000;
#endif
#if COMPARE_QUANTS
    Quantities initialQuants;
    const int comparePerPasses = 20000000;
#endif

    //returns initial conditions of system
    static initialData initialConditions() {

        return Bodyfold::generateRandomCOM();
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
    }


    //bool isEscape() {
//
    //    vector<double> distSquares;
    //    nat j;
    //    for (nat i = 0; i < NUM; i++) {
    //        j = (i + 1) % NUM;
    //        distSquares.emplace_back((bodyfold.posList[i] - bodyfold.posList[j]).squaredNorm());
    //    }
//
    //    if (distSquares.at(0) > ratioSquare * distSquares.at(1))
    //        return confirmEscape(0);
    //    if (ratioSquare * distSquares.at(0) < distSquares.at(1))
    //        return confirmEscape(2);
    //    if (ratioSquare * distSquares.at(2) < distSquares.at(1))
    //        return confirmEscape(1);
//
    //    return NUM;
    //}
//
    //bool confirmEscape(nat i) {
//
    //    int ip = (i+1) % NUM;
    //    int ipp = (i+2) % NUM;
    //    Vector2d twoBod = bodyfold.posList[ip] + (bodyfold.posList[ipp] - bodyfold.posList[ip]) * bodyfold.ratios[i];
//
    //    double specificOrbitalEnergy =1;
//
    //    Vector2d vectorAway = (bodyfold.posList[i] - twoBod).normalized();
//
    //    double velocityAway = bodyfold.velList[i].dot(vectorAway);
//
    //    return (bodyfold.massList[i] * velocityAway * velocityAway / 2 + calcPotentialOf(i) > 0);
//
    //}

    //calculates potential energy
    double calcPotential() {

        double total = 0;

        for (int i = 0; i < NUM; i++)
            for (int j = i + 1; j < NUM; j++)
            {
                if (i != j)
                    total += ((double)(-G * bodyfold.massList[i] * bodyfold.massList[j])) / (bodyfold.posList[i] - bodyfold.posList[j]).norm();

            }

        return total;
    }

#if VISUALIZE
    bool isWindowOpen() {
        return visuals.isOpen();
    }
#endif


public:

    Solver(int givenT, double givenDt, initialData inits=initialConditions()): bodyfold{inits}, T{givenT}, dt{givenDt}
#if VISUALIZE
    , visuals{800, 800, getSystemRadius()}
#endif
#if COMPARE_QUANTS
    , initialQuants{quantities()}
#endif
    {
        dumpSystemState();
    }

    void run() {
        for (int pass = 0; pass*dt < T; pass++) {
            doSymplecticIntegrator();

#if VISUALIZE
            if (pass%framePerPasses == 0) {
                visuals.visualizationLoop(getDrawInfo());
                if (!isWindowOpen())
                    break;
            }
#endif
#if COMPARE_QUANTS
            if (pass%comparePerPasses == 0) {
                compare(pass);
            }
#endif
        }
    }

    //calculates important quantities
    Quantities quantities() {

        Vector2d mom = bodyfold.sumMomentum();
        double kin = bodyfold.sumKineticEnergy();

        double pot = calcPotential();

        return {mom.x(), mom.y(), kin, pot};
    };

#if COMPARE_QUANTS
    void compare(int pass) {
        cout << "----------" << endl;
        cout << "TIME: " << pass*dt << endl;
        Quantities::compare(quantities(), initialQuants);
    }
#endif

#if VISUALIZE
    const vData &getDrawInfo() {
        return bodyfold.posList;
    }

    double getSystemRadius() {

        double maximum = 0;
        for (int i = 0; i < NUM; i++)
            maximum = max(maximum, bodyfold.posList[i].norm());

        return maximum;
    }
#endif

    void dumpSystemState() {

        string frame = "----------------------------\n";
        string bigFrame = "############################\n";

        string txt = bigFrame;
        txt = txt + "SYSTEM STATE:\n";
        txt = txt + frame;
        txt = txt + bodyfold.toString();
        txt = txt + frame;
        txt = txt + quantities().toString();
        txt = txt + bigFrame;

        cout << txt;

    }
};

#endif
