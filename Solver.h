#ifndef MINREP_SOLVER_H
#define MINREP_SOLVER_H

#include <Eigen/Eigen>
#include "Bodyfold.h"
#include "Quantities.h"
#include "Visualizer.h"

using namespace std;
using namespace Eigen;

#define VISUALIZE true
#define COMPARE_QUANTS false
#define HALTCHECK true

#if VISUALIZE
#include <unistd.h>
#endif

#define ORDER 4
static const array<double, ORDER> C = {1/(2*(2-cbrt(2))), (1-cbrt(2))/(2*(2-cbrt(2))), (1-cbrt(2))/(2*(2-cbrt(2))), 1/(2*(2-cbrt(2)))};
static const array<double, ORDER> D = {1/(2-cbrt(2)), -cbrt(2)/(2-cbrt(2)), 1/(2-cbrt(2)), 0};

static const double G = 4*M_PI*M_PI;

typedef uint_fast8_t nat;

class Solver {

private:

    const int T;
    const double dt;


    static const int haltCheckPerPasses = 2000000;
    static const int ratio = 10;
    static const int ratioSquare = ratio*ratio;

    Bodyfold bodyfold;
    vector<double> massRatios;

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

#if HALTCHECK

    tuple<int, int> haltCheck() {

        vector<double> distSquares;

        nat j;
        for (nat i = 0; i < NUM; i++) {
            j = (i + 1) % NUM;
            distSquares.emplace_back((bodyfold.posList[i] - bodyfold.posList[j]).squaredNorm());
        }

        nat escapeCheckBody, escapeCheckStatus;
        tie(escapeCheckBody, escapeCheckStatus) = escapeCheck(distSquares);

        if (escapeCheckStatus == 1)
            return {escapeCheckBody, escapeCheckStatus};
        //if (escapeCheckStatus == 2) {
        //    //Activate ellipse stuff
        //    return {-1, -1};
        //}

        if (isDissolved(distSquares))
            return {-1, 3};

        // To Be Determined
        return {-1, -1};

    }


    tuple<nat, nat> escapeCheck(vector<double> &distSquares) {

        if (distSquares.at(0) > ratioSquare * distSquares.at(1))
            return {0, confirmEscape(distSquares, 0)};
        if (ratioSquare * distSquares.at(0) < distSquares.at(1))
            return {2, confirmEscape(distSquares, 2)};
        if (ratioSquare * distSquares.at(2) < distSquares.at(1))
            return {1, confirmEscape(distSquares, 1)};

        return {-1, -1};
    }

    // 0: Undecided, 1: Escape, 2: Locked?
    // NOTE: Can save many divisions, but this gets calculated so infrequently that it doesn't matter.
    nat confirmEscape(vector<double> &distSquares, nat i) {

        int uno = (i + 1) % NUM;
        int dos = (i + 2) % NUM;

        double mu = bodyfold.massList[uno];
        double md = bodyfold.massList[dos];
        double mu_ud = G*(mu + md);
        Vector2d velDiff_ud = bodyfold.velList[dos] - bodyfold.velList[uno];


        double epsilon_ud = velDiff_ud.squaredNorm()/2 - mu_ud / sqrt(distSquares[uno]);

        // This means the binary isn't bound
        if (epsilon_ud >= 0)
            return 0;

        double ellipseMajor_ud = -mu_ud/epsilon_ud;

        // This means the approximation will not be good at apoapsis
        // Multiply by eps^2 for no division
        if (ratioSquare * ellipseMajor_ud * ellipseMajor_ud > distSquares[i])
            return 0;

        Vector2d binaryCOM = bodyfold.posList[uno]
                            + massRatios[i] * (bodyfold.posList[dos] - bodyfold.posList[uno]);

        Vector2d binaryCOMVel = bodyfold.velList[uno]
                                + massRatios[i] * velDiff_ud;

        Vector2d deltaPosWholeSystem = bodyfold.posList[i] - binaryCOM;
        Vector2d deltaVelWholeSystem = bodyfold.velList[i] - binaryCOMVel;
        double muWholeSystem = mu_ud + G*bodyfold.massList[i];


        double epsilonWholeSystem = deltaVelWholeSystem.squaredNorm()/2
                                    - muWholeSystem / deltaPosWholeSystem.norm();

        if (epsilonWholeSystem > 0)
            // If false then it isn't travelling in escape direction: therefore return code 0
            // If true then escaping away: therefore return code 1
            return deltaPosWholeSystem.dot(deltaVelWholeSystem) > 0;

        //double ellipseMajorWholeSystem = -muWholeSystem/epsilonWholeSystem;

        // Suspicion of Hierarchical triple system. What this is technically is that both
        // the nested and big two body systems are bound.
        return 2;
    }

    //double relativeVelocity_takenOff(const Vector2d &v1, const Vector2d &v2, )

    bool isDissolved(const vector<double> &distSquares) {

        Vector2d relPos;
        Vector2d relVel;

        vector<double> posPotentials;

        nat j, k;
        for (nat i = 0; i < NUM; i++) {
            j = (i + 1) % NUM;
            posPotentials.emplace_back(G*bodyfold.massList[i]*bodyfold.massList[j]/sqrt(distSquares[i]));
        }

        // Check that all bodies are moving away from eachother
        for (nat i = 0; i < NUM; i++) {

            j = (i + 1) % NUM;
            k = (i + 2) % NUM;

            relPos = bodyfold.posList[j] - bodyfold.posList[i];
            relVel = bodyfold.velList[j] - bodyfold.velList[i];

            // If anything going towards anything else: no.
            // New: In worst case, both velocities may decrease by as much as 2U/m in the direction of the other body. Checks it.
            // (v2-v1)*r12 -> (v2 + c1*ehat - (v1 + c2*ohat))*r12 = (v2-v1)*r12 + (c1*ehat - c2*ohat)*r12
            // This is smallest when ehat=-r12_hat, ohat=-ehat. Therefor, worst case: v12*r12 - (c1+c2)|r12|

            double worstCase = 2*posPotentials[j]/bodyfold.massList[j] + 2*posPotentials[k]/bodyfold.massList[i];

            if (relPos.dot(relVel) <= worstCase)
                return false;
        }

        for (int i = 0; i < NUM; i++) {

            int uno = (i + 1) % NUM;
            int dos = (i + 2) % NUM;

            relVel = bodyfold.velList[dos] - bodyfold.velList[uno];

            double deltavUno = (2/bodyfold.massList[uno])*posPotentials[i]; // potential between i and uno
            double deltavDos = (2/bodyfold.massList[dos])*posPotentials[dos]; // potential between dos and i
            double reducedPosPotential = G*(bodyfold.massList[uno]+bodyfold.massList[dos])/sqrt(distSquares[uno]);

            // If not enough energy to escape eachother: no.
            // New: In worst case, both velocities may decrease by as much as 2U/m, in some direction.
            // And so, in worst case, we have |v1 - v2 + c1*e + c2*o| for |e|,|o| <= 1 vectors
            // Geometric arg proves if c = (c1+c2)/2, then e', o' with |e'|=|o'|=1 must exist such that
            // c(e' + o') = c1*e + c2*o
            // Then, of course if v = v1-v2 is the original vector, you reduce its magnitude most by going in the opposite direction until you reach 0.
            // Therefore, have e' + o' be in direction -v, with magnitude as big as possible (which is 2). That is unless you'll go further than 0,
            // Then you just want to make them do a zigzag to reach 0 exactly. This is M.
            double M = max(0.0, relVel.norm() - deltavUno - deltavDos);
            //cout << "M: " << M << endl;
            //cout << "reduced potential: " << reducedPotential << endl;
            //cout << "dvs: " << deltavUno << ", " << deltavDos << endl;
            //cout << "epsilon: " << M*M/2 - reducedPosPotential << endl;
            //cout << "---" << endl;

            if (M*M/2 - reducedPosPotential <= 0)
                return false;
        }

        return true;
    }

#endif

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
        updateAccelerations();
        dumpSystemStateString();

#if HALTCHECK
        int i, j, k;
        for (i = 0; i < NUM; i++) {
            j = (i+1) % NUM;
            k = (i+2) % NUM;
            massRatios.emplace_back(bodyfold.massList[k] / (bodyfold.massList[j] + bodyfold.massList[k]));
        }
#endif
    }

    void run() {

        for (long pass = 0; pass*dt < T; pass++) {

            doSymplecticIntegrator();

#if HALTCHECK
            if (pass%haltCheckPerPasses == 0) {
                tuple<int, int> result = haltCheck();
                cout << get<0>(result) << " " << get<1>(result) << endl;
            }
#endif

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

    void dumpSystemStateString() {

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
