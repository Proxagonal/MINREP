#ifndef MINREP_SOLVER_H
#define MINREP_SOLVER_H

#include <Eigen/Eigen>
#include "Bodyfold.h"
#include "Quantities.h"
#include "Visualizer.h"

using namespace std;
using namespace Eigen;

#define ANALYSE true
#define HALTCHECK true

#define ORDER 4
static const array<double, ORDER> C = {1/(2*(2-cbrt(2))), (1-cbrt(2))/(2*(2-cbrt(2))), (1-cbrt(2))/(2*(2-cbrt(2))), 1/(2*(2-cbrt(2)))};
static const array<double, ORDER> D = {1/(2-cbrt(2)), -cbrt(2)/(2-cbrt(2)), 1/(2-cbrt(2)), 0};

static const double G = 4*M_PI*M_PI;

typedef uint_fast8_t nat;

class Solver {

public:

    const int T;
    const double dt;


    const int haltCheckPerPasses;
    static const int ratio = 10;
    static const int ratioSquare = ratio*ratio;
    vector<double> changingTimes;
    tuple<int, int> haltStatus;
    vector<tuple<int, int>> haltStatuses;
    int seeWhatHappensAttempts = 5;
    bool unresolved = false;
    double systemTime = 0;
    int factor = 2;
    int limFactor = 5;

    Bodyfold bodyfold;
    vector<double> massRatios;

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
        if (escapeCheckStatus == 2) {
            //Activate ellipse stuff
            return {-1, -1};
        }

        if (isDissolved(distSquares))
            return {3, 3};

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

    // 0: Undecided, 1: Escape, 2: Locked
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

        double ellipseMajorWholeSystem = -muWholeSystem/epsilonWholeSystem;

        // Suspicion of Hierarchical triple system
        return 2;
    }

    bool isDissolved(vector<double> &distSquares) {

        Vector2d relPos;
        Vector2d relVel;

        for (int i = 0; i < NUM; i++) {

            int j = (i + 1) % NUM;

            relPos = bodyfold.posList[j] - bodyfold.posList[i];
            relVel = bodyfold.velList[j] - bodyfold.velList[i];

            // If anything going towards anything else: no.
            if (relPos.dot(relVel) <= 0)
                return false;
        }

        // Minimal distance: if we assume all bodies are this far apart and we get dissolution,
        // then it must happen.
        double minDist = sqrt(*ranges::min_element(distSquares));

        double totalEnergy;
        for (int i = 0; i < NUM; i++) {
            int uno = (i + 1) % NUM;
            int dos = (i + 2) % NUM;

            // Calculate body's energy with this worst-case-scenario distance
            totalEnergy = bodyfold.massList[i]*bodyfold.velList[i].squaredNorm()
                    - G*bodyfold.massList[i]*(bodyfold.massList[uno] + bodyfold.massList[dos])/minDist;

            // Must be enough to escape the current potential. Since all bodies are getting
            // further and further away, this will be enough to escape always.
            if (totalEnergy < 0)
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


    Solver(int givenT, double givenDt, double givenCheckTime, initialData inits=initialConditions()):
    bodyfold{inits}, T{givenT}, dt{givenDt}, haltCheckPerPasses{(int)(givenCheckTime/dt)}
    {
        updateAccelerations();

#if HALTCHECK
        int i, j, k;
        for (i = 0; i < NUM; i++) {
            j = (i+1) % NUM;
            k = (i+2) % NUM;
            massRatios.emplace_back(bodyfold.massList[k] / (bodyfold.massList[j] + bodyfold.massList[k]));
        }

        haltStatuses.emplace_back(haltCheck());
        changingTimes.emplace_back(0);
#endif

    }

    void run() {
        int pass;
        for (pass = 0; pass*dt < factor*T; pass++) {
            doSymplecticIntegrator();

#if HALTCHECK

            if (pass%haltCheckPerPasses == 0) {
                haltStatus = haltCheck();
                if (haltStatus != haltStatuses.back()) {
                    changingTimes.emplace_back(pass*dt);
                    haltStatuses.emplace_back(haltStatus);
                    systemTime += pass*dt;
                    pass = 0;
                    factor = 2 - isHalted(haltStatus);
                    if (systemTime >= limFactor*T) {
                        unresolved = true;
                        return;
                    }
                }
            }
#endif
        }

        systemTime += pass*dt;
    }

    void runDry() {
        int pass;
        for (pass = 0; pass*dt < T; pass++) {
            doSymplecticIntegrator();

#if HALTCHECK

            if (pass%haltCheckPerPasses == 0) {
                haltStatus = haltCheck();
                cout << get<0>(haltStatus) << get<1>(haltStatus);
                if (haltStatus != haltStatuses.back()) {
                    changingTimes.emplace_back(pass*dt);
                    haltStatuses.emplace_back(haltStatus);
                }
            }
#endif
        }
    }

    void runDryNoHalt() {
        int pass;
        for (pass = 0; pass * dt < T; pass++)
            doSymplecticIntegrator();
    }

    //calculates important quantities
    Quantities quantities() {

        Vector2d mom = bodyfold.sumMomentum();
        double kin = bodyfold.sumKineticEnergy();

        double pot = calcPotential();

        return {mom.x(), mom.y(), kin, pot};
    };

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

    bool isHalted(tuple<int, int> tup) {
        int x, y;
        tie(x, y) = tup;
        if (x == 3 and y == 3)
            return true;
        if (x != -1 and x != 3 and y == 1)
            return true;
        return false;
    }

};

#endif
