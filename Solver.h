#ifndef MINREP_SOLVER_H
#define MINREP_SOLVER_H

#include <Eigen/Eigen>
#include "Bodyfold.h"
#include "Quantities.h"
#include "Visualizer.h"
#include <stdexcept>


using namespace std;
using namespace Eigen;

#define VISUALIZE false
#define COMPARE_QUANTS false
#define HALTCHECK true

#if VISUALIZE
#include <unistd.h>
#endif

#define ORDER 4
static const array<long double, ORDER> C = {1/(2*(2-cbrt(2))), (1-cbrt(2))/(2*(2-cbrt(2))), (1-cbrt(2))/(2*(2-cbrt(2))), 1/(2*(2-cbrt(2)))};
static const array<long double, ORDER> D = {1/(2-cbrt(2)), -cbrt(2)/(2-cbrt(2)), 1/(2-cbrt(2)), 0};

static const long double G = 4*M_PI*M_PI;

typedef uint_fast8_t nat;

class Solver {

public:

    static const std::array<std::string, 3> GNFOC_Statuses;
    static constexpr int GNFOC_StatusesAmount = GNFOC_Statuses.size();
    vector<tuple<tuple<int, int>, long double>> statuses;

    bool ISI_hEscape = false, ISI_hDiss = false;



    const int T;
    const long double dt;
    const int haltCheckPerPasses = 1/dt;
    long double time = 0;

    static const int ratio = 10;
    static const int ratioSquare = ratio*ratio;

    Bodyfold bodyfold;
    vector<long double> massRatios;

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

        vector<long double> distSquares;

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



    tuple<nat, nat> escapeCheck(const vector<long double> &distSquares) {

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
    nat confirmEscape(const vector<long double> &distSquares, const nat i) {

        int uno = (i + 1) % NUM;
        int dos = (i + 2) % NUM;

        long double mu = bodyfold.massList[uno];
        long double md = bodyfold.massList[dos];
        long double mu_ud = G*(mu + md);
        Vector2d velDiff_ud = bodyfold.velList[dos] - bodyfold.velList[uno];


        long double epsilon_ud = velDiff_ud.squaredNorm()/2 - mu_ud / sqrt(distSquares[uno]);

        // This means the binary isn't bound
        if (epsilon_ud >= 0)
            return 0;

        long double ellipseMajor_ud = -mu_ud/epsilon_ud;

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
        long double muWholeSystem = mu_ud + G*bodyfold.massList[i];


        long double epsilonWholeSystem = deltaVelWholeSystem.squaredNorm()/2
                                    - muWholeSystem / deltaPosWholeSystem.norm();

        if (epsilonWholeSystem > 0)
            // If false then it isn't travelling in escape direction: therefore return code 0
            // If true then escaping away: therefore return code 1
            return deltaPosWholeSystem.dot(deltaVelWholeSystem) > 0;

        //long double ellipseMajorWholeSystem = -muWholeSystem/epsilonWholeSystem;

        // Suspicion of Hierarchical triple system. What this is technically is that both
        // the nested and big two body systems are bound.
        return 2;
    }


    bool isDissolved(const vector<long double> &distSquares) {

        // for index i, sum of max veloicities of j, k that can be gained from potential energy of i
        vector<long double> c1_Plus_c2(NUM, 0);

        for (nat i = 0; i < NUM; i++) {
            nat j = (i + 1) % NUM;

            const long double i_j_Potential_noMass = G/sqrt(distSquares[i]);

            //2*(Uij/mimj)*mi = 2*Uij/mj
            c1_Plus_c2[i] += sqrt(2*i_j_Potential_noMass*bodyfold.massList[i]);
            c1_Plus_c2[j] += sqrt(2*i_j_Potential_noMass*bodyfold.massList[j]);

        }

        // For each i, check for the two other bodies uno, dos, the w.c. velocity condition, AND w.c. epsilon condition
        for (nat i = 0; i < NUM; i++) {

            const int uno = (i + 1) % NUM;
            const int dos = (i + 2) % NUM;

            Vector2d relPos = bodyfold.posList[dos] - bodyfold.posList[uno];
            Vector2d relVel = bodyfold.velList[dos] - bodyfold.velList[uno];

            const long double r12 = sqrt(distSquares[uno]);

            // If anything going towards anything else: no.
            // New: In worst case, both velocities may decrease by as much as 2U/m in the direction of the other body. Checks it.
            // (v2-v1)*r12 -> (v2 + c2*ohat - (v1 + c1*ehat))*r12 = (v2-v1)*r12 + (- c1*ehat + c2*ohat)*r12
            // This is smallest when ehat=r12_hat, ohat=-ehat. Therefor, worst case you need: v12*r12 - (c1+c2)|r12| <= 0

            if (relPos.dot(relVel) - c1_Plus_c2[i] * r12 <= 0)
                return false;


            long double reducedPosPotential = G*(bodyfold.massList[uno]+bodyfold.massList[dos])/r12;

            // If not enough energy to escape eachother: no.
            // New: In worst case, both velocities may decrease by as much as 2U/m, in some direction.
            // And so, in worst case, we have |v1 - v2 + c1*e + c2*o| for |e|,|o| <= 1 vectors
            // Geometric arg proves if c = (c1+c2)/2, then e', o' with |e'|=|o'|=1 must exist such that
            // c(e' + o') = c1*e + c2*o
            // Then, of course if v = v1-v2 is the original vector, you reduce its magnitude most by going in the opposite direction until you reach 0.
            // Therefore, have e' + o' be in direction -v, with magnitude as big as possible (which is 2). That is unless you'll go further than 0,
            // Then you just want to make them do a zigzag to reach 0 exactly. This is M.
            long double M = max(static_cast<long double>(0.0), relVel.norm() - c1_Plus_c2[i]);
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

    bool heuristicDissolution() {
        for (nat i = 0; i < NUM; i++) {

            int j = (i + 1) % NUM;

            Vector2d relPos = bodyfold.posList[j] - bodyfold.posList[i];
            Vector2d relVel = bodyfold.velList[j] - bodyfold.velList[i];

            if (relPos.dot(relVel) <= 0)
                return false;
        }

        return true;
    }

    bool heuristicEscape() {

        vector<long double> distSquares;

        nat j;
        for (nat i = 0; i < NUM; i++) {
            j = (i + 1) % NUM;
            distSquares.emplace_back((bodyfold.posList[i] - bodyfold.posList[j]).squaredNorm());
        }

        int i = -1;

        if (distSquares.at(0) > ratioSquare * distSquares.at(1))
            i = 0;
        if (ratioSquare * distSquares.at(0) < distSquares.at(1))
            i = 2;
        if (ratioSquare * distSquares.at(2) < distSquares.at(1))
            i = 1;

        if (i == -1)
            return false;

        int uno = (i + 1) % NUM;
        int dos = (i + 2) % NUM;

        Vector2d binaryCOMVel = bodyfold.velList[uno]
                        + massRatios[i] * (bodyfold.velList[dos] - bodyfold.velList[uno]);
        Vector2d binaryCOM = bodyfold.posList[uno]
                    + massRatios[i] * (bodyfold.posList[dos] - bodyfold.posList[uno]);

        return ((binaryCOMVel - bodyfold.velList[i]).dot(binaryCOM - bodyfold.posList[i]) > 0);

    }

    static bool heuristicLocker(const bool result, bool &flag, long double &tMaybe, const long double current) {

        if (result && !flag) {
            flag = true;
            tMaybe = current;
        }
        if (!result && flag)
            return false;
        return true;
    }

    static bool heuristicLocker(const bool result, bool &flag) {

        if (result && !flag)
            flag = true;
        if (!result && flag)
            return false;
        return true;
    }

#endif

    //calculates potential energy
    long double calcPotential() {

        long double total = 0;

        for (int i = 0; i < NUM; i++)
            for (int j = i + 1; j < NUM; j++)
            {
                if (i != j)
                    total += ((long double)(-G * bodyfold.massList[i] * bodyfold.massList[j])) / (bodyfold.posList[i] - bodyfold.posList[j]).norm();

            }

        return total;
    }

#if VISUALIZE
    bool isWindowOpen() {
        return visuals.isOpen();
    }
#endif


public:

    Solver(int givenT, long double givenDt, initialData inits=initialConditions()): bodyfold{inits}, T{givenT}, dt{givenDt}
#if VISUALIZE
    , visuals{800, 800, (double)getSystemRadius()}
#endif
#if COMPARE_QUANTS
    , initialQuants{quantities()}
#endif
    {
        updateAccelerations();
        //dumpSystemStateString();

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
            //if (pass%10 == 0 && 1.1*pass*dt > T)
            //    usleep(0);
#endif
#if COMPARE_QUANTS
            if (pass%comparePerPasses == 0) {
                compare(pass);
            }
#endif
        }
    }

    bool run_EANDT_TCYCLE(long double initialEnergy, long double divMax) {

        int timeNeeded = T;
        long pass = 0;

        statuses.emplace_back(haltCheck(), 0);

        for (pass = 0; pass*dt < timeNeeded; pass++) {

            if (pass%haltCheckPerPasses == 0) {
                tuple<int, int> result = haltCheck();
                if (result != get<0>(statuses.back())) {
                    statuses.emplace_back(result, time + pass * dt);
                    timeNeeded = (int)(T - (2*T/3)*(time == 0) + T*(time > 0));
                    time += ceil(pass * dt);
                    pass = 0;
                }

                if (abs((getEnergy() - initialEnergy)/initialEnergy) > divMax) {
                    time += ceil(pass * dt);
                    return false;
                }
            }

            doSymplecticIntegrator();
        }

        time += ceil(pass * dt);
        return true;
    }

    bool run_TTBC(long double initialEnergy, long double divMax) {

        statuses.emplace_back(haltCheck(), 0);

        for (long pass = 0; pass*dt < T; pass++) {

            if (pass%haltCheckPerPasses == 0) {
                tuple<int, int> result = haltCheck();
                if (result != get<0>(statuses.back()))
                    statuses.emplace_back(result, pass * dt);

                if (abs((getEnergy() - initialEnergy)/initialEnergy) > divMax) {
                    time += ceil(pass * dt);
                    return false;
                }
            }

            doSymplecticIntegrator();
        }

        time = T;
        return true;
    }

    int run_GFNOC(long double divMax) {

        long double initialEnergy = getEnergy();

        bool hEscapeRegret = false, hDissRegret = false;

        for (long pass = 0; pass*dt < T; pass++) {

            if (pass%haltCheckPerPasses == 0) {
                tuple<int, int> result = haltCheck();
                if (result != tuple(-1, -1))
                    return 0;
                if (abs((getEnergy() - initialEnergy)/initialEnergy) > divMax)
                    return 1;

                if (pass*dt > 4000) {
                    hEscapeRegret = hEscapeRegret || !heuristicLocker(heuristicEscape(), ISI_hEscape);
                    hDissRegret = hDissRegret || !heuristicLocker(heuristicDissolution(), ISI_hDiss);
                    if (hEscapeRegret && hDissRegret)
                        return 2;
                }
            }

            doSymplecticIntegrator();
        }

        time = T;
        return -1;
    }

    /*
    *         statuses.emplace_back(haltCheck(), 0);

        for (pass = 0; pass*dt < timeNeeded; pass++) {

            if (pass%haltCheckPerPasses == 0) {
                tuple<int, int> result = haltCheck();
                if (result != get<0>(statuses.back())) {
                    statuses.emplace_back(result, time + pass * dt);
     */



    template <size_t N, size_t... Is>
    tuple<Vector2d, Vector2d, Vector2d> as_tuple(std::array<Vector2d, N> const& arr, std::index_sequence<Is...>)
    {
        return std::make_tuple(arr[Is]...);
    }

    // User-facing function that generates an index sequence
    template <size_t N>
    tuple<Vector2d, Vector2d, Vector2d> as_tuple(std::array<Vector2d, N> const& arr)
    {
        return as_tuple(arr, std::make_index_sequence<N>{});
    }

    //calculates important quantities
    Quantities quantities() {

        Vector2d mom = bodyfold.sumMomentum();
        long double kin = bodyfold.sumKineticEnergy();

        long double pot = calcPotential();

        return {mom.x(), mom.y(), kin, pot};
    };

    long double getEnergy() {
        return bodyfold.sumKineticEnergy() + calcPotential();
    }

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

    long double getSystemRadius() {

        long double maximum = 0;
        for (int i = 0; i < NUM; i++)
            maximum = max(maximum, (long double)bodyfold.posList[i].norm());

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

    //calculates important quantities
    static Quantities calcQuantities(const initialData &init) {

        Bodyfold bodyfold{init};

        Vector2d mom = bodyfold.sumMomentum();
        long double kin = bodyfold.sumKineticEnergy();

        long double pot = calcPotential(bodyfold);

        return {mom.x(), mom.y(), kin, pot};
    };

    static long double calcPotential(const Bodyfold &bodyfold) {

        long double total = 0;

        for (int i = 0; i < NUM; i++)
            for (int j = i + 1; j < NUM; j++)
            {
                if (i != j)
                    total += ((long double)(-G * bodyfold.massList[i] * bodyfold.massList[j])) / (bodyfold.posList[i] - bodyfold.posList[j]).norm();

            }

        return total;
    }

};

inline const std::array<std::string, 3> Solver::GNFOC_Statuses = {"Got Prediction", "Energy Inaccurate", "Regretted Heuristic"};


#endif