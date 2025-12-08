#ifndef MINREP_SOLVER_H
#define MINREP_SOLVER_H

#include <Eigen/Eigen>
#include "Bodyfold.h"
#include "Quantities.h"
#include "Visualizer.h"
#include "Kepler.h"

using namespace std;
using namespace Eigen;

#define VISUALIZE true
#define COMPARE_QUANTS true
#define HALTCHECK true
#define SOW true
#define SKIP true
#define SEE true


#define ORDER 4
static const array<double, ORDER> C = {1/(2*(2-cbrt(2))), (1-cbrt(2))/(2*(2-cbrt(2))), (1-cbrt(2))/(2*(2-cbrt(2))), 1/(2*(2-cbrt(2)))};
static const array<double, ORDER> D = {1/(2-cbrt(2)), -cbrt(2)/(2-cbrt(2)), 1/(2-cbrt(2)), 0};

static const long double LD_PI = 3.141592653589793238462643383279L;
static const long double LD_G = 4*LD_PI*LD_PI;
static const long double LD_G_inv = 1/LD_G;
static const double G = LD_G;
static const double G_inv = 1/LD_G;


typedef uint_fast8_t nat;

class Solver {

private:

    const int T;
    const double dt;

    Bodyfold bodyfold;

    //returns initial conditions of system
    static initialData initialConditions() {

        return Bodyfold::generateRandomCOM();
    }

    void doSymplecticIntegrator() {

        for (nat p = 0; p < ORDER-1; p++) {
            updateAccelerations();
            for (nat i = 0; i < NUM; i++) {
                bodyfold.velList[i] += C[p] * dt * bodyfold.accList[i];
                bodyfold.posList[i] += D[p] * dt * bodyfold.velList[i];
            }
        }
        updateAccelerations();
        for (nat i = 0; i < NUM; i++)
            bodyfold.velList[i] += C[3] * dt * bodyfold.accList[i];
    }

    void updateAccelerations() {

        VectorDd mutualVector;

        for (VectorDd &acc : bodyfold.accList)
            acc.setZero();

        nat j;
        for (nat i = 0; i < NUM; i++) {
            j = (i+1)%NUM;
            mutualVector = directedInverseSquare(bodyfold.posList[i], bodyfold.posList[j]);
            bodyfold.accList[i] += bodyfold.massList[j] * mutualVector;
            bodyfold.accList[j] += - bodyfold.massList[i] * mutualVector;
        }
    }

    static VectorDd directedInverseSquare(const VectorDd &pos1, const VectorDd &pos2) {

        VectorDd diff = pos2 - pos1;
        return G * diff / (diff.norm() * diff.squaredNorm());
    }

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

    double sqrdist(nat i);
    bool isForceHierarchy(nat i, double FR);
    bool Solver::isForceHierarchy(nat i, double FR, double smallsqr, double bigsqr);

#if VISUALIZE

    Visualizer visuals;

    const int savePosPerPasses = (1/dt) / 100;
    const double secondPerFrame = 0.5;

    static constexpr double t_frame_approx = 0.012412223522235222087;
    static constexpr double t_symp_approx = 1.8 * pow(10, -7);
    const int framePerPasses = ceil(t_frame_approx/(secondPerFrame*dt - t_symp_approx));

#endif

#if COMPARE_QUANTS
    Quantities initialQuants;
    const int compare_checkPer = 1/dt;
#endif

#if HALTCHECK

    const int halt_checkPer = 1/dt;
    static const int escapeDistanceRatio = 10;
    static const int halt_FR = escapeDistanceRatio*escapeDistanceRatio;

    tuple<int, int> haltCheck();

    inline tuple<nat, nat> escapeCheck();

    // 0: Undecided, 1: Escape, 2: Locked?
    // NOTE: Can save many divisions, but this gets calculated so infrequently that it doesn't matter.
    nat confirmEscape(const nat i);

    bool isDissolved();

#endif

#if SOW || SKIP

    inline static long double cross(Vector2ld &a, Vector2ld &b);
    inline static int sign(double x);
    inline static int zeroone_negpos(bool b);
    inline static Vector2ld rotate(Vector2ld &v, long double angle);

    inline static tuple<VectorDld, VectorDld> subspaceBasis(VectorDld &u, VectorDld &v);
    inline static Vector2ld inSubspace(VectorDld &u, tuple<VectorDld, VectorDld> &basis);
    inline static VectorDld inSpace(Vector2ld &u, tuple<VectorDld, VectorDld> &basis);
    inline static tuple<Vector2ld, Vector2ld, tuple<VectorDld, VectorDld>> toSubspace(VectorDld &u, VectorDld &v);
    inline static tuple<VectorDld, VectorDld> toSpace(Vector2ld &u, Vector2ld &v, tuple<VectorDld, VectorDld> &basis);

    // CIRCULAR OUTER LIKELY FALSE

    static tuple<long double, long double, Vector2ld, Vector2ld> mirrorPath(long double mTotal, Vector2ld &posrel, Vector2ld &velrel);

    static tuple<Vector2ld, Vector2ld> orbitForTime(long double T, long double mTotal, Vector2ld &posrel, Vector2ld &velrel);
    static tuple<Vector2ld, Vector2ld> orbitForTime_CIRCLE(long double T, long double epsilon, long double mu_inv, int SIGN, Vector2ld &posrel, Vector2ld &velrel);
    static tuple<Vector2ld, Vector2ld> orbitForTime_PARABOLA(long double T, Vector2ld &posrel, Vector2ld &velrel, long double mu_inv, long double omega);

#endif
#if SOW

    static constexpr double distanceToLengthPerDt_MAX = 2.8; //NEED FORCE MOD
    const double RsquaredConst = 1/pow(distanceToLengthPerDt_MAX*dt, 2);
    static constexpr double sowingDistanceRatio = pow(10, 2.2);
    static constexpr double sow_FR = sowingDistanceRatio*sowingDistanceRatio;

    long double sowSystem(nat far);
#endif
#if SKIP

    const int skip_checkPer = 1/dt;
    const double skippingDistanceApoapsisRatio = 200; //FORCE
    const long skip_minPasses = pow(10, 7); // About 1 real sec
    const double skip_minTime = skip_minPasses*dt;

    const double skip_dafr = skippingDistanceApoapsisRatio*skippingDistanceApoapsisRatio;

    // (Skip time, whether its big enough)
    tuple<long double, bool> skipSystem(nat far);

    bool checkAFR(nat far);

#endif

#if SEE
    int see_checkPer = 1/(100*dt);

    static const int see_ratio = 4;
    static const int see_detectRatio = 2;

    static const int see_FR = see_ratio*see_ratio;
    static const int see_below_FR = (see_ratio-1)*(see_ratio-1);
    static const int see_detectFR = see_detectRatio*see_detectRatio;

    inline bool isGoingAway(nat far);
    inline bool isWeakWithRatio(nat i, double FR);
    inline int see_0(nat &far);
    inline int see_1(nat &far);
    inline int see_2(nat &far);
    inline int see_3(nat &far);
    inline long double AARatio(nat far);

    array<int(Solver::*)(nat&), 4> see_operator = {&Solver::see_0, &Solver::see_1, &Solver::see_2, &Solver::see_3};
#endif

public:

    Solver(int givenT, double givenDt, initialData inits=initialConditions())
        : bodyfold{inits}, T{givenT}, dt{givenDt}
#if VISUALIZE
        , visuals{800, 800, bodyfold.posList}
#endif
#if COMPARE_QUANTS
        , initialQuants{quantities()}
#endif
    {

#if VISUALIZE
        if (framePerPasses <= 0)
            throw std::domain_error("Unfeasible framerate");
#endif

        updateAccelerations();
        dumpSystemStateString();
    }

    void run() {

#if SKIP
        long passCanCheck = 0;
#endif

#if SOW || SKIP
        long double tSkipped = 0;
#define TIME() (pass*dt + tSkipped)
#else
#define TIME() (pass*dt)
#endif

#if SEE
        double see_startTime;
        int see_status = 0;
        nat see_body;
#endif

        for (long pass = 0; TIME() < T; pass++) {

#if SKIP
            if (pass % skip_checkPer == 0 && pass >= passCanCheck) {
                for (nat i = 0; i < NUM; i++) {

                    nat uno = (i+1)%NUM;
                    nat dos = (i+2)%NUM;

                    double smallDistSquared = sqrdist(uno);
                    double bigDistSquared = sqrdist(dos);

                    if ()
                    if (skip_darSquared*smallDistSquared < bigDistSquared)
                        if (checkAFR(i)) {

                            auto [tSkip, done] = skipSystem(i);
                            done ? (tSkipped += tSkip) : (passCanCheck = pass + tSkip/dt);
                            cout << "-----------------" << " SKIP: " << done * tSkipped << "-----------------" << endl;
                        }
                }
            }
#endif
#if SOW
            for (nat i = 0; i < NUM; i++) {

                nat j = (i + 1) % NUM;

                VectorDd relpos = bodyfold.posList[j] - bodyfold.posList[i];
                VectorDd relvel = bodyfold.velList[j] - bodyfold.velList[i];

                // if both sim-bad condition AND getting worse AND body ratio allows it
                if (relvel.squaredNorm() >= RsquaredConst * relpos.squaredNorm()
                    && relpos.dot(relvel) <= 0)

                    if (isForceHierarchy(i, sow_FR)) {
                        tSkipped += sowSystem((j+1)%NUM);
                        cout << "-----------------" << " CUT " << "-----------------" << endl;
                        break;
                    }
            }
#endif

#if SEE
            if (pass % see_checkPer == 0) {
                switch(see_status) {

                    case 0: see_status = see_0(see_body);
                        if (see_status == 1) {
                            cout << "TIME" << endl;
                            see_startTime = TIME();
                        }
                        break;

                    case 1: see_status = see_1(see_body);
                        if (see_status == 2)
                            cout << "SEE OFFICIAL" << endl;
                        break;

                    case 2: see_status = see_2(see_body); break;

                    case 3: see_status = see_3(see_body);
                        if (see_status == 0)
                            cout << "SEE TIME: " << TIME() - see_startTime << endl;;
                        break;
                }
            }

#endif


            doSymplecticIntegrator();

#if HALTCHECK
            if (pass%halt_checkPer == 0) {
                tuple<int, int> result = haltCheck();
                cout << get<0>(result) << " " << get<1>(result) << endl;
            }
#endif

#if VISUALIZE
            if (pass%savePosPerPasses == 0)
                visuals.addToPaths(bodyfold.posList);

            if (pass%framePerPasses == 0 || (visuals.slowDown() && pass%(1 + framePerPasses/visuals.slowerBy) == 0)) {
                visuals.visualizationLoop(bodyfold.posList);
                if (!visuals.isOpen())
                    break;
            }
#endif
#if COMPARE_QUANTS
            if (pass%compare_checkPer == 0) {
                compare(TIME());
            }
#endif
        }
    }

    //calculates important quantities
    Quantities quantities() {

        VectorDd wpos = bodyfold.getWeightedPosition();
        VectorDd mom = bodyfold.sumMomentum();
        VectorAngd angMom = bodyfold.sumAngularMomentum();

        double kin = bodyfold.sumKineticEnergy();
        double pot = calcPotential();

        return {wpos, mom, angMom, kin, pot};
    };

#if COMPARE_QUANTS
    void compare(const double time) {
        cout << "----------" << endl;
        cout << "TIME: " << time << endl;
        Quantities::compare(quantities(), initialQuants);
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

inline double Solver::sqrdist(nat i) {
    return (bodyfold.posList[i] - bodyfold.posList[(i+1)%NUM]).squaredNorm();
}

// If true, i and i+1 are a tight binary with respect to i+2
inline bool Solver::isForceHierarchy(nat i, double FR) {
    return (bodyfold.massList[(i+1)%NUM] * sqrdist((i+2)%NUM) > FR * bodyfold.massList[(i+2)%NUM] * sqrdist(i%NUM));
}

inline bool Solver::isForceHierarchy(nat i, double FR, double smallsqr, double bigsqr) {
    return (bodyfold.massList[(i+1)%NUM] * bigsqr > FR * bodyfold.massList[(i+2)%NUM] * smallsqr);
}

#include "HALT.h"
#include "SOWSKIP.h"
#include "SEE.h"

#endif
