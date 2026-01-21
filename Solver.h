#ifndef MINREP_SOLVER_H
#define MINREP_SOLVER_H

#include <Eigen/Eigen>
#include "Bodyfold.h"
#include "Kepler.h"

using namespace std;
using namespace Eigen;

#define HALTCHECK true
#define SOW true
#define SKIP true
#define SEE true

#if NUM != 3 && (HALTCHECK || SOW || SKIP || SEE)
#error "Haltchecking, Sowing, Skipping, Sub-escape Excursion Detection: Only avalible for 3-Body Problem."
#endif


#define ORDER 4
static const array<double, ORDER> C = {1/(2*(2-cbrt(2))), (1-cbrt(2))/(2*(2-cbrt(2))), (1-cbrt(2))/(2*(2-cbrt(2))), 1/(2*(2-cbrt(2)))};
static const array<double, ORDER> D = {1/(2-cbrt(2)), -cbrt(2)/(2-cbrt(2)), 1/(2-cbrt(2)), 0};

typedef uint_fast8_t nat;

class Solver {

public:
    const double dt;
    Bodyfold bodyfold;
    const double crossingTime = G * pow(bodyfold.sumMass(), 2.5) / pow(2*abs(bodyfold.sumEnergy()), 1.5);
    const int crossingTimePasses = crossingTime/dt;
    int see_checkPer;

    enum excursionStatus {
        NONE=0,
        SUSPECTED=1,
        GOING=2,
        RETURNING=3
    };

    enum haltStatus {
        UNDETERMINED=-1,
        ESCAPE_0=0,
        ESCAPE_1=1,
        ESCAPE_2=2,
        DISSOLUTION=3
    };

#if HALTCHECK
    int haltCheck();
#endif
#if SEE
    inline long double AARatio(nat far);
#endif

private:

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

#if HALTCHECK

    static const int escapeDistanceRatio = 10;
    static const int halt_edrSquared = escapeDistanceRatio*escapeDistanceRatio;

    inline int escapeCheck(const vector<double> &distSquares);

    // 0: Undecided, 1: Escape, 2: Locked?
    // NOTE: Can save many divisions, but this gets calculated so infrequently that it doesn't matter.
    bool confirmEscape(const vector<double> &distSquares, const nat i);

    bool isDissolved(const vector<double> &distSquares);

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

    static constexpr double distanceToLengthPerDt_MAX = 2.8;
    const double RsquaredConst = 1/pow(distanceToLengthPerDt_MAX*dt, 2);
    static constexpr double sowingDistanceRatio = pow(10, 2.2);
    static constexpr double sow_drSquared = sowingDistanceRatio*sowingDistanceRatio;

    long double sowSystem(nat far);
#endif
#if SKIP

    const int skip_checkPer;
    const double skippingDistanceApoapsisRatio = 200;
    const long skip_minPasses = pow(10, 7); // About 1 real sec
    const double skip_minTime = skip_minPasses*dt;

    const double skip_darSquared = skippingDistanceApoapsisRatio*skippingDistanceApoapsisRatio;

    // (Skip time, whether its big enough)
    tuple<long double, bool> skipSystem(nat far);

    bool checkDAR(nat far);

#endif

#if SEE
    static const int see_ratio = 3;
    static const int see_detectRatio = 2;

    inline bool isGoingAway(nat far);
    inline bool isFarWithRatio(nat i, double ratio);
    inline int see_0(nat &far);
    inline int see_1(nat &far);
    inline int see_2(nat &far);
    inline int see_3(nat &far);
#endif

public:

    Solver(double givenDt, const initialData &inits, double skipChecksPerCT=0.1, int SeeChecksPerCT=100)
        : bodyfold{inits}, dt{givenDt},
            skip_checkPer{(int)(crossingTimePasses / skipChecksPerCT)},
            see_checkPer{(int)(crossingTimePasses / SeeChecksPerCT)}
    {
        updateAccelerations();
    }
    Solver(double givenDt, const Bodyfold &bf, int skipChecksPerCT=1, int SeeChecksPerCT=500)
    : bodyfold{bf}, dt{givenDt},
        skip_checkPer{crossingTimePasses / skipChecksPerCT},
        see_checkPer{crossingTimePasses / SeeChecksPerCT}
    {
        updateAccelerations();
    }


    long pass = 0;
    long passCanCheck = 0;
#if SOW || SKIP
    long double tSkipped = 0;
#endif
#if SEE
    int exc_status = 0;
    nat exc_body;
#endif

    long double time() {
        return pass*dt
#if SOW || SKIP
        +tSkipped
#endif
        ;
    }

    void runIteration() {

#if SKIP
        if (pass % skip_checkPer == 0 && pass >= passCanCheck) {
            for (nat i = 0; i < NUM; i++) {

                nat uno = (i+1)%NUM;
                nat dos = (i+2)%NUM;

                double smallDistSquared = (bodyfold.posList[dos] - bodyfold.posList[uno]).squaredNorm();
                double bigDistSquared = (bodyfold.posList[i] - bodyfold.posList[dos]).squaredNorm();

                if (skip_darSquared*smallDistSquared < bigDistSquared)
                    if (checkDAR(i)) {
                        auto [tSkip, done] = skipSystem(i);
                        done ? (tSkipped += tSkip) : (passCanCheck = pass + tSkip/dt);
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

                if (sow_drSquared*relpos.squaredNorm() < (bodyfold.posList[i] - bodyfold.posList[(j+1)%NUM]).squaredNorm()) {
                    tSkipped += sowSystem((j+1)%NUM);
                    break;
                }
        }
#endif

#if SEE
        if (pass % see_checkPer == 0) {
            switch(exc_status) {

                case 0: exc_status = see_0(exc_body); break;

                case 1: exc_status = see_1(exc_body); break;

                case 2: exc_status = see_2(exc_body); break;

                case 3: exc_status = see_3(exc_body); break;
            }
        }

#endif

        doSymplecticIntegrator();
        pass++;
    }

    void dumpSystemStateString() {

        string frame = "----------------------------\n";
        string bigFrame = "############################\n";

        string txt = bigFrame;
        txt = txt + "SYSTEM STATE:\n";
        txt = txt + frame;
        txt = txt + bodyfold.toString();
        txt = txt + frame;
        txt = txt + bodyfold.quantities().toString();
        txt = txt + bigFrame;

        cout << txt;

    }


    inline static initialData generateRandomNONCOM();
    inline static initialData generateRandomCOM();


#if DIM == 3

    inline static initialData ergodicScatterRing3D(array<double, 3> m, double innerDist, double outerDist, double innerPhase, double incline);
    inline static initialData ergodicScatterRing3D_eccentric(array<double, 3> m, double r_max, double r_min, double outerDist, double innerPhase, double phi, double incline);

    inline static initialData YOGESH_CONSTRUCTOR(double length);

#elif DIM == 2

    static initialData ergodicScatterRing2D(array<double, 3> m, double innerDist, double outerDist, double innerPhase);
    static initialData ergodicScatterRing2D_eccentric_90deg(array<double, 3> m, double r_max, double r_min, double outerDist, double innerPhase);

#endif


};

#include "HALT.h"
#include "SOWSKIP.h"
#include "SEE.h"
#include "Constructors.h"


#endif
