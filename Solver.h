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

#if VISUALIZE
#include <unistd.h>
#endif

#define ORDER 4
static const array<double, ORDER> C = {1/(2*(2-cbrt(2))), (1-cbrt(2))/(2*(2-cbrt(2))), (1-cbrt(2))/(2*(2-cbrt(2))), 1/(2*(2-cbrt(2)))};
static const array<double, ORDER> D = {1/(2-cbrt(2)), -cbrt(2)/(2-cbrt(2)), 1/(2-cbrt(2)), 0};

static const long double LD_PI = 3.141592653589793238462643383279L;
static const long double LD_G = 1;//4*LD_PI*LD_PI;
static const long double LD_G_inv = 1/LD_G;
static const double G = LD_G;
static const double G_inv = 1/LD_G;


typedef uint_fast8_t nat;

class Solver {

public:

    const int T;
    const double dt;
    const int EACheckPerPasses = 1/dt;
    double time = 0;
    double EAMax = 0;


    int SEEStatus = 0;
    double SEEstart;
    nat SEEbody;
    double SEEAARatio;

    int SEECheckPer = 1/(100*dt);
    int SEEratio = 10;
    int SEEDone = 8;
    // S.E.E: (body, t, delta_t, a_big/a_small)
    vector<tuple<nat, double, double, double>> SEEInfo;

    tuple<nat, double, double, double, double> SKIPtemp = make_tuple(0, 0, 0, 0, 0);
    // SKIP: (body, delta_t, a_big/a_small, EA_B, EA_A)
    vector<tuple<nat, double, double, double, double>> SKIPInfo;

    long sowCount = 0;
    tuple<nat, double, double, double, double, double, double> SOWtemp = make_tuple(0, 0, 0, 0, 0, 0, 0);
    // SOW: (body, delta_t, r^2, v*dt ^ 2, R, EA_B, EA_A)
    vector<tuple<nat, double, double, double, double, double, double>> SOWInfo;



    Bodyfold bodyfold;


#if HALTCHECK
    const int haltCheckPerPasses = 1/dt;
    static const int escapeDistanceRatio = 10;
    static const int edrSquared = escapeDistanceRatio*escapeDistanceRatio;
#endif

#if SOW
    static constexpr double distanceToLengthPerDt_MAX = 2.8;
    const double RsquaredConst = 1/pow(distanceToLengthPerDt_MAX*dt, 2);
    static constexpr double sowingDistanceRatio = pow(10, 2.2);
    static constexpr double sow_drSquared = sowingDistanceRatio*sowingDistanceRatio;
#endif

#if SKIP
    const int skipCheckPerPasses = 1/dt;
    const double skippingDistanceApoapsisRatio = 200;
    const double minPassesForSkip = pow(10, 7); // About 1 real sec

    const double minTimeForSkip = minPassesForSkip * dt;
    const double skip_DARSquared = skippingDistanceApoapsisRatio*skippingDistanceApoapsisRatio;
#endif

#if VISUALIZE
    Visualizer visuals;

    const int savePosPerPasses = (1/dt) / 100;
    const double secondPerFrame = 0.04;

    static constexpr double t_frame_approx = 0.012412223522235222087;
    static constexpr double t_symp_approx = 1.8 * pow(10, -7);
    const int framePerPasses = ceil(t_frame_approx/(secondPerFrame*dt - t_symp_approx));
#endif

#if COMPARE_QUANTS
    Quantities initialQuants;
    const int comparePerPasses = 1/dt;
#endif

    //returns initial conditions of system
    //static initialData initialConditions() {
//
    //    return Bodyfold::generateRandomCOM();
    //}

    static  initialData initialConditions() {

        initialData list;

        double m = 5;
        double rad = 20;
        double speed = -sqrt(m*G/(sqrt(3)*rad));

        Vector2d pos;
        Vector2d vel;

        for (int i = 0; i < 3; i++) {
            pos = rad*Vector2d(cos(i*M_PI*2/3), sin(i*M_PI*2/3));
            vel = speed*Vector2d(-sin(i*M_PI*2/3), cos(i*M_PI*2/3));
            list.emplace_back(m, pos, vel);
        }

        return list;
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

#if SOW || SKIP

    inline static long double cross(Vector2ld &a, Vector2ld &b) {
        return a.x()*b.y() - a.y()*b.x();
    }
    inline static int sign(double x) {
        return 1 - 2*signbit(x);
    }
    inline static int zeroone_negpos(bool b) {
        return 2*b - 1;
    }
    inline static Vector2ld rotate(Vector2ld &v, long double angle) {
        long double cosA = cos(angle);
        long double sinA = sin(angle);

        return {v.x() * cosA - v.y() * sinA, v.x() * sinA + v.y() * cosA};
    }

    // CIRCULAR OUTER LIKELY FALSE

    static tuple<long double, long double, Vector2ld, Vector2ld> mirrorPath(long double mTotal, Vector2ld &posrel, Vector2ld &velrel) {

        long double r_rel = posrel.norm();
        long double v2_rel = velrel.squaredNorm();
        long double vdotr = velrel.dot(posrel);

        long double M_inv = 1/mTotal;
        long double mu = LD_G * mTotal;
        long double mu_inv = M_inv * LD_G_inv;

        long double epsilon = v2_rel/2 - mu/r_rel;

        Vector2ld e = mu_inv * ((epsilon + v2_rel/2)*posrel - vdotr * velrel);
        long double e_mag = e.norm();

        if (e_mag == 0)
            throw std::out_of_range("Insane circular orbit with no resolution");

        int SIGN = zeroone_negpos(cross(posrel, velrel) >= 0); //BOUNDARY COND
        long double t, T;
        if (e_mag < 1) {

            long double sqrt_one_minus_e_squared = sqrt(1-e_mag*e_mag);

            long double Eycomp = sqrt_one_minus_e_squared * ( zeroone_negpos(posrel.dot(velrel) >= 0) * abs(cross(e, posrel)));
            long double Excomp = (e_mag*e_mag*r_rel + e.dot(posrel));

            long double E = atan2(Eycomp, Excomp);
            E = 2*LD_PI*(E < 0) + E;

            long double sinE = Eycomp/sqrt(Eycomp*Eycomp + Excomp*Excomp);
            long double Mean = E - e.norm()*sinE; // ok w.r.t. E

            long double timeroot = sqrt(-1/pow(2*epsilon, 3));
            t = 2 * mu * timeroot * (2*LD_PI - Mean);
            T = 2*LD_PI * mu * timeroot;

        } else if (e_mag > 1) {

            long double sqrt_e_squared_m_one = sqrt(e_mag*e_mag - 1);

            long double Hycomp = SIGN * sqrt_e_squared_m_one * ( zeroone_negpos(posrel.dot(velrel) >= 0) * abs(cross(e, posrel)));
            long double Hxcomp = (e_mag*e_mag*r_rel + e.dot(posrel));

            long double H = atanh(Hycomp/Hxcomp);
            long double sinhH = Hycomp/sqrt(Hxcomp*Hxcomp - Hycomp*Hycomp);

            long double Mean = e.norm()*sinhH - H;

            t = 2 * mu * sqrt(1/(pow(2*epsilon, 3))) * abs(Mean);
            T = 0;

        } else { //PARABOLIC
            long double h = cross(posrel, velrel);
            long double D = (r_rel - posrel.x())/posrel.y();

            long double factor = mu_inv*mu_inv*abs(pow(h, 3))/2;

            t = 2 * abs(factor*D*(1+D*D / 3));
            T = 0;
        }

        Vector2ld ehat = e/e_mag;
        Vector2ld ohat(-ehat.y(), ehat.x());

        Vector2ld pos2New = posrel - 2*posrel.dot(ohat)*ohat;
        Vector2ld vel2New = velrel - 2*velrel.dot(ehat)*ehat;

        return {t, T, pos2New, vel2New};
    }

    static tuple<Vector2ld, Vector2ld> orbitForTime(long double T, long double mTotal, Vector2ld &posrel, Vector2ld &velrel) {
        // EDGE CASES:
        // EPSILON = 0, ECC = 1: seperated case
        // ECC = 0: seperated case
        // r cross v = 0: well done (?)
        // r_vec = 0_vec: ha
        // E0top = pm * E0bottom: never

        long double M_inv = 1/mTotal;

        long double mu = LD_G * mTotal;
        long double mu_inv = LD_G_inv * M_inv;

        long double r_rel = posrel.norm();
        int SIGN = zeroone_negpos(cross(posrel, velrel) >= 0);

        long double epsilon = velrel.squaredNorm()/2 - mu/r_rel;

        Vector2ld e = mu_inv * ((epsilon + velrel.squaredNorm()/2)*posrel - velrel.dot(posrel)*velrel);
        long double e_mag = e.norm();
        long double omega = atan2(e.y(), e.x());

        if (e_mag == 0)
            return orbitForTime_CIRCLE(SIGN * T, epsilon, mu_inv, SIGN, posrel, velrel);
        if (e_mag == 1)
            return orbitForTime_PARABOLA(T, posrel, velrel, mu_inv, omega);

        long double sqrt_one_e_squared, cos_E_cosh_H, sin_E_negsinh_H, a, sqrt_2epsilon;
        if (e_mag > 1) {

            sqrt_one_e_squared = sqrt(e_mag*e_mag - 1);
            a = mu/(2*epsilon);
            sqrt_2epsilon = sqrt(2*epsilon);
            long double n = 2*epsilon*mu_inv * sqrt_2epsilon;

            long double E0top = SIGN * sqrt_one_e_squared * ( zeroone_negpos(posrel.dot(velrel) >= 0) * abs(cross(e, posrel)));
            long double E0bottom = (e_mag*e_mag*r_rel + e.dot(posrel));

            long double H0 = atanh(E0top/E0bottom);
            long double sinhH0 = E0top/sqrt(E0bottom*E0bottom - E0top*E0top);

            long double M0 = e_mag*sinhH0 - H0;

            long double M_true = M0 + SIGN*n*T;

            long double H = Kepler::KEPLER(M_true, e_mag);

            cos_E_cosh_H = cosh(H);
            sin_E_negsinh_H = zeroone_negpos(H <= 0) * sqrt(cos_E_cosh_H*cos_E_cosh_H - 1);

        } else if (e_mag < 1) {

            sqrt_one_e_squared = sqrt(1 - e_mag*e_mag);
            a = -mu/(2*epsilon);
            sqrt_2epsilon = sqrt(-2*epsilon);
            long double n = -2*epsilon*mu_inv * sqrt_2epsilon;


            long double E0top = SIGN * sqrt_one_e_squared * ( zeroone_negpos(posrel.dot(velrel) >= 0) * abs(cross(e, posrel))) ;//zeroone_negpos(posrel.dot(velrel) >= 0) * abs(cross(e, posrel))
            long double E0bottom = (e_mag*e_mag*r_rel + e.dot(posrel));

            long double E0 = atan2(E0top, E0bottom);
            long double sinE0 = E0top/sqrt(E0top*E0top + E0bottom*E0bottom);

            long double M0 = E0 - e_mag*sinE0; // NEG PI TO PI

            long double M_true = M0 + SIGN*n*T;
            int toRange = M_true/(2*LD_PI) - (M_true < 0);

            long double E = Kepler::KEPLER(M_true - toRange * 2 * LD_PI, e_mag);

            cos_E_cosh_H = cos(E);
            sin_E_negsinh_H = zeroone_negpos(E <= LD_PI) * sqrt(1 - cos_E_cosh_H*cos_E_cosh_H);
        }

        long double inv_factor = 1/(1 - e_mag*cos_E_cosh_H);
        long double cosv = (cos_E_cosh_H - e_mag) * inv_factor;
        long double sinv = sqrt_one_e_squared * sin_E_negsinh_H * inv_factor;

        long double r = a*sqrt_one_e_squared*sqrt_one_e_squared/(1 + e_mag*cosv);

        long double vfactor = sqrt_2epsilon/sqrt_one_e_squared;
        long double vr = vfactor * e_mag * sinv;
        long double vtheta = vfactor * (1 + e_mag*cosv);

        Vector2ld TRYPOS(r*cosv, r*sinv);
        Vector2ld TRYVEL(vr*cosv - vtheta*sinv, vr*sinv + vtheta*cosv);

        return {rotate(TRYPOS, omega), SIGN*rotate(TRYVEL, omega)};
    }

    static tuple<Vector2ld, Vector2ld> orbitForTime_CIRCLE(long double T, long double epsilon, long double mu_inv, int SIGN, Vector2ld &posrel, Vector2ld &velrel) {

        long double n = -2*epsilon*mu_inv * sqrt(-2*epsilon);
        long double theta = SIGN*n*T;

        return {rotate(posrel, theta), rotate(velrel, theta)};
    }

    static tuple<Vector2ld, Vector2ld> orbitForTime_PARABOLA(long double T, Vector2ld &posrel, Vector2ld &velrel, long double mu_inv, long double omega) {

        Vector2ld posrelAxis = rotate(posrel, -omega);

        long double h = cross(posrel, velrel);
        int SIGN = 2*(h >= 0) - 1;
        h = abs(h);

        long double r_rel = posrel.norm();
        // x=rcosv, y=rsinv makes it make sense:
        long double D = (r_rel - posrelAxis.x())/posrelAxis.y();

        long double factor = mu_inv*mu_inv*pow(h, 3)/2;

        long double T_fromAxis = factor*D*(1+D*D / 3);

        long double T_new = T_fromAxis + SIGN*T;

        long double A = (3/(2*factor)) * T_new;
        long double B = cbrt(A + sqrt(1+A*A));

        long double nu_new = 2*atan(B - 1/B);

        long double r_new = h*h*mu_inv/(1+cos(nu_new));

        long double cosv = cos(nu_new);
        long double sinv = sin(nu_new);

        long double vfactor = 1/(h*mu_inv);

        Vector2ld TRYPOS(r_new * cosv, r_new * sinv);
        Vector2ld TRYVEL(-vfactor * sinv, vfactor * (1 + cosv));

        return {rotate(TRYPOS, omega), SIGN*rotate(TRYVEL, omega)};
    }

#endif
#if SOW
    long double sowSystem(nat far) {

        nat uno = (far+1) % NUM;
        nat dos = (far+2) % NUM;

        long double innerM = bodyfold.massList[uno] + bodyfold.massList[dos];
        long double M = innerM + bodyfold.massList[far];

        Vector2ld COM = (bodyfold.massList[0]*bodyfold.posList[0]
                        + bodyfold.massList[1]*bodyfold.posList[1]
                        + bodyfold.massList[2]*bodyfold.posList[2]).cast<long double>() / M;
        Vector2ld COMvel = (bodyfold.massList[0]*bodyfold.velList[0]
                        + bodyfold.massList[1]*bodyfold.velList[1]
                        + bodyfold.massList[2]*bodyfold.velList[2]).cast<long double>() / M;

        Vector2ld innerPosRel = (bodyfold.posList[dos] - bodyfold.posList[uno]).cast<long double>();
        Vector2ld innerVelRel = (bodyfold.velList[dos] - bodyfold.velList[uno]).cast<long double>();

        auto [T, total, innerPosRelNew, innerVelRelNew] = mirrorPath(innerM, innerPosRel, innerVelRel);

        Vector2ld innerCOM = (bodyfold.massList[uno]*bodyfold.posList[uno] + bodyfold.massList[dos]*bodyfold.posList[dos]).cast<long double>() / innerM;
        Vector2ld innerCOMvel = (bodyfold.massList[uno]*bodyfold.velList[uno] + bodyfold.massList[dos]*bodyfold.velList[dos]).cast<long double>() / innerM;

        Vector2ld outerPosRel = bodyfold.posList[far].cast<long double>() - innerCOM;
        Vector2ld outerVelRel = bodyfold.velList[far].cast<long double>() - innerCOMvel;

        auto [farNewPos, farNewVel] = orbitForTime(T, M, outerPosRel, outerVelRel);

        COM += T*COMvel;
        bodyfold.posList[far] = (COM + innerM/M * farNewPos).cast<double>();
        bodyfold.velList[far] = (COMvel + innerM/M * farNewVel).cast<double>();

        bodyfold.posList[uno] = (COM - (bodyfold.massList[far]/M) * farNewPos - (bodyfold.massList[dos]/innerM) * innerPosRelNew).cast<double>();
        bodyfold.velList[uno] = (COMvel - bodyfold.massList[far]/M * farNewVel - (bodyfold.massList[dos]/innerM) * innerVelRelNew).cast<double>();
        bodyfold.posList[dos] = (COM - bodyfold.massList[far]/M * farNewPos + (bodyfold.massList[uno]/innerM) * innerPosRelNew).cast<double>();
        bodyfold.velList[dos] = (COMvel - bodyfold.massList[far]/M * farNewVel + (bodyfold.massList[uno]/innerM) * innerVelRelNew).cast<double>();

        return T;
    }
#endif
#if SKIP

    // (Skip time, whether its big enough)
    tuple<long double, bool> skipSystem(nat far) {

        nat uno = (far+1) % NUM;
        nat dos = (far+2) % NUM;

        long double innerM = bodyfold.massList[uno] + bodyfold.massList[dos];
        long double M = innerM + bodyfold.massList[far];

        Vector2ld COM = (bodyfold.massList[0]*bodyfold.posList[0]
                        + bodyfold.massList[1]*bodyfold.posList[1]
                        + bodyfold.massList[2]*bodyfold.posList[2]).cast<long double>() / M;
        Vector2ld COMvel = (bodyfold.massList[0]*bodyfold.velList[0]
                        + bodyfold.massList[1]*bodyfold.velList[1]
                        + bodyfold.massList[2]*bodyfold.velList[2]).cast<long double>() / M;

        Vector2ld innerCOM = (bodyfold.massList[uno]*bodyfold.posList[uno] + bodyfold.massList[dos]*bodyfold.posList[dos]).cast<long double>() / innerM;
        Vector2ld innerCOMvel = (bodyfold.massList[uno]*bodyfold.velList[uno] + bodyfold.massList[dos]*bodyfold.velList[dos]).cast<long double>() / innerM;

        Vector2ld outerPosRel = bodyfold.posList[far].cast<long double>() - innerCOM;
        Vector2ld outerVelRel = bodyfold.velList[far].cast<long double>() - innerCOMvel;

        auto [t_toClose, Ttotal, outerPosRelNew, outerVelRelNew] = mirrorPath(M, outerPosRel, outerVelRel);

        if (Ttotal == 0)
            throw std::domain_error("SKIP: NOT ELLIPTIC");
        long double T = t_toClose - Ttotal;

        if (T < minTimeForSkip)
            return {T, false};

        Vector2ld innerPosRel = (bodyfold.posList[dos] - bodyfold.posList[uno]).cast<long double>();
        Vector2ld innerVelRel = (bodyfold.velList[dos] - bodyfold.velList[uno]).cast<long double>();


        auto [innerPosRelNew, innerVelRelNew] = orbitForTime(T, innerM, innerPosRel, innerVelRel);

        COM += T*COMvel;

        bodyfold.posList[far] = (COM + innerM/M * outerPosRelNew).cast<double>();
        bodyfold.velList[far] = (COMvel + innerM/M * outerVelRelNew).cast<double>();

        bodyfold.posList[uno] = (COM - bodyfold.massList[far]/M * outerPosRelNew - (bodyfold.massList[dos]/innerM) * innerPosRelNew).cast<double>();
        bodyfold.velList[uno] = (COMvel - bodyfold.massList[far]/M * outerVelRelNew - (bodyfold.massList[dos]/innerM) * innerVelRelNew).cast<double>();
        bodyfold.posList[dos] = (COM - bodyfold.massList[far]/M * outerPosRelNew + (bodyfold.massList[uno]/innerM) * innerPosRelNew).cast<double>();
        bodyfold.velList[dos] = (COMvel - bodyfold.massList[far]/M * outerVelRelNew + (bodyfold.massList[uno]/innerM) * innerVelRelNew).cast<double>();

        return {T, true};
    }

    bool checkDAR(nat far) {

        int uno = (far + 1) % NUM;
        int dos = (far + 2) % NUM;

        double mu_ud = G*(bodyfold.massList[uno] + bodyfold.massList[dos]);
        Vector2d velDiff_ud = bodyfold.velList[dos] - bodyfold.velList[uno];


        double epsilon_ud = velDiff_ud.squaredNorm()/2 - mu_ud / (bodyfold.posList[dos] - bodyfold.posList[uno]).norm();

        // This means the binary isn't bound
        if (epsilon_ud >= 0)
            return false;


        Vector2d innerCOM = (bodyfold.massList[uno]*bodyfold.posList[uno] + bodyfold.massList[dos]*bodyfold.posList[dos]) / (bodyfold.massList[uno] + bodyfold.massList[dos]);
        Vector2d innerCOMvel = (bodyfold.massList[uno]*bodyfold.velList[uno] + bodyfold.massList[dos]*bodyfold.velList[dos]) / (bodyfold.massList[uno] + bodyfold.massList[dos]);


        Vector2d deltaPosWholeSystem = bodyfold.posList[far] - innerCOM;
        Vector2d deltaVelWholeSystem = bodyfold.velList[far] - innerCOMvel;
        double muWholeSystem = mu_ud + G*bodyfold.massList[far];


        double epsilonWholeSystem = deltaVelWholeSystem.squaredNorm()/2
                                    - muWholeSystem / deltaPosWholeSystem.norm();

        if (epsilonWholeSystem >= 0)
            return false;

        if (deltaPosWholeSystem.dot(deltaVelWholeSystem) <= 0)
            return false;

        double ellipseMajor_ud = -mu_ud/epsilon_ud;

        get<2>(SKIPtemp) = (-muWholeSystem/epsilonWholeSystem)/ellipseMajor_ud;

        // This means the approximation will not be good at apoapsis
        // Multiply by apo^2 for no division
        return (skip_DARSquared * ellipseMajor_ud * ellipseMajor_ud
                < (bodyfold.posList[far] - innerCOM).squaredNorm());
    }

#endif

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


    tuple<nat, nat> escapeCheck(const vector<double> &distSquares) {

        if (distSquares.at(0) > edrSquared * distSquares.at(1))
            return {0, confirmEscape(distSquares, 0)};
        if (edrSquared * distSquares.at(0) < distSquares.at(1))
            return {2, confirmEscape(distSquares, 2)};
        if (edrSquared * distSquares.at(2) < distSquares.at(1))
            return {1, confirmEscape(distSquares, 1)};

        return {-1, -1};
    }

    // 0: Undecided, 1: Escape, 2: Locked?
    // NOTE: Can save many divisions, but this gets calculated so infrequently that it doesn't matter.
    nat confirmEscape(const vector<double> &distSquares, const nat i) {

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
        if (edrSquared * ellipseMajor_ud * ellipseMajor_ud > distSquares[i])
            return 0;

        Vector2d binaryCOM = (mu*bodyfold.posList[uno] + md*bodyfold.posList[dos])/(mu + md);
        Vector2d binaryCOMVel = (mu*bodyfold.velList[uno] + md*bodyfold.velList[dos])/(mu + md);

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


    bool isDissolved(const vector<double> &distSquares) {

        // for index i, sum of max veloicities of j, k that can be gained from potential energy of i
        vector<double> c1_Plus_c2(NUM, 0);

        for (nat i = 0; i < NUM; i++) {
            nat j = (i + 1) % NUM;

            const double i_j_Potential_noMass = G/sqrt(distSquares[i]);

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

            const double r12 = sqrt(distSquares[uno]);

            // If anything going towards anything else: no.
            // New: In worst case, both velocities may decrease by as much as 2U/m in the direction of the other body. Checks it.
            // (v2-v1)*r12 -> (v2 + c2*ohat - (v1 + c1*ehat))*r12 = (v2-v1)*r12 + (- c1*ehat + c2*ohat)*r12
            // This is smallest when ehat=r12_hat, ohat=-ehat. Therefor, worst case you need: v12*r12 - (c1+c2)|r12| <= 0

            if (relPos.dot(relVel) - c1_Plus_c2[i] * r12 <= 0)
                return false;


            double reducedPosPotential = G*(bodyfold.massList[uno]+bodyfold.massList[dos])/r12;

            // If not enough energy to escape eachother: no.
            // New: In worst case, both velocities may decrease by as much as 2U/m, in some direction.
            // And so, in worst case, we have |v1 - v2 + c1*e + c2*o| for |e|,|o| <= 1 vectors
            // Geometric arg proves if c = (c1+c2)/2, then e', o' with |e'|=|o'|=1 must exist such that
            // c(e' + o') = c1*e + c2*o
            // Then, of course if v = v1-v2 is the original vector, you reduce its magnitude most by going in the opposite direction until you reach 0.
            // Therefore, have e' + o' be in direction -v, with magnitude as big as possible (which is 2). That is unless you'll go further than 0,
            // Then you just want to make them do a zigzag to reach 0 exactly. This is M.
            double M = max(0.0, relVel.norm() - c1_Plus_c2[i]);

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
                if (i != j) {
                    cout << (bodyfold.posList[i] - bodyfold.posList[j]).norm() << endl;
                    total += ((double)(-G * bodyfold.massList[i] * bodyfold.massList[j])) / (bodyfold.posList[i] - bodyfold.posList[j]).norm();

                }

            }
        cout << "-----------" << endl;

        return total;
    }

#if VISUALIZE
    bool isWindowOpen() {
        return visuals.isOpen();
    }
    bool isSlower() {
        return visuals.slowDown();
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

#if VISUALIZE
        if (framePerPasses <= 0)
            throw std::domain_error("Infeasable framerate");
#endif

        dumpSystemStateString();
        updateAccelerations();
    }

    void run() {

#if SKIP
        long passCanCheck = 0;
#endif

#if SOW || SKIP
        long double tSkipped = 0;
#endif

        for (long pass = 0; pass*dt < T; pass++) {

#if SKIP
            if (pass % skipCheckPerPasses == 0 && pass >= passCanCheck) {
                for (nat i = 0; i < NUM; i++) {

                    nat uno = (i+1)%NUM;
                    nat dos = (i+2)%NUM;

                    double smallDistSquared = (bodyfold.posList[dos] - bodyfold.posList[uno]).squaredNorm();
                    double bigDistSquared = (bodyfold.posList[i] - bodyfold.posList[dos]).squaredNorm();

                    if (skip_DARSquared*smallDistSquared < bigDistSquared)
                        if (checkDAR(i)) {

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

                Vector2d relpos = bodyfold.posList[j] - bodyfold.posList[i];
                Vector2d relvel = bodyfold.velList[j] - bodyfold.velList[i];

                // if both sim-bad condition AND getting worse AND body ratio allows it
                if (relvel.squaredNorm() >= RsquaredConst * relpos.squaredNorm()
                    && relpos.dot(relvel) <= 0)

                    if (sow_drSquared*relpos.squaredNorm() < (bodyfold.posList[i] - bodyfold.posList[(j+1)%NUM]).squaredNorm()) {
                        tSkipped += sowSystem((j+1)%NUM);
                        cout << "-----------------" << " CUT " << "-----------------" << endl;
                        break;
                    }
            }
#endif
            doSymplecticIntegrator();

#if HALTCHECK
            if (pass%haltCheckPerPasses == 0) {
                tuple<int, int> result = haltCheck();
                cout << get<0>(result) << " " << get<1>(result) << endl;
            }
#endif

#if VISUALIZE
            if (pass%savePosPerPasses == 0)
                visuals.addToPaths(getDrawInfo());

            if (pass%framePerPasses == 0 || (isSlower() && pass%(1 + framePerPasses/visuals.slowerBy) == 0)) {
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

    bool isExcursion(nat far) {


        int uno = (far + 1) % NUM;
        int dos = (far + 2) % NUM;

        double mu_ud = G*(bodyfold.massList[uno] + bodyfold.massList[dos]);
        Vector2d velDiff_ud = bodyfold.velList[dos] - bodyfold.velList[uno];


        double epsilon_ud = velDiff_ud.squaredNorm()/2 - mu_ud / (bodyfold.posList[dos] - bodyfold.posList[uno]).norm();

        // This means the binary isn't bound
        if (epsilon_ud >= 0)
            return false;


        Vector2d innerCOM = (bodyfold.massList[uno]*bodyfold.posList[uno] + bodyfold.massList[dos]*bodyfold.posList[dos]) / (bodyfold.massList[uno] + bodyfold.massList[dos]);
        Vector2d innerCOMvel = (bodyfold.massList[uno]*bodyfold.velList[uno] + bodyfold.massList[dos]*bodyfold.velList[dos]) / (bodyfold.massList[uno] + bodyfold.massList[dos]);


        Vector2d deltaPosWholeSystem = bodyfold.posList[far] - innerCOM;
        Vector2d deltaVelWholeSystem = bodyfold.velList[far] - innerCOMvel;


        if (deltaPosWholeSystem.dot(deltaVelWholeSystem) <= 0)
            return false;

        double ellipseMajor_ud = -mu_ud/epsilon_ud;

        // This means the approximation will not be good at apoapsis
        // Multiply by apo^2 for no division
        return (SEEratio * SEEratio * ellipseMajor_ud * ellipseMajor_ud
                < (bodyfold.posList[far] - innerCOM).squaredNorm());

    }
/*
    tuple<int, tuple<int, int>> run_TSPDT(double initialEnergy, double divMax) {

        double EA;
        long pass;

        long passCanCheck = 0;
        long double tSkipped = 0;

        for (pass = 0; pass*dt + tSkipped < T; pass++) {

            if (pass % skipCheckPerPasses == 0 && pass >= passCanCheck) {
                for (nat i = 0; i < NUM; i++) {

                    nat uno = (i+1)%NUM;
                    nat dos = (i+2)%NUM;

                    double smallDistSquared = (bodyfold.posList[dos] - bodyfold.posList[uno]).squaredNorm();
                    double bigDistSquared = (bodyfold.posList[i] - bodyfold.posList[dos]).squaredNorm();

                    if (skip_DARSquared*smallDistSquared < bigDistSquared)
                        if (checkDAR(i)) {

                            auto [tSkip, done] = skipSystem(i);

                            if (done) {
                                tSkipped += tSkip;

                                get<0>(SKIPtemp) = i;
                                get<1>(SKIPtemp) = tSkip;
                                get<3>(SKIPtemp) = EA;
                                get<4>(SKIPtemp) = abs((getEnergy() - initialEnergy)/initialEnergy);
                                SKIPInfo.emplace_back(SKIPtemp);
                            } else
                                passCanCheck = pass + tSkip/dt;
                        }
                }
            }

            for (nat i = 0; i < NUM; i++) {

                nat j = (i + 1) % NUM;

                Vector2d relpos = bodyfold.posList[j] - bodyfold.posList[i];
                Vector2d relvel = bodyfold.velList[j] - bodyfold.velList[i];

                // if both sim-bad condition AND getting worse AND body ratio allows it
                if (relvel.squaredNorm() >= RsquaredConst * relpos.squaredNorm()
                    && relpos.dot(relvel) <= 0)

                    if (sow_drSquared*relpos.squaredNorm() < (bodyfold.posList[i] - bodyfold.posList[(j+1)%NUM]).squaredNorm()) {

                        double tSkip = sowSystem((j+1)%NUM);

                        if (sowCount % (long)((pow(10, ceil(log10(sowCount))))/10)) {
                            get<0>(SOWtemp) = (j+1)%NUM;
                            get<1>(SOWtemp) = tSkip;
                            get<2>(SOWtemp) = relpos.squaredNorm();
                            get<3>(SOWtemp) = relvel.squaredNorm();
                            get<4>(SOWtemp) = (bodyfold.posList[i] - bodyfold.posList[(j+1)%NUM]).squaredNorm();
                            get<5>(SOWtemp) = EA;
                            get<6>(SOWtemp) = abs((getEnergy() - initialEnergy)/initialEnergy);
                            SOWInfo.emplace_back(SOWtemp);
                        }
                        sowCount++;


                        tSkipped += tSkip;
                        break;
                    }
            }

            if (pass % SEECheckPer == 0) {

                if (SEEStatus == 0)
                    for (nat i = 0; i < NUM; i++) {
                        nat uno = (i+1)%NUM;
                        nat dos = (i+2)%NUM;

                        double smallDistSquared = (bodyfold.posList[dos] - bodyfold.posList[uno]).squaredNorm();
                        double bigDistSquared = (bodyfold.posList[i] - bodyfold.posList[dos]).squaredNorm();

                        if ((SEEratio - 1)*(SEEratio - 1)*smallDistSquared < bigDistSquared) {

                            if (SEEStatus = isExcursion(i)) {
                                SEEstart = pass*dt + tSkipped;
                                SEEbody = i;
                            }
                        }
                    }

                if (SEEStatus == 1) {

                    nat uno = (SEEbody+1)%NUM;
                    nat dos = (SEEbody+2)%NUM;

                    Vector2d innerCOM = (bodyfold.massList[uno]*bodyfold.posList[uno] + bodyfold.massList[dos]*bodyfold.posList[dos]) / (bodyfold.massList[uno] + bodyfold.massList[dos]);
                    Vector2d innerCOMvel = (bodyfold.massList[uno]*bodyfold.velList[uno] + bodyfold.massList[dos]*bodyfold.velList[dos]) / (bodyfold.massList[uno] + bodyfold.massList[dos]);


                    Vector2d deltaPosWholeSystem = bodyfold.posList[SEEbody] - innerCOM;
                    Vector2d deltaVelWholeSystem = bodyfold.velList[SEEbody] - innerCOMvel;


                    if (deltaPosWholeSystem.dot(deltaVelWholeSystem) <= 0) {

                        double mu_ud = G*(bodyfold.massList[uno] + bodyfold.massList[dos]);
                        Vector2d velDiff_ud = bodyfold.velList[dos] - bodyfold.velList[uno];


                        double epsilon_ud = velDiff_ud.squaredNorm()/2 - mu_ud / (bodyfold.posList[dos] - bodyfold.posList[uno]).norm();

                        double muWholeSystem = mu_ud + G*bodyfold.massList[SEEbody];


                        double epsilonWholeSystem = deltaVelWholeSystem.squaredNorm()/2
                                                    - muWholeSystem / deltaPosWholeSystem.norm();

                        double ellipseMajor_ud = -mu_ud/epsilon_ud;
                        double ellipseMajorWholeSystem = -muWholeSystem/epsilonWholeSystem;

                        SEEAARatio = ellipseMajorWholeSystem / ellipseMajor_ud;
                        SEEStatus = 2;
                    }

                }

                if (SEEStatus == 2) {

                    // Idea from OY meet: make the time from 1:10 to actual chaos be derived from
                    // the upper bound, which is 1:10 exactly.

                    THIS IS COMMENTnat uno = (SEEbody+1)%NUM;
                    nat dos = (SEEbody+2)%NUM;

                    Vector2d innerCOM = (bodyfold.massList[uno]*bodyfold.posList[uno] + bodyfold.massList[dos]*bodyfold.posList[dos]) / (bodyfold.massList[uno] + bodyfold.massList[dos]);
                    Vector2d innerCOMvel = (bodyfold.massList[uno]*bodyfold.velList[uno] + bodyfold.massList[dos]*bodyfold.velList[dos]) / (bodyfold.massList[uno] + bodyfold.massList[dos]);


                    Vector2d deltaPosWholeSystem = bodyfold.posList[SEEbody] - innerCOM;
                    Vector2d deltaVelWholeSystem = bodyfold.velList[SEEbody] - innerCOMvel;

                    double mu_ud = G*(bodyfold.massList[uno] + bodyfold.massList[dos]);
                    Vector2d velDiff_ud = bodyfold.velList[dos] - bodyfold.velList[uno];


                    double epsilon_ud = velDiff_ud.squaredNorm()/2 - mu_ud / (bodyfold.posList[dos] - bodyfold.posList[uno]).norm();

                    double muWholeSystem = mu_ud + G*bodyfold.massList[SEEbody];

                    double ellipseMajor_ud = -mu_ud/epsilon_ud;

                }

            }

            doSymplecticIntegrator();

            if (pass%EACheckPerPasses == 0) {
                EA = abs((getEnergy() - initialEnergy)/initialEnergy);
                EAMax = max(EA, EAMax);
                if (EAMax > divMax) {
                    time += pass*dt + tSkipped;
                    return {-1, {0, 0}};
                }
            }

            if (pass%haltCheckPerPasses == 0) {
                tuple<int, int> result = haltCheck();
                if (result != make_tuple(-1, -1)) {
                    time += pass*dt + tSkipped;
                    return {1, result};
                }
            }
        }

        time += pass*dt + tSkipped;

        EA = abs((getEnergy() - initialEnergy)/initialEnergy);
        EAMax = max(EA, EAMax);
        if (EAMax > divMax) {
            time += pass*dt + tSkipped;
            return {-1, {0, 0}};
        }

        if (pass%haltCheckPerPasses == 0) {
            tuple<int, int> result = haltCheck();
            if (result != make_tuple(-1, -1)) {
                time += pass*dt + tSkipped;
                return {1, result};
            }
        }

        return {0, {0, 0}};
    }*/

    //calculates important quantities
    Quantities quantities() {

        Vector2d mom = bodyfold.sumMomentum();
        double kin = bodyfold.sumKineticEnergy();

        double pot = calcPotential();

        return {mom.x(), mom.y(), kin, pot, bodyfold.sumAngularMomentum()};
    };

#if COMPARE_QUANTS
    void compare(long pass) {
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

    //calculates important quantities
    static Quantities calcQuantities(const initialData &init) {

        Bodyfold bodyfold{init};

        Vector2d mom = bodyfold.sumMomentum();
        double kin = bodyfold.sumKineticEnergy();
        double ang = bodyfold.sumAngularMomentum();

        double pot = calcPotential(bodyfold);

        return {mom.x(), mom.y(), kin, pot, ang};
    };

    static double calcPotential(const Bodyfold &bodyfold) {

        double total = 0;

        for (int i = 0; i < NUM; i++)
            for (int j = i + 1; j < NUM; j++)
            {
                if (i != j)
                    total += ((double)(-G * bodyfold.massList[i] * bodyfold.massList[j])) / (bodyfold.posList[i] - bodyfold.posList[j]).norm();

            }

        return total;
    }

    double getEnergy() {
        return bodyfold.sumKineticEnergy() + calcPotential();
    }

};

#endif
