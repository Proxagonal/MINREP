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
#define COMPARE_QUANTS false
#define HALTCHECK true
#define SOW true

#if VISUALIZE
#include <unistd.h>
#endif

#define ORDER 4
static const array<double, ORDER> C = {1/(2*(2-cbrt(2))), (1-cbrt(2))/(2*(2-cbrt(2))), (1-cbrt(2))/(2*(2-cbrt(2))), 1/(2*(2-cbrt(2)))};
static const array<double, ORDER> D = {1/(2-cbrt(2)), -cbrt(2)/(2-cbrt(2)), 1/(2-cbrt(2)), 0};

static const double G = 4*M_PI*M_PI;
static const double G_inv = 1/G;


typedef uint_fast8_t nat;

class Solver {

private:

    const int T;
    const double dt;


#if HALTCHECK
    static const int haltCheckPerPasses = 20000;
    static const int escapeDistanceRatio = 10;
    static const int edrSquared = escapeDistanceRatio*escapeDistanceRatio;
#endif

    Bodyfold bodyfold;

#if SOW
    const double distanceToLengthPerDt_MAX = sqrt(10);
    const double RsquaredConst = 1/pow(distanceToLengthPerDt_MAX*dt, 2);
    const double sowingDistanceRatio = sqrt(1000); //100?
    const double sdrSquared = sowingDistanceRatio*sowingDistanceRatio;
#endif

    vector<double> massRatios;

#if VISUALIZE
    Visualizer visuals;
    const int framePerPasses = 100;
#endif
#if COMPARE_QUANTS
    Quantities initialQuants;
    const int comparePerPasses = 2000;
#endif

#if SOW
    inline static double cross(Vector2d &a, Vector2d &b) {
        return a.x()*b.y() - a.y()*b.x();
    }
    inline static int sign(double x) {
        return 1 - 2*signbit(x);
    }
    inline static int zeroone_negpos(bool b) {
        return 2*b - 1;
    }
    inline static Vector2d rotate(Vector2d &v, double angle) {
        double cosA = cos(angle);
        double sinA = sin(angle);

        return {v.x() * cosA - v.y() * sinA, v.x() * sinA + v.y() * cosA};
    }
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

#if SOW
    tuple<double, Vector2d, Vector2d> innerBinarySpinOnZero(nat i) {

        nat uno = (i+1) % NUM;
        nat dos = (i+2) % NUM;

        Vector2d posrel = bodyfold.posList[dos] - bodyfold.posList[uno];
        Vector2d velrel = bodyfold.velList[dos] - bodyfold.velList[uno];

        double r_rel = posrel.norm();
        double v2_rel = velrel.squaredNorm();
        double vdotr = velrel.dot(posrel);

        double M = bodyfold.massList[uno] + bodyfold.massList[dos];
        double M_inv = 1/M;
        double mu = G*M;
        double mu_inv = M_inv * G_inv;

        double epsilon = v2_rel/2 - mu/r_rel;

        Vector2d e = mu_inv * ((epsilon + v2_rel/2)*posrel - vdotr * velrel);
        double e_mag = e.norm();

        if (e_mag == 0)
            throw std::out_of_range("Insane circular orbit with no resolution");

        int SIGN = zeroone_negpos(cross(posrel, velrel) >= 0); //BOUNDARY COND
        double T;
        if (e_mag < 1) {

            double sqrt_one_minus_e_squared = sqrt(1-e_mag*e_mag);

            double Eycomp = sqrt_one_minus_e_squared * ( zeroone_negpos(posrel.dot(velrel) >= 0) * abs(cross(e, posrel)));
            double Excomp = (e_mag*e_mag*r_rel + e.dot(posrel));

            double E = atan2(Eycomp, Excomp);
            E = 2*M_PI*(E < 0) + E;

            double sinE = Eycomp/sqrt(Eycomp*Eycomp + Excomp*Excomp);
            double Mean = E - e.norm()*sinE; // ok w.r.t. E

            T = 2 * mu * sqrt(-1/pow(2*epsilon, 3)) * (2*M_PI - Mean);

        } else if (e_mag > 1) {

            double sqrt_e_squared_m_one = sqrt(e_mag*e_mag - 1);

            double Hycomp = SIGN * sqrt_e_squared_m_one * ( zeroone_negpos(posrel.dot(velrel) >= 0) * abs(cross(e, posrel)));
            double Hxcomp = (e_mag*e_mag*r_rel + e.dot(posrel));

            double H = atanh(Hycomp/Hxcomp);
            double sinhH = Hycomp/sqrt(Hxcomp*Hxcomp - Hycomp*Hycomp);

            double Mean = e.norm()*sinhH - H;

            T = 2 * mu * sqrt(1/(pow(2*epsilon, 3))) * abs(Mean);

        } else { //PARABOLIC
            double h = cross(posrel, velrel);
            double D = (r_rel - posrel.x())/posrel.y();

            double factor = mu_inv*mu_inv*abs(pow(h, 3))/2;

            T = 2 * abs(factor*D*(1+D*D / 3));

        }

        Vector2d ehat = e/e_mag;
        Vector2d ohat(-ehat.y(), ehat.x());

        Vector2d pos2New = posrel - 2*posrel.dot(ohat)*ohat;
        Vector2d vel2New = velrel - 2*velrel.dot(ehat)*ehat;

        Vector2d COM = (bodyfold.massList[uno]*bodyfold.posList[uno] + bodyfold.massList[dos]*bodyfold.posList[dos]) * M_inv;
        Vector2d COMvel = (bodyfold.massList[uno]*bodyfold.velList[uno] + bodyfold.massList[dos]*bodyfold.velList[dos]) * M_inv;

        bodyfold.posList[uno] = - bodyfold.massList[dos]*M_inv * pos2New;
        bodyfold.velList[uno] = - bodyfold.massList[dos]*M_inv * vel2New;
        bodyfold.posList[dos] = + bodyfold.massList[uno]*M_inv * pos2New;
        bodyfold.velList[dos] = + bodyfold.massList[uno]*M_inv * vel2New;

        return {T, COM, COMvel};
    }

    tuple<Vector2d, Vector2d> outerBinaryOnCOM_CIRCLE(double T, double epsilon, double mu_inv, int SIGN, Vector2d &posrel, Vector2d &velrel) {

        double n = -2*epsilon*mu_inv * sqrt(-2*epsilon);
        double theta = SIGN*n*T;

        return {rotate(posrel, theta), rotate(velrel, theta)};
    }

    tuple<Vector2d, Vector2d> outerBinaryOnCOM_PARABOLA(double T, Vector2d &posrel, Vector2d &velrel, double mu_inv, double omega) {

        Vector2d posrelAxis = rotate(posrel, -omega);

        double h = cross(posrel, velrel);
        int SIGN = 2*(h >= 0) - 1;
        h = abs(h);

        double r_rel = posrel.norm();
        // x=rcosv, y=rsinv makes it make sense:
        double D = (r_rel - posrelAxis.x())/posrelAxis.y();

        double factor = mu_inv*mu_inv*pow(h, 3)/2;

        double T_fromAxis = factor*D*(1+D*D / 3);

        double T_new = T_fromAxis + SIGN*T;

        double A = (3/(2*factor)) * T_new;
        double B = cbrt(A + sqrt(1+A*A));

        double nu_new = 2*atan(B - 1/B);

        double r_new = h*h*mu_inv/(1+cos(nu_new));

        double cosv = cos(nu_new);
        double sinv = sin(nu_new);

        double vfactor = 1/(h*mu_inv);

        Vector2d TRYPOS(r_new * cosv, r_new * sinv);
        Vector2d TRYVEL(-vfactor * sinv, vfactor * (1 + cosv));

        return {rotate(TRYPOS, omega), SIGN*rotate(TRYVEL, omega)};
    }

    tuple<Vector2d, Vector2d> outerBinaryOnCOMForT(double T, Vector2d &innerCOM, Vector2d &innerCOMvel, nat i) {

        // EDGE CASES:
        // EPSILON = 0, ECC = 1: seperated case
        // ECC = 0: seperated case
        // r cross v = 0: well done (?)
        // r_vec = 0_vec: ha
        // E0top = pm * E0bottom: never

        double M = bodyfold.massList[0] + bodyfold.massList[1] + bodyfold.massList[2];
        double M_inv = 1/M;

        double mu = G*M;
        double mu_inv = G_inv * M_inv;


        Vector2d posrel = bodyfold.posList[i] - innerCOM;
        Vector2d velrel = bodyfold.velList[i] - innerCOMvel;

        double r_rel = posrel.norm();
        int SIGN = zeroone_negpos(cross(posrel, velrel) >= 0);

        double epsilon = velrel.squaredNorm()/2 - mu/r_rel;

        Vector2d e = mu_inv * ((epsilon + velrel.squaredNorm()/2)*posrel - velrel.dot(posrel)*velrel);
        double e_mag = e.norm();
        double omega = atan2(e.y(), e.x());

        if (e_mag == 0)
            return outerBinaryOnCOM_CIRCLE(SIGN * T, epsilon, mu_inv, SIGN, posrel, velrel);
        if (e_mag == 1)
            return outerBinaryOnCOM_PARABOLA(T, posrel, velrel, mu_inv, omega);

        double sqrt_one_e_squared, cos_E_cosh_H, sin_E_negsinh_H, a, sqrt_2epsilon;
        if (e_mag > 1) {

            sqrt_one_e_squared = sqrt(e_mag*e_mag - 1);
            a = mu/(2*epsilon);
            sqrt_2epsilon = sqrt(2*epsilon);
            double n = 2*epsilon*mu_inv * sqrt_2epsilon;

            double E0top = SIGN * sqrt_one_e_squared * ( zeroone_negpos(posrel.dot(velrel) >= 0) * abs(cross(e, posrel)));
            double E0bottom = (e_mag*e_mag*r_rel + e.dot(posrel));

            double H0 = atanh(E0top/E0bottom);
            double sinhH0 = E0top/sqrt(E0bottom*E0bottom - E0top*E0top);

            double M0 = e_mag*sinhH0 - H0;

            double M_true = M0 + SIGN*n*T;

            double H = Kepler::KEPLER(M_true, e_mag);

            cos_E_cosh_H = cosh(H);
            sin_E_negsinh_H = zeroone_negpos(H <= 0) * sqrt(cos_E_cosh_H*cos_E_cosh_H - 1);

        } else if (e_mag < 1) {

            sqrt_one_e_squared = sqrt(1 - e_mag*e_mag);
            a = -mu/(2*epsilon);
            sqrt_2epsilon = sqrt(-2*epsilon);
            double n = -2*epsilon*mu_inv * sqrt_2epsilon;


            double E0top = SIGN * sqrt_one_e_squared * ( zeroone_negpos(posrel.dot(velrel) >= 0) * abs(cross(e, posrel))) ;//zeroone_negpos(posrel.dot(velrel) >= 0) * abs(cross(e, posrel))
            double E0bottom = (e_mag*e_mag*r_rel + e.dot(posrel));

            double E0 = atan2(E0top, E0bottom);
            double sinE0 = E0top/sqrt(E0top*E0top + E0bottom*E0bottom);

            double M0 = E0 - e_mag*sinE0; // NEG PI TO PI

            double M_true = M0 + SIGN*n*T;
            int toRange = M_true/(2*M_PI) - (M_true < 0);

            double E = Kepler::KEPLER(M_true - toRange * 2 * M_PI, e_mag);

            cos_E_cosh_H = cos(E);
            sin_E_negsinh_H = zeroone_negpos(E <= M_PI) * sqrt(1 - cos_E_cosh_H*cos_E_cosh_H);
        }

        double inv_factor = 1/(1 - e_mag*cos_E_cosh_H);
        double cosv = (cos_E_cosh_H - e_mag) * inv_factor;
        double sinv = sqrt_one_e_squared * sin_E_negsinh_H * inv_factor;

        double r = a*sqrt_one_e_squared*sqrt_one_e_squared/(1 + e_mag*cosv);

        double vfactor = sqrt_2epsilon/sqrt_one_e_squared;
        double vr = vfactor * e_mag * sinv;
        double vtheta = vfactor * (1 + e_mag*cosv);

        Vector2d TRYPOS(r*cosv, r*sinv);
        Vector2d TRYVEL(vr*cosv - vtheta*sinv, vr*sinv + vtheta*cosv);

        return {rotate(TRYPOS, omega), SIGN*rotate(TRYVEL, omega)};
    }

    double binaryApproximationRun(nat i) {

        nat uno = (i+1) % NUM;
        nat dos = (i+2) % NUM;

        double innerM = bodyfold.massList[uno] + bodyfold.massList[dos];
        double M = innerM + bodyfold.massList[i];

        Vector2d COM = (bodyfold.massList[0]*bodyfold.posList[0]
                        + bodyfold.massList[1]*bodyfold.posList[1]
                        + bodyfold.massList[2]*bodyfold.posList[2]) / M;
        Vector2d COMvel = (bodyfold.massList[0]*bodyfold.velList[0]
                        + bodyfold.massList[1]*bodyfold.velList[1]
                        + bodyfold.massList[2]*bodyfold.velList[2]) / M;


        auto [T, innerCOM, innerCOMvel] = innerBinarySpinOnZero(i);

        auto [iNewPos, iNewVel] = outerBinaryOnCOMForT(T, innerCOM, innerCOMvel, i);

        COM += T*COMvel;
        bodyfold.posList[i] = COM + innerM/M * iNewPos;
        bodyfold.velList[i] = COMvel + innerM/M * iNewVel;

        bodyfold.posList[uno] += COM - bodyfold.massList[i]/M * iNewPos;
        bodyfold.velList[uno] += COMvel - bodyfold.massList[i]/M * iNewVel;
        bodyfold.posList[dos] += COM - bodyfold.massList[i]/M * iNewPos;
        bodyfold.velList[dos] += COMvel - bodyfold.massList[i]/M * iNewVel;

        return T;
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

#if SOW
        double tSkipped = 0;
#endif
        
        for (long pass = 0; pass*dt < T; pass++) {

#if SOW
            for (int i = 0; i < NUM; i++) {

                int j = (i + 1) % NUM;

                Vector2d relpos = bodyfold.posList[j] - bodyfold.posList[i];
                Vector2d relvel = bodyfold.velList[j] - bodyfold.velList[i];

                // if both sim-bad condition AND getting worse AND body ratio allows it
                if (relvel.squaredNorm() >= RsquaredConst * relpos.squaredNorm()
                    && relpos.dot(relvel) <= 0)

                    if (sdrSquared*relpos.squaredNorm() < (bodyfold.posList[i] - bodyfold.posList[(j+1)%NUM]).squaredNorm()) {
                        tSkipped += binaryApproximationRun((j+1)%NUM);
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
                cout << pass*dt << endl;
            }
#endif

#if VISUALIZE
            if (pass%framePerPasses == 0 || (isSlower() && pass%(framePerPasses/visuals.slowerBy) == 0)) {
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

};

#endif
