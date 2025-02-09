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
#define HALTCHECK false

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

    const double distanceToLengthPerDt_MAX = 2000;
    const double RsquaredConst = 1/pow(distanceToLengthPerDt_MAX*dt, 2);
    const double sowingDistanceRatio = 0; //100?
    const double sdrSquared = sowingDistanceRatio*sowingDistanceRatio;

    vector<double> massRatios;

#if VISUALIZE
    Visualizer visuals;
    const int framePerPasses = 100;
#endif
#if COMPARE_QUANTS
    Quantities initialQuants;
    const int comparePerPasses = 20000;
#endif

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
    } //MAKE SURE GOOD

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
        cout << e.transpose() << endl;

        //int SIGN = 1 - 2*signbit(cross(e, posrel)); // conf(cross)

        double EHycomp = -sqrt(2*abs(epsilon)) * abs(vdotr); // not sure about minus sign at all
        double EHxcomp = v2_rel*r_rel - mu;

        double T;
        if (epsilon <= 0) { //USE RADIAL KEPLERS EQUATION

            double E = atan2(EHycomp, EHxcomp);
            E = 2*M_PI*(E < 0) + E;

            double sinE = EHycomp/sqrt(EHycomp*EHycomp + EHxcomp*EHxcomp); //ok w.r.t E
            double Mean = E - e.norm()*sinE; // ok w.r.t. E

            T = 2 * mu * sqrt(-1/pow(2*epsilon, 3)) * (2*M_PI - Mean);

        } else {

            double H = atanh(EHycomp/EHxcomp);
            double sinhH = EHycomp/sqrt(EHxcomp*EHxcomp - EHycomp*EHycomp);

            double Mean = e.norm()*sinhH - H;

            T = 2 * mu * sqrt(1/(pow(2*epsilon, 3))) * abs(Mean);
        }

        Vector2d ehat = e.normalized();
        Vector2d ohat(-ehat.y(), ehat.x());

        Vector2d pos2New = posrel - 2*posrel.dot(ohat)*ohat;
        Vector2d vel2New = velrel - 2*velrel.dot(ehat)*ehat;

        //----- PURE APPROX:
        //YOU NEED TO CALCULATE NEW COM, COMvel ACCORDING TO 2BP with body i, and also correct body i

        Vector2d COM = (bodyfold.massList[uno]*bodyfold.posList[uno] + bodyfold.massList[dos]*bodyfold.posList[dos]) * M_inv;
        Vector2d COMvel = (bodyfold.massList[uno]*bodyfold.velList[uno] + bodyfold.massList[dos]*bodyfold.velList[dos]) * M_inv;

        bodyfold.posList[uno] = - bodyfold.massList[dos]*M_inv * pos2New;
        bodyfold.velList[uno] = - bodyfold.massList[dos]*M_inv * vel2New;
        bodyfold.posList[dos] = + bodyfold.massList[uno]*M_inv * pos2New;
        bodyfold.velList[dos] = + bodyfold.massList[uno]*M_inv * vel2New;

        return {T, COM, COMvel};
    }

    tuple<Vector2d, Vector2d> outerBinaryOnCOMForT(double T, Vector2d &innerCOM, Vector2d &innerCOMvel, nat i) {
        nat uno = (i+1) % NUM;
        nat dos = (i+2) % NUM;

        cout << (int)i << endl;

        Vector2d posrel = bodyfold.posList[i] - innerCOM;
        Vector2d velrel = bodyfold.velList[i] - innerCOMvel;

        //return {posrel, {0,0}};

        cout << "Inner COM: " << innerCOM.transpose() << endl;
        cout << "i: " << bodyfold.posList[i].transpose() << endl;

        cout << "pos, vel: " << posrel.transpose() << " | " << velrel.transpose() << endl;

        double r_rel = posrel.norm();

        //double M_reduced = bodyfold.massList[i]*(bodyfold.massList[uno] + bodyfold.massList[dos])/(bodyfold.massList[0] + bodyfold.massList[1] + bodyfold.massList[2]);
        //double M_inv = 1/M_reduced;
        double M = bodyfold.massList[0] + bodyfold.massList[1] + bodyfold.massList[2];
        double M_inv = 1/M;

        double mu = G*M;
        double mu_inv = G_inv * M_inv;

        double epsilon = velrel.squaredNorm()/2 - mu/r_rel;

        if (epsilon > 0)
            return outerBinaryOnCOMForTHYPER(T, innerCOM, innerCOMvel, i);


        cout << "mu: " << mu << endl;
        cout << "r, epsilon: " << r_rel << ", " << epsilon << endl;
        cout << "r dot v: " << posrel.dot(velrel) << endl;

        //Vector2d e = mu_inv * ((epsilon + velrel.squaredNorm()/2)*posrel - velrel.dot(posrel)*velrel);
        Vector2d e = (velrel.squaredNorm()/mu - 1/r_rel)*posrel - velrel.dot(posrel)*velrel/mu;
        double e_mag = e.norm();
        cout << "e, e mag: " << e.transpose() << ", " << e_mag << endl;

        double sqrt_one_m_e_squared = sqrt(1-e_mag*e_mag);

        //TRY

        double v0 = acos(e.dot(posrel)/(e_mag*r_rel));
        if (posrel.dot(velrel) < 0)
            v0 = 2*M_PI - v0;
        cout << "true anomaly: " << v0 << endl;

        double sinv0 = sin(v0);
        double cosv0 = cos(v0);

        double omega = atan2(e.y(), e.x());
        int SIGN = 1;
        if (cross(posrel, velrel) < 0)
            SIGN = -SIGN;

        double a = -mu/(2*epsilon); // TEST TRIVIAL
        double R = a*(1-e_mag*e_mag)/(1+e_mag*cosv0);

        //return {{R*cos(v0-omega), R*sin(v0-omega)}, velrel};

        cout << "e cross r: " << cross(e, posrel) << endl;

        double E0top = SIGN * sqrt_one_m_e_squared * ( zeroone_negpos(posrel.dot(velrel) >= 0) * abs(cross(e, posrel))) ;//zeroone_negpos(posrel.dot(velrel) >= 0) * abs(cross(e, posrel))
        double E0bottom = (e_mag*e_mag*r_rel + e.dot(posrel));

        double E0 = atan2(E0top, E0bottom);
        cout << "E0 before: " << E0 << endl;
        E0 = 2*M_PI*(E0 < 0) + E0;
        //double sinE0 = E0top/sqrt(E0top*E0top + E0bottom*E0bottom);*

        double sinE0 = sin(E0);
        cout << "Ey, Ex, E0_after, sinE0: " <<  E0top << ", " << E0bottom << ", " << E0 << ", " << sinE0 << endl;


        double M0 = E0 - e_mag*sinE0;
        cout << "HEYO " << M0 << endl;

        double n = sqrt(mu/pow(a,3));

        double M_true = M0 + SIGN*n*T;
        int toRange = M_true/(2*M_PI) - (M_true < 0);

        double E = toRange * 2 * M_PI + Kepler::KEPLER(M_true - toRange * 2 * M_PI, e_mag);
        cout << M_true << ", " << e_mag << ": " << E << endl;

        double cosE = cos(E);
        double cosv = (cosE - e_mag)/(1 - e_mag*cosE);
        double sinv = sqrt_one_m_e_squared*sin(E)/(1-e_mag*cosE);
        //double v = atan2(sinv, cosv) + omega;
        //double sinv = zeroone_negpos(E <= M_PI)*sqrt(1-cosv*cosv); // cosv, sinv good given E, e

        //double r = a*(1-e_mag*e_mag)/(1+e_mag*cosv);

        double vfactor = SIGN/(sqrt(a/mu)*sqrt_one_m_e_squared);
        double vr = vfactor*e_mag*sinv;
        double vtheta = vfactor*(1+e_mag*cosv);

        cout << "ITS ELLIPTING TIME" << endl;

        Vector2d TRYPOS(a*cos(E)-a*e_mag, a*sqrt_one_m_e_squared*sin(E));
        Vector2d TRYVEL(vr*cosv - vtheta*sinv, vr*sinv + vtheta*cosv);


        return {rotate(TRYPOS, omega), rotate(TRYVEL, omega)};
        //return {{r*cos(v), r*sin(v)}, {vr*cos(v) - vtheta*sin(v), vr*sin(v) + vtheta*cos(v)}};
    }

    tuple<Vector2d, Vector2d> outerBinaryOnCOMForTHYPER(double T, Vector2d &innerCOM, Vector2d &innerCOMvel, nat i) {
        nat uno = (i+1) % NUM;
        nat dos = (i+2) % NUM;

        cout << (int)i << endl;

        Vector2d posrel = bodyfold.posList[i] - innerCOM;
        Vector2d velrel = bodyfold.velList[i] - innerCOMvel;

        //return {posrel, {0,0}};

        //cout << "Inner COM: " << innerCOM.transpose() << endl;
        //cout << "i: " << bodyfold.posList[i].transpose() << endl;

        //cout << "pos, vel: " << posrel.transpose() << " | " << velrel.transpose() << endl;

        double r_rel = posrel.norm();

        //double M_reduced = bodyfold.massList[i]*(bodyfold.massList[uno] + bodyfold.massList[dos])/(bodyfold.massList[0] + bodyfold.massList[1] + bodyfold.massList[2]);
        //double M_inv = 1/M_reduced;
        double M = bodyfold.massList[0] + bodyfold.massList[1] + bodyfold.massList[2];
        double M_inv = 1/M;

        double mu = G*M;
        double mu_inv = G_inv * M_inv;

        double epsilon = velrel.squaredNorm()/2 - mu/r_rel;
        //cout << "mu: " << mu << endl;
        //cout << "r, epsilon: " << r_rel << ", " << epsilon << endl;
        //cout << "r dot v: " << posrel.dot(velrel) << endl;

        //Vector2d e = mu_inv * ((epsilon + velrel.squaredNorm()/2)*posrel - velrel.dot(posrel)*velrel);
        Vector2d e = (velrel.squaredNorm()/mu - 1/r_rel)*posrel - velrel.dot(posrel)*velrel/mu;
        double e_mag = e.norm();
        //cout << "e, e mag: " << e.transpose() << ", " << e_mag << endl;

        double sqrt_e_squared_m_one = sqrt(e_mag*e_mag-1);

        //TRY

        //double v0 = acos(e.dot(posrel)/(e_mag*r_rel));
        //if (posrel.dot(velrel) < 0)
        //    v0 = 2*M_PI - v0;
        //cout << "true anomaly: " << v0 << endl;
//
        //double sinv0 = sin(v0);
        //double cosv0 = cos(v0);
//
        double omega = atan2(e.y(), e.x());
        int SIGN = 1;
        if (cross(posrel, velrel) < 0)
            SIGN = -SIGN;

        double a = mu/(2*epsilon); // TEST TRIVIAL
        //double R = a*(1-e_mag*e_mag)/(1+e_mag*cosv0);

        //return {{R*cos(v0-omega), R*sin(v0-omega)}, velrel};

        //cout << "e cross r: " << cross(e, posrel) << endl;

        double E0top = SIGN * sqrt_e_squared_m_one * ( zeroone_negpos(posrel.dot(velrel) >= 0) * abs(cross(e, posrel)));
        double E0bottom = (e_mag*e_mag*r_rel + e.dot(posrel));

        double H0 = atanh(E0top/E0bottom);
        //cout << "H0 before: " << H0 << endl;
        //double sinE0 = E0top/sqrt(E0top*E0top + E0bottom*E0bottom);*

        //cout << "Ey, Ex, E0_after, sinE0: " <<  E0top << ", " << E0bottom << ", " << E0 << ", " << sinE0 << endl;

        //double coshH = cosh(H0);
        //double cosv = (coshH - e_mag)/(1 - e_mag*coshH);
        //double sinv = -sqrt_e_squared_m_one*sinh(H0)/(1-e_mag*coshH);
        //double v = atan2(sinv, cosv) + omega;
        ////double sinv = zeroone_negpos(E <= M_PI)*sqrt(1-cosv*cosv); // cosv, sinv good given E, e
//
        //double r = a*(e_mag*e_mag-1)/(1+e_mag*cosv);
//
        //return {{r*cos(v), r*sin(v)}, velrel};


        double M0 = e_mag*sinh(H0) - H0;
        //cout << "HEYO " << M0 << endl;

        double n = sqrt(mu/pow(a,3));

        //double M_true = n*T + M0;
        //int toRange = M_true/(2*M_PI) - (M_true < 0);
        double M_true = M0 + SIGN*n*T;

        double H = Kepler::KEPLER(M_true, e_mag);
        //cout << M0 << ", " << e_mag << ": " << H << endl;

        double coshH = cosh(H);
        double cosv = (coshH - e_mag)/(1 - e_mag*coshH);
        double sinv = -sqrt_e_squared_m_one*sinh(H)/(1-e_mag*coshH);
        double vfactor = SIGN/(sqrt(a/mu)*sqrt_e_squared_m_one);
        double vr = vfactor*e_mag*sinv;
        double vtheta = vfactor*(1+e_mag*cosv);

        Vector2d TRYPOS(a*(e_mag - cosh(H)), a*sqrt_e_squared_m_one*sinh(H));
        Vector2d TRYVEL(vr*cosv - vtheta*sinv, vr*sinv + vtheta*cosv);

        cout << "ITS HYPERING TIME" << endl;

        return {rotate(TRYPOS, omega), rotate(TRYVEL, omega)};
/*
        double coshH = cosh(H);
        double cosv = (coshH - e_mag)/(1 - e_mag*coshH);
        double sinv = -sqrt_e_squared_m_one*sinh(H)/(1-e_mag*coshH);
        //double v = atan2(sinv, cosv) + omega;
        //double sinv = zeroone_negpos(E <= M_PI)*sqrt(1-cosv*cosv); // cosv, sinv good given E, e

        double r = a*(e_mag*e_mag-1)/(1+e_mag*cosv);

        double vfactor = 1/(sqrt(-a/mu)*sqrt_e_squared_m_one);
        double vr = vfactor*e_mag*sinv;
        double vtheta = vfactor*(1+e_mag*cosv);
        cout << "ITS HYPERING TIME" << endl;

        return {{r*cos(v), r*sin(v)}, {vr*cos(v) - vtheta*sin(v), vr*sin(v) + vtheta*cos(v)}};*/
    }

    double binaryApproximationRun(nat i) {
        cout << bodyfold.toString() << endl;


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

        bodyfold.posList[i] = COM + innerM/M * iNewPos;
        bodyfold.velList[i] = COMvel + innerM/M * iNewVel;

        bodyfold.posList[uno] += COM - bodyfold.massList[i]/M * iNewPos;
        bodyfold.velList[uno] += COMvel - bodyfold.massList[i]/M * iNewVel;
        bodyfold.posList[dos] += COM - bodyfold.massList[i]/M * iNewPos;
        bodyfold.velList[dos] += COMvel - bodyfold.massList[i]/M * iNewVel;

        return T;
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
        
        for (long pass = 0; pass*dt < T; pass++) {

            doSymplecticIntegrator();

#if HALTCHECK
            if (pass%haltCheckPerPasses == 0) {
                tuple<int, int> result = haltCheck();
                cout << get<0>(result) << " " << get<1>(result) << endl;
                cout << pass*dt << endl;
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

    void run_vdt() {

        bool TEMPFLAGDELETEAFTERSOWWORKS = false;

        for (long pass = 0; pass*dt < T; pass++) {

            for (int i = 0; i < NUM; i++) {

                if (TEMPFLAGDELETEAFTERSOWWORKS)
                    break;

                int j = (i + 1) % NUM;

                Vector2d relpos = bodyfold.posList[j] - bodyfold.posList[i];
                Vector2d relvel = bodyfold.velList[j] - bodyfold.velList[i];

                // if both sim-bad condition AND getting worse AND body ratio allows it
                if (relvel.squaredNorm() >= RsquaredConst * relpos.squaredNorm()
                    && relpos.dot(relvel) <= 0)

                    if (sdrSquared*relpos.squaredNorm() < (bodyfold.posList[i] - bodyfold.posList[(j+1)%NUM]).squaredNorm()) {
                        cout << "---------------------------------" << endl;
                        cout << "---------------CUT---------------" << endl;
                        cout << "---------------------------------" << endl;
                        double T = binaryApproximationRun((j+1)%NUM);
                        cout << "Skip: " << T << endl;
                        TEMPFLAGDELETEAFTERSOWWORKS = false;
                        break;
                    }
            }

            doSymplecticIntegrator();

#if HALTCHECK
            if (pass%haltCheckPerPasses == 0) {
                tuple<int, int> result = haltCheck();
                cout << get<0>(result) << " " << get<1>(result) << endl;
                cout << pass*dt << endl;
            }
#endif

#if VISUALIZE
            if (pass%framePerPasses == 0 || (isSlower() && pass%(framePerPasses/10) == 0)) {
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
