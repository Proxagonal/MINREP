#ifndef SOWSKIP_H
#define SOWSKIP_H


#if SOW || SKIP

#include "Solver.h"


inline long double Solver::cross(Vector2ld &a, Vector2ld &b) {
    return a.x()*b.y() - a.y()*b.x();
}
inline int Solver::sign(double x) {
    return 1 - 2*signbit(x);
}
inline int Solver::zeroone_negpos(bool b) {
    return 2*b - 1;
}
inline Vector2ld Solver::rotate(Vector2ld &v, long double angle) {
    long double cosA = cos(angle);
    long double sinA = sin(angle);

    return {v.x() * cosA - v.y() * sinA, v.x() * sinA + v.y() * cosA};
}

inline tuple<VectorDld, VectorDld> Solver::subspaceBasis(VectorDld &u, VectorDld &v) {

    if (u.squaredNorm() == 0) {
        if (v.squaredNorm() == 0)
            throw std::domain_error("r12=0, v12=0");
        return subspaceBasis(v, u);
    }
    if (u.squaredNorm()*v.squaredNorm() == pow(u.dot(v), 2)) {

        VectorDld vArb = VectorDld::Zero();
        vArb(0) = -u(1);
        vArb(1) = u(0);
        if (u(0) == 0 && u(1) == 0)
            vArb(0) = 1;
        else
            vArb.normalize();

        return {u.normalized(), vArb};

    }

    VectorDld uhat = u.normalized();

    return {uhat, (v - v.dot(uhat)*uhat).normalized()};
}

inline Vector2ld Solver::inSubspace(VectorDld &u, tuple<VectorDld, VectorDld> &basis) {

    auto &[a, b] = basis;
    return {u.dot(a), u.dot(b)};
}

inline VectorDld Solver::inSpace(Vector2ld &u, tuple<VectorDld, VectorDld> &basis) {

    auto &[a, b] = basis;
    return u.x()*a + u.y()*b;
}

inline tuple<Vector2ld, Vector2ld, tuple<VectorDld, VectorDld>> Solver::toSubspace(VectorDld &u, VectorDld &v) {

    auto basis = subspaceBasis(u, v);

    return {inSubspace(u, basis), inSubspace(v, basis), basis};
}

inline tuple<VectorDld, VectorDld> Solver::toSpace(Vector2ld &u, Vector2ld &v, tuple<VectorDld, VectorDld> &basis) {

    return {inSpace(u, basis), inSpace(v, basis)};
}

// CIRCULAR OUTER LIKELY FALSE

tuple<long double, long double, Vector2ld, Vector2ld> Solver::mirrorPath(long double mTotal, Vector2ld &posrel, Vector2ld &velrel) {

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

tuple<Vector2ld, Vector2ld> Solver::orbitForTime(long double T, long double mTotal, Vector2ld &posrel, Vector2ld &velrel) {
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

tuple<Vector2ld, Vector2ld> Solver::orbitForTime_CIRCLE(long double T, long double epsilon, long double mu_inv, int SIGN, Vector2ld &posrel, Vector2ld &velrel) {

    long double n = -2*epsilon*mu_inv * sqrt(-2*epsilon);
    long double theta = SIGN*n*T;

    return {rotate(posrel, theta), rotate(velrel, theta)};
}

tuple<Vector2ld, Vector2ld> Solver::orbitForTime_PARABOLA(long double T, Vector2ld &posrel, Vector2ld &velrel, long double mu_inv, long double omega) {

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
long double Solver::sowSystem(nat far) {

    nat uno = (far+1) % NUM;
    nat dos = (far+2) % NUM;

    long double innerM = bodyfold.massList[uno] + bodyfold.massList[dos];
    long double M = innerM + bodyfold.massList[far];

    VectorDld COM = (bodyfold.massList[0]*bodyfold.posList[0]
                    + bodyfold.massList[1]*bodyfold.posList[1]
                    + bodyfold.massList[2]*bodyfold.posList[2]).cast<long double>() / M;
    VectorDld COMvel = (bodyfold.massList[0]*bodyfold.velList[0]
                    + bodyfold.massList[1]*bodyfold.velList[1]
                    + bodyfold.massList[2]*bodyfold.velList[2]).cast<long double>() / M;

    VectorDld innerPosRel = (bodyfold.posList[dos] - bodyfold.posList[uno]).cast<long double>();
    VectorDld innerVelRel = (bodyfold.velList[dos] - bodyfold.velList[uno]).cast<long double>();

    auto [IPR_2d, IVR_2d, innerBasis] = toSubspace(innerPosRel, innerVelRel);

    auto [T, total, IPRN_2d, IVRN_2d] = mirrorPath(innerM, IPR_2d, IVR_2d);

    auto [innerPosRelNew, innerVelRelNew] = toSpace(IPRN_2d, IVRN_2d, innerBasis);

    VectorDld innerCOM = (bodyfold.massList[uno]*bodyfold.posList[uno] + bodyfold.massList[dos]*bodyfold.posList[dos]).cast<long double>() / innerM;
    VectorDld innerCOMvel = (bodyfold.massList[uno]*bodyfold.velList[uno] + bodyfold.massList[dos]*bodyfold.velList[dos]).cast<long double>() / innerM;

    VectorDld outerPosRel = bodyfold.posList[far].cast<long double>() - innerCOM;
    VectorDld outerVelRel = bodyfold.velList[far].cast<long double>() - innerCOMvel;

    auto [OPR_2d, OVR_2d, outerBasis] = toSubspace(outerPosRel, outerVelRel);

    auto [FNP_2d, FNV_2d] = orbitForTime(T, M, OPR_2d, OVR_2d);

    auto [farNewPos, farNewVel] = toSpace(FNP_2d, FNV_2d, outerBasis);

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
tuple<long double, bool> Solver::skipSystem(nat far) {

    nat uno = (far+1) % NUM;
    nat dos = (far+2) % NUM;

    long double innerM = bodyfold.massList[uno] + bodyfold.massList[dos];
    long double M = innerM + bodyfold.massList[far];

    VectorDld COM = (bodyfold.massList[0]*bodyfold.posList[0]
                    + bodyfold.massList[1]*bodyfold.posList[1]
                    + bodyfold.massList[2]*bodyfold.posList[2]).cast<long double>() / M;
    VectorDld COMvel = (bodyfold.massList[0]*bodyfold.velList[0]
                    + bodyfold.massList[1]*bodyfold.velList[1]
                    + bodyfold.massList[2]*bodyfold.velList[2]).cast<long double>() / M;

    VectorDld innerCOM = (bodyfold.massList[uno]*bodyfold.posList[uno] + bodyfold.massList[dos]*bodyfold.posList[dos]).cast<long double>() / innerM;
    VectorDld innerCOMvel = (bodyfold.massList[uno]*bodyfold.velList[uno] + bodyfold.massList[dos]*bodyfold.velList[dos]).cast<long double>() / innerM;

    VectorDld outerPosRel = bodyfold.posList[far].cast<long double>() - innerCOM;
    VectorDld outerVelRel = bodyfold.velList[far].cast<long double>() - innerCOMvel;


    auto [OPR_2d, OVR_2d, outerBasis] = toSubspace(outerPosRel, outerVelRel);

    auto [t_toClose, Ttotal, OPRN_2d, OVRN_2d] = mirrorPath(M, OPR_2d, OVR_2d);

    auto [outerPosRelNew, outerVelRelNew] = toSpace(OPRN_2d, OVRN_2d, outerBasis);


    if (Ttotal == 0)
        throw std::domain_error("SKIP: NOT ELLIPTIC");
    long double T = t_toClose - Ttotal;

    if (T < skip_minTime)
        return {T, false};

    VectorDld innerPosRel = (bodyfold.posList[dos] - bodyfold.posList[uno]).cast<long double>();
    VectorDld innerVelRel = (bodyfold.velList[dos] - bodyfold.velList[uno]).cast<long double>();

    auto [IPR_2d, IVR_2d, innerBasis] = toSubspace(innerPosRel, innerVelRel);

    auto [IPRN_2d, IVRN_2d] = orbitForTime(T, innerM, IPR_2d, IVR_2d);

    auto [innerPosRelNew, innerVelRelNew] = toSpace(IPRN_2d, IVRN_2d, innerBasis);


    COM += T*COMvel;

    bodyfold.posList[far] = (COM + innerM/M * outerPosRelNew).cast<double>();
    bodyfold.velList[far] = (COMvel + innerM/M * outerVelRelNew).cast<double>();

    bodyfold.posList[uno] = (COM - bodyfold.massList[far]/M * outerPosRelNew - (bodyfold.massList[dos]/innerM) * innerPosRelNew).cast<double>();
    bodyfold.velList[uno] = (COMvel - bodyfold.massList[far]/M * outerVelRelNew - (bodyfold.massList[dos]/innerM) * innerVelRelNew).cast<double>();
    bodyfold.posList[dos] = (COM - bodyfold.massList[far]/M * outerPosRelNew + (bodyfold.massList[uno]/innerM) * innerPosRelNew).cast<double>();
    bodyfold.velList[dos] = (COMvel - bodyfold.massList[far]/M * outerVelRelNew + (bodyfold.massList[uno]/innerM) * innerVelRelNew).cast<double>();

    return {T, true};
}

bool Solver::checkDAR(nat far) {

    int uno = (far + 1) % NUM;
    int dos = (far + 2) % NUM;

    double mu_ud = G*(bodyfold.massList[uno] + bodyfold.massList[dos]);
    VectorDd velDiff_ud = bodyfold.velList[dos] - bodyfold.velList[uno];


    double epsilon_ud = velDiff_ud.squaredNorm()/2 - mu_ud / (bodyfold.posList[dos] - bodyfold.posList[uno]).norm();

    // This means the binary isn't bound
    if (epsilon_ud >= 0)
        return false;


    VectorDd innerCOM = (bodyfold.massList[uno]*bodyfold.posList[uno] + bodyfold.massList[dos]*bodyfold.posList[dos]) / (bodyfold.massList[uno] + bodyfold.massList[dos]);
    VectorDd innerCOMvel = (bodyfold.massList[uno]*bodyfold.velList[uno] + bodyfold.massList[dos]*bodyfold.velList[dos]) / (bodyfold.massList[uno] + bodyfold.massList[dos]);


    VectorDd deltaPosWholeSystem = bodyfold.posList[far] - innerCOM;
    VectorDd deltaVelWholeSystem = bodyfold.velList[far] - innerCOMvel;
    double muWholeSystem = mu_ud + G*bodyfold.massList[far];


    double epsilonWholeSystem = deltaVelWholeSystem.squaredNorm()/2
                                - muWholeSystem / deltaPosWholeSystem.norm();

    if (epsilonWholeSystem >= 0)
        return false;

    if (deltaPosWholeSystem.dot(deltaVelWholeSystem) <= 0)
        return false;

    double ellipseMajor_ud = -mu_ud/epsilon_ud;

    // This means the approximation will not be good at apoapsis
    // Multiply by apo^2 for no division
    return (skip_darSquared * ellipseMajor_ud * ellipseMajor_ud
            < (bodyfold.posList[far] - innerCOM).squaredNorm());
}

#endif


#endif
