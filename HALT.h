#ifndef HALT_H
#define HALT_H

#if HALTCHECK

#include "Solver.h"


inline tuple<int, int> Solver::haltCheck() {

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


inline tuple<nat, nat> Solver::escapeCheck(const vector<double> &distSquares) {

    if (distSquares.at(0) > halt_edrSquared * distSquares.at(1))
        return {0, confirmEscape(distSquares, 0)};
    if (halt_edrSquared * distSquares.at(0) < distSquares.at(1))
        return {2, confirmEscape(distSquares, 2)};
    if (halt_edrSquared * distSquares.at(2) < distSquares.at(1))
        return {1, confirmEscape(distSquares, 1)};

    return {-1, -1};
}

// 0: Undecided, 1: Escape, 2: Locked?
// NOTE: Can save many divisions, but this gets calculated so infrequently that it doesn't matter.
inline nat Solver::confirmEscape(const vector<double> &distSquares, const nat i) {

    int uno = (i + 1) % NUM;
    int dos = (i + 2) % NUM;

    double mu = bodyfold.massList[uno];
    double md = bodyfold.massList[dos];
    double mu_ud = G*(mu + md);
    VectorDd velDiff_ud = bodyfold.velList[dos] - bodyfold.velList[uno];


    double epsilon_ud = velDiff_ud.squaredNorm()/2 - mu_ud / sqrt(distSquares[uno]);

    // This means the binary isn't bound
    if (epsilon_ud >= 0)
        return 0;

    double ellipseMajor_ud = -mu_ud/epsilon_ud;

    // This means the approximation will not be good at apoapsis
    // Multiply by eps^2 for no division
    if (halt_edrSquared * ellipseMajor_ud * ellipseMajor_ud > distSquares[i])
        return 0;

    VectorDd binaryCOM = (mu*bodyfold.posList[uno] + md*bodyfold.posList[dos])/(mu + md);
    VectorDd binaryCOMVel = (mu*bodyfold.velList[uno] + md*bodyfold.velList[dos])/(mu + md);

    VectorDd deltaPosWholeSystem = bodyfold.posList[i] - binaryCOM;
    VectorDd deltaVelWholeSystem = bodyfold.velList[i] - binaryCOMVel;
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


inline bool Solver::isDissolved(const vector<double> &distSquares) {

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

        VectorDd relPos = bodyfold.posList[dos] - bodyfold.posList[uno];
        VectorDd relVel = bodyfold.velList[dos] - bodyfold.velList[uno];

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
#endif