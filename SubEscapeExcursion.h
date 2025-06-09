#ifndef SUBESCAPEEXCURSION_H
#define SUBESCAPEEXCURSION_H

#include "Solver.h"

inline bool Solver::isGoingAway(nat far) {

    nat uno = (far+1)%NUM;
    nat dos = (far+2)%NUM;

    VectorDd innerWeightedPos = (bodyfold.massList[uno]*bodyfold.posList[uno] + bodyfold.massList[dos]*bodyfold.posList[dos]);// / (bodyfold.massList[uno] + bodyfold.massList[dos]);
    VectorDd innerWeightedVel = (bodyfold.massList[uno]*bodyfold.velList[uno] + bodyfold.massList[dos]*bodyfold.velList[dos]);// / (bodyfold.massList[uno] + bodyfold.massList[dos]);

    VectorDd MassMultipliedDeltaPosWholeSystem = (bodyfold.massList[uno] + bodyfold.massList[dos])*bodyfold.posList[SEE_body] - innerWeightedPos;
    VectorDd MassMultipliedDeltaVelWholeSystem = (bodyfold.massList[uno] + bodyfold.massList[dos])*bodyfold.velList[SEE_body] - innerWeightedVel;

    return (MassMultipliedDeltaPosWholeSystem.dot(MassMultipliedDeltaVelWholeSystem) > 0);
}

inline bool Solver::isFarWithRatio(nat i, double ratio) {

    nat uno = (i+1)%NUM;
    nat dos = (i+2)%NUM;

    double smallDistSquared = (bodyfold.posList[dos] - bodyfold.posList[uno]).squaredNorm();
    double bigDistSquared = (bodyfold.posList[i] - bodyfold.posList[dos]).squaredNorm();

    return (ratio*ratio * smallDistSquared < bigDistSquared);
}


inline nat Solver::SEE_0() {

    for (nat i = 0; i < NUM; i++)

        if (isFarWithRatio(i, SEE_detectRatio + 1) && isGoingAway(i))
            return i;

    return NUM;
}

inline int Solver::SEE_1(nat far) {

    if (!isGoingAway(far) || !isFarWithRatio(far, SEE_detectRatio + 1))
        return -1;

    nat uno = (far+1)%NUM;
    nat dos = (far+2)%NUM;

    if (isFarWithRatio(far, SEE_ratio - 1)) {

        double mu_ud = G*(bodyfold.massList[uno] + bodyfold.massList[dos]);
        VectorDd velDiff_ud = bodyfold.velList[dos] - bodyfold.velList[uno];

        double epsilon_ud = velDiff_ud.squaredNorm()/2 - mu_ud / (bodyfold.posList[dos] - bodyfold.posList[uno]).norm();

        // This means the binary isn't bound
        if (epsilon_ud >= 0)
            return -1;

        double ellipseMajor_ud = -mu_ud/epsilon_ud;

        VectorDd MassMultipliedDeltaPosWholeSystem = (bodyfold.massList[uno] + bodyfold.massList[dos])*bodyfold.posList[far]
                                                    - (bodyfold.massList[uno]*bodyfold.posList[uno] + bodyfold.massList[dos]*bodyfold.posList[dos]);

        // This means the approximation will not be good at apoapsis
        // Multiply by apo^2 for no division
        return (pow(SEE_ratio * ellipseMajor_ud * (bodyfold.massList[uno] + bodyfold.massList[dos]), 2)
            < MassMultipliedDeltaPosWholeSystem.squaredNorm());
    }

    return 0;
}

inline bool Solver::SEE_2(nat far) {

    return !isGoingAway(far);
}

inline bool Solver::SEE_3(nat far) {

    return !isFarWithRatio(far, SEE_detectRatio + 1);
}



inline long double Solver::AARatio(nat far) {

    nat uno = (SEE_body+1)%NUM;
    nat dos = (SEE_body+2)%NUM;

    VectorDd deltaPosWholeSystem = bodyfold.posList[SEE_body] - (bodyfold.massList[uno]*bodyfold.posList[uno] + bodyfold.massList[dos]*bodyfold.posList[dos])/(bodyfold.massList[uno] + bodyfold.massList[dos]);
    VectorDd deltaVelWholeSystem = bodyfold.velList[SEE_body] - (bodyfold.massList[uno]*bodyfold.velList[uno] + bodyfold.massList[dos]*bodyfold.velList[dos])/(bodyfold.massList[uno] + bodyfold.massList[dos]);

    double mu_ud = G*(bodyfold.massList[uno] + bodyfold.massList[dos]);
    VectorDd velDiff_ud = bodyfold.velList[dos] - bodyfold.velList[uno];


    double epsilon_ud = velDiff_ud.squaredNorm()/2 - mu_ud / (bodyfold.posList[dos] - bodyfold.posList[uno]).norm();

    double muWholeSystem = mu_ud + G*bodyfold.massList[SEE_body];


    double epsilonWholeSystem = deltaVelWholeSystem.squaredNorm()/2
                                - muWholeSystem / deltaPosWholeSystem.norm();

    double ellipseMajor_ud = -mu_ud/epsilon_ud;
    double ellipseMajorWholeSystem = -muWholeSystem/epsilonWholeSystem;

    return ellipseMajorWholeSystem / ellipseMajor_ud;
}



#endif
