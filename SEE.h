#ifndef SEE_H
#define SEE_H

#if SEE

#include "Solver.h"

inline bool Solver::isGoingAway(nat far) {

    nat uno = (far+1)%NUM;
    nat dos = (far+2)%NUM;

    VectorDd innerWeightedPos = (bodyfold.massList[uno]*bodyfold.posList[uno] + bodyfold.massList[dos]*bodyfold.posList[dos]);// / (bodyfold.massList[uno] + bodyfold.massList[dos]);
    VectorDd innerWeightedVel = (bodyfold.massList[uno]*bodyfold.velList[uno] + bodyfold.massList[dos]*bodyfold.velList[dos]);// / (bodyfold.massList[uno] + bodyfold.massList[dos]);

    VectorDd MassMultipliedDeltaPosWholeSystem = (bodyfold.massList[uno] + bodyfold.massList[dos])*bodyfold.posList[far] - innerWeightedPos;
    VectorDd MassMultipliedDeltaVelWholeSystem = (bodyfold.massList[uno] + bodyfold.massList[dos])*bodyfold.velList[far] - innerWeightedVel;

    return (MassMultipliedDeltaPosWholeSystem.dot(MassMultipliedDeltaVelWholeSystem) > 0);
}

inline bool Solver::isWeakWithRatio(nat i, double FR) {

    nat uno = (i+1)%NUM;
    nat dos = (i+2)%NUM;

    double smallDistSquared = sqrdist(uno);
    double bigDistSquared1 = sqrdist(dos);
    double bigDistSquared2 = sqrdist(i);

    return (bodyfold.massList[(i+2)%NUM] * bigDistSquared2 > FR * bodyfold.massList[i%NUM] * smallDistSquared)
            && (bodyfold.massList[(i+1)%NUM] * bigDistSquared1 > FR * bodyfold.massList[i%NUM] * smallDistSquared);
}


inline int Solver::see_0(nat &far) {

    for (far = 0; far < NUM; far++)

        if (isWeakWithRatio(far, see_detectFR) && isGoingAway(far))
            return 1;

    return 0;
}

inline int Solver::see_1(nat &far) {

    if (!isGoingAway(far) || !isWeakWithRatio(far, see_detectFR))
        return 0;

    nat uno = (far+1)%NUM;
    nat dos = (far+2)%NUM;

    if (isWeakWithRatio(far, see_below_FR)) {

        double mu_ud = G*(bodyfold.massList[uno] + bodyfold.massList[dos]);
        VectorDd velDiff_ud = bodyfold.velList[dos] - bodyfold.velList[uno];

        double epsilon_ud = velDiff_ud.squaredNorm()/2 - mu_ud / (bodyfold.posList[dos] - bodyfold.posList[uno]).norm();

        // This means the binary isn't bound
        if (epsilon_ud >= 0)
            return 0;

        double ellipseMajor_ud = -mu_ud/epsilon_ud;

        VectorDd MassMultipliedDeltaPosWholeSystem = (bodyfold.massList[uno] + bodyfold.massList[dos])*bodyfold.posList[far]
                                                    - (bodyfold.massList[uno]*bodyfold.posList[uno] + bodyfold.massList[dos]*bodyfold.posList[dos]);


        //cout << (bodyfold.posList[uno] - bodyfold.posList[dos]).norm()
        //<< ", " << (bodyfold.posList[uno] - bodyfold.posList[far]).norm()
        //<< ", " << (bodyfold.posList[dos] - bodyfold.posList[far]).norm() << endl;
        //cout << ellipseMajor_ud << endl;

        // This means the approximation will not be good at apoapsis
        // Multiply by apo^2 for no division
        if (isForceHierarchy(uno, see_FR, pow(ellipseMajor_ud * (bodyfold.massList[uno] + bodyfold.massList[dos]), 2), MassMultipliedDeltaPosWholeSystem.squaredNorm()))
            return 2;
        return 1;
    }

    return 1;
}

inline int Solver::see_2(nat &far) {

    if (isGoingAway(far))
        return 2;
    return 3;
}

inline int Solver::see_3(nat &far) {

    if (isWeakWithRatio(far, see_detectFR))
        return 3;
    return 0;
}



inline long double Solver::AARatio(nat far) {

    nat uno = (far+1)%NUM;
    nat dos = (far+2)%NUM;

    VectorDd deltaPosWholeSystem = bodyfold.posList[far] - (bodyfold.massList[uno]*bodyfold.posList[uno] + bodyfold.massList[dos]*bodyfold.posList[dos])/(bodyfold.massList[uno] + bodyfold.massList[dos]);
    VectorDd deltaVelWholeSystem = bodyfold.velList[far] - (bodyfold.massList[uno]*bodyfold.velList[uno] + bodyfold.massList[dos]*bodyfold.velList[dos])/(bodyfold.massList[uno] + bodyfold.massList[dos]);

    double mu_ud = G*(bodyfold.massList[uno] + bodyfold.massList[dos]);
    VectorDd velDiff_ud = bodyfold.velList[dos] - bodyfold.velList[uno];


    double epsilon_ud = velDiff_ud.squaredNorm()/2 - mu_ud / (bodyfold.posList[dos] - bodyfold.posList[uno]).norm();

    double muWholeSystem = mu_ud + G*bodyfold.massList[far];


    double epsilonWholeSystem = deltaVelWholeSystem.squaredNorm()/2
                                - muWholeSystem / deltaPosWholeSystem.norm();

    double ellipseMajor_ud = -mu_ud/epsilon_ud;
    double ellipseMajorWholeSystem = -muWholeSystem/epsilonWholeSystem;

    return ellipseMajorWholeSystem / ellipseMajor_ud;
}



#endif
#endif
