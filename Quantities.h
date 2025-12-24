#ifndef MINREP_QUANTITIES_H
#define MINREP_QUANTITIES_H

#include <iostream>
#include <Eigen/Eigen>
#include "Consts.h"

using namespace std;
using namespace Eigen;


class Quantities {

    static inline const string posName = "COM";
    static inline const string momName = "Total Momentum";
    static inline const string angMomName = "Total Angular Momentum";

    static constexpr int SIZE = 3;
    static inline const array<string, SIZE> names = {"Total Energy", "Kinetic Energy", "Potential Energy"};


    static double deviation(double x, double y) {
        if (y == 0)
            return x;
        return (x-y)/y;
    }

    static string stringVectorDeviation(const VectorDd &x, const VectorDd &y) {

        stringstream ss;

        for (int i = 0; i < x.size(); i++) {
            if (abs(y(i)) < pow(10, -9))
                ss << "0 -> " << x(i) << ", ";
            else
                ss << deviation(x(i), y(i)) << ", ";
        }

        return ss.str();
    }
#if DIM != 3
    static string stringVectorDeviation(const VectorAngd &x, const VectorAngd &y) {

        stringstream ss;
        for (int i = 0; i < x.size(); i++) {
            if (y(i) == 0)
                ss << "0 -> " << x(i) << ", ";
            else
                ss << deviation(x(i), y(i)) << ", ";
        }

        return ss.str();
    }
#endif

public:


    const VectorDd com, mom;
    const VectorAngd angMom;
    const array<double, SIZE> quants;


    Quantities (const VectorDd &compos,
                const VectorDd &momentum,
                const VectorAngd &angularMomentum,
                double kineticEnergy,
                double potentialEnergy):
                com{compos},
                mom{momentum},
                angMom{angularMomentum},
                quants{kineticEnergy + potentialEnergy,kineticEnergy,potentialEnergy}
    {
    }

    string toString(const streamsize accuracy = DEFAULTDIGITS) const {

        stringstream ss;
        ss.precision(accuracy);


        ss << posName << ": " << com.transpose() << "\n";
        ss << momName << ": " << mom.transpose()<< "\n";
        ss << angMomName << ": " << angMom.transpose() << "\n";

        for (int i = 0; i < SIZE; i++)
            ss << names.at(i) << ": " << quants.at(i) << "\n";

        return ss.str();
    }

    static string compare(const Quantities &now, const Quantities &init, const streamsize accuracy = DEFAULTDIGITS) {

        stringstream ss;
        ss.precision(accuracy);

        ss << posName << ": " << stringVectorDeviation(now.com, init.com) << endl;
        ss << momName << ": " << stringVectorDeviation(now.mom, init.mom) << endl;
        ss << angMomName << ": " << stringVectorDeviation(now.angMom, init.angMom) << endl;


        // kinetic energy and potential energy aren't supposed to be conserved, so I don't print them
        for (int i = 0; i < SIZE; i++) {

            if (i == 1 || i == 2)
                continue;

            ss << names.at(i) << ": ";

            if (init.quants.at(i) == 0)
                ss << "0 -> " << now.quants.at(i) << endl;
            else
                ss << deviation(now.quants.at(i), init.quants.at(i)) << endl;
        }

        return ss.str();
    }

};

#endif //MINREP_QUANTITIES_H
