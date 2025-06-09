#ifndef MINREP_QUANTITIES_H
#define MINREP_QUANTITIES_H

#include <iostream>
#include <Eigen/Eigen>
#include "Consts.h"

using namespace std;
using namespace Eigen;

#define SIZE 3

static const string posName = "Position";
static const string momName = "Momentum";
static const string angMomName = "Angular Momentum";

static const array<string, SIZE> names = {"Total Energy", "Kinetic Energy", "Potential Energy"};


class Quantities {

private:
    constexpr static int ALLDIGITS = std::numeric_limits<double>::max_digits10;

    static double deviation(double x, double y) {
        if (y == 0)
            return x;
        return (x-y)/y;
    }

    static string stringVectorDeviation(VectorDd &x, VectorDd &y) {

        stringstream ss;
        for (int i = 0; i < x.size(); i++) {
            if (abs(y(i)) < pow(10, -5))
                ss << "0 -> " << x(i) << ", ";
            else
                ss << deviation(x(i), y(i)) << ", ";
        }

        return ss.str();
    }
#if DIM != 3
    static string stringVectorDeviation(VectorAngd &x, VectorAngd &y) {

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


    VectorDd wpos, mom;
    VectorAngd angMom;
    array<double, SIZE> quants;


    Quantities (VectorDd &wposition,
                VectorDd &momentum,
                VectorAngd &angularMomentum,
                double kineticEnergy,
                double potentialEnergy):
                wpos{wposition},
                mom{momentum},
                angMom{angularMomentum},
                quants{kineticEnergy + potentialEnergy,kineticEnergy,potentialEnergy}
    {
    }

    string toString() {

        stringstream ss;
        ss.precision(ALLDIGITS);


        ss << posName << ": " << wpos.transpose() << "\n";
        ss << momName << ": " << mom.transpose()<< "\n";
        ss << angMomName << ": " << angMom.transpose() << "\n";

        for (int i = 0; i < SIZE; i++)
            ss << names.at(i) << ": " << quants.at(i) << "\n";

        return ss.str();
    }

    static void compare(Quantities now, Quantities &init) {

        cout << stringVectorDeviation(now.wpos, init.wpos) << endl;
        cout << stringVectorDeviation(now.mom, init.mom) << endl;
        cout << stringVectorDeviation(now.angMom, init.angMom) << endl;


        // kinetic energy and potential energy aren't supposed to be conserved, so I don't print them
        for (int i = 0; i < SIZE; i++) {

            if (i == 1 || i == 2)
                continue;

            cout << names.at(i) << ": ";

            if (init.quants.at(i) == 0)
                cout << "0 -> " << now.quants.at(i) << endl;
            else
                cout << deviation(now.quants.at(i), init.quants.at(i)) << endl;
        }
    }

    double E() {
        return quants.at(0);
    }

};

#endif //MINREP_QUANTITIES_H
