#ifndef MINREP_QUANTITIES_H
#define MINREP_QUANTITIES_H

#include <iostream>
#include <Eigen/Eigen>
#include <cmath>

using namespace std;
using namespace Eigen;

#define SIZE 6

static const array<string, SIZE> names = {"Momentum X", "Momentum Y", "Total Energy", "Angular Momentum", "Kinetic Energy", "Potential Energy"};
static const array<string, SIZE> namesWithColons = {"Momentum X: ", "Momentum Y: ", "Total Energy: ", "Angular Momentum: ", "Kinetic Energy: ", "Potential Energy: "};


class Quantities {

public:

    array<double, SIZE> quants;


    Quantities (double xMomentum,
                double yMomentum,
                double angularMomentum,
                double kineticEnergy,
                double potentialEnergy):
                quants{xMomentum,yMomentum,kineticEnergy + potentialEnergy,angularMomentum,kineticEnergy,potentialEnergy}
    {
    }

    string toString() {

        string txt = "";

        for (int i = 0; i < SIZE; i++)
            txt += namesWithColons.at(i) + to_string(quants.at(i)) + "\n";

        return txt;
    }

    static double deviation(double x, double y) {
        if (y == 0)
            return x;
        return (x-y)/y;
    }

    static void compare(Quantities now, Quantities init) {

        // kinetic energy and potential energy aren't supposed to be conserved, so I don't print them
        for (int i = 0; i<SIZE; i++) {
            if (init.quants.at(i) == 0) {
                cout << namesWithColons.at(i) << "0 -> " << deviation(now.quants.at(i), init.quants.at(i)) << endl;
                continue;
            }
            cout << namesWithColons.at(i) << deviation(now.quants.at(i), init.quants.at(i)) << endl;

        }
    }

};

#endif //MINREP_QUANTITIES_H
