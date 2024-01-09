#ifndef MINREP_QUANTITIES_H
#define MINREP_QUANTITIES_H

#include <iostream>
#include <Eigen/Eigen>
#include <cmath>

using namespace std;
using namespace Eigen;

#define SIZE 6

static const array<string, SIZE> names = {"Momentum X", "Momentum Y", "Total Energy", "Angular Momentum", "Kinetic Energy", "Potential Energy"};

class Quantities {

public:

    long double xMomentum;
    long double yMomentum;
    long double energy;
    long double angularMomentum;
    long double kineticEnergy;
    long double potentialEnergy;

    Quantities (long double xMomentum,
                long double yMomentum,
                long double energy,
                long double angularMomentum,
                long double kineticEnergy,
                long double potentialEnergy):

                xMomentum{xMomentum},
                yMomentum{yMomentum},
                energy{energy},
                angularMomentum{angularMomentum},
                kineticEnergy{kineticEnergy},
                potentialEnergy{potentialEnergy}
                {
    }

    Quantities (Vector2<long double> momentum,
                long double angularMomentum,
                long double kineticEnergy,
                long double potentialEnergy):

            xMomentum{momentum.x()},
            yMomentum{momentum.y()},
            energy{kineticEnergy + potentialEnergy},
            angularMomentum{angularMomentum},
            kineticEnergy{kineticEnergy},
            potentialEnergy{potentialEnergy}
    {
    }

    Quantities () {

    }

    array<long double, SIZE> inArray() {
        return  {xMomentum,
                yMomentum,
                energy,
                angularMomentum,
                kineticEnergy,
                potentialEnergy};
    }

    string toString() {

        array<long double, SIZE> arr = inArray();

        string txt = "";

        for (int i = 0; i < SIZE; i++)
            txt += names.at(i) + ": " + to_string(arr.at(i)) + "\n";

        return txt;
    }



};

#endif //MINREP_QUANTITIES_H
