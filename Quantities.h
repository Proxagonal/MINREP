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

    long double xMomentum;
    long double yMomentum;
    long double energy;
    long double angularMomentum;
    long double kineticEnergy;
    long double potentialEnergy;


    Quantities (long double xMomentum,
                long double yMomentum,
                long double angularMomentum,
                long double kineticEnergy,
                long double potentialEnergy):

                xMomentum{xMomentum},
                yMomentum{yMomentum},
                energy{kineticEnergy + potentialEnergy},
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
            txt += namesWithColons.at(i) + to_string(arr.at(i)) + "\n";

        return txt;
    }

    static void compare(vector<long double> now, vector<long double> init) {

        // kinetic energy and potential energy aren't supposed to be conserved, so I don't print them
        for (int i = 0; i<SIZE; i++) {
            if (init.at(i) == 0) {
                cout << namesWithColons.at(i) << "0 -> " << deviation(now.at(i), init.at(i)) << endl;
                continue;
            }
            cout << namesWithColons.at(i) << deviation(now.at(i), init.at(i)) << endl;

        }
    }

    static long double deviation(long double x, long double y) {
        if (y == 0)
            return x;
        return (x-y)/y;
    }

};

#endif //MINREP_QUANTITIES_H
