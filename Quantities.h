#ifndef MINREP_QUANTITIES_H
#define MINREP_QUANTITIES_H

#include <iostream>
#include <Eigen/Eigen>
#include <cmath>

using namespace std;
using namespace Eigen;

#define SIZE 5

static const array<string, SIZE> names = {"Momentum X", "Momentum Y", "Total Energy", "Kinetic Energy", "Potential Energy"};
static const array<string, SIZE> namesWithColons = {"Momentum X: ", "Momentum Y: ", "Total Energy: ", "Kinetic Energy: ", "Potential Energy: "};


class Quantities {

private:
    constexpr static int ALLDIGITS = std::numeric_limits<double>::max_digits10;

public:

    array<double, SIZE> quants;


    Quantities (double xMomentum,
                double yMomentum,
                double kineticEnergy,
                double potentialEnergy):
                quants{xMomentum,yMomentum,kineticEnergy + potentialEnergy,kineticEnergy,potentialEnergy}
    {
    }

    string toString() {

        stringstream ss;
        ss.precision(ALLDIGITS);

        for (int i = 0; i < SIZE; i++)
            ss << namesWithColons.at(i) << quants.at(i) << "\n";

        return ss.str();
    }

    double E() {
        return quants.at(2);
    }

    static double deviation(double x, double y) {
        if (y == 0)
            return x;
        return (x-y)/y;
    }

    static void compare(Quantities now, Quantities init) {

        // kinetic energy and potential energy aren't supposed to be conserved, so I don't print them
        for (int i = 0; i < SIZE - 2; i++) {

            cout << namesWithColons.at(i);
            // for momentum:
            if (i <= 1)
                cout << "0 -> " << now.quants.at(i) << endl;
            else
                cout << deviation(now.quants.at(i), init.quants.at(i)) << endl;
        }
    }

};

#endif //MINREP_QUANTITIES_H
