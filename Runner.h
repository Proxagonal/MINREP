#ifndef MINREP_RUNNER_H
#define MINREP_RUNNER_H

#include <iostream>
#include "Solver.h"
#include "Visualizer.h"
#include "Quantities.h"

using namespace std;
using namespace Eigen;


#define COMPARE_QUANTS true
#define VISUALIZE true



class Runner {

    Solver solver;
    const Quantities initialQuantities = solver.quantities();

    #ifdef VISUALIZE
    Visualizer visuals{800, 800};
    #endif

private:

    const int T = 100;
    const double dt = 0.001;
    int i = 0;

    bool isWindowOpen() {
        if (VISUALIZE)
            return visuals.isOpen();
        return true;
    }

    void quantComparison() {
        if (fmod(i*dt, 5) == 0) {

            cout << "----------" << endl;
            cout << "TIME: " << i*dt << endl;
            Quantities::compare(solver.quantities(), initialQuantities);
        }
    }

public:

    Runner() {
    }

    void run() {

        while (isWindowOpen() && i < T/dt) {
            i++;

            solver.passTime(dt);

            if (VISUALIZE)
                visuals.visualizationLoop(solver.getBodiesInfo());

            if (COMPARE_QUANTS)
                quantComparison();
        }

    }

};


#endif //MINREP_RUNNER_H
