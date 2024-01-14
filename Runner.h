#ifndef MINREP_RUNNER_H
#define MINREP_RUNNER_H

#include <iostream>
#include "Solver.h"
#include "Visualizer.h"
#include "Quantities.h"

using namespace std;
using namespace Eigen;


#define VISUALIZE true
#define COMPARE_QUANTS true


class Runner {

private:

    const int T = 1000;
    const double frameTime = 0.001;
    int i = 0;

    Solver solver{frameTime};
    const Quantities initialQuantities = solver.quantities();

#if VISUALIZE
    Visualizer visuals{800, 800, solver.getSystemRadius()};
#endif



    bool isWindowOpen() {
#if VISUALIZE
        return visuals.isOpen();
#endif
        return true;
    }

    void visualizationLoop() {
#if VISUALIZE
        visuals.visualizationLoop(solver.getBodiesInfo());
#endif
    }

    void quantComparison() {
        if (fmod(i*frameTime, 5) == 0) {

            cout << "----------" << endl;
            cout << "TIME: " << i*frameTime << endl;
            Quantities::compare(solver.quantities(), initialQuantities);
        }
    }

public:

    Runner() {
    }

    void run() {

        while (isWindowOpen() && i*frameTime < T) {

            i++;

            solver.passTime();

            if (VISUALIZE)
                visualizationLoop();

            if (COMPARE_QUANTS)
                quantComparison();
        }
    }

};


#endif //MINREP_RUNNER_H
