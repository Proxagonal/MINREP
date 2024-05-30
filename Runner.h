#ifndef MINREP_RUNNER_H
#define MINREP_RUNNER_H

#include <iostream>
#include "Solver.h"
#include "Visualizer.h"
#include "Quantities.h"

using namespace std;
using namespace Eigen;


#define VISUALIZE false
#define COMPARE_QUANTS false


class Runner {

private:

    const int T = 10;
    const double frameTime = 0.001;
    int i = 0;

    Solver solver{frameTime, T};
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
        visuals.visualizationLoop(solver.getDrawInfo());
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

#if VISUALIZE
        while (isWindowOpen() && i*frameTime < T) {
#else
        while (i*frameTime < T) {
#endif
            i++;

            solver.passTime();

            if (VISUALIZE)
                visualizationLoop();

            if (COMPARE_QUANTS)
                quantComparison();
        }

        // So compiler doesn't delete literally everything
        cout << solver.quantities().toString() << endl;
    }

};


#endif //MINREP_RUNNER_H
