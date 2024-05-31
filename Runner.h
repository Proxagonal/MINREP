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
#define defaultT 50
#define defaultFrameTime 0.001


class Runner {

private:

    const int T;
    const double frameTime;

    Solver solver;
    const Quantities initialQuantities;

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

    void quantComparison(int i) {
        if (fmod(i*frameTime, 5) == 0) {

            cout << "----------" << endl;
            cout << "TIME: " << i*frameTime << endl;
            Quantities::compare(solver.quantities(), initialQuantities);
        }
    }


public:

    Runner(): T{defaultT},
                frameTime{defaultFrameTime},
                solver{frameTime, T},
                initialQuantities{solver.quantities()} {
    }

    Runner(initialData &givenInit, int givenT): T{givenT},
              frameTime{defaultFrameTime},
              solver{frameTime, T, givenInit},
              initialQuantities{solver.quantities()} {
    }

    void run() {

        int i = 0;

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
                quantComparison(i);
        }

        solver.dumpSystemState();
    }

};


#endif //MINREP_RUNNER_H
