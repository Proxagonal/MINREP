#ifndef MINREP_RUNNER_H
#define MINREP_RUNNER_H

#include <iostream>
#include <Eigen/Eigen>
#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <SFML/System.hpp>
#include "Solver.h"
#include "Body.h"
#include "Visualizer.h"

using namespace std;
using namespace Eigen;

#define COMPARE_QUANTS true
#define VISUALIZE true

class Runner {

    Solver solver;
    const Quantities initialQuantities = solver.quantities();
    Visualizer vs;

#ifdef VISUALIZE
    Visualizer visuals(800, 800);
#endif

private:
    int i = 0;
    const int T = 100;
    double dt = 0.001;

public:

    Runner() {
    }

    //void visualizationLoop(vector<Body> &bodiesInfo) {
    //    if (VISUALIZE)
    //        visuals.visualizationLoop(bodiesInfo);
    //}

    void run() {


        while (i < T/dt) {
            solver.passTime(dt);
        }

    }

};


#endif //MINREP_RUNNER_H
