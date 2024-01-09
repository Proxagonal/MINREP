#include "Solver.h"
#include "SimpSim.h"
#include <chrono>


using namespace std;
using namespace Eigen;


long double deviation(long double x, long double y);
void compare(vector<long double> now, vector<long double> init);


int main() {

    Solver solver;
    SimpSim sim(800, 800);
    sim.simulate(solver);

    //double dt = 0.001;
    //int T = 300;
    //int steps = T/dt;

    //vector<long double> initQuants = solver.quantsInfo();

    //auto start = chrono::high_resolution_clock::now();
//
    //for (int i = 0; i < steps; i++) {
//
    //    solver.passTime(dt);

        //if (fmod(i*dt, 1) == 0) {
        //    vector<long double> quants = solver.quantsInfo();
//
        //    cout << "----------" << endl;
        //    cout << "TIME: " << i*dt << endl;
        //    compare(quants, initQuants);
        //}
    //}

    //auto stop = chrono::high_resolution_clock::now();
//
    //auto duration = duration_cast<chrono::milliseconds>(stop - start);
    //cout << duration.count() << endl;
//
//
    //Solver solver2;
//
    //start = chrono::high_resolution_clock::now();
//
    //for (int i = 0; i < steps; i++) {
    //    solver2.passTimeVERLET(dt);
    //}
//
    //stop = chrono::high_resolution_clock::now();
//
    //duration = duration_cast<chrono::milliseconds>(stop - start);
    //cout << duration.count() << endl;



    return 0;
}

//compares conserved quantities initially to now
void compare(vector<long double> now, vector<long double> init) {

    vector<string> names = {"Momentum_x: ", "Momentum_y: ", "Total Energy: ", "Kinetic: ", "Potential: "};

    // kinetic energy and potential energy aren't supposed to be conserved, so I don't print them
    for (int i = 0; i<3; i++)
        cout << names.at(i) << deviation(now.at(i), init.at(i)) << endl;
}

long double deviation(long double x, long double y) {
    return (x-y)/y;
}





