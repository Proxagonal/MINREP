#ifndef NBPCPP_SIMPSIM_H
#define NBPCPP_SIMPSIM_H

#include <iostream>
#include <Eigen/Eigen>
#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <SFML/System.hpp>
#include "Solver.h"
#include "Body.h"
#include "sfLine.h"

#define RAD 100
#define SYSRAD 10000

using namespace std;
using namespace Eigen;

class SimpSim {

private:

    const double zoomSpeed = 0.05;
    const double initialZoomFactor = 6;

    sf::RenderWindow window;
    sf::View view;

    void drawBody(Body &body) {

        sf::CircleShape shape(RAD);

        shape.setFillColor(sf::Color::White);
        shape.setOrigin(RAD, RAD);
        shape.setPosition(body.position.x(), body.position.y());

        window.draw(shape);
    }

    void drawPos(Vector2<long double> vec) {

        sf::CircleShape shape(RAD);

        shape.setFillColor(sf::Color::Green);
        shape.setOrigin(RAD, RAD);
        shape.setPosition(vec.x(), vec.y());

        window.draw(shape);

    }


    void zoom(double scrollDelta) {

        view.setSize(view.getSize().x * (1 - scrollDelta * zoomSpeed),
                     view.getSize().y * (1 - scrollDelta * zoomSpeed));
        window.setView(view);
    }

    long double getSimulationSize() {
        return min(view.getSize().x, view.getSize().y);
    }

public:

    SimpSim(int winX, int winY):
            window{sf::VideoMode(winX, winY), "NBPCPP"} {

        view.setCenter(0, 0);
        view.setRotation(180);
        view.setSize(view.getSize().x, -view.getSize().y);

        double scale = initialZoomFactor * SYSRAD;

        view.setSize(-scale, scale * winY/winX);

        window.setView(view);
    }

    void simulate(Solver solver) {

        double dt = 0.001;
        int T = 100;
        int i = 0;
        vector<Vector2<long double>> points;
        Vector2<long double> initCOM = solver.calcCOM();
        Vector2<long double> COMVelocity = solver.calcCOMVelocity();

        vector<long double> initQuants = solver.quantsInfo();
        points.emplace_back(solver.calcCOM());

        while (window.isOpen() && i < T/dt) {
            i++;

            sf::Event event;

            while (window.pollEvent(event)) {

                if (event.type == sf::Event::Closed)
                    window.close();

                if (event.type == sf::Event::MouseWheelMoved) {
                    zoom(event.mouseWheel.delta);
                    break;
                }
            }

            solver.passTime(dt);

            if (fmod(i*dt, 1) == 0) {

                vector<long double> quants = solver.quantsInfo();

                cout << "----------" << endl;
                cout << "TIME: " << i*dt << endl;
                compare(quants, initQuants);
            }
            if (i % 100 == 0)
                points.emplace_back(solver.calcCOM());


            window.clear();

            vector<Body> bodiesInfo = solver.getBodiesInfo();
            for (Body body: bodiesInfo)
                drawBody(body);

            drawPos(initCOM + i*dt * COMVelocity);


            for (int j = 0; j < points.size() - 1; j++) {
                sfLine line(sf::Vector2f(points.at(j).x(), points.at(j).y()),
                            sf::Vector2f(points.at(j+1).x(), points.at(j+1).y()));
                line.thickness = 1000;
                line.color = sf::Color::Red;
                line.draw(window);
            }


            window.display();


        }
    }

    //compares conserved quantities initially to now
    void compare(vector<long double> now, vector<long double> init) {

        vector<string> names = {"Momentum_x: ", "Momentum_y: ", "Total Energy: ", "Kinetic: ", "Potential: "};

        // kinetic energy and potential energy aren't supposed to be conserved, so I don't print them
        for (int i = 0; i<5; i++) {
            if (init.at(i) == 0) {
                cout << names.at(i) << "0 -> " << deviation(now.at(i), init.at(i)) << endl;
                continue;
            }
            cout << names.at(i) << deviation(now.at(i), init.at(i)) << endl;

        }
    }

    long double deviation(long double x, long double y) {
        if (y == 0)
            return x;
        return (x-y)/y;
    }
};

#endif //MINREP_SIMPSIM_H
