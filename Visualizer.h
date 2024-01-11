#ifndef NBPCPP_SIMPSIM_H
#define NBPCPP_SIMPSIM_H

#include <iostream>
#include <Eigen/Eigen>
#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <SFML/System.hpp>
#include "Solver.h"
#include "Body.h"

#define RAD 1

using namespace std;
using namespace Eigen;

class Visualizer {

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

    void zoom(double scrollDelta) {

        view.setSize(view.getSize().x * (1 - scrollDelta * zoomSpeed),
                     view.getSize().y * (1 - scrollDelta * zoomSpeed));
        window.setView(view);
    }

public:

    Visualizer(int winX, int winY, double sysrad):
            window{sf::VideoMode(winX, winY), "NBP"} {
        view.setCenter(0, 0);
        view.setRotation(180);
        view.setSize(view.getSize().x, -view.getSize().y);

        double scale = initialZoomFactor * sysrad;

        view.setSize(-scale, scale * winY/winX);

        window.setView(view);
    }

    bool isOpen() {
        return window.isOpen();
    }

    void visualizationLoop(const vector<Body> &bodiesInfo) {

        sf::Event event;

        while (window.pollEvent(event)) {

            if (event.type == sf::Event::Closed)
                window.close();

            if (event.type == sf::Event::MouseWheelMoved) {
                zoom(event.mouseWheel.delta);
                break;
            }
        }

        window.clear();

        for (Body body: bodiesInfo)
            drawBody(body);

        window.display();
    }
};

#endif //MINREP_SIMPSIM_H
