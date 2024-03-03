#ifndef NBPCPP_SIMPSIM_H
#define NBPCPP_SIMPSIM_H

#include <iostream>
#include <Eigen/Eigen>
#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <SFML/System.hpp>
#include "Solver.h"

#define RAD 0.1

using namespace std;
using namespace Eigen;

class Visualizer {

private:

    const double zoomSpeed = 0.05;
    const double initialZoomFactor = 6;

    sf::RenderWindow window;
    sf::View view;

    array<vector<Vector2d>, NUM> paths;

    void drawPos(const Vector2d &pos) {

        sf::CircleShape shape(RAD);

        shape.setFillColor(sf::Color::White);
        shape.setOrigin(RAD, RAD);
        shape.setPosition(pos.x(), pos.y());

        window.draw(shape);
    }

    void drawPath(const vector<Vector2d> path) {

        int amount = 1500;
        int init = std::max(0, (int)path.size() - amount);

        sf::VertexArray lines(sf::LinesStrip, path.size() - init);


        for (int i = init; i<path.size(); i++)
            lines[i-init] = sf::Vector2f(path[i].x(), path[i].y());

        window.draw(lines);
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

    void visualizationLoop(const array<Vector2d, NUM> &posList) {

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

        for (int i = 0; i < NUM; i++) {
            paths[i].emplace_back(posList[i]);
            drawPath(paths[i]);
            drawPos(posList[i]);
        }

        window.display();
    }
};

#endif //MINREP_SIMPSIM_H
