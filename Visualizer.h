#ifndef NBPCPP_SIMPSIM_H
#define NBPCPP_SIMPSIM_H

#include <iostream>
#include <Eigen/Eigen>
#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <SFML/System.hpp>
#include <unistd.h>
#include "Solver.h"

#define RAD 0.5

using namespace std;
using namespace Eigen;

class Visualizer {

private:

    static constexpr double zoomSpeed = 0.05;
    static constexpr double initialZoomFactor = 6;
    const vector<sf::Color> colors = {sf::Color(255, 50, 50),
                                      sf::Color(50, 255, 50),
                                      sf::Color(50, 50, 255)};
    static constexpr int wSkips = 1000/100;
    int wCount = 0;
    int pathLength = 100000;


    sf::RenderWindow window;
    sf::View view;

    array<vector<Vector2d>, NUM> paths;
    int pathStart = 0;
    int trueLength = 0;

    bool moving = false;
    sf::Vector2i anchor;
    sf::Vector2i mouseMove;

    void drawPos(const Vector2d &pos, int i) {

        sf::CircleShape shape(RAD);

        //shape.setFillColor(sf::Color::White);
        shape.setFillColor(colors[i]);
        shape.setOrigin(RAD, RAD);
        shape.setPosition(pos.x(), pos.y());

        window.draw(shape);
    }

    void drawPath(const vector<Vector2d> path, int j) {

        sf::VertexArray lines(sf::LinesStrip, trueLength);


        for (int i = 0; i < trueLength; i++) {
            lines[i] = sf::Vector2f(path[(i + pathStart)%pathLength].x(), path[(i + pathStart)%pathLength].y());
            lines[i].color = colors[j];
        }

        window.draw(lines);
    }

    void zoom(double scrollDelta) {

        view.setSize(view.getSize().x * (1 - scrollDelta * zoomSpeed),
                     view.getSize().y * (1 - scrollDelta * zoomSpeed));
        window.setView(view);
    }

public:

    static const int slowerBy = 30;

    Visualizer(int winX, int winY, double sysrad):
            window{sf::VideoMode(winX, winY), "NBP"} {
        view.setCenter(0, 0);
        view.setRotation(180);
        view.setSize(view.getSize().x, -view.getSize().y);

        double scale = initialZoomFactor * sysrad;

        view.setSize(-scale, scale * winY/winX);

        window.setView(view);

        auto pos = sf::Vector2i(1000,70);
        window.setPosition(pos);
    }

    bool isOpen() {
        return window.isOpen();
    }
    bool slowDown() {
        return sf::Keyboard::isKeyPressed(sf::Keyboard::S);
    }

    void visualizationLoop(const array<Vector2d, NUM> &posList) {

        wCount--;
        if (wCount > 0)
            return;

        sf::Event event;

        while (window.pollEvent(event)) {

            if (event.type == sf::Event::Closed)
                window.close();

            if(event.type == sf::Event::KeyPressed)
                if(event.key.code == sf::Keyboard::W)
                    wCount = wSkips;

            if (event.type == sf::Event::MouseWheelMoved)
                zoom(event.mouseWheel.delta);

            if (event.type == sf::Event::MouseButtonPressed) {
                if (event.mouseButton.button == sf::Mouse::Left) {

                    anchor = sf::Vector2i(event.mouseButton.x, event.mouseButton.y);
                    mouseMove = anchor;
                    moving = true;
                }
            }
            if (event.type == sf::Event::MouseButtonReleased) {
                if (event.mouseButton.button == sf::Mouse::Left) {
                    moving = false;
                }
            }
            if (moving && event.type == sf::Event::MouseMoved) {
                mouseMove = sf::Vector2i(event.mouseMove.x, event.mouseMove.y);
            }
        }

        if (moving) {
            view.setCenter(view.getCenter() - window.mapPixelToCoords(mouseMove) + window.mapPixelToCoords(anchor));
            window.setView(view);
            anchor = sf::Vector2i(mouseMove.x, mouseMove.y);
        }

        window.clear();

        for (int i = 0; i < NUM; i++) {
            drawPath(paths[i], i);
            drawPos(posList[i], i);
        }

        window.display();
    }

    void addToPaths(const array<Vector2d, NUM> &posList) {

        if (trueLength < pathLength) {
            for (int i = 0; i < NUM; i++)
                paths[i].emplace_back(posList[i]);
            trueLength++;
            return;
        }

        for (int i = 0; i < NUM; i++)
            paths[i][pathStart] = posList[i];
        pathStart++;
        pathStart = pathStart%pathLength;
    }
};

#endif //MINREP_SIMPSIM_H
