#ifndef NBPCPP_SIMPSIM_H
#define NBPCPP_SIMPSIM_H

#include <Eigen/Eigen>
#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <SFML/System.hpp>
#include <unistd.h>

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
    const int pathLength = 1000;
    double farRate = 10;


    sf::RenderWindow window;
    sf::View view;

    array<sf::VertexArray, NUM> paths;
    int pathStart = 0;
    int trueLength = 0;

    bool moving = false;
    sf::Vector2i anchor;
    sf::Vector2i mouseMove;


    double effectiveOtherDistance(const VectorDd &pos) {
        VectorDd otherCoords = pos;
        otherCoords(0) = 0;
        otherCoords(1) = 0;

        return otherCoords.sum() / (DIM*farRate);
    }

    double effectiveRadius(const VectorDd &pos) {
        if (DIM <= 2)
            return RAD;

        VectorDd otherCoords = pos;
        otherCoords(0) = 0;
        otherCoords(1) = 0;

        return RAD*(1 + tanh(effectiveOtherDistance(pos)));
    }

    inline sf::Color effectiveColor(const VectorDd &pos, const sf::Color &color) {
        return color;
    }

    void drawPos(const VectorDd &pos, int i) {

        double rad = effectiveRadius(pos);

        sf::CircleShape shape(rad);

        shape.setFillColor(effectiveColor(pos, colors[i]));
        shape.setOrigin(rad, rad);
        shape.setPosition(pos.x(), pos.y());

        window.draw(shape);
    }

    void drawPath(sf::VertexArray &path, int j) {

        if (trueLength < pathLength) {
            window.draw(&path[0], trueLength, sf::LinesStrip);
            return;
        }

        window.draw(&path[pathStart], pathLength, sf::LinesStrip);
    }

    void zoom(double scrollDelta) {

        view.setSize(view.getSize().x * (1 - scrollDelta * zoomSpeed),
                     view.getSize().y * (1 - scrollDelta * zoomSpeed));
        window.setView(view);
    }

    static sf::Vector2f toSFML(const VectorDd &v) {
        return sf::Vector2f(v.x(), v.y());
    }

    // BREAKS FOR NUM =/= 3
    array<int, NUM> bodiesOrdered(const array<VectorDd, NUM> &posList) {

        int a = 0, b = 1, c = 2;

        if (effectiveRadius(posList[a]) > effectiveRadius(posList[b])) swap(a, b);
        if (effectiveRadius(posList[a]) > effectiveRadius(posList[c])) swap(a, c);
        if (effectiveRadius(posList[b]) > effectiveRadius(posList[c])) swap(b, c);

        return {a, b, c};
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

        auto pos = sf::Vector2i(1000,70); //HUH
        window.setPosition(pos);

        for (int i = 0; i < NUM; i++)
            paths[i] = sf::VertexArray(sf::LinesStrip, 2*pathLength);
    }

    bool isOpen() {
        return window.isOpen();
    }
    bool slowDown() {
        return sf::Keyboard::isKeyPressed(sf::Keyboard::S);
    }

    void visualizationLoop(const array<VectorDd, NUM> &posList) {

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

        for (int i = 0; i < NUM; i++)
            drawPath(paths[i], i);
        for (int i : bodiesOrdered(posList))
            drawPos(posList[i], i);

        window.display();
    }

    void addToPaths(const array<VectorDd, NUM> &posList) {

        if (trueLength < pathLength) {
            for (int i = 0; i < NUM; i++) {

                auto pos = toSFML(posList[i]);
                auto color = effectiveColor(posList[i], colors[i]);

                paths[i][trueLength] = pos;
                paths[i][trueLength].color = color;
                paths[i][trueLength + pathLength] = pos;
                paths[i][trueLength + pathLength].color = color;

            }
            trueLength++;
            return;
        }

        for (int i = 0; i < NUM; i++) {

            auto pos = toSFML(posList[i]);
            auto color = effectiveColor(posList[i], colors[i]);

            paths[i][(pathStart + 2*pathLength)%(2*pathLength)] = pos;
            paths[i][(pathStart + 2*pathLength)%(2*pathLength)].color = color;
            paths[i][(pathStart - pathLength + 2*pathLength)%(2*pathLength)] = pos;
            paths[i][(pathStart - pathLength + 2*pathLength)%(2*pathLength)].color = color;
        }
        pathStart++;
        pathStart = pathStart%pathLength;
    }
};

#endif //MINREP_SIMPSIM_H
