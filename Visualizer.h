#ifndef NBPCPP_SIMPSIM_H
#define NBPCPP_SIMPSIM_H

#include <Eigen/Eigen>
#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <SFML/System.hpp>
#include <unistd.h>

#include "Consts.h"

using namespace std;
using namespace Eigen;

class Visualizer {

private:

    inline static const vector<sf::Color> constcolors = {sf::Color(255, 50, 50),
                                  sf::Color(50, 255, 50),
                                  sf::Color(50, 50, 255)};
    const vector<sf::Color> colors = makeColors();


    // rand is used purposefully: use srand to get consistent colors.
    vector<sf::Color> makeColors()
    {
        vector<sf::Color> gencolors = constcolors;

        for (int i = constcolors.size(); i < NUM; ++i)
            gencolors.emplace_back(rand() % 256, rand() % 256, rand() % 256);

        return gencolors;
    }

    double radius;
    static constexpr double zoomSpeed = 0.05;
    static constexpr double initialZoomFactor = 6;
    static constexpr int wSkips = 10;
    int wCount = 0;
    const int pathLength;
    const double farRate;


    sf::RenderWindow window;
    sf::View view;

    array<sf::VertexArray, NUM> paths;
    int pathStart = 0;
    int trueLength = 0;

    bool moving = false;
    sf::Vector2i anchor;
    sf::Vector2i mouseMove;


    static Vector2d projection(const VectorDd &pos) {
        return {pos.x(), pos.y()};
    }

    double effectiveOtherDistance(const VectorDd &pos) {
        return (pos.sum() - projection(pos).sum()) / (DIM*farRate);
    }

    double effectiveRadius(const VectorDd &pos) {
        if (DIM <= 2)
            return radius;

        return radius*(1 + tanh(effectiveOtherDistance(pos)));
    }

    inline sf::Color effectiveColor(const VectorDd &pos, const sf::Color &color) {
        return color;
    }

    void drawPos(const VectorDd &pos, int i) {

        double rad = effectiveRadius(pos);

        sf::CircleShape shape(rad);

        shape.setFillColor(effectiveColor(pos, colors[i]));
        shape.setOrigin(rad, rad);

        Vector2d proj = projection(pos);

        shape.setPosition(proj.x(), proj.y());

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
        Vector2d proj = projection(v);
        return sf::Vector2f(proj.x(), proj.y());
    }

    array<int, NUM> bodiesOrdered(const vData &posList) {

        array<int, NUM> idx;
        for (int i = 0; i < NUM; i++)
            idx[i] = i;

        for (int i = 0; i < NUM; i++)
            for (int j = i + 1; j < NUM; j++)
                if (effectiveRadius(posList[idx[i]]) >
                    effectiveRadius(posList[idx[j]]))
                    swap(idx[i], idx[j]);

        return idx;
    }

    vData getPoslist(const initialData &init) {

        vData posList;

        for (int i = 0; i < NUM; i++)
            posList[i] = get<1>(init[i]);

        return posList;

    }

    double getSystemRadius(const vData &posList) const {

        double maximum = 0;
        for (const auto &pos : posList)
            maximum = max(maximum, pos.norm());

        return maximum;
    }

    double getMinDistance(const vData &posList) const {

        double minimum = numeric_limits<double>::infinity();

        for (int i = 0; i < NUM; i++)
            for (int j = i + 1; j < NUM; j++)
                minimum = min(minimum, (posList[i] - posList[j]).squaredNorm());

        return sqrt(minimum);
    }


public:

    Visualizer(int winX, int winY, double sysrad, double bodyRadius, double farRate, int pathLength, int savePosPerPasses, int framesPerPasses, int slowingFactor=30):
            radius{bodyRadius},
            farRate{farRate},
            pathLength{pathLength},
            savePosPerPasses{savePosPerPasses},
            framePerPasses{framesPerPasses},
            slowerBy{slowingFactor},
            window{sf::VideoMode(winX, winY), "NBP"} {
        view.setCenter(0, 0);
        view.setRotation(180);
        view.setSize(view.getSize().x, -view.getSize().y);

        double scale = initialZoomFactor * sysrad;

        view.setSize(-scale, scale * winY/winX);

        window.setView(view);

        sf::Vector2i pos(1000,70);
        window.setPosition(pos);

        for (int i = 0; i < NUM; i++)
            paths[i] = sf::VertexArray(sf::LinesStrip, 2*pathLength);
    }

    Visualizer(int winX, int winY, const vData &posList, double dt, int savePosPerYear, int framesPerYear, int slowingFactor)
        : Visualizer(winX, winY,
        getSystemRadius(posList),
        getMinDistance(posList)/20,
        getSystemRadius(posList),
        100,
        (int)(1/(savePosPerYear*dt)),
        (int)(1/(framesPerYear*dt)),
        slowingFactor)
    {}

    Visualizer(int winX, int winY, const initialData &init, double dt, int savePosPerYear=100, int framesPerYear=30, int slowingFactor=30)
    : Visualizer(winX, winY, getPoslist(init), dt, savePosPerYear, framesPerYear, slowingFactor) {}

    bool isOpen() {
        return window.isOpen();
    }
    bool slowDown() {
        return sf::Keyboard::isKeyPressed(sf::Keyboard::S);
    }

    int savePosPerPasses = 100;
    int framePerPasses = 100;
    int slowerBy = 30;
    bool easyVisualize(long pass, const vData &posList) {

        if (pass%savePosPerPasses == 0)
            addToPaths(posList);

        if (pass%framePerPasses == 0 || (slowDown() && pass%(1 + framePerPasses/slowerBy) == 0))
            visualizeIteration(posList);

        return isOpen();
    }

    void visualizeIteration(const vData &posList) {

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

    void addToPaths(const vData &posList) {

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
