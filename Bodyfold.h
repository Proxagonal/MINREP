#ifndef MINREP_BODYFOLD_H
#define MINREP_BODYFOLD_H

#include <iostream>
#include <Eigen/Eigen>
#include <cmath>
#include <random>

using namespace std;
using namespace Eigen;

#define NUM 3

typedef vector<tuple<double, Vector2d, Vector2d>> initialData;
typedef array<Vector2d, NUM> vData;
typedef array<double, NUM> Data;

struct Bodyfold {

private:

    constexpr static int ALLDIGITS = std::numeric_limits<double>::max_digits10;

    constexpr static double massMin = 0.5;
    constexpr static double massMax = 2;
    constexpr static double systemRadius = 35;
    constexpr static double velocityMax = 19;

    static Vector2d toCartesian(double rad, double theta) {
        return {rad*cos(theta), rad*sin(theta)};
    }

    static Vector2d randomOnRadius(double r) {
        return toCartesian(r * sqrt(rand01()), M_2_PI * rand01());
    }

    static double rand01() {

        static std::random_device rd;
        static std::mt19937_64 gen(rd());
        static std::uniform_real_distribution<double> dist{0, 1};

        return dist(gen);

        //return (double) rand() / RAND_MAX;
    }

public:

    vData posList;
    vData velList;
    vData accList;
    Data massList;

    Bodyfold(const initialData& init) {

        if (init.size() != NUM)
            throw invalid_argument("Not Correct Body Amount");

        int i = 0;
        for (auto const &[mass, pos, vel] : init) {
            posList[i] = pos;
            velList[i] = vel;
            accList[i] = Vector2d(0, 0);
            massList[i] = mass;
            i++;
        }
    };

    double sumKineticEnergy() {

        double kin = 0;
        for (int i = 0; i < NUM; i++)
            kin += massList[i] * velList[i].squaredNorm() / 2;

        return kin;
    };

    Vector2d sumMomentum() {

        Vector2d mom(0,0);
        for (int i = 0; i < NUM; i++)
            mom += massList[i] * velList[i];

        return mom;
    }

    double sumMass() {

        double mass = 0;
        for (int i = 0; i < NUM; i++)
            mass += massList[i];

        return mass;
    }

    Vector2d getCOMVelocity() {

        return sumMomentum()/sumMass();

    }

    Vector2d getCOMPosition() {

        Vector2d com(0,0);

        for (int i = 0; i < NUM; i++)
            com += massList[i] * posList[i];

        return com/sumMass();
    }

    // Symmetries: rotation, scale, mass sum, COM, p_COM
    static initialData generateRandomCOM() {

        initialData list;

        Vector2d pos;
        Vector2d vel;
        double mass;

        for (int i = 0; i < NUM; i++) {
            pos = randomOnRadius(systemRadius);
            vel = randomOnRadius(velocityMax);
            mass = massMin + (massMax - massMin) * rand01();
            list.emplace_back(mass, pos, vel);
        }

        return transformToCOMSystem(list);
    }

    static initialData generateRandomCOM_no3() {

        initialData list;

        Vector2d pos;
        Vector2d vel;
        double mass;

        for (int i = 0; i < NUM; i++) {
            pos = randomOnRadius(systemRadius);
            vel = randomOnRadius(velocityMax);
            mass = (i < 2)*(massMin + (massMax - massMin) * rand01());
            list.emplace_back(mass, pos, vel);
        }

        return transformToCOMSystem(list);
    }

    static initialData transformToCOMSystem(initialData &init) {

        initialData COMMED;

        double massSum = 0;
        Vector2d weightedPoses(0, 0);
        Vector2d weightedVels(0, 0);

        for (auto const &[mass, pos, vel] : init) {
            massSum += mass;
            weightedPoses += mass*pos;
            weightedVels += mass*vel;
        }

        Vector2d COM = weightedPoses/massSum;
        Vector2d COMVel = weightedVels/massSum;

        for (auto const &[mass, pos, vel] : init)
            COMMED.emplace_back(mass, pos - COM, vel - COMVel);

        return COMMED;
    }

    string toString() {

        stringstream ss;
        ss.precision(ALLDIGITS);

        for (int i = 0; i < NUM; i++) {

            ss << "Body #" << i << ": " << endl;
            ss << "Mass: " << massList[i] << endl;
            ss << "Position: " << posList[i].transpose() << endl;
            ss << "Velocity: " << velList[i].transpose() << endl;
            ss << "Acceleration: " << accList[i].transpose() << endl;

        }

        return ss.str();
    }

    static string toString(initialData &initialConditions) {

        stringstream ss;
        ss.precision(ALLDIGITS);

        for (int i = 0; i < NUM; i++) {

            ss << "Body #" << i << ": " << endl;
            ss << "Mass: " << get<0>(initialConditions.at(i)) << endl;
            ss << "Position: " << get<1>(initialConditions.at(i)).transpose() << endl;
            ss << "Velocity: " << get<2>(initialConditions.at(i)).transpose() << endl;
        }

        return ss.str();
    }

    static initialData stringToInitialData(string &str) {

        initialData bodies;
        istringstream iss(str);
        string line;
        double mass, posX, posY, velX, velY;
        while (getline(iss, line)) {
            if (line.find("Mass") != string::npos) {
                stringstream(line.substr(line.find(":") + 1)) >> mass;
                getline(iss, line);  // Position line
                stringstream(line.substr(line.find(":") + 1)) >> posX >> posY;
                getline(iss, line);  // Velocity line
                stringstream(line.substr(line.find(":") + 1)) >> velX >> velY;
                bodies.emplace_back(mass, Eigen::Vector2d(posX, posY), Eigen::Vector2d(velX, velY));
            }
        }

        return bodies;
    }

};

#endif