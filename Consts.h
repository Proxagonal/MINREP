#ifndef CONSTS_H
#define CONSTS_H

#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;



#define NUM 3
#define DIM 2
#define DIM_C2 (DIM*(DIM-1)/2)

typedef Vector<double, DIM> VectorDd;
typedef Vector<long double, DIM> VectorDld;

typedef Vector<double, 2> Vector2d;
typedef Vector<long double, 2> Vector2ld;

typedef vector<tuple<double, VectorDd, VectorDd>> initialData;
typedef array<VectorDd, NUM> vData;
typedef array<double, NUM> Data;

typedef Vector<double, DIM_C2> VectorAngd;

static inline int angularIndex(int i, int j) {

    return DIM_C2 - (DIM - i)*(DIM - i - 1)/2 + j - i - 1;

}

static const long double LD_PI = 3.141592653589793238462643383279L;
static const long double LD_G = 4*LD_PI*LD_PI;
static const long double LD_G_inv = 1/LD_G;
static const double G = LD_G;
static const double G_inv = 1/LD_G;

static double rand01() {

    static std::random_device rd;
    static std::mt19937_64 gen(rd());
    static std::uniform_real_distribution<double> dist{0, 1};

    return dist(gen);
}

const static streamsize DEFAULTDIGITS = std::cout.precision();
constexpr static streamsize ALLDIGITS = std::numeric_limits<double>::max_digits10;
#define DNAN numeric_limits<double>::quiet_NaN()


#endif
