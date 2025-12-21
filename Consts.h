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




#endif
