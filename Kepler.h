#ifndef KEPLER_H
#define KEPLER_H

#include <cmath>
#include <iomanip>

#include <unistd.h>

#define ITERLIM 10
static const double SMALL = std::numeric_limits<double>::epsilon();
static const double threeCbrt2 = 3*std::cbrt(2);

typedef struct
{
    double f, df;
}
newton_state_t;

struct function_fdf
{
    void (* fdf) (double x, void * params, double * f, double * df);
    void * params;
};

static int newton_iterate (newton_state_t * vstate, function_fdf * fdf, double * root)
{

    double root_new, f_new, df_new;

    if (vstate->df == 0.0)
    {
        return -1; //error
    }

    root_new = *root - (vstate->f / vstate->df);

    *root = root_new;

    fdf->fdf(root_new, fdf->params, &f_new, &df_new);

    vstate->f = f_new ;
    vstate->df = df_new ;

    if (!std::isfinite(f_new))
    {
        return -1; //error
    }

    if (!std::isfinite(df_new))
    {
        return -1; //error
    }

    return 0; //success
}

static int root_test_residual_abs (double f, double epsabs)
{
    if (f < epsabs)
        return 0; //success

    return 1 ; //continue
}

static void keplerEllipticalEvaluator_fdf(double E, void* M_e, double *y, double* dy) {

    double* M_e_double = (double*)M_e;
    double M = *M_e_double;
    double e = *(M_e_double+1);

    double cosE = cos(E);
    *y = E - e*sqrt(1-cosE*cosE) * (2 * (E<=M_PI) - 1) - M;
    *dy = 1 - e*cosE;
}

static void keplerHyperbolicEvaluator_fdf(double E, void* M_e, double *y, double* dy) {

    double* M_e_double = (double*)M_e;
    double M = *M_e_double;
    double e = *(M_e_double+1);

    double coshE = cosh(E);
    *y = E + M - e*sqrt(coshE*coshE-1) * (2 * (E>=0) - 1);
    *dy = 1 - e*coshE;
}

static double ellipticalTaylor(double M, double e) {
    if (e == 0)
        return M;
    double eSquared = e*e;
    double p = std::cbrt(162*eSquared*M + sqrt(26244*eSquared*eSquared*M*M - 23328*pow(e*(e-1), 3)));

    return p/(e*threeCbrt2) + 2*threeCbrt2*(e-1)/p + 0.145*eSquared*M;
}

static double hyperbolicTaylor(double M, double e) {
    if (e == 0)
        return M;
    double eSquared = e*e;
    double p = std::cbrt(162*eSquared*M + sqrt(26244*eSquared*eSquared*M*M + 23328*pow(e*(e-1), 3)));

    return p/(e*threeCbrt2) - 2*threeCbrt2*(e-1)/p -0.07*M/eSquared;
}

static double keplerInitial(double M, double e) {
    if (e < 1) {
        if (M > M_PI)
            return 2*M_PI - ellipticalTaylor(2*M_PI - M, e);
        return ellipticalTaylor(M, e);
    }
    if (e>1) {
        if (M <= 10)
            return hyperbolicTaylor(M, e);
        return 3.3/M + asinh(M/e);
    }
    return -1;
}

namespace Kepler {

    static double KEPLER(double M, double e) {

        double params[2] = {M, e};

        function_fdf fdf;
        fdf.fdf = (e < 1 ? &keplerEllipticalEvaluator_fdf : &keplerHyperbolicEvaluator_fdf);
        fdf.params = params;

        newton_state_t vstate;

        double root = keplerInitial(M, e);
        fdf.fdf(root, params, &vstate.f, &vstate.df);

        double prevroot = -1, prevdist, dist = 1<<5;

        size_t iter = 0;
        do {
            prevroot = root;
            prevdist = dist;

            iter++;
            if (newton_iterate(&vstate, &fdf, &root) != 0)
                throw std::out_of_range("Infinity or 1/0 " + std::to_string(e) + ", " + std::to_string(M));
            dist = std::abs(vstate.f);

            if (prevdist <= dist) {
                root = prevroot;
                dist = prevdist;
                break;
            }
        } while (iter < ITERLIM || root_test_residual_abs(dist, SMALL));

        if (dist > pow(10, -8)) {
            std::ostringstream out;
            out.precision(20);
            out << std::fixed << "Catastrophic Convergence " << e << ", " << M << ": " << root << ", delta=" << dist;
            throw std::out_of_range(out.str());
        }

        return root;
    }
}



#endif //KEPLER_H
