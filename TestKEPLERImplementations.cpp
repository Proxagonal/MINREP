#include <cmath>
#include <gsl/gsl_roots.h>
#include <stdio.h>
#include <functional>

static const double threeCbrt2 = 3*std::cbrt(2);

double ellipticalTaylor(double M, double e) {
    if (e == 0)
        return M;
    double eSquared = e*e;
    double p = std::cbrt(162*eSquared*M + sqrt(26244*eSquared*eSquared*M*M - 23328*pow(e*(e-1), 3)));

    return p/(e*threeCbrt2) + 2*threeCbrt2*(e-1)/p + 0.145*eSquared*M;
}

double hyperbolicTaylor(double M, double e) {
    if (e == 0)
        return M;
    double eSquared = e*e;
    double p = std::cbrt(162*eSquared*M + sqrt(26244*eSquared*eSquared*M*M + 23328*pow(e*(e-1), 3)));

    return p/(e*threeCbrt2) - 2*threeCbrt2*(e-1)/p -0.07*M/eSquared;
}

double keplerInitial(double M, double e) {
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

}

void keplerEllipticalEvaluator(double E, void* M_e, double *y, double* dy) {

    double* M_e_double = (double*)M_e;
    double M = *M_e_double;
    double e = *(M_e_double+1);

    double cosE = cos(E);
    *y = E - e*sqrt(1-cosE*cosE) * (2 * (E<=M_PI) - 1) - M;
    *dy = 1 - e*cosE;
}

void keplerHyperbolicEvaluator(double E, void* M_e, double *y, double* dy) {

    double* M_e_double = (double*)M_e;
    double M = *M_e_double;
    double e = *(M_e_double+1);

    double coshE = cosh(E);
    *y = E + M - e*sqrt(coshE*coshE-1) * (2 * (E>=0) - 1);
    *dy = 1 - e*coshE;
}

double KEPLER(double M, double e) {

    double params[2] = {M, e};

    gsl_function_fdf fdf;
    fdf.fdf = keplerEllipticalEvaluator;
    fdf.params = (void*)params;

    gsl_root_fdfsolver *s = gsl_root_fdfsolver_alloc(gsl_root_fdfsolver_newton);
    gsl_root_fdfsolver_set(s, &fdf, x0);

    int status;
    size_t iter = 0;
    do {
        iter++;
        status = gsl_root_fdfsolver_iterate(s);
        root = gsl_root_fdfsolver_root(s);
        status = gsl_root_test_residual(func_f(root, NULL), 1e-7);
    } while (status == GSL_CONTINUE && iter < 100);

    printf("status = %s\n", gsl_strerror(status));
    printf("root = %.7f\n", root);

    gsl_root_fdfsolver_free(s);

}

int main() {

    double M = 0.5*M_PI;
    double e = 0.2;



}