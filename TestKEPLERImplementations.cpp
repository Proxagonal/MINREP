#include <cmath>
#include <gsl/gsl_roots.h>
#include <gsl/gsl_errno.h>
#include <iostream>

using std::cout;
using std::endl;

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
    return -1;
}

void keplerEllipticalEvaluator_fdf(double E, void* M_e, double *y, double* dy) {

    double* M_e_double = (double*)M_e;
    double M = *M_e_double;
    double e = *(M_e_double+1);

    double cosE = cos(E);
    *y = E - e*sqrt(1-cosE*cosE) * (2 * (E<=M_PI) - 1) - M;
    *dy = 1 - e*cosE;
}

void keplerHyperbolicEvaluator_fdf(double E, void* M_e, double *y, double* dy) {

    double* M_e_double = (double*)M_e;
    double M = *M_e_double;
    double e = *(M_e_double+1);

    double coshE = cosh(E);
    *y = E + M - e*sqrt(coshE*coshE-1) * (2 * (E>=0) - 1);
    *dy = 1 - e*coshE;
}

double keplerEllipticalEvaluator_f(double E, void* M_e) {

    double* M_e_double = (double*)M_e;
    double M = *M_e_double;
    double e = *(M_e_double+1);

    return E - e*sin(E) - M;
}

double keplerHyperbolicEvaluator_f(double E, void* M_e) {

    double* M_e_double = (double*)M_e;
    double M = *M_e_double;
    double e = *(M_e_double+1);

    return E + M - e*sinh(E);
}

double keplerEllipticalEvaluator_df(double E, void* M_e) {

    double* M_e_double = (double*)M_e;
    double e = *(M_e_double+1);

    return 1 - e*cos(E);
}

double keplerHyperbolicEvaluator_df(double E, void* M_e) {

    double* M_e_double = (double*)M_e;
    double e = *(M_e_double+1);

    return 1 - e*cosh(E);
}

double keplerEllipticalValue(double E, void* M_e) {

    double* M_e_double = (double*)M_e;
    double M = *M_e_double;
    double e = *(M_e_double+1);

    double cosE = cos(E);
    return E - e*sqrt(1-cosE*cosE) * (2 * (E<=M_PI) - 1) - M;
}

double keplerHyperbolicValue(double E, void* M_e) {

    double* M_e_double = (double*)M_e;
    double M = *M_e_double;
    double e = *(M_e_double+1);

    double coshE = cosh(E);
    return E + M - e*sqrt(coshE*coshE-1) * (2 * (E>=0) - 1);
}

double KEPLER(double M, double e) {

    double params[2] = {M, e};

    gsl_function_fdf fdf;
    fdf.f = &keplerEllipticalEvaluator_f;
    fdf.df = &keplerEllipticalEvaluator_df;
    fdf.fdf = (e < 1 ? &keplerEllipticalEvaluator_fdf : &keplerHyperbolicEvaluator_fdf);
    fdf.params = params;


    double (*value)(double, void*) = (e < 1 ? &keplerEllipticalValue : &keplerHyperbolicValue);

    cout << "yo" << endl;

    const gsl_root_fdfsolver_type * T = gsl_root_fdfsolver_newton;
    gsl_root_fdfsolver * s = gsl_root_fdfsolver_alloc(T);
    cout << "yo" << endl;

    double init = keplerInitial(M, e);

    //s->fdf = &fdf;
    //s->root = init;
//
    //cout << "HEYO" << GSL_FN_FDF_EVAL_F (&fdf, init) << endl;
//
//
    //cout << ((s->type->set) (s->state, s->fdf, &(s->root))) << endl;


    cout << "DONE" << endl;

    gsl_root_fdfsolver_set(s, &fdf, init);
    cout << "yo" << endl;

    double root;

    int status;
    size_t iter = 0;
    do {
        iter++;
        status = gsl_root_fdfsolver_iterate(s);
        root = gsl_root_fdfsolver_root(s);
        status = gsl_root_test_residual(value(root, params), 1e-7);
    } while (status == GSL_CONTINUE && iter < 100);

    printf("status = %s\n", gsl_strerror(status));
    printf("root = %.7f\n", root);

    gsl_root_fdfsolver_free(s);

    return root;
}

int main() {

    double M = 0.5*M_PI;
    double e = 0.2;

    cout << KEPLER(M, e) << endl;
}