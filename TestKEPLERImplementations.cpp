#include <cmath>
#include <iomanip>
#include <gsl/gsl_errno.h>
#include <iostream>
#include <chrono>
#include <err.h>

using std::cout;
using std::endl;
using std::tuple;
using std::numeric_limits;

#define ITERLIM 10
#define NCHECKS 5


static const double threeCbrt2 = 3*std::cbrt(2);

static const double SMALL = numeric_limits<double>::epsilon();
static const int max_digits = numeric_limits<double>::max_digits10;

void printMils(std::chrono::steady_clock::time_point start, std::chrono::steady_clock::time_point end) {
    cout << (duration_cast<std::chrono::milliseconds>(end - start)).count() << endl;
}

std::chrono::steady_clock::time_point now() {
    return std::chrono::steady_clock::now();
}

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

#define GSL_FN_FDF_EVAL_F(FDF,x) (*((FDF)->f))(x,(FDF)->params)
#define GSL_FN_FDF_EVAL_DF(FDF,x) (*((FDF)->df))(x,(FDF)->params)
#define GSL_FN_FDF_EVAL_F_DF(FDF,x,y,dy) (*((FDF)->fdf))(x,(FDF)->params,(y),(dy))

typedef struct
{
    double f, df;
}
newton_state_t;

struct gsl_function_fdf
{
    void (* fdf) (double x, void * params, double * f, double * df);
    void * params;
};

static int
newton_init (newton_state_t * vstate, gsl_function_fdf * fdf, double * root)
{
    const double x = *root ;

    GSL_FN_FDF_EVAL_F_DF(fdf, *root, &vstate->f, &vstate->df);

    return GSL_SUCCESS;
}

static int
newton_iterate (newton_state_t * vstate, gsl_function_fdf * fdf, double * root)
{

    double root_new, f_new, df_new;

    if (vstate->df == 0.0)
    {
        GSL_ERROR("derivative is zero", GSL_EZERODIV);
    }

    root_new = *root - (vstate->f / vstate->df);

    *root = root_new ;

    GSL_FN_FDF_EVAL_F_DF(fdf, root_new, &f_new, &df_new);

    vstate->f = f_new ;
    vstate->df = df_new ;

    if (!std::isfinite(f_new))
    {
        GSL_ERROR ("function value is not finite", GSL_EBADFUNC);
    }

    if (!std::isfinite(df_new))
    {
        GSL_ERROR ("derivative value is not finite", GSL_EBADFUNC);
    }

    return GSL_SUCCESS;
}

int
root_test_residual (double f, double epsabs)
{
    if (std::abs(f) < epsabs)
        return GSL_SUCCESS;

    return GSL_CONTINUE ;
}

int
root_test_residual_abs (double f, double epsabs)
{
    if (f < epsabs)
        return GSL_SUCCESS;

    return GSL_CONTINUE ;
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

//tuple<int, double, int, double> KEPLER(double M, double e) {
//
//    double params[2] = {M, e};
//
//    gsl_function_fdf fdf;
//    fdf.fdf = (e < 1 ? &keplerEllipticalEvaluator_fdf : &keplerHyperbolicEvaluator_fdf);
//    fdf.params = params;
//
//    newton_state_t vstate;
//
//    double root = keplerInitial(M, e);
//    fdf.fdf(root, params, &vstate.f, &vstate.df);
//
//    double prevroot = -1, prevdist, dist = 1<<10;
//
//    int status;
//    size_t iter = 0;
//    do {
//        prevroot = root;
//        prevdist = dist;
//
//        iter++;
//        newton_iterate(&vstate, &fdf, &root);
//        dist = std::abs(fdf.f(root, params));
//
//        if (prevdist <= dist) {
//            status = GSL_SUCCESS;
//            root = prevroot;
//            dist = prevdist;
//            break;
//        }
//
//       status = root_test_residual_abs(dist, SMALL);
//    } while (status == GSL_CONTINUE && iter < ITERLIM);
//
//    if (dist > pow(10, -8))
//        status = GSL_ETOL;
//
//    return {status, root, iter ,dist};
//}


double KEPLER_PRACTICAL(double M, double e) {

    double params[2] = {M, e};

    gsl_function_fdf fdf;
    fdf.fdf = (e < 1 ? &keplerEllipticalEvaluator_fdf : &keplerHyperbolicEvaluator_fdf);
    fdf.params = params;

    newton_state_t vstate;

    double root = keplerInitial(M, e);
    fdf.fdf(root, params, &vstate.f, &vstate.df);

    double prevroot = -1, prevdist, dist = 1<<10;

    size_t iter = 0;
    do {
        prevroot = root;
        prevdist = dist;

        iter++;
        if (newton_iterate(&vstate, &fdf, &root) != GSL_SUCCESS)
            throw std::out_of_range("Infinity or 1/0 " + std::to_string(e) + ", " + std::to_string(M));
        dist = std::abs(vstate.f);

        if (prevdist <= dist) {
            root = prevroot;
            dist = prevdist;
            break;
        }
    } while (iter < ITERLIM || root_test_residual_abs(dist, SMALL));

    if (dist > pow(10, -8))
        throw std::out_of_range("Catastrophic Convergence " + std::to_string(e) + ", " + std::to_string(M));

    return root;
}

int main() {

    double worstAccuracy = 0;
    int worstN = 0;
    long NLIST[20] = {0};
    long ACCLIST[40] = {0};

    cout << "epsilon: " << SMALL << endl;
    cout << std::setprecision(max_digits + 1);
    cout << "max digits: " << max_digits << endl;


    long long COUNT = 0, FAILS = 0;

    double e_lim = 100;
    double hyp_M_lim = 1000;
    int e_num = pow(10, 4.5);
    int M_num = pow(10, 4.5);

    auto start = now();

    for (int i_e = 0; i_e < e_num; i_e++) {

        double e = i_e * e_lim/e_num;

        if (e == 1)
            continue;

        double M_lim = {e < 1 ? 2*M_PI : hyp_M_lim};

        for (int i_M = 0; i_M < M_num; i_M++) {

            double M = i_M * M_lim/M_num;

            auto [status, root, N, dist] = KEPLER(M, e);

            bool fail = (status == GSL_CONTINUE);


            NLIST[std::min(N, 19)] += 1;
            int exp = ((dist == 0) ? 0 : 1 + (int)std::floor(std::log10(std::fabs(dist))));
            int index = std::max(0, std::min(39, 20 + exp));
            ACCLIST[index] += 1;

            //if (exp > -11 && dist != 0) {
            //    cout << "HEYO" << endl;
            //    cout << exp << endl;
            //    cout << "M, e: " << M << ", " << e << endl;
            //    cout << "status, root, N, value: " << gsl_strerror(status) << ", " << root << ", " << N << ", " << dist << endl;
            //    return 1;
            //}

            if (dist > worstAccuracy) {
                worstAccuracy = dist;
                cout << "Worst Accuracy so far:" << endl;
                cout << "M, e: " << M << ", " << e << endl;
                cout << "status, root, N, value: " << gsl_strerror(status) << ", " << root << ", " << N << ", " << dist << endl;
            }

            if (N > worstN) {
                worstN = N;
                cout << "Worst NUM so far:" << endl;
                cout << "M, e: " << M << ", " << e << endl;
                cout << "status, root, N, value: " << gsl_strerror(status) << ", " << root << ", " << N << ", " << dist << endl;
            }

            if (fail) {
                FAILS++;
                cout << "FAIL: " << gsl_strerror(status) << endl;
                cout << "M, e: " << M << ", " << e << endl;
                cout << "status, root, N: " << gsl_strerror(status) << ", " << root << ", " << N << ", " << dist << endl;
            }

            if (!fail && status != GSL_SUCCESS) {
                cout << "UNEXPECTED ERROR: " << gsl_strerror(status) << endl;
                cout << "M, e: " << M << ", " << e << endl;
                cout << "status, root, N: " << gsl_strerror(status) << ", " << root << ", " << N << ", " << dist << endl;
            }

            COUNT++;
        }
    }

    cout << COUNT << endl;
    cout << FAILS << endl;

    int i;
    for (i = 0; i < 20; i++)
        cout << NLIST[i] << ", ";
    cout << endl;
    for (i = 0; i < 40; i++)
        cout << ACCLIST[i] << ", ";
    cout << endl;

    cout << std::fixed << std::setprecision(2);
    for (i = 0; i < 20; i++)
        cout << 100*NLIST[i]/(double)COUNT << ", ";
    cout << endl;
    for (i = 0; i < 40; i++)
        cout << 100*ACCLIST[i]/(double)COUNT << ", ";
    cout << endl;

    cout << "time: ";
    printMils(start, now());

}