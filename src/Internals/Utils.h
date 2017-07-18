#ifndef UTILS_H
#define UTILS_H

/* INCLUDES ******************************************************************/

#include "../Eigen-3.3/Eigen/Core"
#include "../Eigen-3.3/Eigen/QR"

/* FUNCTION DECLARATION ******************************************************/

/**
 * @brief Converts an angle from degrees to radians.
 *
 * @param degrees The angle in degrees to convert.
 *
 * @return The angle in radians.
 */
double deg2rad(double degrees);

/**
 * @brief Converts an angle from radians to degrees.
 *
 * @param radians The angle in radians to convert.
 *
 * @return The angle in degrees.
 */
double rad2deg(double radians);

/**
 * @brief Evaluates a polynomial.
 *
 * @param coeffs The coefficients of the polynomial.
 * @param x      The value to evaluate.
 *
 * @return The result of the evaluation of the polynomial.
 */
double polyeval(Eigen::VectorXd coeffs, double x);

/**
 * @brief Fits a polynomial to a series of points. 
 * Adapted from https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716.
 *
 * @param xvals The values for X.
 * @param yvals The values for Y.
 *
 * @return The coefficients of the resulting polynomial.
 */
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);

#endif /* UTILS_H */