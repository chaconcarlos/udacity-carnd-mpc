/* INCLUDES ******************************************************************/

#include "Utils.h"

#include <math.h>

/* STATIC FUNCTIONS **********************************************************/

/**
 * @brief Gets the value of PI.
 *
 * @return The value of PI.
 */
static constexpr
double pi() 
{ 
  return M_PI; 
}

/* FUNCTION IMPLEMENTATION ***************************************************/

double
 deg2rad(double degrees) 
{ 
  return degrees * pi() / 180; 
}

double 
rad2deg(double radians)
{ 
  return radians * 180 / pi();
}

double 
polyeval(Eigen::VectorXd coeffs, double x) 
{
  double result = 0.0;

  for (int i = 0; i < coeffs.size(); ++i) 
  	result += coeffs[i] * pow(x, i);
  
  return result;
}

Eigen::VectorXd 
polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) 
{
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);

  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); ++i) 
  	A(i, 0) = 1.0;

  for (int j = 0; j < xvals.size(); ++j) 
  {
    for (int i = 0; i < order; i++) 
      A(j, i + 1) = A(j, i) * xvals(j);
  }

  auto Q      = A.householderQr();
  auto result = Q.solve(yvals);

  return result;
}