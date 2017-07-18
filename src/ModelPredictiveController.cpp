/* INCLUDES ******************************************************************/

#include "ModelPredictiveController.h"

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <limits>

#include "Internals/Utils.h"

/* USINGS ********************************************************************/

using CppAD::AD;

/* DEFINITIONS ***************************************************************/

static const size_t STEP_COUNT                      = 10;
static const double CTE_COST_WEIGHT                 = 20000;
static const double EPSI_COST_WEIGHT                = 20000;
static const double SPEED_COST_WEIGHT               = 100;
static const double ACTUATOR_COST_WEIGHT            = 1;
static const double STEER_CHANGE_COST_WEIGHT        = 200;
static const double ACCELERATION_CHANGE_COST_WEIGHT = 200;
static const double REFERENCE_SPEED                 = 70;
static const double MAX_DEGREE_RADIANS              = deg2rad(25);
static const double MAX_ACCELERATION                = 1.0;
static const size_t x_start                         = 0;
static const size_t y_start                         = x_start     + STEP_COUNT;
static const size_t psi_start                       = y_start     + STEP_COUNT;
static const size_t v_start                         = psi_start   + STEP_COUNT;
static const size_t cte_start                       = v_start     + STEP_COUNT;
static const size_t epsi_start                      = cte_start   + STEP_COUNT;
static const size_t delta_start                     = epsi_start  + STEP_COUNT;
static const size_t a_start                         = delta_start + STEP_COUNT - 1;

/* FUNCTORS ******************************************************************/

class FG_eval
{

  public:

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

  public:

    FG_eval(Eigen::VectorXd coeffs, double lf, double delta)
    : m_lf(lf)
    , m_delta(delta)
    , m_coeffs(coeffs)
    {
    }

    void operator()(ADvector &fg, const ADvector &vars)
    {
      fg[0] = 0;

      // Objective term 1: Keep close to reference values
      for (size_t t = 0; t < STEP_COUNT; ++t)
      {
        fg[0] += CTE_COST_WEIGHT   * CppAD::pow(vars[cte_start  + t], 2);
        fg[0] += EPSI_COST_WEIGHT  * CppAD::pow(vars[epsi_start + t], 2);
        fg[0] += SPEED_COST_WEIGHT * CppAD::pow(vars[v_start    + t] - REFERENCE_SPEED, 2);
      }

      // Objective term 2:  Avoid to actuate, as much as possible
      for (size_t t = 0; t < STEP_COUNT - 1; ++t)
      {
        fg[0] += ACTUATOR_COST_WEIGHT * CppAD::pow(vars[delta_start + t], 2);
        fg[0] += ACTUATOR_COST_WEIGHT * CppAD::pow(vars[a_start     + t], 2);
      }

      // Objective term 3:  Enforce actuators smoothness in change
      for (size_t t = 0; t < STEP_COUNT - 2; ++t)
      {
        fg[0] += STEER_CHANGE_COST_WEIGHT        * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
        fg[0] += ACCELERATION_CHANGE_COST_WEIGHT * CppAD::pow(vars[a_start     + t + 1] - vars[a_start     + t], 2);
      }

      // Initial constraints
      fg[1 + x_start]    = vars[x_start];
      fg[1 + y_start]    = vars[y_start];
      fg[1 + psi_start]  = vars[psi_start];
      fg[1 + v_start]    = vars[v_start];
      fg[1 + cte_start]  = vars[cte_start];
      fg[1 + epsi_start] = vars[epsi_start];

      for (size_t t = 1; t < STEP_COUNT; ++t)
      {
        const AD<double> x0      = vars[x_start    + t - 1];
        const AD<double> y0      = vars[y_start    + t - 1];
        const AD<double> psi0    = vars[psi_start  + t - 1];
        const AD<double> v0      = vars[v_start    + t - 1];
        const AD<double> cte0    = vars[cte_start  + t - 1];
        const AD<double> epsi0   = vars[epsi_start + t - 1];

        const AD<double> x1      = vars[x_start    + t];
        const AD<double> y1      = vars[y_start    + t];
        const AD<double> psi1    = vars[psi_start  + t];
        const AD<double> v1      = vars[v_start    + t];
        const AD<double> cte1    = vars[cte_start  + t];
        const AD<double> epsi1   = vars[epsi_start + t];

        const AD<double> delta0  = vars[delta_start + t - 1];
        const AD<double> a0      = vars[a_start     + t - 1];
        const AD<double> f0      = m_coeffs[0] + m_coeffs[1] * x0 + m_coeffs[2] * x0 * x0 + m_coeffs[3] * x0 * x0 * x0;
        const AD<double> psides0 = CppAD::atan(m_coeffs[1] + 2 * m_coeffs[2] * x0 + 3 * m_coeffs[3] * x0 * x0);

        // Setup other model constraints
        fg[1 + x_start + t]     = x1    - (x0 + v0 * CppAD::cos(psi0) * m_delta);
        fg[1 + y_start + t]     = y1    - (y0 + v0 * CppAD::sin(psi0) * m_delta);
        fg[1 + psi_start + t]   = psi1  - (psi0 - v0 * delta0 / m_lf * m_delta);
        fg[1 + v_start + t]     = v1    - (v0 + a0 * m_delta);
        fg[1 + cte_start + t]   = cte1  - (f0 - y0 + (v0 * CppAD::sin(epsi0) * m_delta));
        fg[1 + epsi_start + t]  = epsi1 - (psi0 - psides0 - v0 * delta0 / m_lf * m_delta);
      }
    }

  private:

    double          m_lf;
    double          m_delta;
    Eigen::VectorXd m_coeffs;
};

/* CLASS IMPLEMENTATION ******************************************************/

ModelPredictiveController::ModelPredictiveController(double lf, double delta)
: m_lf(lf)
, m_delta(delta)
{
}

ModelPredictiveController::~ModelPredictiveController()
{
}

MpcResult
ModelPredictiveController::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs)
{
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  double x    = state[0];
  double y    = state[1];
  double psi  = state[2];
  double v    = state[3];
  double cte  = state[4];
  double epsi = state[5];

  // Set the number of model variables (includes both states and inputs).
  size_t n_vars = STEP_COUNT * state.size() + (STEP_COUNT - 1) * 2;

  // Set the number of constraints
  size_t n_constraints = STEP_COUNT * state.size();

  // Initial all of independent variables to zero.
  Dvector vars(n_vars);

  for (size_t i = 0; i < n_vars; i++)
    vars[i] = 0.0;

  Dvector vars_lower_bounds(n_vars);
  Dvector vars_upper_bounds(n_vars);

  // Set limits for non-actuators (avoid numerical issues during optimization)
  for (size_t i = 0; i < delta_start; ++i)
  {
    vars_lower_bounds[i] = - std::numeric_limits<double>::max();
    vars_upper_bounds[i] = + std::numeric_limits<double>::max();
  }

  // Set upper and lower constraints for steering
  for (size_t i = delta_start; i < a_start; ++i)
  {
    vars_lower_bounds[i] = - MAX_DEGREE_RADIANS;
    vars_upper_bounds[i] = + MAX_DEGREE_RADIANS;
  }

  for (size_t i = a_start; i < n_vars; ++i)
  {
    vars_lower_bounds[i] = - MAX_ACCELERATION;
    vars_upper_bounds[i] = + MAX_ACCELERATION;
  }

  // Initialize to zero lower and upper limits for the constraints
  Dvector constraints_lower_bounds(n_constraints);
  Dvector constraints_upper_bounds(n_constraints);

  for (size_t i = 0; i < n_constraints; ++i)
  {
    constraints_lower_bounds[i] = 0;
    constraints_upper_bounds[i] = 0;
  }

  // Force the solver to start from current state in optimization space
  constraints_lower_bounds[x_start] = x;        constraints_upper_bounds[x_start] = x;
  constraints_lower_bounds[y_start] = y;        constraints_upper_bounds[y_start] = y;
  constraints_lower_bounds[psi_start] = psi;    constraints_upper_bounds[psi_start] = psi;
  constraints_lower_bounds[v_start] = v;        constraints_upper_bounds[v_start] = v;
  constraints_lower_bounds[cte_start] = cte;    constraints_upper_bounds[cte_start] = cte;
  constraints_lower_bounds[epsi_start] = epsi;  constraints_upper_bounds[epsi_start] = epsi;

  // Object that computes objective and constraints
  FG_eval fg_eval(coeffs, m_lf, m_delta);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
          options, vars, vars_lower_bounds, vars_upper_bounds, constraints_lower_bounds,
          constraints_upper_bounds, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  std::vector<double> nextPointsX;
  std::vector<double> nextPointsY;

  // Add "future" solutions (where MPC is going)
  for (size_t i = 0; i < STEP_COUNT; ++i)
  {
    nextPointsX.push_back(solution.x[x_start + i]);
    nextPointsY.push_back(solution.x[y_start + i]);
  }

  // Return the first actuator values. The variables can be accessed with `solution.x[i]`.
  MpcResult result(solution.x[delta_start], solution.x[a_start], nextPointsX, nextPointsY);

  return result;
}
