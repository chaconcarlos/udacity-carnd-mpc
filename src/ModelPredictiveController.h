#ifndef MPC_H
#define MPC_H

/* INCLUDES ******************************************************************/

#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "MpcResult.h"

/* CLASS DECLARATION *********************************************************/

/**
 * @brief Implements a Model Predictive Controller (MPC)
 */
class ModelPredictiveController
{
  public:

    ModelPredictiveController(double lf, double delta);

    virtual ~ModelPredictiveController();

    MpcResult Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

  private:

    double m_lf;
    double m_delta;
};

#endif /* MPC_H */
