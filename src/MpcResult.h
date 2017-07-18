#ifndef MPC_RESULT_H
#define MPC_RESULT_H

/* INCLUDES ******************************************************************/

#include <vector>

/* CLASS DECLARATION *********************************************************/

class MpcResult
{
  public:

    MpcResult(
      double steerinAngle,
      double throttle,
      const std::vector<double>& pointsX,
      const std::vector<double>& pointsY);

    virtual ~MpcResult();

  public:

    double getSteeringAngle() const;

    double getThrottle() const;

    std::vector<double> getPointsX() const;

    std::vector<double> getPointsY() const;

  private:

    double              m_steeringAngle;
    double              m_throttle;
    std::vector<double> m_pointsX;
    std::vector<double> m_pointsY;
};

#endif /* MPC_RESULT_H */
