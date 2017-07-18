/* INCLUDES ******************************************************************/

#include "MpcResult.h"

/* CLASS IMPLEMENTATION ******************************************************/

MpcResult::MpcResult(
	double steerinAngle, 
	double throttle, 
	const std::vector<double>& pointsX,
	const std::vector<double>& pointsY)
: m_steeringAngle(steerinAngle)
, m_throttle(throttle)
, m_pointsX(pointsX)
, m_pointsY(pointsY)
{
}

MpcResult::~MpcResult()
{
}

double
MpcResult::getSteeringAngle() const
{
	return m_steeringAngle;
}

double
MpcResult::getThrottle() const
{
	return m_throttle;
}

std::vector<double> 
MpcResult::getPointsX() const
{
	return m_pointsX;
}

std::vector<double> 
MpcResult::getPointsY() const
{
	return m_pointsY;
}
