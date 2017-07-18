/* INCLUDES ******************************************************************/

#include "VehicleState.h"

/* CLASS IMPLEMENTATION ******************************************************/

VehicleState::VehicleState(double x, 
	double y,
	double psi,
	double speed, 
	double steeringAngle,
	double throttle,
	const std::vector<double>& pointsX, 
	const std::vector<double>& pointsY)
: m_x(x)
, m_y(y)
, m_psi(psi)
, m_speed(speed)
, m_steeringAngle(steeringAngle)
, m_throttle(throttle)
, m_pointsX(Eigen::VectorXd(pointsX.size()))
, m_pointsY(Eigen::VectorXd(pointsX.size()))
{
	const double cosPsi = cos(psi);
	const double sinPsi = sin(psi);

	for (size_t i = 0; i < pointsX.size(); ++i) 
	{
		const double xn = pointsX[i] - x;
		const double yn = pointsY[i] - y;

		m_pointsX[i] =  xn * cosPsi + yn * sinPsi;
		m_pointsY[i] = -xn * sinPsi + yn * cosPsi;
	}
}

VehicleState::~VehicleState()
{
}

double 
VehicleState::getX() const
{
	return m_x;
}

double 
VehicleState::getY() const
{
	return m_y;
}

double 
VehicleState::getPsi() const
{
	return m_psi;
}

double 
VehicleState::getSpeed() const
{
	return m_speed;
}

double 
VehicleState::getSteeringAngle() const
{
	return m_steeringAngle;
}

double 
VehicleState::getThrottle() const
{
	return m_throttle;
}

Eigen::VectorXd
VehicleState::getPointsX() const
{
	return m_pointsX;
}

Eigen::VectorXd 
VehicleState::getPointsY() const
{
	return m_pointsY;
}