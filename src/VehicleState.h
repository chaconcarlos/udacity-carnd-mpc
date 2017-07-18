#ifndef VEHICLE_STATE_H
#define VEHICLE_STATE_H

/* INCLUDES ******************************************************************/

#include <cstring>
#include <vector>

#include "Eigen-3.3/Eigen/Core"

/* CLASS DECLARATION *********************************************************/

/**
 * @brief Represents the localization state of a vehicle.
 */
class VehicleState
{
	public:

		/**
		 * @brief Initializes an instance of the VehicleState class.
		 *
		 * @param x       The vehicle position in X.
		 * @param y       The vehicle position in Y.
		 * @param psi     The vehicle yaw angle.
		 * @param speed   The vehicle speed.
		 * @param pointsX The vehicle trajectory points X values.
		 * @param pointsY The vehicle trajectory points Y values.
		 */
		VehicleState(
			double x, 
			double y,
			double psi,
			double speed, 
			double steeringAngle,
			double throttle,
			const std::vector<double>& pointsX, 
			const std::vector<double>& pointsY);
		
		/**
		 * @brief Finalizes an instance of the VehicleState class.
		 */
		virtual ~VehicleState();

	public:

		/**
		 * @brief Gets the position in X for the vehicle.
		 * 
		 * @return The position in X for the vehicle.
		 */
		double getX() const;

		/**
		 * @brief Gets the position in Y for the vehicle.
		 * 
		 * @return The position in Y for the vehicle.
		 */
		double getY() const;

		/**
		 * @brief Gets the vehicle yaw angle.
		 * 
		 * @return The vehicle yaw angle.
		 */
		double getPsi() const;

		/**
		 * @brief Gets the vehicle speed.
		 * 
		 * @return The vehicle speed.
		 */
		double getSpeed() const;

		/**
		 * @brief Gets the vehicle steering angle.
		 * 
		 * @return The vehicle steering angle.
		 */
		double getSteeringAngle() const;

		/**
		 * @brief Gets the vehicle throttle.
		 * 
		 * @return The vehicle throttle.
		 */
		double getThrottle() const;

		/**
		 * @brief Gets the vehicle trajectory points X values.
		 * 
		 * @return The vehicle trajectory points X values
		 */
		Eigen::VectorXd getPointsX() const;

		/**
		 * @brief Gets the vehicle trajectory points Y values.
		 * 
		 * @return The vehicle trajectory points Y values
		 */
		Eigen::VectorXd getPointsY() const;

	private:

		double          m_x;
		double          m_y;
		double          m_psi;
		double          m_speed;
		double          m_steeringAngle;
		double          m_throttle;
		Eigen::VectorXd m_pointsX;
		Eigen::VectorXd m_pointsY;
};

#endif /* VEHICLE_STATE_H */