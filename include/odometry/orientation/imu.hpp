#pragma once

#include "api.h"
#include "units/units.hpp"
#include "orientation.hpp"

namespace Pronounce {

	/**
	 * @brief IMU wrapper for orientation
	 * 
	 * @authors Alex Dickhans (alexDickhans)
	 */
	class IMU : public Orientation {
	private:
		/**
		 * @brief Reference to the imu
		 * 
		 */
		pros::Imu& imu;
	public:
		/**
		 * @brief Construct a new IMU object with a reference to the imu
		 * 
		 * @param imu 
		 */
		IMU(pros::Imu& imu) : imu(imu), Orientation(0.0) {}

		/**
		 * @brief Update the imu
		 * 
		 */
		void update() {
			Angle imuAngle = toRadians(imu.get_rotation());

			this->setAngle(imuAngle);
		}

		/**
		 * @brief Reset all the values
		 * 
		 */
		void reset() {
			this->imu.reset();
			this->setAngle(0.0);
		}

		~IMU() {}
	};
} // namespace Pronounce
