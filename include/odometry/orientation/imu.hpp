#pragma once

#include "api.h"
#include "units/units.hpp"
#include "orientation.hpp"
#include "pros/apix.h"

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
			if (pros::c::registry_get_plugged_type(15) == pros::c::v5_device_e_t::E_DEVICE_IMU) {
				Angle imuAngle = imu.get_rotation() * 1_deg;

				std::cout << imuAngle.Convert(degree) << std::endl;

				this->setAngle(imuAngle);
			}
		}

		/**
		 * @brief Reset all the values
		 *
		 */
		void reset() {
			if (pros::c::registry_get_plugged_type(17) == pros::c::v5_device_e_t::E_DEVICE_IMU) {
				this->imu.reset();
				this->setAngle(0.0);
			}
		}

		~IMU() {}
	};
} // namespace Pronounce
