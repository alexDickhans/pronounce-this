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
class IMU : public Orientation, public pros::Imu {
	private:
		/**
		 * @brief Reference to the imu
		 *
		 */
	public:
		/**
		 * @brief Construct a new IMU object with a reference to the imu
		 *
		 * @param imu
		 */
		IMU(const std::uint8_t port) : pros::Imu(port), Orientation(0.0) {}

		/**
		 * @brief Update the imu
		 *
		 */
		void update() {
			if (pros::c::registry_get_plugged_type(_port - 1) == pros::c::v5_device_e_t::E_DEVICE_IMU) {
				Angle imuAngle = (isfinite(this->get_rotation()) ? this->get_rotation() : 0.0) * 1_deg;
				// std::cout << imuAngle.getValue() << std::endl;

				this->setAngle(imuAngle);
			}
		}

		/**
		 * @brief Reset all the values
		 *
		 */
		void reset() override {
			// if this is broken change the imu api to make _port public
			if (pros::c::registry_get_plugged_type(_port - 1) == pros::c::v5_device_e_t::E_DEVICE_IMU) {
				this->reset();
				this->setAngle(0.0);
			}
		}

		~IMU() {}
	};
} // namespace Pronounce
