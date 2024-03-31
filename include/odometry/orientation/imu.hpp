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
		IMU(pros::Imu &imu) : Orientation(0.0), imu(imu) {}

		/**
		 * @brief Update the imu
		 *
		 */
		void update() {
			if (imu.is_installed()) {
				Angle imuAngle = (isfinite(imu.get_rotation()) ? imu.get_rotation() : 0.0) * 1_deg;
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
			if (imu.is_installed()) {
				this->reset();
				this->setAngle(0.0);
			}
		}

		void setRotation(Angle rotation) {
			imu.set_rotation(rotation.Convert(degree));
		}

		~IMU() {}
	};
} // namespace Pronounce
