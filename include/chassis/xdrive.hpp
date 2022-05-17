#pragma once

#include "api.h"
#include "drivetrain.hpp"
#include "utils/vector.hpp"

namespace Pronounce {	
class XDrive {
	private:

		pros::Motor* frontRight;
		pros::Motor* frontLeft;
		pros::Motor* backRight;
		pros::Motor* backLeft;

		double trackWidth;
		double wheelAngle;

	public:

		XDrive(pros::Motor* frontRight,
			pros::Motor* frontLeft,
			pros::Motor* backRight,
			pros::Motor* backLeft) {

			this->frontRight = frontRight;
			this->frontLeft = frontLeft;
			this->backRight = backRight;
			this->backLeft = backLeft;
		}

		void setDriveVectorVelocity(Vector vector, double rotation) {
			this->frontLeft->move_velocity(vector.getCartesian().getY() + vector.getCartesian().getX() + rotation);
			this->frontRight->move_velocity(vector.getCartesian().getY() - vector.getCartesian().getX() - rotation);
			this->backLeft->move_velocity(vector.getCartesian().getY() - vector.getCartesian().getX() + rotation);
			this->backRight->move_velocity(vector.getCartesian().getY() + vector.getCartesian().getX() - rotation);
		}

		void setDriveVectorVelocity(Vector vector) {
			this->setDriveVectorVelocity(vector, 0);	
		}

		/**
		 * Get the average drivetrain temperature
		 *
		 * @return Average of the 4 wheels
		 */
		double getAvgTemp() {
			double total = this->frontLeft->get_temperature() +
				this->frontRight->get_temperature() +
				this->backLeft->get_temperature() +
				this->backRight->get_temperature();
			return total / 4;
		}

	};
}