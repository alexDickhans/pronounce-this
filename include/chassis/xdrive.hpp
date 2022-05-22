#pragma once

#include "api.h"
#include "drivetrain.hpp"
#include "abstractHolonomicDrivetrain.hpp"
#include "utils/vector.hpp"

namespace Pronounce {
	class XDrive : public Drivetrain, AbstractHolonomicDrivetrain {
	private:

		pros::Motor* frontLeftMotor;
		pros::Motor* frontRightMotor;
		pros::Motor* backLeftMotor;
		pros::Motor* backRightMotor;

		double trackWidth;
		double wheelAngle;

	public:
		XDrive(pros::Motor* frontLeft,
			pros::Motor* frontRight,
			pros::Motor* backLeft,
			pros::Motor* backRight) {
			frontLeftMotor = frontLeft;
			frontRightMotor = frontRight;
			backLeftMotor = backLeft;
			backRightMotor = backRight;

			trackWidth = 15;
			wheelAngle = M_PI_4;
		}

		XDrive(pros::Motor* frontLeft, pros::Motor* frontRight, pros::Motor* backLeft, pros::Motor* backRight, double trackWidth, double wheelAngle) {
			frontLeftMotor = frontLeft;
			frontRightMotor = frontRight;
			backLeftMotor = backLeft;
			backRightMotor = backRight;

			this->trackWidth = trackWidth;
			this->wheelAngle = wheelAngle;

		}

		void setDriveVectorVelocity(Vector vector, double rotation) {
			this->frontLeftMotor->move_velocity(vector.getCartesian().getY() + vector.getCartesian().getX() + rotation);
			this->frontRightMotor->move_velocity(vector.getCartesian().getY() - vector.getCartesian().getX() - rotation);
			this->backLeftMotor->move_velocity(vector.getCartesian().getY() - vector.getCartesian().getX() + rotation);
			this->backRightMotor->move_velocity(vector.getCartesian().getY() + vector.getCartesian().getX() - rotation);
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
			double total = this->frontLeftMotor->get_temperature() +
				this->frontRightMotor->get_temperature() +
				this->backLeftMotor->get_temperature() +
				this->backRightMotor->get_temperature();
			return total / 4;
		}

	};
}