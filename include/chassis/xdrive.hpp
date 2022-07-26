#pragma once

#include "api.h"
#include "drivetrain.hpp"
#include "abstractHolonomicDrivetrain.hpp"
#include "utils/vector.hpp"

namespace Pronounce {
	class XDrive : public Drivetrain, public AbstractHolonomicDrivetrain {
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
			double frontLeftVelocity = (vector.getCartesian().getY() + vector.getCartesian().getX()).getValue() + rotation;
			double frontRightVelocity = (vector.getCartesian().getY() - vector.getCartesian().getX()).getValue() - rotation;
			double backLeftVelocity = (vector.getCartesian().getY() - vector.getCartesian().getX()).getValue() + rotation;
			double backRightVelocity = (vector.getCartesian().getY() + vector.getCartesian().getX()).getValue() - rotation;

			double maxVal = abs(frontLeftVelocity);
			maxVal = maxVal > abs(frontRightVelocity) ? maxVal : abs(frontRightVelocity);
			maxVal = maxVal > abs(backLeftVelocity) ? maxVal : abs(backLeftVelocity);
			maxVal = maxVal > abs(backRightVelocity) ? maxVal : abs(backRightVelocity);

			if (maxVal > 200.0) {
				double adjustment = maxVal/200.0;

				frontLeftVelocity *= adjustment;
				frontRightVelocity *= adjustment;
				backLeftVelocity *= adjustment;
				backRightVelocity *= adjustment;
			} 

			this->frontLeftMotor->move_velocity(frontLeftVelocity);
			this->frontRightMotor->move_velocity(frontRightVelocity);
			this->backLeftMotor->move_velocity(backLeftVelocity);
			this->backRightMotor->move_velocity(backRightVelocity);
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

		~XDrive() {}

	};
}