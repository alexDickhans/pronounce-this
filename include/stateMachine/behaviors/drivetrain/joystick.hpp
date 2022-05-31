#pragma once

#include "odometry/odometry.hpp"
#include "stateMachine/behavior.hpp"
#include "api.h"
#include "chassis/abstractHolonomicDrivetrain.hpp"
#include "utils/utils.hpp"
#include "math.h"

namespace Pronounce {
	class JoystickDrivetrain : public Behavior {
	private:
		double deadband = 0.10;
		bool fieldOriented = false;
		bool targeting = false;
		double exponentializeValue = 1.0;
		double outputMultiplier = 1.0;

		/**
		 * @brief Used for field oriented and targeting control
		 *
		 */
		Odometry* odometry;

		pros::Controller* controller;

		AbstractHolonomicDrivetrain* drivetrain;

		Vector filterVector(Vector vector) {
			vector.setMagnitude(vector.getMagnitude() < deadband ? 0.0 : vector.getMagnitude());
			return vector;
		}

		double filterAxis(double axis) {
			return axis < deadband ? 0.0 : axis;
		}

		Vector exponentialize(Vector vector) {
			vector.setMagnitude(pow(vector.getMagnitude(), exponentializeValue));
			return vector;
		}

	public:
		JoystickDrivetrain(double deadband, bool fieldOriented, bool targeting, double exponentializerValue, double outputMultiplier, Odometry* odometry, pros::Controller* controller, AbstractHolonomicDrivetrain* drivetrain) {
			this->deadband = deadband;
			this->fieldOriented = fieldOriented;
			this->targeting = targeting;
			this->exponentializeValue = exponentializerValue;
			this->outputMultiplier = outputMultiplier;
			this->odometry = odometry;
			this->controller = controller;
			this->drivetrain = drivetrain;
		}

		void initialize() {}

		void update() {
			std::cout << "drivetrain" << std::endl;

			if (outputMultiplier == 0.0) {
				drivetrain->setDriveVectorVelocity(Vector(0.0, 0.0), 0.0);
				return;
			}

			double x = map(controller->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X), -127.0, 127.0, -1.0, 1.0);
			double y = map(controller->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), -127.0, 127.0, -1.0, 1.0);
			double turn = map(controller->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X), -127.0, 127.0, -1.0, 1.0);

			Vector driveVector(new Point(x, y));
			driveVector = filterVector(driveVector);

			if (fieldOriented) {
				driveVector.rotate(-odometry->getTheta());
			}

			driveVector = driveVector.scale(outputMultiplier);

			drivetrain->setDriveVectorVelocity(driveVector, turn * outputMultiplier);
		}

		void exit() {
			drivetrain->setDriveVectorVelocity(Vector(0.0, 0.0));
		}

		bool isDone() {
			return false;
		}

		~JoystickDrivetrain() {}
	};
} // namespace Pronounce
