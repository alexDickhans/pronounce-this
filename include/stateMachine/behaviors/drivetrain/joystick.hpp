#pragma once

#include "odometry/continuousOdometry/continuousOdometry.hpp"
#include "stateMachine/behavior.hpp"
#include "api.h"
#include "chassis/abstractHolonomicDrivetrain.hpp"
#include "utils/utils.hpp"
#include "math.h"
#include "utils/runningAverage.hpp"

// TODO: mode running average stuff to another place
// TODO: Change to jerk and acceleration limiting
#define RUNNING_AVERAGE_TRANSLATION 20
#define RUNNING_AVERAGE_ROTATION 5

// TODO: Add docstrings

namespace Pronounce {
	class JoystickDrivetrain : public Behavior {
	private:
		double deadband = 0.10;
		bool targeting = false;
		double exponentializeValue = 1.0;
		double outputMultiplier = 1.0;

		/**
		 * @brief Used for field oriented and targeting control
		 *
		 */
		ContinuousOdometry* odometry;

		pros::Controller* controller;

		AbstractHolonomicDrivetrain* drivetrain;

		double filterAxis(double axis) {
			return axis < deadband ? 0.0 : axis;
		}

	public:
		JoystickDrivetrain(std::string name, double deadband, bool targeting, double exponentializerValue, double outputMultiplier, ContinuousOdometry* odometry, pros::Controller* controller, AbstractHolonomicDrivetrain* drivetrain) : Behavior(name) {
			this->deadband = deadband;
			this->targeting = targeting;
			this->exponentializeValue = exponentializerValue;
			this->outputMultiplier = outputMultiplier;
			this->odometry = odometry;
			this->controller = controller;
			this->drivetrain = drivetrain;
		}

		void initialize() {}

		void update() {

			if (outputMultiplier == 0.0) {
				drivetrain->setDriveVectorVelocity(Vector(0.0, 0.0), 0.0);
				return;
			}

			movingAverageX->add(map(controller->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X), -127.0, 127.0, -1.0, 1.0));
			movingAverageY->add(map(controller->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), -127.0, 127.0, -1.0, 1.0));
			movingAverageTurn->add(map(controller->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X), -127.0, 127.0, -1.0, 1.0));

			double x = movingAverageX->getAverage();
			double y = movingAverageY->getAverage();
			double turn = movingAverageTurn->getAverage();

			Vector driveVector(new Point(x, y));
			
			if (fieldOriented) {
				Point drivePoint = driveVector.getCartesian();

				double x = drivePoint.getX().getValue() * cos(odometry->getAngle()) - drivePoint.getY().getValue() * sin(odometry->getAngle());
				double y = drivePoint.getX().getValue() * sin(odometry->getAngle()) + drivePoint.getY().getValue() * cos(odometry->getAngle());

				driveVector = Vector(Point(x, y));
			}

			// driveVector = filterVector(driveVector);

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
