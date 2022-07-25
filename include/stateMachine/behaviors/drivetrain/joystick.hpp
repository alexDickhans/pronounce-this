#pragma once

#include "odometry/continuousOdometry/continuousOdometry.hpp"
#include "stateMachine/behavior.hpp"
#include "api.h"
#include "chassis/abstractHolonomicDrivetrain.hpp"
#include "utils/utils.hpp"
#include "math.h"
#include "utils/runningAverage.hpp"

#define RUNNING_AVERAGE_TRANSLATION 20
#define RUNNING_AVERAGE_ROTATION 5

namespace Pronounce {
	class JoystickDrivetrain : public Behavior {
	private:
		double deadband = 0.10;
		bool fieldOriented = false;
		bool targeting = false;
		double exponentializeValue = 1.0;
		double outputMultiplier = 1.0;

		RunningAverage<RUNNING_AVERAGE_TRANSLATION>* movingAverageX;
		RunningAverage<RUNNING_AVERAGE_TRANSLATION>* movingAverageY;
		RunningAverage<RUNNING_AVERAGE_ROTATION>* movingAverageTurn;

		/**
		 * @brief Used for field oriented and targeting control
		 *
		 */
		ContinuousOdometry* odometry;

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
		JoystickDrivetrain(double deadband, bool fieldOriented, bool targeting, double exponentializerValue, double outputMultiplier, RunningAverage<RUNNING_AVERAGE_TRANSLATION>* movingAverageX, RunningAverage<RUNNING_AVERAGE_TRANSLATION>* movingAverageY, RunningAverage<RUNNING_AVERAGE_ROTATION>* movingAverageTurn, ContinuousOdometry* odometry, pros::Controller* controller, AbstractHolonomicDrivetrain* drivetrain) {
			this->deadband = deadband;
			this->fieldOriented = fieldOriented;
			this->targeting = targeting;
			this->exponentializeValue = exponentializerValue;
			this->outputMultiplier = outputMultiplier;
			this->odometry = odometry;
			this->controller = controller;
			this->drivetrain = drivetrain;
			this->movingAverageX = movingAverageX;
			this->movingAverageY = movingAverageY;
			this->movingAverageTurn = movingAverageTurn;
		}

		void initialize() {}

		void update() {
			std::cout << "drivetrain" << std::endl;

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
			driveVector = filterVector(driveVector);

			if (fieldOriented) {
				driveVector.rotate(odometry->getTheta());
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
