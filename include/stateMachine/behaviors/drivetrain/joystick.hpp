#pragma once

#include "odometry/continuousOdometry/continuousOdometry.hpp"
#include "stateMachine/behavior.hpp"
#include "api.h"
#include "utils/utils.hpp"
#include "math.h"
#include "utils/runningAverage.hpp"
#include "chassis/abstractTankDrivetrain.hpp"
#include "hardware/hardware.hpp"

namespace Pronounce {
	class JoystickDrivetrain : public Behavior {
	private:
		double deadband = 0.10;
		bool targeting = false;
		double exponentializeValue = 1.0;
		QSpeed maxDriveSpeed;

		/**
		 * @brief Used for field oriented and targeting control
		 *
		 */
		ContinuousOdometry& odometry;

		pros::Controller& controller;

		AbstractTankDrivetrain& drivetrain;

		double filterAxis(double axis) {
			return axis < deadband ? 0.0 : axis;
		}

	public:
		JoystickDrivetrain(std::string name, ContinuousOdometry& odometry, pros::Controller& controller, AbstractTankDrivetrain& drivetrain, double deadband, bool targeting, double exponentializerValue, QSpeed maxSpeed) : Behavior(name), odometry(odometry), controller(controller), drivetrain(drivetrain) {
			this->deadband = deadband;
			this->targeting = targeting;
			this->exponentializeValue = exponentializerValue;
			this->maxDriveSpeed = maxDriveSpeed;
		}

		void initialize() {}

		void update() {
			drivetrainMutex.take();

			if (maxDriveSpeed == 0.0_in/second) {
				drivetrain.skidSteerVelocity(0.0, 0.0);
				drivetrainMutex.give();
				return;
			}

			double y = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 127.0;
			double turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 127.0;

			drivetrain.skidSteerVelocity(y * maxDriveSpeed, turn);

			drivetrainMutex.give();
		}

		void exit() {
			drivetrain.skidSteerVelocity(0.0, 0.0);
		}

		bool isDone() {
			return false;
		}

		~JoystickDrivetrain() {}
	};
} // namespace Pronounce
