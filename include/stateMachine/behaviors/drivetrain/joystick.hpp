#pragma once

#include "odometry/continuousOdometry/continuousOdometry.hpp"
#include "stateMachine/behavior.hpp"
#include "utils/utils.hpp"
#include <cmath>
#include "utils/runningAverage.hpp"
#include "chassis/abstractTankDrivetrain.hpp"
#include "hardware/hardware.hpp"
#include "hardwareAbstractions/joystick/joystick.hpp"

namespace Pronounce {

	class JoystickDrivetrain : public Behavior {
	private:
		double deadband = 0.02;
		QSpeed maxDriveSpeed;

		/**
		 * @brief Used for field oriented and targeting control
		 *
		 */
		ContinuousOdometry& odometry;

		AbstractJoystick* controller;

		AbstractTankDrivetrain& drivetrain;

		bool arcade;

		double filterAxis(double axis) {
			return abs(axis) < deadband ? 0.0 : axis;
		}

	public:

		JoystickDrivetrain(std::string name, ContinuousOdometry& odometry, AbstractJoystick* controller, AbstractTankDrivetrain& drivetrain, double deadband, QSpeed maxSpeed) : Behavior(name), odometry(odometry), controller(controller), drivetrain(drivetrain) {
			this->deadband = deadband;
			this->maxDriveSpeed = maxSpeed;
			this->arcade = false;
		}

		void initialize() override {
			if (maxDriveSpeed == 0.0_in / second) {
				drivetrain.tankSteerVoltage(0.0, 0.0);
				drivetrainMutex.give();
				return;
			}
		}

		void update() override {
			drivetrainMutex.take();

			if (maxDriveSpeed == 0.0_in / second) {
				// drivetrain.tankSteerVoltage(0.0, 0.0);
				drivetrainMutex.give();
				return;
			}

			double power;
			double turn;

			if (controller->get_digital_new_press(E_CONTROLLER_DIGITAL_X)) {
				arcade = !arcade;
			}

			if (arcade) {
				power = filterAxis(controller->get_analog(E_CONTROLLER_ANALOG_LEFT_Y) / 127.0);
				turn = filterAxis(controller->get_analog(E_CONTROLLER_ANALOG_RIGHT_X) / 127.0);
			}
			else {
				double left = filterAxis(controller->get_analog(E_CONTROLLER_ANALOG_LEFT_Y) / 127.0);
				double right = filterAxis(controller->get_analog(E_CONTROLLER_ANALOG_RIGHT_Y) / 127.0);
				power = (left + right) / 2.0;
				turn = (left - right) / 2.0;
			}

			double left = power + turn;
			double right = power - turn;

			std::cout << "DriverInputLeft: " << left << std::endl;
			std::cout << "DriverInputRight: " << right << std::endl;

			drivetrain.tankSteerVoltage(left * 11040.0 + signnum_c(left) * 960.0, right * 11040.0 + signnum_c(right) * 960.0);

			drivetrainMutex.give();
		}

		void exit() {
			drivetrain.skidSteerVelocity(0.0, 0.0);
		}

		bool isDone() {
			return false;
		}

		~JoystickDrivetrain() = default;
	};
} // namespace Pronounce