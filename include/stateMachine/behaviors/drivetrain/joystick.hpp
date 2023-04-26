#pragma once

#include "odometry/continuousOdometry/continuousOdometry.hpp"
#include "stateMachine/behavior.hpp"
#include "utils/utils.hpp"
#include "math.h"
#include "utils/runningAverage.hpp"
#include "chassis/abstractTankDrivetrain.hpp"
#include "hardware/hardware.hpp"
#include "hardwareAbstractions/joystick/joystick.hpp"

namespace Pronounce {
	class JoystickDrivetrain: public Behavior {
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

		AbstractJoystick* controller;

		AbstractTankDrivetrain& drivetrain;

		PID visionPid;

		RunningAverage<10> visionSensorX;

		bool arcade;

		double filterAxis(double axis) {
			return axis < deadband ? 0.0 : axis;
		}

		double visionSensorAngle;

	public:

		JoystickDrivetrain(std::string name, ContinuousOdometry& odometry, AbstractJoystick* controller, AbstractTankDrivetrain& drivetrain, PID visionPid, double deadband, bool targeting, double exponentializerValue, QSpeed maxSpeed): Behavior(name), odometry(odometry), controller(controller), drivetrain(drivetrain) {
			this->deadband = deadband;
			this->targeting = targeting;
			this->exponentializeValue = exponentializerValue;
			this->maxDriveSpeed = maxSpeed;
			this->visionPid = visionPid;
			this->arcade = false;
			this->visionSensorAngle = 0.0;
		}

		void initialize() {
			if (maxDriveSpeed == 0.0_in / second) {
				drivetrain.tankSteerVoltage(0.0, 0.0);
				drivetrainMutex.give();
				return;
			}

			this->visionSensorAngle = (aimingVisionSensor.get_by_size(0).x_middle_coord * 73_deg/314.0 + odometry.getAngle()).getValue();
			this->visionSensorAngle = false ? 0.0 : visionSensorAngle + (5_deg).getValue();
		}

		void update() {
			drivetrainMutex.take();

			if (((aimingVisionSensor.get_by_size(0).angle) * 0.1_deg + odometry.getAngle() + 3_deg).getValue() != visionSensorAngle) {
				this->visionSensorAngle = (aimingVisionSensor.get_by_size(0).x_middle_coord * 73_deg/314.0 + odometry.getAngle() + 5_deg).getValue();
			}

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
				power = controller->get_analog(E_CONTROLLER_ANALOG_LEFT_Y) / 127.0;
				turn = controller->get_analog(E_CONTROLLER_ANALOG_RIGHT_X) / 127.0;
				turn = pow(fabs(turn), 1.3) * signnum_c(turn);
			}
			else {
				double left = controller->get_analog(E_CONTROLLER_ANALOG_LEFT_Y) / 127.0;
				double right = controller->get_analog(E_CONTROLLER_ANALOG_RIGHT_Y) / 127.0;
				power = (left + right) / 2.0;
				turn = (left - right) / 2.0;
			}


			if (targeting) {
				visionPid.setTarget(visionSensorAngle);
				turn = visionPid.update(odometry.getAngle().getValue());
			} else {
				// aimingVisionSensor.clear_led();
			}

			double left = power + turn;
			double right = power - turn;

			std::cout << "DriverInputLeft: " << left << std::endl;
			std::cout << "DriverInputRight: " << right << std::endl;

			drivetrain.tankSteerVoltage(left * 12000.0, right * 12000.0);

			leftVoltage = left * 12000;
			rightVoltage = right * 12000;

			drivetrainMutex.give();
		}

		void exit() {
			// drivetrain.skidSteerVelocity(0.0, 0.0);
			if (targeting) {
				visionSensorAngle = 0.0;
			}
		}

		bool isDone() {
			return false;
		}

		~JoystickDrivetrain() {}
	};
} // namespace Pronounce
