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

		PID visionPid;

		RunningAverage<20> visionSensorX;

		double filterAxis(double axis) {
			return axis < deadband ? 0.0 : axis;
		}

	public:
		JoystickDrivetrain(std::string name, ContinuousOdometry& odometry, pros::Controller& controller, AbstractTankDrivetrain& drivetrain, PID visionPid, double deadband, bool targeting, double exponentializerValue, QSpeed maxSpeed) : Behavior(name), odometry(odometry), controller(controller), drivetrain(drivetrain) {
			this->deadband = deadband;
			this->targeting = targeting;
			this->exponentializeValue = exponentializerValue;
			this->maxDriveSpeed = maxSpeed;
			this->visionPid = visionPid;

		}

		void initialize() {}

		void update() {
			drivetrainMutex.take();

			if (maxDriveSpeed == 0.0_in/second) {
				drivetrain.skidSteerVelocity(0.0, 0.0);
				drivetrainMutex.give();
				return;
			}

			double left = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 127.0;
			double right = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) / 127.0;

			double power = (left + right) / 2.0; //  controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 127.0;
			double turn = (left-right)/2.0; //  controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 127.0;
			
			if (targeting && aimingVisionSensor.get_object_count() >= 1) {
				visionSensorX.add(-(aimingVisionSensor.get_by_size(0).x_middle_coord));
				turn = visionPid.update(visionSensorX.getAverage());
			}

			left = power + turn;
			right = power - turn;

			std::cout << "DriverInputLeft: " << left << std::endl;
			std::cout << "DriverInputRight: " << right << std::endl;

			drivetrain.tankSteerVoltage(left*12000, right*12000);

			leftVoltage = left*12000;
			rightVoltage = right*12000;

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
