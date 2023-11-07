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

#define DRIVE_DEADBAND 0.1f
#define DRIVE_SLEW 0.02f
#define CD_TURN_NONLINEARITY                                                   \
  0.65 // This factor determines how fast the wheel
       // traverses the "non linear" sine curve
#define CD_NEG_INERTIA_SCALAR 4.0
#define CD_SENSITIVITY 1.0

	// We apply a sinusoidal curve (twice) to the joystick input to give finer
	// control at small inputs.
	static double _turnRemapping(double iturn) {
		double denominator = sin(M_PI / 2 * CD_TURN_NONLINEARITY);
		double firstRemapIteration =
			sin(M_PI / 2 * CD_TURN_NONLINEARITY * iturn) / denominator;
		return sin(M_PI / 2 * CD_TURN_NONLINEARITY * firstRemapIteration) / denominator;
	}

	// On each iteration of the drive controller (where we aren't point turning) we
	// constrain the accumulators to the range [-1, 1].
	double quickStopAccumlator = 0.0;
	double negInertiaAccumlator = 0.0;
	static void _updateAccumulators() {
		if (negInertiaAccumlator > 1) {
			negInertiaAccumlator -= 1;
		}
		else if (negInertiaAccumlator < -1) {
			negInertiaAccumlator += 1;
		}
		else {
			negInertiaAccumlator = 0;
		}

		if (quickStopAccumlator > 1) {
			quickStopAccumlator -= 1;
		}
		else if (quickStopAccumlator < -1) {
			quickStopAccumlator += 1;
		}
		else {
			quickStopAccumlator = 0.0;
		}
	}

	double prevTurn = 0.0;
	double prevThrottle = 0.0;
	std::pair<double, double> cheesyDrive(double ithrottle, double iturn) {
		bool turnInPlace = false;
		double linearCmd = ithrottle;
		if (fabs(ithrottle) < DRIVE_DEADBAND && fabs(iturn) > DRIVE_DEADBAND) {
			// The controller joysticks can output values near zero when they are
			// not actually pressed. In the case of small inputs like this, we
			// override the throttle value to 0.
			linearCmd = 0.0;
			turnInPlace = true;
		}
		else if (ithrottle - prevThrottle > DRIVE_SLEW) {
			linearCmd = prevThrottle + DRIVE_SLEW;
		}
		else if (ithrottle - prevThrottle < -(DRIVE_SLEW * 2)) {
			// We double the drive slew rate for the reverse direction to get
			// faster stopping.
			linearCmd = prevThrottle - (DRIVE_SLEW * 2);
		}

		double remappedTurn = _turnRemapping(iturn);

		double left, right;
		if (turnInPlace) {
			// The remappedTurn value is squared when turning in place. This
			// provides even more fine control over small speed values.
			left = remappedTurn * std::abs(remappedTurn);
			right = -remappedTurn * std::abs(remappedTurn);

		}
		else {
			double negInertiaPower = (iturn - prevTurn) * CD_NEG_INERTIA_SCALAR;
			negInertiaAccumlator += negInertiaPower;

			double angularCmd =
				abs(linearCmd) *  // the more linear vel, the faster we turn
				(remappedTurn + negInertiaAccumlator) *
				CD_SENSITIVITY -  // we can scale down the turning amount by a
				// constant
				quickStopAccumlator;

			right = left = linearCmd;
			left += angularCmd;
			right -= angularCmd;

			_updateAccumulators();
		}

		prevTurn = iturn;
		prevThrottle = ithrottle;

		return std::make_pair(left, right);
	}

	class JoystickDrivetrain : public Behavior {
	private:
		double deadband = 0.10;
		double exponentializeValue = 1.0;
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
			return axis < deadband ? 0.0 : axis;
		}

	public:

		JoystickDrivetrain(std::string name, ContinuousOdometry& odometry, AbstractJoystick* controller, AbstractTankDrivetrain& drivetrain, double deadband, double exponentializerValue, QSpeed maxSpeed) : Behavior(name), odometry(odometry), controller(controller), drivetrain(drivetrain) {
			this->deadband = deadband;
			this->exponentializeValue = exponentializerValue;
			this->maxDriveSpeed = maxSpeed;
			this->arcade = true;
		}

		void initialize() {
			if (maxDriveSpeed == 0.0_in / second) {
				drivetrain.tankSteerVoltage(0.0, 0.0);
				drivetrainMutex.give();
				return;
			}
		}

		void update() {
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
				power = controller->get_analog(E_CONTROLLER_ANALOG_LEFT_Y) / 127.0;
				turn = controller->get_analog(E_CONTROLLER_ANALOG_RIGHT_X) / 127.0;

				std::pair<double, double> outputs = cheesyDrive(power, turn);

				double left = outputs.first;
				double right = outputs.second;
				power = (left + right) / 2.0;
				turn = (left - right) / 2.0;
			}
			else {
				double left = controller->get_analog(E_CONTROLLER_ANALOG_LEFT_Y) / 127.0;
				double right = controller->get_analog(E_CONTROLLER_ANALOG_RIGHT_Y) / 127.0;
				power = (left + right) / 2.0;
				turn = (left - right) / 2.0;
			}


			double left = power + turn;
			double right = power - turn;

			std::cout << "DriverInputLeft: " << left << std::endl;
			std::cout << "DriverInputRight: " << right << std::endl;

			drivetrain.tankSteerVoltage(left * 12000.0, right * 12000.0);

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