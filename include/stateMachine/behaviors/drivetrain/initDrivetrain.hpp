#pragma once

#include "joystick.hpp"
#include "api.h"
#include "utils/motorGroup.hpp"
#include "stateMachine/wait.hpp"
#include "stateMachine/stateController.hpp"
#include "stateMachine/behavior.hpp"
#include "odometry/threeWheelOdom.hpp"
#include "position/trackingWheel.hpp"
#include "odometry/gpsOdometry.hpp"
#include "chassis/xdrive.hpp"
#include "driver/controller.hpp"

namespace Pronounce {

	// Controllers
	Pronounce::Controller master(pros::E_CONTROLLER_MASTER);
	Pronounce::Controller partner(pros::E_CONTROLLER_PARTNER);

	// Drive Motors
	pros::Motor frontLeftMotor(5, pros::E_MOTOR_GEARSET_18, false);
	pros::Motor frontRightMotor(6, pros::E_MOTOR_GEARSET_18, true);
	pros::Motor backLeftMotor(7, pros::E_MOTOR_GEARSET_18, false);
	pros::Motor backRightMotor(8, pros::E_MOTOR_GEARSET_18, true);

	// Inertial Measurement Unit
	pros::Imu imu(19);

	pros::Rotation leftEncoder(9);
	pros::Rotation rightEncoder(10);
	pros::Rotation backEncoder(11);

	// Odom wheels
	Pronounce::TrackingWheel leftOdomWheel(&leftEncoder);
	Pronounce::TrackingWheel rightOdomWheel(&rightEncoder);
	Pronounce::TrackingWheel backOdomWheel(&backEncoder);

	// GPS sensor
	pros::Gps gps(4, 0, 0, 90, 0.2, 0.2);
	GpsOdometry gpsOdometry(&gps);

	ThreeWheelOdom odometry(&leftOdomWheel, &rightOdomWheel, &backOdomWheel, &imu);

	XDrive drivetrain(&frontLeftMotor, &frontRightMotor, &backLeftMotor, &backRightMotor);

	JoystickDrivetrain fieldRelativeJoystick(0.10, true, false, 2.4, 200.0, &odometry, &master, &drivetrain);
	JoystickDrivetrain fieldRelativeTargetingJoystick(0.10, true, true, 2.4, 200.0, &odometry, &master, &drivetrain);
	JoystickDrivetrain normalJoystick(0.10, false, false, 2.4, 200.0, &odometry, &master, &drivetrain);
	JoystickDrivetrain normalTargetingJoystick(0.10, false, true, 2.4, 200.0, &odometry, &master, &drivetrain);

	JoystickDrivetrain drivetrainStopped(0.10, false, true, 2.4, 0.0, &odometry, &master, &drivetrain);

	StateController drivetrainStateController(&drivetrainStopped);

	void initDrivetrain() {

		// Motor brake modes
		frontLeftMotor.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
		frontRightMotor.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
		backLeftMotor.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
		backRightMotor.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);

		// odometry.setUseImu(false);
		// Left/Right fraction
		// 1.072124756
		// Left 99.57
		// Right 100.57
		double turningFactor = (((100.35 / 100.0) - 1.0) / 2);
		double tuningFactor = 0.998791278;
		leftOdomWheel.setRadius(2.75 / 2);
		leftOdomWheel.setTuningFactor(tuningFactor * (1 - turningFactor));
		rightOdomWheel.setRadius(2.75 / 2);
		rightOdomWheel.setTuningFactor(tuningFactor * (1 + turningFactor));
		backOdomWheel.setRadius(2.75 / 2);
		backOdomWheel.setTuningFactor(tuningFactor * 1.0);

		leftEncoder.set_reversed(true);
		rightEncoder.set_reversed(false);

		odometry.setLeftOffset(8.75/2);
		odometry.setRightOffset(8.75/2);
		odometry.setBackOffset(-3);

		odometry.setMaxMovement(0);

		pros::Task::delay(10);

		odometry.reset(new Position());
	}

} // namespace Pronounce
