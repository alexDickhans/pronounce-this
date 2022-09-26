#pragma once

#include "joystick.hpp"
#include "api.h"
#include "utils/motorGroup.hpp"
#include "stateMachine/wait.hpp"
#include "stateMachine/stateController.hpp"
#include "stateMachine/behavior.hpp"
#include "odometry/continuousOdometry/continuousOdometry.hpp"
#include "position/trackingWheel.hpp"
#include "chassis/xdrive.hpp"
#include "driver/controller.hpp"
#include "utils/exponentialMovingAverage.hpp"
#include "utils/runningAverage.hpp"
#include "odometry/orientation/imu.hpp"
#include "odometry/odomFuser.hpp"
#include "motionControl/omniMotionProfiling.hpp"
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
#include "utils/path.hpp"
#include "utils/quadraticSplinePath.hpp"
#include "motionControl/rotationController.hpp"
#include "motionControl/omniMovement.hpp"

// TODO: Clean up
// TODO: move declarations to another place
// TODO: Add comments

namespace Pronounce {

	// Controllers
	Pronounce::Controller master(pros::E_CONTROLLER_MASTER);
	Pronounce::Controller partner(pros::E_CONTROLLER_PARTNER);

	// Drive Motors
	pros::Motor frontLeftMotor(1, pros::E_MOTOR_GEARSET_18, false);
	pros::Motor frontRightMotor(10, pros::E_MOTOR_GEARSET_18, true);
	pros::Motor backLeftMotor(2, pros::E_MOTOR_GEARSET_18, false);
	pros::Motor backRightMotor(9, pros::E_MOTOR_GEARSET_18, true);

	// Inertial Measurement Unit
	pros::Imu imu(19);
	IMU imuOrientation(imu);

	AvgOrientation averageImu;

	pros::Rotation leftEncoder(6);
	pros::Rotation rightEncoder(8);
	pros::Rotation backEncoder(7);

	// Odom wheels
	Pronounce::TrackingWheel leftOdomWheel(&leftEncoder);
	Pronounce::TrackingWheel rightOdomWheel(&rightEncoder);
	Pronounce::TrackingWheel backOdomWheel(&backEncoder);

	// GPS sensor
	pros::Gps gps(4, 0, 0, 90, 0.2, 0.2);
	GpsOdometry gpsOdometry(gps, 7.5_in, 7.5_in, 180_deg);

	ThreeWheelOdom threeWheelOdom(&leftOdomWheel, &rightOdomWheel, &backOdomWheel, &averageImu);

	XDrive drivetrain(&frontLeftMotor, &frontRightMotor, &backLeftMotor, &backRightMotor);

	RunningAverage<RUNNING_AVERAGE_TRANSLATION> movingAverageX;
	RunningAverage<RUNNING_AVERAGE_TRANSLATION> movingAverageY;
	RunningAverage<RUNNING_AVERAGE_ROTATION> movingAverageTurn;

	OdomFuser odometry(threeWheelOdom);
	
	JoystickDrivetrain fieldRelativeJoystick("FieldRelativeNormal", 0.10, true, false, 2.4, 200.0, &movingAverageX, &movingAverageY, &movingAverageTurn, &odometry, &master, &drivetrain);
	JoystickDrivetrain fieldRelativeTargetingJoystick("FieldRelativeTargetting", 0.10, true, true, 2.4, 200.0, &movingAverageX, &movingAverageY, &movingAverageTurn, &odometry, &master, &drivetrain);
	JoystickDrivetrain normalJoystick("RobotRelativeJoystick", 0.10, false, false, 2.4, 200.0, &movingAverageX, &movingAverageY, &movingAverageTurn, &odometry, &master, &drivetrain);
	JoystickDrivetrain normalTargetingJoystick("RobotRelativeTargeting", 0.10, false, true, 2.4, 200.0, &movingAverageX, &movingAverageY, &movingAverageTurn, &odometry, &master, &drivetrain);

	JoystickDrivetrain drivetrainStopped("DrivetrainStopped", 0.10, false, true, 2.4, 0.0, &movingAverageX, &movingAverageY, &movingAverageTurn, &odometry, &master, &drivetrain);

	ProfileConstraints defaultProfileConstraints;
	ProfileConstraints stackIntakeProfileConstraints;
	ProfileConstraints rowIntakeProfileConstraints;

	SinusoidalVelocityProfile testVelocityProfile(24_in, defaultProfileConstraints);
	SinusoidalVelocityProfile frontRollerToAllianceStackVelocityProfile(24_in, defaultProfileConstraints);
	SinusoidalVelocityProfile intakeAllianceStackVelocityProfile(24_in, stackIntakeProfileConstraints);
	SinusoidalVelocityProfile allianceStackToAllianceDiscsVelocityProfile(24_in, rowIntakeProfileConstraints);
	SinusoidalVelocityProfile allianceDiscsToRollerVelocityProfile(24_in, defaultProfileConstraints);
	
	PID turningPid(380, 0, 2000);
	
	OmniMotionProfiling testProfile("MotionProfilingTest", testVelocityProfile, drivetrain, &turningPid, &odometry, 0.0_deg, 0_deg);
	OmniMotionProfiling frontRollerToAllianceStack("FrontRollerToAllianceStack", frontRollerToAllianceStackVelocityProfile, drivetrain, &turningPid, &odometry, 45.0_deg, 0_deg);
	OmniMotionProfiling intakeAllianceStack("IntakeAllianceStack", intakeAllianceStackVelocityProfile, drivetrain, &turningPid, &odometry, 0.0_deg, -45_deg);
	OmniMotionProfiling allianceStackToAllianceDiscs("AllianceStackToAllianceDiscs", allianceStackToAllianceDiscsVelocityProfile, drivetrain, &turningPid, &odometry, 0.0_deg, -45_deg);
	OmniMotionProfiling allianceDiscsToRoller("AllianceDiscsToRoller", allianceDiscsToRollerVelocityProfile, drivetrain, &turningPid, &odometry, -45.0_deg, 180_deg);

	// BezierPath testPath;
	// PurePursuitProfile defaultPurePursuitProfile = {10.0_in, testVelocityProfile};
	// OmniPurePursuit testPurePursuit(&drivetrain, &odometry, defaultPurePursuitProfile, testPath);
	RotationController turnTo90("TurnTo90", &drivetrain, &odometry, &turningPid, 90_deg);
	Wait turnTo905s(&turnTo90, 3000);
	RotationController turnTo315("TurnTo315", &drivetrain, &odometry, &turningPid, 315_deg);
	Wait turnTo905s(&turnTo315, 1500);
	RotationController turnTo180("TurnTo180", &drivetrain, &odometry, &turningPid, 180_deg);
	Wait turnTo905s(&turnTo180, 1500);
	OmniMovement drivetrainRoller("DrivetrainRoller", &drivetrain, 50, -90_deg);
	Wait drivetrainRollerWait(&drivetrainRoller, 2000);

	StateController drivetrainStateController("DrivetrainStateController", &drivetrainStopped);

	void initDrivetrain() {

		defaultProfileConstraints.maxVelocity = 34_in/1_s;
		defaultProfileConstraints.maxAcceleration = 60 * inchs2;
		defaultProfileConstraints.maxJerk = 3.0;

		stackIntakeProfileConstraints.maxVelocity = 10_in/1_s;
		stackIntakeProfileConstraints.maxAcceleration = 60 * inchs2;
		stackIntakeProfileConstraints.maxJerk = 3.0;

		rowIntakeProfileConstraints.maxVelocity = 20_in/1_s;
		rowIntakeProfileConstraints.maxAcceleration = 60 * inchs2;
		rowIntakeProfileConstraints.maxJerk = 3.0;

		testVelocityProfile.setProfileConstraints(defaultProfileConstraints);
		frontRollerToAllianceStackVelocityProfile.setProfileConstraints(defaultProfileConstraints);
		intakeAllianceStackVelocityProfile.setProfileConstraints(stackIntakeProfileConstraints);
		allianceStackToAllianceDiscsVelocityProfile.setProfileConstraints(rowIntakeProfileConstraints);
		allianceDiscsToRollerVelocityProfile.setProfileConstraints(defaultProfileConstraints);

		testVelocityProfile.calculate(100);
		frontRollerToAllianceStackVelocityProfile.calculate(100);
		intakeAllianceStackVelocityProfile.calculate(100);
		allianceStackToAllianceDiscsVelocityProfile.calculate(100);
		allianceDiscsToRollerVelocityProfile.calculate(100);

		testProfile.setVelocityProfile(testVelocityProfile);
		frontRollerToAllianceStack.setVelocityProfile(frontRollerToAllianceStackVelocityProfile);
		intakeAllianceStack.setVelocityProfile(intakeAllianceStackVelocityProfile);
		allianceStackToAllianceDiscs.setVelocityProfile(allianceStackToAllianceDiscsVelocityProfile);
		allianceDiscsToRoller.setVelocityProfile(allianceDiscsToRollerVelocityProfile);

		// Motor brake modes
		frontLeftMotor.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
		frontRightMotor.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
		backLeftMotor.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
		backRightMotor.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);

		averageImu.addOrientation(&imuOrientation);

		double turningFactor = (((100.0 / 100.0) - 1.0) / 2);
		double tuningFactor = 1.0;
		leftOdomWheel.setRadius(2.75_in / 2.0);
		leftOdomWheel.setTuningFactor(tuningFactor * (1 - turningFactor));
		rightOdomWheel.setRadius(2.75_in / 2.0);
		rightOdomWheel.setTuningFactor(tuningFactor * (1 + turningFactor));
		backOdomWheel.setRadius(2.75_in / 2.0);
		backOdomWheel.setTuningFactor(tuningFactor * 1.0);

		leftEncoder.set_reversed(true);
		rightEncoder.set_reversed(false);
		backEncoder.set_reversed(false);

		threeWheelOdom.setLeftOffset(11.75_in/2.0);
		threeWheelOdom.setRightOffset(11.75_in/2.0);
		threeWheelOdom.setBackOffset(0.0_in);

		pros::Task::delay(10);

		threeWheelOdom.reset(new Pose2D(18.0_in, 122.5_in - 24_in, 0.0));
	}

} // namespace Pronounce
