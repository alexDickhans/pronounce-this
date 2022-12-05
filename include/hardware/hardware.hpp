#pragma once

#include "api.h"
#include "odometry/orientation/imu.hpp"
#include "odometry/orientation/avgOrientation.hpp"
#include "position/motorOdom.hpp"
#include "position/trackingWheel.hpp"
#include "odometry/continuousOdometry/threeWheelOdom.hpp"
#include "odometry/interruptOdometry/gpsOdometry.hpp"
#include "odometry/odomFuser.hpp"
#include "pros/rtos.hpp"
#include "pronounceLedStrip/ledStrip.hpp"
#include "hardwareAbstractions/joystick/joystick.hpp"
#include "pros/apix.h"

#ifndef SIM
#include "hardwareAbstractions/joystick/robotJoystick.hpp"
#else
#include "hardwareAbstractions/joystick/simJoystick.hpp"
#endif // !SIM

namespace Pronounce {

	pros::Mutex controllerMutex;

#ifndef SIM
	AbstractJoystick* master = new RobotJoystick(controller_id_e_t::E_CONTROLLER_MASTER);
#else
	AbstractJoystick* master = new SimJoystick(controller_id_e_t::E_CONTROLLER_MASTER);
#endif // !SIM

	pros::Mutex drivetrainMutex;

	pros::Motor leftDrive1(9, pros::E_MOTOR_GEAR_600, true);
	pros::Motor leftDrive2(8, pros::E_MOTOR_GEAR_600, true);
	pros::Motor leftDrive3(7, pros::E_MOTOR_GEAR_600, false);
	pros::Motor rightDrive1(2, pros::E_MOTOR_GEAR_600, false);
	pros::Motor rightDrive2(3, pros::E_MOTOR_GEAR_600, false);
	pros::Motor rightDrive3(4, pros::E_MOTOR_GEAR_600, true);

	pros::Motor_Group leftDriveMotors({ leftDrive1, leftDrive2, leftDrive3 });
	pros::Motor_Group rightDriveMotors({ rightDrive1, rightDrive2, rightDrive3 });

	TankDrivetrain drivetrain(7.5_in, 61_in / second, leftDriveMotors, rightDriveMotors, 600);

	pros::Mutex ptoMutex;

	pros::Motor leftPtoMotor(16, pros::E_MOTOR_GEAR_600, true);
	pros::Motor rightPtoMotor(15, pros::E_MOTOR_GEAR_600, false);

	int32_t leftVoltage = 0;
	int32_t rightVoltage = 0;

	// Catapult
	pros::Rotation catapultLimitSwitch(14);

	MotorOdom leftDrive1Odom(std::make_shared<pros::Motor>(leftDrive2), 3.25_in);
	MotorOdom rightDrive1Odom(std::make_shared<pros::Motor>(rightDrive2), 3.25_in);

	// Inertial Measurement Unit
	pros::Imu imu(18);
	IMU imuOrientation(imu);

	pros::Mutex odometryMutex;

	ThreeWheelOdom threeWheelOdom(&leftDrive1Odom, &rightDrive1Odom, new OdomWheel(), &imuOrientation);

	// GPS sensor
	pros::Gps gps(4, 0, 0, 90, 0.2, 0.2);
	GpsOdometry gpsOdometry(gps, 7.5_in, 7.5_in, 180_deg);

	OdomFuser odometry(threeWheelOdom);

	pros::Mutex endgameMutex;

	pros::ADIDigitalOut pistonBoost('b', false);
	pros::ADIDigitalOut pistonOverfill('a', false);

	pros::ADIAnalogIn catapultLineSensor('b');
	pros::ADIAnalogIn intakeLineSensor('c');
	pros::ADIDigitalOut endgameDigitalOutputs('d', false);

	pros::ADILed leftLeds({ 20, 'a' }, 20);
	pros::ADILed rightLeds({ 20, 'b' }, 20);

	PronounceLedLib::AnimationColors blueColors = { 0x0000008B, 0x00004B4B };
	PronounceLedLib::AnimationColors redColors = { 0x008C2427, 0x008C142C };
	PronounceLedLib::AnimationColors orangeColors = { 0x00804000, 0x00854300 };
	PronounceLedLib::AnimationColors greenColors = { 0x00008000, 0x00308020 };
	PronounceLedLib::AnimationColors whiteColors = { 0x00808080, 0x00707070 };

	PronounceLedLib::LedStripController leftLedController(leftLeds, orangeColors, 0.7);
	PronounceLedLib::LedStripController rightLedController(rightLeds, orangeColors, 0.7);

	pros::Vision aimingVisionSensor(17, pros::E_VISION_ZERO_CENTER);

	pros::vision_signature_s_t RED_GOAL;
	pros::vision_signature_s_t BLUE_GOAL;

	void initHardware() {

		drivetrainMutex.take();

		leftDriveMotors.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
		rightDriveMotors.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
		leftPtoMotor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		rightPtoMotor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

		drivetrainMutex.give();

		odometryMutex.take();

		double turningFactor = 360.0 / 600.0;
		double tuningFactor = 1.0;
		leftDrive1Odom.setRadius(3.25_in / 2.0);
		leftDrive1Odom.setTuningFactor(turningFactor);
		rightDrive1Odom.setRadius(3.25_in / 2.0);
		rightDrive1Odom.setTuningFactor(turningFactor);

		threeWheelOdom.setLeftOffset(10_in / 1.5);
		threeWheelOdom.setRightOffset(10.0_in / 1.5);
		threeWheelOdom.setBackOffset(0.0_in);

		pros::Task::delay(50);

		threeWheelOdom.reset(Pose2D(0.0_in, 0.0_in, 0.0_deg));

		imu.reset();

		while (imu.is_calibrating() && !(pros::c::registry_get_plugged_type(18) == pros::c::v5_device_e_t::E_DEVICE_NONE))
			pros::Task::delay(50);

		odometryMutex.give();

		RED_GOAL = aimingVisionSensor.signature_from_utility(1, 6167, 8375, 7270, -977, 77, -450, 3.000, 0);
		BLUE_GOAL = aimingVisionSensor.signature_from_utility(2, -1317, -595, -956, 3619, 5717, 4668, 3.000, 0);

		aimingVisionSensor.set_signature(1, &RED_GOAL);
		aimingVisionSensor.set_signature(2, &BLUE_GOAL);
		aimingVisionSensor.set_exposure(116);

		// catapultLimitSwitch.reverse();
	}
} // namespace Pronoucne
