#pragma once

#include "api.h"
#include "odometry/orientation/imu.hpp"
#include "odometry/orientation/avgOrientation.hpp"
#include "position/motorOdom.hpp"
#include "odometry/continuousOdometry/threeWheelOdom.hpp"
#include "odometry/interruptOdometry/gpsOdometry.hpp"
#include "odometry/odomFuser.hpp"
#include "pros/rtos.hpp"
#include "hardwareAbstractions/joystick/joystick.hpp"
#include "pros/apix.h"
#include <map>
#include "telemetryRadio/telemetryManager.hpp"
#include "auton.h"
#include "locolib/locolib.hpp"
#include "units/units.hpp"
#include "odometry/continuousOdometry/particleFilterOdometry.hpp"
#include "odometry/orientation/gpsOrientation.hpp"
#include "constants.hpp"

#ifndef SIM

#include "hardwareAbstractions/joystick/robotJoystick.hpp"
#include "Logger/logger.hpp"

#else
#include "hardwareAbstractions/joystick/simJoystick.hpp"
#endif // !SIM

namespace Pronounce {

	Logger* logger = nullptr;

	pros::Mutex robotMutex;

	bool isSkills = AUTON == 5;

#ifndef SIM
	RobotJoystick *master = new RobotJoystick(controller_id_e_t::E_CONTROLLER_MASTER);
#else
	AbstractJoystick* master = new SimJoystick(controller_id_e_t::E_CONTROLLER_MASTER);
#endif // !SIM

	pros::Mutex drivetrainMutex;

	pros::Motor leftDrive1(19, pros::E_MOTOR_GEAR_600, true);
	pros::Motor leftDrive2(18, pros::E_MOTOR_GEAR_600, true);
	pros::Motor leftDrive3(17, pros::E_MOTOR_GEAR_600, true);
	pros::Motor leftDrive4(16, pros::E_MOTOR_GEAR_600, true);
	pros::Motor rightDrive1(8, pros::E_MOTOR_GEAR_600, false);
	pros::Motor rightDrive2(12, pros::E_MOTOR_GEAR_600, false);
	pros::Motor rightDrive3(13, pros::E_MOTOR_GEAR_600, false);
	pros::Motor rightDrive4(14, pros::E_MOTOR_GEAR_600, false);

	pros::Motor_Group leftDriveMotors({leftDrive1, leftDrive2, leftDrive3, leftDrive4});
	pros::Motor_Group rightDriveMotors({rightDrive1, rightDrive2, rightDrive3, rightDrive4});
//	pros::Gps gps(10, -(5.25_in).Convert(metre), -(4.1_in).Convert(metre));
//	GpsOrientation gpsOrientation(gps, 90_deg);

	MotorOdom leftDrive1Odom(std::make_shared<pros::Motor>(leftDrive2), 1.625_in);
	MotorOdom rightDrive1Odom(std::make_shared<pros::Motor>(rightDrive2), 1.625_in);

	TankDrivetrain drivetrain(Constants::trackWidth, 76.57632093_in / second, &leftDriveMotors, &rightDriveMotors,
	                          600.0 * (revolution / minute));

	pros::Motor intakeMotor(20, pros::E_MOTOR_GEARSET_18, false);

	pros::Motor_Group catapultMotors({7, -4});

	pros::ADIDigitalOut leftSolenoid('A', false);
	pros::ADIDigitalOut rightSolenoid('B', false);
	pros::ADIDigitalOut hangSolenoid('C', false);
	pros::ADIDigitalOut AWPSolenoid('F', false);

	pros::Motor_Group intakeMotors({intakeMotor});
	// Inertial Measurement Unit
	pros::Imu imu(3);
	IMU imuOrientation(3);

	pros::Distance distanceSensor(10);
	pros::Distance hopperDistanceSensor(5);
	pros::Distance catapultDistance(6);

	pros::Mutex odometryMutex;

	PT::TelemetryRadio telemetryRadio(2, new PT::PassThroughEncoding());

	ThreeWheelOdom threeWheelOdom(&leftDrive1Odom, &rightDrive1Odom, new OdomWheel(), &imuOrientation);

	OdomFuser odometry(threeWheelOdom);

	void initHardware() {
		Log("Hardware Init");

		leftDriveMotors.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
		rightDriveMotors.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);

		intakeMotors.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);

		double turningFactor = 450.0 / 600.0;
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

		if (isSkills) {
			Log("Skills");
			if (pros::c::registry_get_plugged_type(catapultMotors.at(0).get_port() - 1) !=
			    pros::c::v5_device_e_t::E_DEVICE_MOTOR ||
			    pros::c::registry_get_plugged_type(catapultMotors.at(1).get_port() - 1) !=
			    pros::c::v5_device_e_t::E_DEVICE_MOTOR) {
				Log("Catapult not plugged in");
				master->getController()->rumble(".-.-.-.-");
			} else if (pros::c::registry_get_plugged_type(intakeMotor.get_port() - 1) ==
			           pros::c::v5_device_e_t::E_DEVICE_MOTOR) {
				Log("Intake plugged in");
				master->getController()->rumble(". . . . ");
			}
		} else {
			Log("Competition");
			if (pros::c::registry_get_plugged_type(intakeMotor.get_port() - 1) !=
			    pros::c::v5_device_e_t::E_DEVICE_MOTOR) {
				Log("Intake not plugged in");
				master->getController()->rumble(".-.-.-.-");
			} else if (pros::c::registry_get_plugged_type(catapultMotors.at(0).get_port() - 1) ==
			           pros::c::v5_device_e_t::E_DEVICE_MOTOR ||
			           pros::c::registry_get_plugged_type(catapultMotors.at(1).get_port() - 1) ==
			           pros::c::v5_device_e_t::E_DEVICE_MOTOR) {
				Log("Catapult plugged in");
				master->getController()->rumble("........");
			}
		}

		if (pros::c::registry_get_plugged_type(imu._port - 1) == pros::c::v5_device_e_t::E_DEVICE_IMU) {
			imu.reset();
			Log("Imu: calibrate");

			while (imu.is_calibrating())
				pros::Task::delay(50);

			Log("Imu: done calibrating");
			master->getController()->rumble(".");
		}

		Log("Hardware Init Done");
	}
} // namespace Pronoucne