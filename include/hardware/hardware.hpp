#pragma once

#include "api.h"
#include "odometry/orientation/imu.hpp"
#include "odometry/orientation/avgOrientation.hpp"
#include "position/motorOdom.hpp"
#include "odometry/continuousOdometry/threeWheelOdom.hpp"
#include "odometry/odomFuser.hpp"
#include "pros/rtos.hpp"
#include "hardwareAbstractions/joystick/joystick.hpp"
#include "pros/apix.h"
#include <map>
#include "auton.h"
#include "units/units.hpp"
#include "odometry/orientation/gpsOrientation.hpp"
#include "constants.hpp"
#include "pros/motor_group.hpp"

#ifndef SIM

#include "hardwareAbstractions/joystick/robotJoystick.hpp"
#include "Logger/logger.hpp"

#else
#include "hardwareAbstractions/joystick/simJoystick.hpp"
#endif // !SIM

namespace Pronounce {

	pros::Mutex robotMutex;

	bool isSkills = AUTON == 5;

#ifndef SIM
	RobotJoystick *master = new RobotJoystick(controller_id_e_t::E_CONTROLLER_MASTER);
#else
	AbstractJoystick* master = new SimJoystick(controller_id_e_t::E_CONTROLLER_MASTER);
#endif // !SIM

	pros::Mutex drivetrainMutex;

	pros::MotorGroup leftDriveMotors({-18, -19, -20}, pros::MotorGears::blue, pros::MotorEncoderUnits::rotations);
	pros::MotorGroup rightDriveMotors({11, 12, 13}, pros::MotorGears::blue, pros::MotorEncoderUnits::rotations);

	TankDrivetrain drivetrain(Constants::trackWidth, 76.57632093_in / second, leftDriveMotors, rightDriveMotors,
	                          600.0 * (revolution / minute));

	pros::MotorGroup catapultMotors({-1, 10});

	pros::MotorGroup winch({-2}, pros::v5::MotorGears::red);

	pros::adi::DigitalOut frontLeftSolenoid('G', false);
	pros::adi::DigitalOut frontRightSolenoid('F', false);
	pros::adi::DigitalOut backLeftSolenoid('D', false);
	pros::adi::DigitalOut backRightSolenoid('E', false);

	pros::MotorGroup intakeMotors({17}, pros::MotorGears::blue);
	// Inertial Measurement Unit
	pros::Imu imu(14);
	IMU imuOrientation(14);

	pros::Distance hopperDistanceSensor(16);
	pros::Distance catapultDistance(8);

	ThreeWheelOdom threeWheelOdom(new OdomWheel(), new OdomWheel(), new OdomWheel(), &imuOrientation);

	OdomFuser odometry(threeWheelOdom);

	void initHardware() {
		Log("Hardware Init");
		leftDriveMotors.set_gearing_all(pros::MotorGears::blue);
		rightDriveMotors.set_gearing_all(pros::MotorGears::blue);
		leftDriveMotors.set_zero_position_all(0.0);
		rightDriveMotors.set_zero_position_all(0.0);

		drivetrain.setBrakeMode(pros::MotorBrake::coast);

		intakeMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

		threeWheelOdom.setLeftOffset(10_in / 1.5);
		threeWheelOdom.setRightOffset(10.0_in / 1.5);
		threeWheelOdom.setBackOffset(0.0_in);

		pros::Task::delay(50);

		threeWheelOdom.reset(Pose2D(0.0_in, 0.0_in, 0.0_deg));

		if (isSkills) {
			Log("Skills");

			if (pros::c::registry_get_plugged_type(catapultMotors.get_port_all().at(0) - 1) !=
			    pros::c::v5_device_e_t::E_DEVICE_MOTOR ||
			    pros::c::registry_get_plugged_type(catapultMotors.get_port_all().at(1) - 1) !=
			    pros::c::v5_device_e_t::E_DEVICE_MOTOR) {
				Log("Catapult not plugged in");
				master->getController()->rumble(".-.-.-.-");
			} else if (pros::c::registry_get_plugged_type(intakeMotors.get_port() - 1) ==
			           pros::c::v5_device_e_t::E_DEVICE_MOTOR) {
				Log("Intake plugged in");
				master->getController()->rumble(". . . . ");
			}
		} else {
			Log("Match");

			if (pros::c::registry_get_plugged_type(intakeMotors.get_port() - 1) !=
			    pros::c::v5_device_e_t::E_DEVICE_MOTOR) {
				Log("Intake not plugged in");
				master->getController()->rumble(".-.-.-.-");
			} else if (pros::c::registry_get_plugged_type(catapultMotors.get_port_all().at(0) - 1) ==
			           pros::c::v5_device_e_t::E_DEVICE_MOTOR ||
			           pros::c::registry_get_plugged_type(catapultMotors.get_port_all().at(1) - 1) ==
			           pros::c::v5_device_e_t::E_DEVICE_MOTOR) {
				Log("Catapult plugged in");
				master->getController()->rumble("........");
			}
		}

		if (pros::c::registry_get_plugged_type(imu.get_port() - 1) == pros::c::v5_device_e_t::E_DEVICE_IMU && isSkills) {
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