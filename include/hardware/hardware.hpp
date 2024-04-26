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

	pros::Motor leftDrive1(1, pros::E_MOTOR_GEAR_600, false);
	pros::Motor leftDrive2(2, pros::E_MOTOR_GEAR_600, true);
	pros::Motor leftDrive3(3, pros::E_MOTOR_GEAR_600, false);
	pros::Motor leftDrive4(4, pros::E_MOTOR_GEAR_600, true);
	pros::Motor rightDrive1(18, pros::E_MOTOR_GEAR_600, false);
	pros::Motor rightDrive2(10, pros::E_MOTOR_GEAR_600, true);
	pros::Motor rightDrive3(8, pros::E_MOTOR_GEAR_600, false);
	pros::Motor rightDrive4(9, pros::E_MOTOR_GEAR_600, true);

	pros::Motor_Group leftDriveMotors({leftDrive1, leftDrive2, leftDrive3, leftDrive4});
	pros::Motor_Group rightDriveMotors({rightDrive1, rightDrive2, rightDrive3, rightDrive4});
//	pros::Gps gps(10, -(5.25_in).Convert(metre), -(4.1_in).Convert(metre));
//	GpsOrientation gpsOrientation(gps, 90_deg);

	MotorOdom leftDrive1Odom(std::make_shared<pros::Motor>(leftDrive2), 1.625_in);
	MotorOdom rightDrive1Odom(std::make_shared<pros::Motor>(rightDrive2), 1.625_in);

	TankDrivetrain drivetrain(19_in, 76.57632093_in / second, &leftDriveMotors, &rightDriveMotors,
	                          600.0 * (revolution / minute));

	pros::Motor intakeMotor(5, pros::E_MOTOR_GEARSET_18, false);

	pros::ADIDigitalOut leftSolenoid('A', false);
	pros::ADIDigitalOut rightSolenoid('B', false);
	pros::ADIDigitalOut hangSolenoid('C', false);
	pros::ADIDigitalOut AWPSolenoid('F', false);

	pros::Motor_Group intakeMotors({intakeMotor});
	// Inertial Measurement Unit
	pros::Imu imu(10);
	IMU imuOrientation(10);

	pros::Mutex odometryMutex;

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

		imu.reset(true);

		Log("Hardware Init Done");
	}
} // namespace Pronoucne