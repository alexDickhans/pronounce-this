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
#include <map>

#ifndef SIM
#include "hardwareAbstractions/joystick/robotJoystick.hpp"
#else
#include "hardwareAbstractions/joystick/simJoystick.hpp"
#endif // !SIM

namespace Pronounce {

	enum GameMode {
		Skills = 0,
		Red = 1,
		Blue = 2,
	};
	
	#if AUTON == 4
		GameMode gameMode = GameMode::Skills;
	#else
		GameMode gameMode = GameMode::Red;
	#endif // AUTON == 4

	pros::Mutex controllerMutex;

#ifndef SIM
	AbstractJoystick* master = new RobotJoystick(controller_id_e_t::E_CONTROLLER_MASTER);
#else
	AbstractJoystick* master = new SimJoystick(controller_id_e_t::E_CONTROLLER_MASTER);
#endif // !SIM

	pros::Mutex drivetrainMutex;

	pros::Motor leftDrive1(5, pros::E_MOTOR_GEAR_600, true);
	pros::Motor leftDrive2(9, pros::E_MOTOR_GEAR_600, true);
	pros::Motor rightDrive1(2, pros::E_MOTOR_GEAR_600, false);
	pros::Motor rightDrive2(10, pros::E_MOTOR_GEAR_600, false);

	pros::Motor_Group leftDriveMotors({ leftDrive1, leftDrive2 });
	pros::Motor_Group rightDriveMotors({ rightDrive1, rightDrive2 });

	MotorOdom leftDrive1Odom(std::make_shared<pros::Motor>(leftDrive2), 1.625_in);
	MotorOdom rightDrive1Odom(std::make_shared<pros::Motor>(rightDrive2), 1.625_in);

	TankDrivetrain drivetrain(23_in, 61.26105675_in / second, &leftDriveMotors, &rightDriveMotors, 600);

	pros::Motor leftIntakeMotor(4, pros::E_MOTOR_GEAR_600, false);
	pros::Motor rightIntakeMotor(8, pros::E_MOTOR_GEAR_600, true);

	pros::Motor_Group intakeMotors({leftIntakeMotor, rightIntakeMotor});

	// Inertial Measurement Unit
	pros::Imu imu(11);
	IMU imuOrientation(imu);

	pros::Mutex odometryMutex;

	ThreeWheelOdom threeWheelOdom(&leftDrive1Odom, &rightDrive1Odom, new OdomWheel(), &imuOrientation);

	OdomFuser odometry(threeWheelOdom);

	void initHardware() {

		drivetrainMutex.take();

		leftDriveMotors.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
		rightDriveMotors.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);

		leftDrive1.set_current_limit(10000);
		leftDrive2.set_current_limit(10000);
		// leftDrive3.set_current_limit(10000);

		rightDrive1.set_current_limit(10000);
		rightDrive2.set_current_limit(10000);
		// std::cout << "CurrentLimit: " << rightDrive3.set_current_limit(10000) << std::endl;

		intakeMotors.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);

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

		if (pros::c::registry_get_plugged_type(10) == pros::c::v5_device_e_t::E_DEVICE_IMU) {
			imu.reset();

			while (imu.is_calibrating())
				pros::Task::delay(50);
		}

		odometryMutex.give();

		// catapultLimitSwitch.reverse();
	}

	int checkPorts(lv_obj_t* table) {
		std::map<uint8_t, pros::c::v5_device_e_t> portsList;

		portsList.emplace(8, pros::c::v5_device_e_t::E_DEVICE_MOTOR);
		portsList.emplace(9, pros::c::v5_device_e_t::E_DEVICE_MOTOR);
		portsList.emplace(7, pros::c::v5_device_e_t::E_DEVICE_MOTOR);
		portsList.emplace(2, pros::c::v5_device_e_t::E_DEVICE_MOTOR);
		portsList.emplace(3, pros::c::v5_device_e_t::E_DEVICE_MOTOR);
		portsList.emplace(4, pros::c::v5_device_e_t::E_DEVICE_MOTOR);
		portsList.emplace(16, pros::c::v5_device_e_t::E_DEVICE_MOTOR);
		portsList.emplace(15, pros::c::v5_device_e_t::E_DEVICE_MOTOR);
		portsList.emplace(14, pros::c::v5_device_e_t::E_DEVICE_ROTATION);
		portsList.emplace(21, pros::c::v5_device_e_t::E_DEVICE_RADIO);
		portsList.emplace(18, pros::c::v5_device_e_t::E_DEVICE_IMU);

		lv_table_set_col_cnt(table, 3);
		lv_table_set_row_cnt(table, portsList.size());

		int count = 0;
		int missingCount = 0;

		for (auto i = portsList.begin(); i != portsList.end(); i++) {
			lv_table_set_cell_value(table, count, 0, std::to_string(i->first).c_str());
			lv_table_set_cell_value(table, count, 1, std::to_string(i->second).c_str());
			lv_table_set_cell_value(table, count, 2, std::to_string(pros::c::registry_get_plugged_type(i->first-1) == i->second).c_str());

			if (pros::c::registry_get_plugged_type(i->first-1) != i->second) {
				missingCount ++;
				std::cout << "PortMissing: " << std::to_string(i->first) << std::endl;
			}

			count ++;
		}

		return missingCount;
	}
} // namespace Pronoucne