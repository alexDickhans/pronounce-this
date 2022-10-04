#pragma once

#include "api.h"
#include "odometry/orientation/imu.hpp"
#include "odometry/orientation/avgOrientation.hpp"
#include "position/motorOdom.hpp"
#include "position/trackingWheel.hpp"
#include "odometry/continuousOdometry/threeWheelOdom.hpp"
#include "odometry/interruptOdometry/gpsOdometry.hpp"
#include "odometry/odomFuser.hpp"

namespace Pronounce {
	pros::Controller master(pros::controller_id_e_t::E_CONTROLLER_MASTER);
	pros::Controller partner(pros::controller_id_e_t::E_CONTROLLER_PARTNER);

	pros::Motor leftDrive1(1, pros::E_MOTOR_GEAR_600, false);
	pros::Motor leftDrive2(1, pros::E_MOTOR_GEAR_600, false);
	pros::Motor leftDrive3(1, pros::E_MOTOR_GEAR_600, false);
	pros::Motor rightDrive1(1, pros::E_MOTOR_GEAR_600, false);
	pros::Motor rightDrive2(1, pros::E_MOTOR_GEAR_600, false);
	pros::Motor rightDrive3(1, pros::E_MOTOR_GEAR_600, false);

	pros::Motor leftPtoMotor(1, pros::E_MOTOR_GEAR_600, false);
	pros::Motor rightPtoMotor(1, pros::E_MOTOR_GEAR_600, false);

	pros::Motor_Group leftDriveMotors({leftDrive1, leftDrive2, leftDrive3});
	pros::Motor_Group rightDriveMotor({rightDrive1, rightDrive2, rightDrive3});

	pros::Rotation backEncoder(7);
	TrackingWheel backOdomWheel(&backEncoder);
	MotorOdom leftDrive1Odom(&leftDrive1, 2.75_in);
	MotorOdom rightDrive1Odom(&rightDrive1, 2.75_in);

	ThreeWheelOdom threeWheelOdom(&leftDrive1Odom, &rightDrive1Odom, &backOdomWheel, &averageImu);

	// Inertial Measurement Unit
	// pros::Imu imu(19);
	// IMU imuOrientation(imu);

	AvgOrientation averageImu;

	// GPS sensor
	pros::Gps gps(4, 0, 0, 90, 0.2, 0.2);
	GpsOdometry gpsOdometry(gps, 7.5_in, 7.5_in, 180_deg);

	OdomFuser odometry(threeWheelOdom);

	void initHardware() {

		double turningFactor = (((100.0 / 100.0) - 1.0) / 2);
		double tuningFactor = 1.0;
		leftDrive1Odom.setRadius(2.75_in / 2.0);
		leftDrive1Odom.setTuningFactor(tuningFactor * (1 - turningFactor));
		rightDrive1Odom.setRadius(2.75_in / 2.0);
		rightDrive1Odom.setTuningFactor(tuningFactor * (1 + turningFactor));
		backOdomWheel.setRadius(2.75_in / 2.0);
		backOdomWheel.setTuningFactor(tuningFactor * 1.0);

		backEncoder.set_reversed(false);

		threeWheelOdom.setLeftOffset(10_in/2.0);
		threeWheelOdom.setRightOffset(10.0_in/2.0);
		threeWheelOdom.setBackOffset(0.0_in);

		pros::Task::delay(10);

		threeWheelOdom.reset(new Pose2D(34.0_in, 12.0_in, 0.0_deg));
	}
} // namespace Pronoucne
