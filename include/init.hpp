#pragma once

#include "api.h"
#include "chassis/tankDrive.hpp"
#include "position/odomWheel.hpp"
#include "odometry/threeWheelOdom.hpp"
#include "motionControl/tankPurePursuit.hpp"
#include "driver/controller.hpp"

namespace Pronounce {

	/**
	 * Initialize all sensors
	 */
	void initSensors(pros::Imu imu) {
		printf("Initializing sensors\n");

		imu.reset();

		// Not needed with 3 wheel odom
		// Wait until IMU is calibrated
		// while (imu.is_calibrating()) {
		// 	pros::delay(20);
		// }

		//	printf("IMU calibrated\n");

		printf("Sensors initialized\n");
	}


	/**
	 * Initialize the controller
	 */
	void initController(Controller controller, Pronounce::TankDrivetrain* drivetrain, Pronounce::Odometry* odometry, void(&renderThread)()) {
		controller.setDrivetrain(drivetrain);
		controller.setOdometry(odometry);
		pros::Task renderTask(renderThread);
	}


	void initDrivetrain(OdomWheel leftOdomWheel, OdomWheel rightOdomWheel, pros::Rotation leftEncoder, pros::Rotation rightEncoder, Pronounce::ThreeWheelOdom odometry, TankPurePursuit purePursuit, void(&updateDrivetrain)()) {
		printf("Init drivetrain\n");

		// odometry.setUseImu(true);
		leftOdomWheel.setRadius(2.75 / 2);
		leftOdomWheel.setTuningFactor(1.005);
		rightOdomWheel.setRadius(2.75 / 2);
		rightOdomWheel.setTuningFactor(1.0017);

		leftEncoder.set_reversed(true);
		rightEncoder.set_reversed(true);

		odometry.setLeftOffset(4.5 * 0.957);
		odometry.setRightOffset(4.5 * 0.957);
		odometry.setBackOffset(0);
		// odometry.setBackOffset(2.5);

		odometry.setMaxMovement(1);

		purePursuit.setNormalizeDistance(10);
		purePursuit.setSpeed(150);
		purePursuit.setLookahead(15);
		purePursuit.setStopDistance(0.5);
		purePursuit.setMaxAcceleration(300);

		pros::Task purePursuitTask = pros::Task(updateDrivetrain, "Pure Pursuit");

		odometry.reset(new Position());

		printf("Drivetrain Init done\n");
	}

}