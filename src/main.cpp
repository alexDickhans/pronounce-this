#include "main.h"

// LVGL
std::shared_ptr<lv_obj_t> tabview;

RobotStatus robotStatus;
ModeLogic modeLogic(&robotStatus);
TeleopModeLogic teleopModeLogic(new pros::Controller(CONTROLLER_MASTER), new pros::Controller(CONTROLLER_PARTNER));

pros::Mutex robotMutex;

// SECTION Auton

/**
 * @brief Runs during auton period before auton
 *
 */
int preAutonRun() {
	teleopController.useDefaultBehavior();
	drivetrainStateController.setCurrentBehavior(&drivetrainStopped);
	drivetrainStateController.setDefaultBehavior(&drivetrainStopped);
	leftDriveMotors.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
	rightDriveMotors.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
	return 0;
}

void turnTo(Angle angle, int waitTimeMS) {
	RotationController angleRotation("AngleTurn", drivetrain, odometry, turningPid, angle, drivetrainMutex);

	drivetrainStateController.setCurrentBehavior(&angleRotation);

	pros::Task::delay(waitTimeMS);

	drivetrainStateController.useDefaultBehavior();

	pros::Task::delay(10);
}

void move(QLength distance, ProfileConstraints profileConstraints, QCurvature curvature) {
	TankMotionProfiling motionProfiling("moveDistance", &drivetrain, profileConstraints, distance, &odometry, &distancePid, drivetrainMutex, curvature);

	drivetrainStateController.setCurrentBehavior(&motionProfiling);

	pros::delay(50);

	while(!drivetrainStateController.isDone()) 
		pros::Task::delay(10);

	drivetrainStateController.setCurrentBehavior(&drivetrainStopped);
}

void move(QLength distance, ProfileConstraints profileConstraints, QCurvature curvature, Angle startAngle) {
	TankMotionProfiling motionProfiling("moveDistance", &drivetrain, profileConstraints, distance, &odometry, &distancePid, drivetrainMutex, curvature, startAngle, &movingTurnPid);

	drivetrainStateController.setCurrentBehavior(&motionProfiling);

	pros::delay(50);

	while(!drivetrainStateController.isDone()) 
		pros::Task::delay(10);

	drivetrainStateController.setCurrentBehavior(&drivetrainStopped);
}

void shootWhileMoving(QLength distance, QSpeed speed, Angle angle, double waitTime = 100, bool off = false) {

	TankMotionProfiling motionProfiling("moveDistance", &drivetrain, { speed, 200_in / second / second, 0.0 }, distance, &odometry, &distancePid, drivetrainMutex, 0.0, angle, &movingTurnPid);

	drivetrainStateController.setCurrentBehavior(&motionProfiling);

	pros::delay(50);

	pros::Task::delay(waitTime);

	ptoStateExtensionController.setCurrentBehavior(off ? &ptoCatapultLaunchOff : &ptoCatapultLaunch);

	while(!drivetrainStateController.isDone()) 
		pros::Task::delay(10);

	drivetrainStateController.setCurrentBehavior(&drivetrainStopped);
}

int tuneTurnPid() {

	threeWheelOdom.setPose(Pose2D(0_in, 0_in, 0_deg));

	// turnTo(180_deg, 1000);

	// move(50_in, defaultProfileConstraints, 0.0, 5_deg);

	// pros::Task::delay(20);

	turnTo(-90_deg, 1500);

	turnTo(90_deg, 1500);

	turnTo(270_deg, 1500);

	turnTo(-90_deg, 1500);

	// pros::Task::delay(200);

	// move(50_in, defaultProfileConstraints, 0.0, 180_deg);

	// pros::delay(50);

	return 0;
}

int tuneDistancePid() {

	threeWheelOdom.setPose(Pose2D(0_in, 0_in, 0_deg));

	move(24_in, defaultProfileConstraints, 0.0, 0_deg);

	// turnTo(90_deg, 1000);

	// move(24_in, defaultProfileConstraints, 0.0, 90_deg);
	// move(-24_in, defaultProfileConstraints, 0.0, 90_deg);

	// turnTo(0_deg, 1000);

	// move(-30_in, defaultProfileConstraints, 0.0, 0_deg);

	pros::Task::delay(50);

	return 0;
}

int testRamseteAuton() {

	threeWheelOdom.setPose(Pose2D(0_in, 0_in, 0_deg));
	ptoStateController.setCurrentBehavior(&ptoIntakeStopped);

	drivetrainStateController.setCurrentBehavior(&testRamsete);

	pros::Task::delay(10000);

	drivetrainStateController.setCurrentBehavior(&drivetrainStopped);

	return 0;
}

int skills() {
	threeWheelOdom.reset(Pose2D(34_in, 12_in, 180_deg));

	ptoStateController.setCurrentBehavior(&ptoIntakeStopped);

	move(3_in, defaultProfileConstraints, 0.0, 180_deg);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostBoosting);

	ptoStateController.setCurrentBehavior(&ptoIntaking);

	turnTo(175_deg, 300);

	ptoStateController.setCurrentBehavior(&ptoIntakeStopped);

	move(-3_in, defaultProfileConstraints, 0.0, 180_deg);

	turnTo(347_deg, 800);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);

	turnTo(347_deg, 400);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostNone);

	pros::Task::delay(500);

	intakeSolenoid.set_value(true);

	move(17_in, { 40_in / second, 130_in / second / second, 0.0 }, 0.0, 351_deg);

	intakeSolenoid.set_value(false);

	pros::Task::delay(1300);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostBoosting);

	shootWhileMoving(3_in, 10_in/second, 349_deg, 0, true);

	pros::Task::delay(300);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostNone);

	turnTo(270_deg, 200);

	ptoStateController.setCurrentBehavior(&ptoIntakeStopped);

	move(30_in, defaultProfileConstraints, 0.0, 270_deg);

	ptoStateController.setCurrentBehavior(&ptoIntaking);

	turnTo(275_deg, 400);

	ptoStateController.setCurrentBehavior(&ptoIntakeStopped);

	move(-3_in, defaultProfileConstraints, 0.0, 270_deg);

	ptoStateController.setCurrentBehavior(&ptoIntaking);

	turnTo(405_deg, 800);

	move(64_in, defaultProfileConstraints, 0.0, 405_deg);

	turnTo(318_deg, 500);

	shootWhileMoving(3_in, 10_in/second, 318_deg, 50);

	move(-3_in, defaultProfileConstraints, 0.0, 318_deg);

	turnTo(290_deg, 800);

	move(33_in, {20_in/second, 125_in/second/second, 0.0}, 0.0);

	move(-33_in, {70_in/second, 125_in/second/second, 0.0}, 0.0, 270_deg);

	turnTo(310_deg, 800);

	shootWhileMoving(3_in, 10_in/second, 310_deg, 50);

	move(-3_in, defaultProfileConstraints, 0.0, 313_deg);

	turnTo(345_deg, 800);

	move(34_in, {20_in/second, 125_in/second/second, 0.0}, 0.0);

	move(-37_in, {70_in/second, 125_in/second/second, 0.0}, 0.0, 360_deg);

	turnTo(317_deg, 800);

	shootWhileMoving(3_in, 10_in/second, 317_deg, 50);

	move(-3_in, defaultProfileConstraints, 0.0, 317_deg);

	turnTo(410_deg, 800);

	move(60_in, {40_in/second, 125_in/second/second, 0.0}, 0.0, 410_deg);

	ptoStateController.setCurrentBehavior(&ptoIntakeStopped);

	move(12_in, defaultProfileConstraints, -50_deg/12_in, 415_deg);

	ptoStateController.setCurrentBehavior(&ptoIntaking);

	move(-10_in, defaultProfileConstraints, 0.0, 360_deg);

	turnTo(265_deg, 800);

	move(35_in, defaultProfileConstraints, 0.0, 265_deg);

	// Match Loader

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunchOff);

	pros::Task::delay(500);

	turnTo(180_deg, 800);

	move(-10_in, defaultProfileConstraints, 0.0);

	ptoStateController.setCurrentBehavior(&ptoIntakeStopped);

	pros::Task::delay(800);

	ptoStateController.setCurrentBehavior(&ptoIntaking);

	move(5_in, defaultProfileConstraints, 0.0, 180_deg);

	turnTo(264_deg, 800);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunchOff);

	pros::Task::delay(600);

	turnTo(180_deg, 800);

	move(-5_in, defaultProfileConstraints, 0.0, 180_deg);

	pros::Task::delay(800);

	ptoStateController.setCurrentBehavior(&ptoIntaking);

	move(5_in, defaultProfileConstraints, 0.0, 180_deg);

	turnTo(264_deg, 800);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);

	pros::Task::delay(300);

	turnTo(135_deg, 800);

	move(30_in, defaultProfileConstraints, 0.0, 135_deg);

	turnTo(90_deg, 600);

	move(43_in, { 40_in / second, 130_in / second / second, 0.0 }, 0.0, 90_deg);

	move(-6_in, defaultProfileConstraints, 0.0);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostBoosting);

	turnTo(183_deg, 800);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);

	pros::Task::delay(400);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostNone);

	turnTo(225_deg, 800);

	move(65_in, defaultProfileConstraints, 0.0, 225_deg);

	turnTo(135_deg, 800);

	shootWhileMoving(3_in, 10_in/second, 135_deg, 50);

	move(-3_in, defaultProfileConstraints, 0.0, 135_deg);

	turnTo(100_deg, 800);

	move(48_in, {25_in/second, 125_in/second/second, 0.0}, 0.0);

	turnTo(188_deg, 800);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);

	pros::Task::delay(500);

	move(-74_in, defaultProfileConstraints, 0.0, 180_deg);

	turnTo(45_deg, 800);

	endgameStateController.setCurrentBehavior(&endgameEnabled);

	turnTo(45_deg, 1000000);

	return 0;
}

int close8Disc() {
	threeWheelOdom.reset(Pose2D(34_in, 12_in, 180_deg));

	ptoStateController.setCurrentBehavior(&ptoIntakeStopped);

	move(3_in, defaultProfileConstraints, 0.0, 180_deg);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostBoosting);

	ptoStateController.setCurrentBehavior(&ptoIntaking);

	turnTo(175_deg, 80);

	ptoStateController.setCurrentBehavior(&ptoIntakeStopped);

	move(-3_in, defaultProfileConstraints, 0.0, 180_deg);

	turnTo(347_deg, 800);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);

	turnTo(347_deg, 400);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostNone);

	turnTo(405_deg, 600);

	move(40_in, { 25_in / second, 130_in / second / second, 0.0 }, 0.0, 405_deg);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostBoosting);

	turnTo(332_deg, 800);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);

	pros::Task::delay(500);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostNone);

	turnTo(330_deg, 200);

	move(-35_in, defaultProfileConstraints, 0.0, 330_deg);
	
	turnTo(370_deg, 800);

	move(25_in, {25_in/second, 75_in/second/second, 0.0}, 0.0);

	move(-25_in, defaultProfileConstraints, 0.0, 360_deg);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostBoosting);

	turnTo(328_deg, 500);

	TankMotionProfiling motionProfiling("moveDistance", &drivetrain, {40_in/second, 200_in/second/second, 0.0}, 25_in, &odometry, &distancePid, drivetrainMutex, 0.0, 328_deg, &movingTurnPid);

	drivetrainStateController.setCurrentBehavior(&motionProfiling);

	pros::Task::delay(500);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);

	pros::Task::delay(500);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostNone);

	drivetrainStateController.useDefaultBehavior();

	return 0;
}

int closeFullAWP() {
	threeWheelOdom.reset(Pose2D(34_in, 12_in, 180_deg));

	ptoStateController.setCurrentBehavior(&ptoIntakeStopped);

	move(3_in, defaultProfileConstraints, 0.0, 180_deg);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostBoosting);

	ptoStateController.setCurrentBehavior(&ptoIntaking);

	turnTo(175_deg, 80);

	ptoStateController.setCurrentBehavior(&ptoIntakeStopped);

	move(-3_in, defaultProfileConstraints, 0.0, 180_deg);

	turnTo(347_deg, 800);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);

	turnTo(347_deg, 400);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostNone);

	turnTo(405_deg, 600);

	move(48_in, { 40_in / second, 100_in / second / second, 0.0 }, 0.0, 405_deg);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostBoosting);

	turnTo(323_deg, 700);

	shootWhileMoving(5_in, 10_in/second, 323_deg, 100);

	move(-5_in, defaultProfileConstraints, 0.0, 323_deg);

	turnTo(323_deg, 200);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostNone);

	turnTo(405_deg, 600);

	move(55_in, { 50_in / second, 75_in / second / second, 0.0 }, 0.0, 405_deg);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostBoosting);

	turnTo(290_deg, 600);
	
	shootWhileMoving(5_in, 50_in/second, 290_deg, 100, true);

	pros::Task::delay(200);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostNone);

	turnTo(405_deg, 600);

	move(40_in, defaultProfileConstraints, 45_deg/40_in, 405_deg);

	ptoStateExtensionController.setCurrentBehavior(new Behavior());

	ptoStateController.setCurrentBehavior(&ptoIntaking);

	turnTo(456_deg, 70);
	
	ptoStateController.setCurrentBehavior(&ptoIntakeStopped);

	return 0;
}

int right8disc() {
	threeWheelOdom.reset(Pose2D(86_in, 130_in, 294_deg));

	pistonBoostStateController.setCurrentBehavior(&pistonBoostBoosting);

	pros::Task::delay(200);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);

	pros::Task::delay(500);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostNone);

	move(40_in, defaultProfileConstraints, 156_deg/40_in, 294_deg);

	ptoStateController.setCurrentBehavior(&ptoIntakeStopped);

	move(8_in, defaultProfileConstraints, 0.0, 450_deg);

	ptoStateController.setCurrentBehavior(&ptoIntaking);

	turnTo(455_deg, 100);

	ptoStateController.setCurrentBehavior(&ptoIntakeStopped);

	move(-5_in, defaultProfileConstraints, 0.0, 450_deg);

	ptoStateController.setCurrentBehavior(&ptoIntaking);

	turnTo(590_deg, 800);

	move(66_in, { 50_in / second, 75_in / second / second, 0.0 }, 0.0, 590_deg);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostBoosting);

	turnTo(681_deg, 600);

	shootWhileMoving(5_in, 10_in/second, 681_deg, 0, false);

	move(-2_in, defaultProfileConstraints, 0.0, 681_deg);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostNone);

	turnTo(835_deg, 800);

	move(40_in, {25_in/second, 100_in/second/second, 0.0}, 0.0);

	move(-40_in, defaultProfileConstraints, 0.0, 845_deg);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostBoosting);

	turnTo(677_deg, 800);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);

	turnTo(677_deg, 400);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostNone);

	return 0;
}

int right9Disc() {
	threeWheelOdom.reset(Pose2D(86_in, 130_in, 294_deg));

	pistonBoostStateController.setCurrentBehavior(&pistonBoostBoosting);

	move(20_in, defaultProfileConstraints, 0.0, 294_deg);

	pros::Task::delay(500);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);

	pros::Task::delay(400);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostNone);
	
	move(-20_in, defaultProfileConstraints, 0.0, 294_deg);

	move(40_in, defaultProfileConstraints, 156_deg/40_in, 294_deg);

	ptoStateController.setCurrentBehavior(&ptoIntakeStopped);

	move(8_in, defaultProfileConstraints, 0.0, 450_deg);

	ptoStateController.setCurrentBehavior(&ptoIntaking);

	turnTo(455_deg, 100);

	ptoStateController.setCurrentBehavior(&ptoIntakeStopped);

	move(-5_in, defaultProfileConstraints, 0.0, 450_deg);

	ptoStateController.setCurrentBehavior(&ptoIntaking);

	turnTo(590_deg, 800);

	move(66_in, intakeProfileConstraints, 0.0, 590_deg);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostBoosting);

	turnTo(681_deg, 800);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);

	turnTo(681_deg, 400);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostNone);

	turnTo(835_deg, 800);

	move(40_in, {25_in/second, 100_in/second/second, 0.0}, 0.0);

	move(-37_in, defaultProfileConstraints, 0.0, 845_deg);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostBoosting);

	turnTo(677_deg, 800);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);

	turnTo(677_deg, 400);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostNone);

	return 0;
}

int right12Disc() {
	threeWheelOdom.reset(Pose2D(86_in, 130_in, 294_deg));

	pistonBoostStateController.setCurrentBehavior(&pistonBoostBoosting);

	move(26_in, { 70_in / second, 100_in / second / second, 0.0 }, 0.0, 294_deg);

	pros::delay(200);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);

	pros::delay(400);

	move(-3_in, defaultProfileConstraints, 0.0, 294_deg);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostNone);
	
	turnTo(265_deg, 300);

	move(-17_in, defaultProfileConstraints, 0.0, 265_deg);

	turnTo(315_deg, 400);

	intakeSolenoid.set_value(true);

	move(12_in, defaultProfileConstraints, 0.0, 315_deg);

	intakeSolenoid.set_value(false);

	pros::Task::delay(300);

	move(-22_in, defaultProfileConstraints, 0.0, 315_deg);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostBoosting);

	turnTo(291_deg, 400);

	shootWhileMoving(10_in, 65_in/second, 291_deg, 200, true);

	pros::Task::delay(100);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostNone);

	turnTo(405_deg, 400);

	move(25_in, defaultProfileConstraints, 45_deg/25_in, 405_deg);

	ptoStateController.setCurrentBehavior(&ptoIntaking);

	move(-8_in, defaultProfileConstraints, 0.0, 450_deg);

	turnTo(590_deg, 600);

	move(65_in, defaultProfileConstraints, 0.0, 590_deg);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostBoosting);

	turnTo(675_deg, 400);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);

	turnTo(675_deg, 300);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostNone);

	turnTo(810_deg, 800);

	move(40_in, defaultProfileConstraints, 0.0);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostBoosting);

	turnTo(675_deg, 600);

	shootWhileMoving(20_in, 60_in/second, 675_deg, 300, false);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostNone);

	return 0;
}

int close9Disc() {
	threeWheelOdom.reset(Pose2D(34_in, 12_in, 0_deg));

	move(24_in, { 20_in / second, 200_in / second / second, 0.0 }, 0.0);

	move(-24_in, defaultProfileConstraints, 0.0);

	turnTo(-35_deg, 500);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostBoosting);

	shootWhileMoving(30_in, 40_in/second, -35_deg, 500);

	pros::Task::delay(200);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostNone);

	turnTo(-135_deg, 500);

	move(40_in, { 25_in / second, 200_in / second / second, 0.0 }, 0.0, -135_deg);
	
	ptoStateController.setCurrentBehavior(&ptoIntakeStopped);

	move(9_in, defaultProfileConstraints, -45_deg/10_in, -135_deg);

	ptoStateController.setCurrentBehavior(&ptoIntaking);

	move(-11_in, stackIntakeProfileConstraints, -50_deg/-8.8_in, -180_deg);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostBoosting);

	turnTo(-15_deg, 500);

	intakeSolenoid.set_value(true);

	move(-8_in, defaultProfileConstraints, 0.0, -15_deg);

	shootWhileMoving(8_in, 50_in/second, -15_deg);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostNone);

	turnTo(-45_deg, 500);

	move(6_in, defaultProfileConstraints, 0.0, -45_deg);

	intakeSolenoid.set_value(false);

	pros::Task::delay(600);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostBoosting);

	move(-13_in, defaultProfileConstraints, 0.0, -45_deg);

	turnTo(-16_deg, 500);

	shootWhileMoving(8_in, 50_in/second, -16_deg);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostNone);

	pros::Task::delay(500);

	return 0;
}

int testLongShot() {
	threeWheelOdom.reset(Pose2D(0_in, 0_in, 0_deg));

	pistonBoostStateController.setCurrentBehavior(&pistonBoostBoosting);

	TankMotionProfiling motionProfiling("moveDistance", &drivetrain, {30_in/second, 150_in/second/second, 150_in/second/second/second}, 10_in, &odometry, &distancePid, drivetrainMutex, 0.0, 0_deg, &movingTurnPid);

	drivetrainStateController.setCurrentBehavior(&motionProfiling);

	pros::delay(300);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);
	
	pros::delay(1000);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostNone);

	drivetrainStateController.setCurrentBehavior(&drivetrainStopped);

	return 0;
}

int testMatchLoad() {

	threeWheelOdom.reset(Pose2D(34_in, 12_in, 180_deg));

	ptoStateController.setCurrentBehavior(&ptoIntakeStopped);

	pros::Task::delay(1000);

	ptoStateController.setCurrentBehavior(&ptoIntaking);

	move(5_in, defaultProfileConstraints, 0.0, 180_deg);

	turnTo(263_deg, 800);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunchOff);

	move(-1_in, defaultProfileConstraints, 0.0);

	pros::Task::delay(500);

	turnTo(180_deg, 800);

	move(-5_in, defaultProfileConstraints, 0.0, 180_deg);

	pros::Task::delay(1000);

	ptoStateController.setCurrentBehavior(&ptoIntaking);

	move(5_in, defaultProfileConstraints, 0.0, 180_deg);

	turnTo(263_deg, 800);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunchOff);

	move(-1_in, defaultProfileConstraints, 0.0);

	pros::Task::delay(500);
}

int postAuton() {
	pros::Task::delay(100);

	drivetrainStateController.setCurrentBehavior(&drivetrainStopped);

	return 0;
}

// !SECTION

// SECTION INIT

void update() {

	pros::Task::delay(200);
	
	robotMutex.take();
	modeLogic.initialize();
	robotMutex.give();

	uint32_t startTime = pros::millis();
	uint32_t startTimeMicros = pros::micros();

	while (true) {
		// Create stuff for exact delay
		startTime = pros::millis();
		startTimeMicros = pros::micros();

		robotMutex.take();
		odometry.update();
		modeLogic.update();
		robotMutex.give();

		// Wait a maximum of 10 milliseconds
		pros::delay(std::min(10 - (pros::millis() - startTime), (long unsigned int) 10));

		std::cout << "FrameTime: " << pros::micros() - startTimeMicros << std::endl;
	}
}

void updateDisplay() {

	// Odom
	std::shared_ptr<lv_obj_t> odomTab = std::shared_ptr<lv_obj_t>(lv_tabview_add_tab(tabview.get(), "Odom"));
	std::shared_ptr<lv_obj_t> odomLabel = std::shared_ptr<lv_obj_t>(lv_label_create(odomTab.get(), NULL));

	std::shared_ptr<lv_obj_t> portsTab = std::shared_ptr<lv_obj_t>(lv_tabview_add_tab(tabview.get(), "Ports"));
	std::shared_ptr<lv_obj_t> portsPage = std::shared_ptr<lv_obj_t>(lv_page_create(portsTab.get(), NULL));
	std::shared_ptr<lv_obj_t> portsLabel = std::shared_ptr<lv_obj_t>(lv_label_create(portsTab.get(), NULL));
	std::shared_ptr<lv_obj_t> portsTable = std::shared_ptr<lv_obj_t>(lv_table_create(portsPage.get(), NULL));
	
	lv_obj_set_width(portsPage.get(), 400);
	lv_obj_set_height(portsPage.get(), 100);

	int count = checkPorts(portsTable.get());

	std::cout << count << std::endl;

	if (count > 0) {
		lv_label_set_text(portsLabel.get(), "SOMETHING IS MISSING");
		// lv_label_set_style(portsLabel.get(), )
	} else {
		lv_label_set_text(portsLabel.get(), "All Good");
	}

	// Drivetrain
	std::shared_ptr<lv_obj_t> drivetrainTab = std::shared_ptr<lv_obj_t>(lv_tabview_add_tab(tabview.get(), "Drivetrain"));
	std::shared_ptr<lv_obj_t> drivetrainTable = std::shared_ptr<lv_obj_t>(lv_table_create(drivetrainTab.get(), NULL));

	lv_table_set_row_cnt(drivetrainTable.get(), 4);
	lv_table_set_col_cnt(drivetrainTable.get(), 2);

	lv_table_set_col_width(drivetrainTable.get(), 0, 200);
	lv_table_set_col_width(drivetrainTable.get(), 1, 200);

	// Flywheels
	std::shared_ptr<lv_obj_t> flywheelTab = std::shared_ptr<lv_obj_t>(lv_tabview_add_tab(tabview.get(), "PTO"));
	std::shared_ptr<lv_obj_t> flywheelLabel = std::shared_ptr<lv_obj_t>(lv_label_create(flywheelTab.get(), NULL));

	while (true) {
		// Odometry
		lv_label_set_text(odomLabel.get(), (odometry.getPosition().to_string()
			+ "\nL: " + std::to_string(leftDrive1Odom.getPosition().Convert(inch)) +
			", R: " + std::to_string(rightDrive1Odom.getPosition().Convert(inch))).c_str());

		// Drivetrain
		lv_table_set_cell_value(drivetrainTable.get(), 0, 0, (std::to_string(leftDrive1.get_temperature()) + " C").c_str());
		lv_table_set_cell_value(drivetrainTable.get(), 1, 0, (std::to_string(leftDrive2.get_temperature()) + " C").c_str());
		lv_table_set_cell_value(drivetrainTable.get(), 2, 0, (std::to_string(leftDrive3.get_temperature()) + " C").c_str());
		lv_table_set_cell_value(drivetrainTable.get(), 3, 0, (std::to_string(leftPtoMotor.get_temperature()) + " C").c_str());
		lv_table_set_cell_value(drivetrainTable.get(), 0, 1, (std::to_string(rightDrive1.get_temperature()) + " C").c_str());
		lv_table_set_cell_value(drivetrainTable.get(), 1, 1, (std::to_string(rightDrive2.get_temperature()) + " C").c_str());
		lv_table_set_cell_value(drivetrainTable.get(), 2, 1, (std::to_string(rightDrive3.get_temperature()) + " C").c_str());
		lv_table_set_cell_value(drivetrainTable.get(), 3, 1, (std::to_string(rightPtoMotor.get_temperature()) + " C").c_str());

		// Flywheel
		lv_label_set_text(flywheelLabel.get(), ("Speed: " + std::to_string(leftPtoMotor.get_actual_velocity())).c_str());

		pros::Task::delay(50);
	}
}

void ledUpdate() {
	while(1) {
		leftLedController.update();
		pros::Task::delay(10);
		rightLedController.update();
		pros::Task::delay(10);
	}
}

void initDisplay() {
	pros::Task display(updateDisplay, TASK_PRIORITY_MIN);
	pros::Task leds(ledUpdate, TASK_PRIORITY_MIN);
}

/**
 * Runs when the robot starts up
 */
void initialize() {

	std::cout << "INIT" << std::endl;

	robotMutex.take();

	lv_init();
	tabview = std::shared_ptr<lv_obj_t>(lv_tabview_create(lv_scr_act(), NULL));

	// Initialize functions
	initHardware();
	initDrivetrain();
	initBoost();
	initPto();
	initEndgame();
	initBehaviors();
	initDisplay();

	robotMutex.give();

	pros::Task modeLogicTask = pros::Task(update, TASK_PRIORITY_MAX);
}

// !SECTION

// SECTION Disabled
/**
 * Runs while the robot is disabled i.e. before and after match, between auton
 * and teleop period
 */
void disabled() {
	robotMutex.take();
	teleopController.useDefaultBehavior();
	robotMutex.give();

	// Create a label
	lv_obj_t* disabledLabel = lv_label_create(lv_scr_act(), NULL);
	lv_obj_align(disabledLabel, NULL, LV_ALIGN_CENTER, 0, 0);
	lv_label_set_text(disabledLabel, "Robot Disabled.");

	while (true) {
		// Nothing right now, other than a little display to show that the robot
		// is disabled.

		pros::delay(200);
	}
}

// !SECTION

// SECTION Competition Initialize

/**
 * Starts when connected to the field
 */
void competition_initialize() {
}

// !SECTION

// SECTION Auton

/**
 * Runs during the autonomous. NO user control
 */
void autonomous() {
	preAutonRun();

	#if AUTON == 0
		closeFullAWP();
	#endif // !1
	#if AUTON == 1
		close9Disc();
	#endif // !1
	#if AUTON == 2
		close8Disc();
	#endif // !1
	#if AUTON == 3
		right8disc();
	#endif // !1
	#if AUTON == 4
		skills();
	#endif // !1

	postAuton();
}

// !SECTION

// SECTION Operator Control

/**
 * Runs during operator/teleop control
 */
void opcontrol() {

	#if AUTON == 4
		preAutonRun();

		pros::Task skillsTask = pros::Task(skills);

		while (true) {
			if (master->get_digital(E_CONTROLLER_DIGITAL_X)) {
				skillsTask.suspend();
				postAuton();
				robotMutex.take();
				teleopController.setCurrentBehavior(&teleopModeLogic);
				drivetrainStateController.setDefaultBehavior(&normalJoystick);
				robotMutex.give();
			}

			pros::Task::delay(10);
		}
	#else
		robotMutex.take();
		teleopController.setCurrentBehavior(&teleopModeLogic);
		drivetrainStateController.setDefaultBehavior(&normalJoystick);
		robotMutex.give();
	#endif // DEBUG
}

// !SECTION
