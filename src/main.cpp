#include "main.h"

// Controllers
Pronounce::Controller master(pros::E_CONTROLLER_MASTER);
Pronounce::Controller partner(pros::E_CONTROLLER_PARTNER);

// Motors

// Drive Motors
pros::Motor frontRightMotor(3, pros::E_MOTOR_GEARSET_06, true);
pros::Motor midRightMotor(4, pros::E_MOTOR_GEARSET_06, true);
pros::Motor backRightMotor(5, pros::E_MOTOR_GEARSET_06, false);
pros::Motor frontLeftMotor(6, pros::E_MOTOR_GEARSET_06, false);
pros::Motor midLeftMotor(7, pros::E_MOTOR_GEARSET_06, false);
pros::Motor backLeftMotor(8, pros::E_MOTOR_GEARSET_06, true);

pros::Motor lift(2, false);

pros::Motor intake(1, true);

pros::ADIDigitalOut frontGrabber(1, false);
pros::ADIDigitalOut backGrabber(2, false);
pros::ADIDigitalOut backTilter(3, false);
pros::ADIDigitalOut frontHook(4, false);
pros::ADIDigitalIn frontGrabberBumperSwitch(5);
pros::ADIDigitalIn backGrabberBumperSwitch(6);

// Inertial Measurement Unit
pros::Imu imu(5);

pros::Rotation leftEncoder(10);
pros::Rotation rightEncoder(9);

// Odom wheels
Pronounce::TrackingWheel leftOdomWheel(&leftEncoder);
Pronounce::TrackingWheel rightOdomWheel(&rightEncoder);
OdomWheel nullOdomWheel;

// GPS sensor
pros::Gps gps(4, 0, 0, 90, 0.2, 0.2);
GpsOdometry gpsOdometry(&gps);

// ThreeWheelOdom odometry(&leftOdomWheel, &rightOdomWheel, &backOdomWheel, &imu);
ThreeWheelOdom odometry(&leftOdomWheel, &rightOdomWheel, &nullOdomWheel, &imu);

TankDrivetrain drivetrain(&frontLeftMotor, &frontRightMotor, &midLeftMotor, &midRightMotor, &backLeftMotor, &backRightMotor, &imu, 9.0);

Pronounce::TankPurePursuit purePursuit(&drivetrain, &odometry, new PID(0.6, 0, 2.0), 20);

Balance balance(&drivetrain, &imu, new BangBang(20, true, -50), new PID(0, 0, 0));

MotorButton liftButton(&master, &lift, DIGITAL_L1, DIGITAL_L2, 200, 0, -200, 0, 0);
MotorButton intakeButton(&master, &intake, DIGITAL_R2, DIGITAL_Y, 200, 0, -200, 0, 0);

SolenoidButton frontGrabberButton(&master, DIGITAL_A);
SolenoidButton frontHookButton(&master, DIGITAL_X);
SolenoidButton backGrabberButton(&master, DIGITAL_R1, DIGITAL_R1);
SolenoidButton backGrabberButton2(&master, DIGITAL_R1, DIGITAL_R1);

// Autonomous Selector
Pronounce::AutonSelector autonomousSelector;

// LVGL
lv_obj_t* tabview;

#define DRIFT_MIN 7.0

bool preDriverTasksDone = false;
bool disableIntake = true;

int driverMode = 0;

bool backButtonStatus = false;
bool backGrabberManual = false;
uint32_t lastChange = 0;

// SECTION Auton

void backGrabberChange(bool backButtonStatus2) {
	if (backButtonStatus2 == backButtonStatus) {
		return;
	}
	backButtonStatus = backButtonStatus2;
	lastChange = pros::millis();
}

void waitForDone(double distance, double timeout) {
	uint32_t startTime = pros::millis();

	// Wait for done
	while (!purePursuit.isDone(distance) && pros::millis() - startTime < timeout && purePursuit.isFollowing()) {
		pros::Task::delay(50);
	}
}

void waitForDone(double distance) {
	// Wait for done
	while (!purePursuit.isDone(distance)) {
		pros::Task::delay(30);
	}
}

void waitForDone() {
	waitForDone(0.5, 15000);
}

void waitForDoneOrientation() {
	pros::Task::delay(100);

	uint32_t startTime = pros::millis();

	pros::Task::delay(300);

	while (!purePursuit.isDoneOrientation(0.1) && pros::millis() - startTime < 3000) {
		pros::Task::delay(50);
	}

	pros::Task::delay(500);
}

void waitForDoneOrientation(double margin) {
	pros::Task::delay(100);

	uint32_t startTime = pros::millis();

	pros::Task::delay(300);

	while (!purePursuit.isDoneOrientation(margin) && pros::millis() - startTime < 3000) {
		pros::Task::delay(50);
	}

	pros::Task::delay(500);
}

void placeOnPlatform() {
	pros::Task::delay(100);

	liftButton.setAutonomousAuthority(2000);

	waitForDone();

	purePursuit.setEnabled(false);
	purePursuit.setFollowing(false);

	liftButton.setAutonomousAuthority(1100);

	pros::Task::delay(300);

	frontGrabberButton.setButtonStatus(NEUTRAL);

	pros::Task::delay(200);

	drivetrain.skidSteerVelocity(-150, 0);

	pros::Task::delay(200);

	drivetrain.skidSteerVelocity(-60, 0);

	pros::Task::delay(200);

	liftButton.setAutonomousAuthority(2000);

	drivetrain.skidSteerVelocity(-50, 0);

	pros::Task::delay(100);

	drivetrain.skidSteerVelocity(0, 0);

	pros::Task::delay(100);

	purePursuit.setEnabled(true);
	purePursuit.setFollowing(true);
}

void balanceRobot(double angle) {
	balance.getOrientationController()->setTarget(angle);

	purePursuit.setFollowing(false);
	purePursuit.setEnabled(false);

	drivetrain.skidSteerVelocity(200, 0);

	pros::Task::delay(1500);

	drivetrain.getLeftMotors().set_brake_mode(MOTOR_BRAKE_HOLD);
	drivetrain.getRightMotors().set_brake_mode(MOTOR_BRAKE_HOLD);

	balance.setEnabled(true);

	pros::Task::delay(50000);

	balance.setEnabled(false);
}

void turn(double angle) {
	purePursuit.setFollowing(true);
	purePursuit.setOrientationControl(true);

	purePursuit.setTargetOrientation(angle);

	waitForDoneOrientation();

	purePursuit.setOrientationControl(false);
}

void turn(double angle, double sensitivity) {
	purePursuit.setFollowing(true);
	purePursuit.setOrientationControl(true);

	purePursuit.setTargetOrientation(angle);

	waitForDoneOrientation(sensitivity);

	purePursuit.setOrientationControl(false);
}

void grabFromHook() {
	purePursuit.setEnabled(false);
	purePursuit.setFollowing(false);

	frontHookButton.setButtonStatus(POSITIVE);

	pros::Task::delay(200);

	drivetrain.skidSteerVelocity(-600, 0);

	pros::Task::delay(250);

	drivetrain.skidSteerVelocity(0, 0);

	frontHookButton.setButtonStatus(NEUTRAL);

	//purePursuit.setEnabled(true);
	//purePursuit.setFollowing(true);

	//turn(odometry.getPosition()->getTheta() - 0.1);

	pros::Task::delay(200);

	purePursuit.setEnabled(false);
	purePursuit.setFollowing(false);

	pros::Task::delay(200);

	drivetrain.skidSteerVelocity(600, 0);

	pros::Task::delay(500);

	frontGrabberButton.setButtonStatus(POSITIVE);

	pros::Task::delay(200);

	purePursuit.setEnabled(true);
	purePursuit.setFollowing(true);
}

/**
 * @brief Runs during auton period before auton
 *
 */
int preAutonRun() {

	purePursuit.setEnabled(true);

	frontGrabberButton.setAutonomous(true);
	backGrabberButton.setAutonomous(true);
	liftButton.setAutonomous(true);
	liftButton.setAutonomousPosition(true);
	intakeButton.setAutonomous(true);

	drivetrain.getLeftMotors().set_brake_mode(MOTOR_BRAKE_HOLD);
	drivetrain.getRightMotors().set_brake_mode(MOTOR_BRAKE_HOLD);

	return 0;
}

void deployBackGrabber() {
	backGrabberManual = true;

	backGrabberButton.setButtonStatus(POSITIVE);

	pros::Task::delay(100);

	backGrabberButton.setButtonStatus(NEUTRAL);

	backGrabberManual = false;
}

void leftSteal() {
	purePursuit.setSpeed(70);

	frontGrabberButton.setButtonStatus(NEUTRAL);
	backGrabberChange(false);
	frontHookButton.setButtonStatus(POSITIVE);

	double oldLookahead = purePursuit.getLookahead();

	purePursuit.setLookahead(25);

	purePursuit.setEnabled(true);
	purePursuit.setFollowing(true);

	purePursuit.setCurrentPathIndex(leftHomeZoneToLeftNeutralStealIndex);

	waitForDone();

	frontHookButton.setButtonStatus(NEUTRAL);

	purePursuit.setCurrentPathIndex(leftNeutralStealToLeftHomeZoneIndex);

	while (odometry.getPosition()->getY() > 40) {
		pros::Task::delay(50);
	}

	purePursuit.setSpeed(45);

	waitForDone();

	purePursuit.setLookahead(oldLookahead);

	grabFromHook();
}

void midSteal() {
	purePursuit.setSpeed(70);

	double oldLookahead = purePursuit.getLookahead();

	purePursuit.setLookahead(25);

	frontGrabberButton.setButtonStatus(NEUTRAL);
	frontHookButton.setButtonStatus(POSITIVE);

	purePursuit.setEnabled(true);
	purePursuit.setFollowing(true);

	purePursuit.setCurrentPathIndex(rightHomeZoneToMidNeutralIndex);

	waitForDone();

	frontHookButton.setButtonStatus(NEUTRAL);

	purePursuit.setCurrentPathIndex(midNeutralToRightHomeZoneIndex);

	purePursuit.setSpeed(50);

	while (odometry.getPosition()->getY() > 40) {
		pros::Task::delay(50);
	}

	for (int i = 5; i > 3; i--) {
		purePursuit.setSpeed(i * 10);
		pros::Task::delay(200);
	}

	waitForDone();

	purePursuit.setLookahead(oldLookahead);

	grabFromHook();
}

void rightSteal() {
	purePursuit.setSpeed(70);

	frontGrabberButton.setButtonStatus(NEUTRAL);
	backGrabberChange(false);
	frontHookButton.setButtonStatus(POSITIVE);

	double oldLookahead = purePursuit.getLookahead();

	purePursuit.setLookahead(20);

	purePursuit.setEnabled(true);
	purePursuit.setFollowing(true);

	purePursuit.setCurrentPathIndex(rightHomeZoneToRightNeutralIndex);

	waitForDone();

	frontHookButton.setButtonStatus(NEUTRAL);

	purePursuit.setCurrentPathIndex(rightNeutralToRightHomeZoneIndex);

	purePursuit.setSpeed(65);

	while (odometry.getPosition()->getY() > 40) {
		pros::Task::delay(50);
	}

	purePursuit.setSpeed(45);

	waitForDone();

	purePursuit.setLookahead(oldLookahead);

	grabFromHook();
}

void rightRightSteal() {
	purePursuit.setSpeed(70);

	frontGrabberButton.setButtonStatus(NEUTRAL);
	frontHookButton.setButtonStatus(POSITIVE);

	purePursuit.setEnabled(true);
	purePursuit.setFollowing(true);

	purePursuit.setCurrentPathIndex(farRightHomeZoneToRightNeutralIndex);

	waitForDone();

	frontHookButton.setButtonStatus(NEUTRAL);

	purePursuit.setCurrentPathIndex(rightNeutralToFarRightHomeZoneIndex);

	purePursuit.setSpeed(65);

	while (odometry.getPosition()->getY() > 40) {
		pros::Task::delay(50);
	}

	purePursuit.setSpeed(45);

	waitForDone();

	grabFromHook();
}

void rightMidSteal() {
	purePursuit.setSpeed(70);

	frontGrabberButton.setButtonStatus(NEUTRAL);
	backGrabberChange(false);
	frontHookButton.setButtonStatus(POSITIVE);

	double oldLookahead = purePursuit.getLookahead();

	purePursuit.setLookahead(30);

	purePursuit.setEnabled(true);
	purePursuit.setFollowing(true);

	purePursuit.setCurrentPathIndex(rightHomeZoneToMidNeutralIndex);

	waitForDone();

	frontHookButton.setButtonStatus(NEUTRAL);

	purePursuit.setCurrentPathIndex(midNeutralToRightHomeZoneIndex);

	while (odometry.getPosition()->getY() > 40) {
		pros::Task::delay(50);
	}

	purePursuit.setSpeed(45);

	waitForDone();

	purePursuit.setLookahead(oldLookahead);

	grabFromHook();
}

void leftAWP() {
	liftButton.setAutonomousAuthority(600);

	pros::Task::delay(100);

	purePursuit.setSpeed(20);

	purePursuit.setCurrentPathIndex(leftAllianceToRightHomeZoneIndex);

	pros::Task::delay(200);

	intakeButton.setButtonStatus(POSITIVE);

	waitForDone(20);

	purePursuit.setSpeed(0);

	pros::Task::delay(500);

	purePursuit.setFollowing(false);
}

void rightAWP() {
	turn(-M_PI_2 * 0.85);

	purePursuit.setCurrentPathIndex(enterRightHomeZoneToRightAllianceIndex);

	waitForDone(20, 2500);

	purePursuit.setSpeed(25);

	waitForDone(8, 2500);

	backGrabberChange(true);

	waitForDone(0.1, 1000);

	pros::Task::delay(100);

	purePursuit.setCurrentPathIndex(rightAllianceToRightRingsIndex);

	liftButton.setAutonomousAuthority(2000);

	intakeButton.setButtonStatus(POSITIVE);

	waitForDone(0.1, 3000);

	purePursuit.setCurrentPathIndex(rightRingsToRightHomeZoneIndex);

	waitForDone();
}

void rightAWPToLeftAWP() {
	purePursuit.setSpeed(40);

	turn(-M_PI_2);

	purePursuit.setCurrentPathIndex(rightPlatformToRightAllianceIndex);

	waitForDone(20, 2500);

	purePursuit.setSpeed(40);

	waitForDone(8, 2500);

	backGrabberChange(true);

	waitForDone(0.1, 2500);

	intakeButton.setButtonStatus(POSITIVE);
	liftButton.setAutonomousAuthority(600);

	purePursuit.setCurrentPathIndex(rightAllianceToLeftAllianceIndex);

	purePursuit.setSpeed(50);

	waitForDone(70);

	intakeButton.setButtonStatus(NEUTRAL);

	backGrabberChange(false);

	purePursuit.setSpeed(50);

	pros::Task::delay(300);

	turn(0);

	waitForDone();

	backGrabberChange(true);

	pros::Task::delay(200);

	intakeButton.setButtonStatus(POSITIVE);

	purePursuit.setSpeed(40);

	for (int i = 0; i < 5; i++) {
		purePursuit.setCurrentPathIndex(leftAllianceToPreloadsIndex);

		waitForDone();

		purePursuit.setCurrentPathIndex(preloadsToLeftAllianceIndex);

		waitForDone();
	}

	intakeButton.setButtonStatus(NEUTRAL);
}

void rightToLeftAWP() {
	turn(-M_PI_2);

	purePursuit.setCurrentPathIndex(rightHomeZoneToLeftAllianceIndex);

	waitForDone();

	backGrabberChange(true);

	pros::Task::delay(200);

	leftAWP();
}

/*
	SECTION Left side
*/

/**
 * @brief Steal the left side goal and get the left AWP
 *
 * @return int
 */
int leftStealLeftAWP() {
	odometry.reset(new Position(27, 18, 0.17));

	pros::Task deployBack(deployBackGrabber);

	leftSteal();

	purePursuit.setCurrentPathIndex(leftHomeZoneToLeftAllianceIndex);

	waitForDone();

	backGrabberChange(true);

	leftAWP();

	pros::Task::delay(2000);

	backGrabberChange(false);

	return 0;
}

/**
 * @brief Steal the left side goal and get the full AWP
 *
 * @return int
 */
int leftStealFullAWP() {
	odometry.reset(new Position(27, 18, 0.17));

	leftSteal();

	purePursuit.setCurrentPathIndex(leftHomeZoneToLeftAllianceIndex);

	waitForDone();

	backGrabberChange(true);

	leftAWP();

	waitForDone();

	backGrabberChange(false);

	pros::Task::delay(1000);

	purePursuit.setFollowing(true);

	rightAWP();

	backGrabberChange(false);

	return 0;
}

/*
	SECTION Right side
*/

/**
 * @brief Steal the right side goal and get the right AWP
 *
 * @return int
 */
int rightStealRightAWP() {
	odometry.reset(new Position(107, 19.5, 0.0));

	pros::Task deployBack(deployBackGrabber);

	rightSteal();

	rightAWP();

	backGrabberChange(false);

	pros::Task::delay(200);

	return 0;
}

/**
 * @brief Steal the right side goal and get the full AWp
 *
 * @return int
 */
int rightStealFullAWP() {
	odometry.reset(new Position(105.7, 19.5, 0.0));

	rightSteal();

	rightAWPToLeftAWP();

	backGrabberChange(false);

	pros::Task::delay(200);

	return 0;
}

/**
 * @brief Steal the right side goal starting from the right right side and get the right side AWP
 *
 * @return int
 */
int rightRightStealToRightAWP() {
	odometry.reset(new Position(121, 18, toRadians(-15)));

	pros::Task deployBack(deployBackGrabber);

	rightRightSteal();

	rightAWP();

	return 0;
}

/**
 * @brief Steal the right side goal starting from the right right side and get the right side AWP
 *
 * @return int
 */
int rightRightStealToFullAWP() {
	odometry.reset(new Position(121, 18, toRadians(-15)));

	pros::Task deployBack(deployBackGrabber);
	
	rightRightSteal();

	rightAWPToLeftAWP();

	return 0;
}

/**
 * @brief Steal the mid goal from the right side and get the Left AWP
 *
 * @return int
 */
int midStealToLeftAWP() {
	odometry.reset(new Position(115, 19.5, 0.0));

	pros::Task deployBack(deployBackGrabber);

	midSteal();

	rightToLeftAWP();

	return 0;
}

/**
 * @brief Steal the mid goal from the right side and get the Left AWP
 *
 * @return int
 */
int midStealToRightAWP() {
	odometry.reset(new Position(99.5, 18, 25.0));

	pros::Task deployBack(deployBackGrabber);

	midSteal();

	rightAWP();

	return 0;
}

/**
 * @brief Steal the mid goal from the right side and get the full AWP
 *
 * @return int
 */
int midStealToFullAWP() {
	odometry.reset(new Position(99.5, 18, toRadians(-30)));

	pros::Task deployBack(deployBackGrabber);

	midSteal();

	rightAWPToLeftAWP();

	return 0;
}

int skills() {
	odometry.reset(new Position(26.5, 11, -M_PI_2));

	backGrabberButton.setButtonStatus(ButtonStatus::NEUTRAL);
	frontGrabberButton.setButtonStatus(ButtonStatus::NEUTRAL);

	purePursuit.setMaxAcceleration(65);

	// Move back
	// Timed programming, our favorite!
	// This is all the safegaurds I have to bypass
	purePursuit.setFollowing(false);
	purePursuit.setEnabled(false);

	backGrabberChange(false);

	drivetrain.skidSteerVelocity(-400, 0);

	pros::Task::delay(200);

	drivetrain.skidSteerVelocity(0, 0);

	backGrabberChange(true);

	purePursuit.setFollowing(true);
	purePursuit.setEnabled(true);

	purePursuit.setLookahead(12);
	purePursuit.setSpeed(65);

	pros::Task::delay(500);

	purePursuit.setCurrentPathIndex(leftHomeZoneToLeftNeutralGoalIndex);

	liftButton.setAutonomousAuthority(600);

	pros::Task::delay(500);

	intakeButton.setButtonStatus(POSITIVE);

	while (odometry.getPosition()->getY() < 38) {
		pros::Task::delay(50);
	}

	purePursuit.setSpeed(20);

	waitForDone(25);

	liftButton.setAutonomousAuthority(0);

	waitForDone();

	frontGrabberButton.setButtonStatus(POSITIVE);

	purePursuit.setSpeed(65);

	purePursuit.setCurrentPathIndex(leftNeutralGoalToFarHomeZoneIndex);

	liftButton.setAutonomousAuthority(2000);

	waitForDone(0.5, 3000);

	purePursuit.setSpeed(60);

	turn(0, 0.1);

	purePursuit.setFollowing(false);

	placeOnPlatform();

	turn(-M_PI_2);

	purePursuit.setCurrentPathIndex(farLeftHomeZoneToFarRightGoalDropOffIndex);

	waitForDone(2);

	intakeButton.setButtonStatus(NEUTRAL);

	backGrabberChange(false);

	pros::Task::delay(600);

	waitForDone();

	purePursuit.setSpeed(60);

	turn(M_PI_2 + M_PI_4);

	purePursuit.setCurrentPathIndex(farRightDropOffToFarLeftAllianceGoalIndex);

	purePursuit.setSpeed(60);

	waitForDone(30);

	liftButton.setAutonomousAuthority(600);

	purePursuit.setEnabled(false);
	purePursuit.setFollowing(false);

	pros::Task::delay(500);

	purePursuit.setEnabled(true);
	purePursuit.setFollowing(true);

	turn(M_PI_2);

	purePursuit.setSpeed(20);

	waitForDone(0.5, 2000);

	backGrabberChange(true);

	pros::Task::delay(100);

	purePursuit.setSpeed(30);

	turn(M_PI*0.9);

	purePursuit.setCurrentPathIndex(farLeftAllianceToMidGoalIndex);

	intakeButton.setButtonStatus(POSITIVE);

	purePursuit.setSpeed(30);

	waitForDone(20);

	purePursuit.setSpeed(20);

	waitForDone(10);

	liftButton.setAutonomousAuthority(0);

	waitForDone();

	frontGrabberButton.setButtonStatus(POSITIVE);

	pros::Task::delay(300);

	purePursuit.setSpeed(60);

	turn(0);

	purePursuit.setCurrentPathIndex(midGoalToFarPlatformIndex);

	liftButton.setAutonomousAuthority(2000);

	purePursuit.setSpeed(40);

	placeOnPlatform();

	turn(M_PI_2);

	purePursuit.setCurrentPathIndex(farPlatformToFarLeftAllianceDropOffIndex);

	liftButton.setAutonomousAuthority(0);

	waitForDone();

	turn(M_PI_2);

	intakeButton.setButtonStatus(NEUTRAL);

	pros::Task::delay(200);

	backGrabberChange(false);

	pros::Task::delay(500);

	purePursuit.setCurrentPathIndex(farLeftAllianceDropOffToFarRightDropOffIndex);

	waitForDone();

	frontGrabberButton.setButtonStatus(POSITIVE);

	pros::Task::delay(200);

	liftButton.setAutonomousAuthority(2000);

	pros::Task::delay(1000);

	turn(0);

	pros::Task::delay(200);

	purePursuit.setCurrentPathIndex(farRightDropOffToPlatformIndex);

	placeOnPlatform();

	purePursuit.setCurrentPathIndex(platformToEnterFarRightHomeZoneIndex);

	waitForDone();

	turn(-M_PI_2);

	purePursuit.setCurrentPathIndex(enterRightHomeZoneToFarRightAllianceIndex);

	waitForDone();

	backGrabberChange(true);

	pros::Task::delay(200);

	purePursuit.setCurrentPathIndex(farRightAllianceToRightHomeZoneIndex);

	pros::Task::delay(200);

	intakeButton.setButtonStatus(POSITIVE);

	purePursuit.setSpeed(60);

	while (odometry.getPosition()->getY() > 85) {
		pros::Task::delay(50);
	}

	purePursuit.setSpeed(40);

	while (odometry.getPosition()->getY() > 40) {
		pros::Task::delay(50);
	}

	purePursuit.setSpeed(10);

	while (odometry.getPosition()->getY() > 30) {
		pros::Task::delay(50);
	}

	backGrabberChange(false);

	intakeButton.setButtonStatus(NEUTRAL);

	pros::Task::delay(500);

	purePursuit.setSpeed(60);

	waitForDone();

	turn(-M_PI_2 - M_PI_4);

	purePursuit.setCurrentPathIndex(rightHomeZoneToRightAllianceIndex);

	waitForDone();

	backGrabberChange(true);

	pros::Task::delay(100);

	turn(0);

	purePursuit.setCurrentPathIndex(rightAllianceToRightNeutralIndex);

	waitForDone();

	frontGrabberButton.setButtonStatus(POSITIVE);

	purePursuit.setCurrentPathIndex(rightNeutralToPlatformIndex);

	pros::Task::delay(500);

	liftButton.setAutonomousAuthority(2000);

	placeOnPlatform();

	purePursuit.setCurrentPathIndex(rightPlatformToFarRightGoalDropOff2Index);

	liftButton.setAutonomousAuthority(0);

	waitForDone();

	turn(-M_PI_2);

	backGrabberChange(false);
	intakeButton.setButtonStatus(NEUTRAL);

	purePursuit.setCurrentPathIndex(farRightGoalDropOff2ToFarLeftAllianceDropOffIndex);

	waitForDone(20);

	turn(M_PI_2);

	purePursuit.setCurrentPathIndex(farLeftAllianceDropOffToFarRightGoalDropOff2Index);

	waitForDone();

	frontGrabberButton.setButtonStatus(POSITIVE);

	pros::Task::delay(200);

	purePursuit.setCurrentPathIndex(farRightGoalDropOff2ToFarLeftAllianceDropOffIndex);

	pros::Task::delay(500);

	liftButton.setAutonomousAuthority(2000);

	waitForDone();

	backGrabberChange(true);

	pros::Task::delay(100);

	intakeButton.setButtonStatus(POSITIVE);

	turn(0);

	purePursuit.setCurrentPathIndex(farLeftAllianceDropOffToPlatformIndex);

	placeOnPlatform();

	turn(M_PI_2);

	purePursuit.setCurrentPathIndex(enterFarLeftHomeZoneToNearRightPlatformIndex);

	liftButton.setAutonomousAuthority(600);

	while (odometry.getPosition()->getY() > 40) {
		pros::Task::delay(50);
	}

	purePursuit.setSpeed(20);

	liftButton.setAutonomousAuthority(0);

	while (odometry.getPosition()->getY() > 35) {
		pros::Task::delay(50);
	}

	frontGrabberButton.setButtonStatus(POSITIVE);

	pros::Task::delay(100);

	liftButton.setAutonomousAuthority(1600);

	purePursuit.setSpeed(40);

	waitForDone();

	turn(-M_PI_2);

	liftButton.setAutonomousAuthority(100);

	pros::Task::delay(1000);

	balanceRobot(-M_PI_2);

	pros::Task::delay(500);

	printf("Skills path done\n");

	return 0;
}

/**
 * @brief Test auton
 *
 * @return 0
 */
int testOrientationAuton() {

	printf("Test Orientation Auton\n");

	bool grabFront = true;
	bool grabBack = true;

	if (grabFront) {
		frontGrabberButton.setButtonStatus(NEUTRAL);

		pros::Task::delay(500);

		frontGrabberButton.setButtonStatus(POSITIVE);

		pros::Task::delay(500);

		liftButton.setAutonomousAuthority(2000);

		pros::Task::delay(1000);
	}

	if (grabBack) {
		backGrabberChange(true);

		pros::Task::delay(500);
	}

	purePursuit.setCurrentPathIndex(zeroPathIndex);

	turn(M_PI);

	printf("Done with 180\n");

	turn(0);

	printf("Done with 0\n");

	pros::Task::delay(1000);

	return 0;
}

/**
 * @brief Test auton
 *
 * @return 0
 */
int tuneOdom() {

	printf("Test Auton\n");

	odometry.reset(new Position(0, 0, 0));

	purePursuit.setCurrentPathIndex(0);
	purePursuit.setSpeed(60);

	//purePursuit.setCurrentPathIndex(wiggleIndex);
	purePursuit.setCurrentPathIndex(testPathIndex);
	purePursuit.setFollowing(true);

	waitForDone();

	purePursuit.setFollowing(false);

	pros::Task::delay(500);

	return 0;
}

int testBalanceAuton() {
	odometry.reset(new Position(0, 0, M_PI_2));

	pros::Task::delay(200);

	frontGrabberButton.setButtonStatus(POSITIVE);

	pros::Task::delay(500);

	backGrabberChange(true);

	pros::Task::delay(500);

	balanceRobot(M_PI_2);

	return 0;
}

int autonTemplate() {
	odometry.reset(new Position(0, 0, 0));

	purePursuit.setSpeed(30);

	pros::Task::delay(500);

	return 0;
}

int postAuton() {
	purePursuit.setFollowing(false);
	purePursuit.setEnabled(false);

	frontGrabberButton.setAutonomous(false);
	backGrabberButton.setAutonomous(false);
	liftButton.setAutonomous(false);
	intakeButton.setAutonomous(false);

	balance.setEnabled(false);

	drivetrain.getLeftMotors().set_brake_mode(MOTOR_BRAKE_COAST);
	drivetrain.getRightMotors().set_brake_mode(MOTOR_BRAKE_COAST);

	return 0;
}

// !SECTION

// SECTION INIT
/**
 * Render thread to update items on the controller
 */
void renderThread() {
	master.renderFunc();
}

void updateDrivetrain() {

	while (1) {
		uint32_t startTime = pros::millis();
		odometry.update();

		purePursuit.update();
		balance.update();

		// Has to be in increments of 10 because of the VEX sensor update time
		pros::Task::delay_until(&startTime, 20);
	}
}

void updateDisplay() {

	// Odom
    lv_obj_t* odomTab = lv_tabview_add_tab(tabview, "Odom");
	lv_obj_t* odomLabel = lv_label_create(odomTab, NULL);

	// Odom
    lv_obj_t* purePursuitTab = lv_tabview_add_tab(tabview, "Pure pursuit");
	lv_obj_t* purePursuitLabel = lv_label_create(purePursuitTab, NULL);

	// Balance
    lv_obj_t* balanceTab = lv_tabview_add_tab(tabview, "Balance");
	lv_obj_t* balanceLabel = lv_label_create(balanceTab, NULL);

	// Drivetrain
    lv_obj_t* drivetrainTab = lv_tabview_add_tab(tabview, "Drivetrain");
	lv_obj_t* drivetrainTable = lv_table_create(drivetrainTab, NULL);

	lv_table_set_row_cnt(drivetrainTable, 3);
	lv_table_set_col_cnt(drivetrainTable, 2);

	lv_table_set_col_width(drivetrainTable, 0, 200);
	lv_table_set_col_width(drivetrainTable, 1, 200);

	while (1) {
		// Odometry
		lv_label_set_text(odomLabel, (odometry.getPosition()->to_string()
						 + "\nL: " + std::to_string(leftOdomWheel.getPosition()) + 
						 ", R: " + std::to_string(rightOdomWheel.getPosition())).c_str());
		
		// Balance
		lv_label_set_text(balanceLabel, ("Angle: " + std::to_string(balance.getLinearController()->getLastInput()) + "\n").c_str());

		// Drivetrain
		lv_table_set_cell_value(drivetrainTable, 0, 0, (std::to_string(frontLeftMotor.get_temperature()) + " C").c_str());
		lv_table_set_cell_value(drivetrainTable, 1, 0, (std::to_string(midLeftMotor.get_temperature()) + " C").c_str());
		lv_table_set_cell_value(drivetrainTable, 2, 0, (std::to_string(backLeftMotor.get_temperature()) + " C").c_str());
		lv_table_set_cell_value(drivetrainTable, 0, 1, (std::to_string(frontRightMotor.get_temperature()) + " C").c_str());
		lv_table_set_cell_value(drivetrainTable, 1, 1, (std::to_string(midRightMotor.get_temperature()) + " C").c_str());
		lv_table_set_cell_value(drivetrainTable, 2, 1, (std::to_string(backRightMotor.get_temperature()) + " C").c_str());

		// Pure pursuit
		lv_label_set_text(purePursuitLabel, ("Distance Remaining on path: " + std::to_string(purePursuit.getPointData().distanceFromEnd)).c_str());

		pros::Task::delay(100);
	}
}

void initDisplay() {
	pros::Task updateDisplayTask = pros::Task(updateDisplay);
}

/**
 * Initialize all sensors
 */
void initSensors() {
	printf("Initializing sensors\n");

	imu.reset();

	// Wait until IMU is calibrated
	// while (imu.is_calibrating()) {
	// 	pros::delay(20);
	// }

	//	printf("IMU calibrated\n");

	printf("Sensors initialized\n");
}

void updateMotors() {
	backGrabberButton.setAutonomous(true);
	backGrabberButton2.setAutonomous(true);

	while (1) {
		frontGrabberButton.update();
		frontHookButton.update();

		if (!((pros::c::competition_get_status() & COMPETITION_AUTONOMOUS) != 0)) {
			if (master.get_digital_new_press(DIGITAL_R1)) {
				backGrabberChange(!backButtonStatus);
			}
		}

		if (backButtonStatus && !backGrabberManual) {
			backGrabberButton2.setButtonStatus(POSITIVE);
			if (pros::millis() - lastChange > 100) {
				backGrabberButton.setButtonStatus(POSITIVE);
			}
		}
		else if (!backGrabberManual) {
			backGrabberButton.setButtonStatus(NEUTRAL);
			if (pros::millis() - lastChange > 150) {
				backGrabberButton2.setButtonStatus(NEUTRAL);
			}
		}

		backGrabberButton.update();
		backGrabberButton2.update();

		liftButton.update();

		intakeButton.update();

		pros::Task::delay(20);
	}
}

/**
 * Initialize all motors
 */
void initMotors() {
	// Motor brake modes
	frontLeftMotor.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
	frontRightMotor.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
	backLeftMotor.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
	backRightMotor.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
	lift.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);

	frontGrabberButton.setSolenoid(&frontGrabber);
	frontGrabberButton.setSingleToggle(true);
	frontGrabberButton.setButtonStatus(POSITIVE);

	frontHookButton.setSolenoid(&frontHook);
	frontHookButton.setSingleToggle(true);

	backGrabberButton.setSolenoid(&backTilter);
	backGrabberButton.setSingleToggle(true);

	backGrabberButton2.setSolenoid(&backGrabber);
	backGrabberButton2.setSingleToggle(true);

	intakeButton.setSingleToggle(true);
	intakeButton.setDejam(true);
	intakeButton.setDejamAuthority(-200);
	intakeButton.setDejamSpeed(25);
	intakeButton.setDejamTime(100);

	liftButton.setMultiplier(1.4);

	pros::Task updateButtons(updateMotors, "Update buttons");
}

void initDrivetrain() {
	printf("Init drivetrain\n");

	// odometry.setUseImu(false);
	// Left/Right fraction
	// 1.072124756
	// Left 99.57
	// Right 100.57
	double turningFactor = (((100.0 / 100.0) - 1.0) / 2);
	double tuningFactor = 1.010901883;
	leftOdomWheel.setRadius(2.75 / 2);
	leftOdomWheel.setTuningFactor(tuningFactor * (1 - turningFactor));
	rightOdomWheel.setRadius(2.75 / 2);
	rightOdomWheel.setTuningFactor(tuningFactor * (1 + turningFactor));

	leftEncoder.set_reversed(true);
	rightEncoder.set_reversed(false);

	odometry.setLeftOffset(3.303827647);
	odometry.setRightOffset(3.303827647);
	odometry.setBackOffset(0);

	odometry.setMaxMovement(0);

	purePursuit.setOutputMultiplier(600.0 / 65.0);
	purePursuit.setNormalizeDistance(10);
	purePursuit.setSpeed(45);
	purePursuit.setLookahead(15);
	purePursuit.setStopDistance(0.5);
	purePursuit.setMaxAcceleration(400);

	pros::Task::delay(10);

	pros::Task purePursuitTask = pros::Task(updateDrivetrain, "Pure Pursuit");

	odometry.reset(new Position());

	printf("Drivetrain Init done\n");
}

/**
 * Initialize the controller
 */
void initController() {
	master.setDrivetrain(&drivetrain);
	master.setOdometry(&odometry);
	partner.setDrivetrain(&drivetrain);
	partner.setOdometry(&odometry);
	pros::Task renderTask(renderThread);
}

// Run selector as task
void runSelector() {
	pros::Task::delay(1000);
	autonomousSelector.choose();
}

/**
 * Initialize the Auton Selector
 */
void initSelector() {
	autonomousSelector.addAuton(Auton("Test Orientation", testOrientationAuton));
	autonomousSelector.addAuton(Auton("Test Balance", testBalanceAuton));
	autonomousSelector.setDefaultAuton(0);
	autonomousSelector.setPreAuton(Auton("Pre auton", preAutonRun));
	autonomousSelector.setPreAuton(Auton("Post auton", postAuton));

	pros::Task selectorTask(runSelector, "Auton Selector");
}

/**
 * Initialize the logger for data collection after the match
 */
void initLogger() {

	Logger::setDefaultLogger(
		std::make_shared<Logger>(
			TimeUtilFactory::createDefault().getTimer(),
			"/usd/log.txt",
			Logger::LogLevel::debug // Show everything
			)
	);
	Logger::getDefaultLogger()->debug<std::string>("LOGGER: Logger initialized");
}

/**
 * Filter and apply the quadratic function.
 */
double filterAxis(pros::Controller controller, pros::controller_analog_e_t controllerAxis) {
	// Remove drift
	double controllerValue = controller.get_analog(controllerAxis);
	double controllerFilter = abs(controllerValue) < DRIFT_MIN ? 0.0 : controllerValue;

	// Apply quadratic function 
	// f(x) = controllerFilter / 127.0 ^ 3 * 127.0
	double quadraticFilter = pow(controllerFilter / 127.0, 3) * 620;

	// Return solution
	return quadraticFilter;
}

/**
 * Runs when the robot starts up
 */
void initialize() {

	printf("Initialize");

	lv_init();
	tabview = lv_tabview_create(lv_scr_act(), NULL);

	printf("LVGL Init");

	// Initialize functions
	initSensors();
	initMotors();
	autoPaths(&purePursuit);
	initDrivetrain();
	initController();
	initLogger();
	initDisplay();
	// initSelector();

	printf("Init done\n");
}

// !SECTION

// SECTION Disabled
/**
 * Runs while the robot is disabled i.e. before and after match, between auton
 * and teleop period
 */
void disabled() {

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
	// autonomousSelector.choose();

}

// !SECTION

// SECTION Auton

/**
 * Runs during the autonomous. NO user control
 */
void autonomous() {
	// This calls the user selection, all the functions prototypes are in 
	// autonRoutines.hpp and the implementation is autonRoutines.cpp
	// autonomousSelector.run();
	preAutonRun();
	skills();
	postAuton();

	// autonomousSelector.run();
}

// !SECTION

// SECTION Operator Control

/**
 * Runs during operator/teleop control
 */
void opcontrol() {

	printf("OpControl");

	postAuton();

	if (!partner.is_connected()) {
		frontGrabberButton.setButtonStatus(POSITIVE);
	}
	else {
		frontGrabberButton.setButtonStatus(NEUTRAL);
	}

	liftButton.setAutonomous(true);
	liftButton.setAutonomousPosition(false);

	intakeButton.setAutonomous(true);

	//lv_obj_t* infoLabel = lv_label_create(lv_scr_act(), NULL);
	// lv_label_set_text(infoLabel, "");

	// Driver Control Loop
	while (true) {

		if (driverMode > 0 && !balance.isEnabled()) {
			// Filter and calculate magnitudes
			int leftY = filterAxis(master, ANALOG_LEFT_Y);
			int rightX = filterAxis(master, ANALOG_RIGHT_X);

			drivetrain.skidSteerVelocity(leftY, rightX);
		}
		else if (!balance.isEnabled()) {
			int leftY = filterAxis(master, ANALOG_LEFT_Y);
			int rightY = filterAxis(master, ANALOG_RIGHT_Y);

			drivetrain.tankSteerVelocity(leftY, rightY);
		}

		if (frontGrabberBumperSwitch.get_new_press()) {
			frontGrabberButton.setButtonStatus(Pronounce::ButtonStatus::POSITIVE);
		}

		if (backGrabberBumperSwitch.get_new_press()) {
			backGrabberButton.setButtonStatus(Pronounce::ButtonStatus::POSITIVE);
		}


		if (master.get_digital_new_press(DIGITAL_RIGHT)) {
			drivetrain.getLeftMotors().set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			drivetrain.getRightMotors().set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		}
		else if (master.get_digital_new_press(DIGITAL_LEFT)) {
			drivetrain.getLeftMotors().set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			drivetrain.getRightMotors().set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		}

		if (master.get_digital_new_press(DIGITAL_B)) {
			balance.setEnabled(!balance.isEnabled());
			balance.getOrientationController()->setTarget(imu.get_heading());
			drivetrain.getLeftMotors().set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			drivetrain.getRightMotors().set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		}

		// if (master.get_digital_new_press(DIGITAL_X)) {
		// 	odometry.reset(new Position());
		// }

		if (master.get_digital_new_press(DIGITAL_UP)) {
			driverMode = 0;
		}
		if (master.get_digital_new_press(DIGITAL_DOWN)) {
			driverMode = 2;
		}

		if (partner.is_connected()) {
			if ((partner.get_digital(DIGITAL_L1) && !master.get_digital(DIGITAL_L2) && !partner.get_digital(DIGITAL_L2)) || master.get_digital(DIGITAL_L1) && !(master.get_digital(DIGITAL_L1) && master.get_digital(DIGITAL_L2))) {
				liftButton.setButtonStatus(Pronounce::ButtonStatus::POSITIVE);
			}
			else if ((partner.get_digital(DIGITAL_L2) && !master.get_digital(DIGITAL_L1) && !partner.get_digital(DIGITAL_L1)) || master.get_digital(DIGITAL_L2) && !(master.get_digital(DIGITAL_L1) && master.get_digital(DIGITAL_L2))) {
				liftButton.setButtonStatus(Pronounce::ButtonStatus::NEGATIVE);
			}
			else {
				liftButton.setButtonStatus(Pronounce::ButtonStatus::NEUTRAL);
			}

			if (master.get_digital_new_press(DIGITAL_R2) || partner.get_digital_new_press(DIGITAL_R2)) {
				intakeButton.setButtonStatus(intakeButton.getButtonStatus() == POSITIVE ? NEUTRAL : POSITIVE);
			}
			else if (master.get_digital_new_press(DIGITAL_Y) || partner.get_digital_new_press(DIGITAL_R1)) {
				intakeButton.setButtonStatus(intakeButton.getButtonStatus() == NEGATIVE ? NEUTRAL : NEGATIVE);
			}
		}
		else {
			if (master.get_digital(DIGITAL_L1) && !(master.get_digital(DIGITAL_L1) && master.get_digital(DIGITAL_L2))) {
				liftButton.setButtonStatus(Pronounce::ButtonStatus::POSITIVE);
			}
			else if (master.get_digital(DIGITAL_L2) && !(master.get_digital(DIGITAL_L1) && master.get_digital(DIGITAL_L2))) {
				liftButton.setButtonStatus(Pronounce::ButtonStatus::NEGATIVE);
			}
			else {
				liftButton.setButtonStatus(Pronounce::ButtonStatus::NEUTRAL);
			}

			if (master.get_digital_new_press(DIGITAL_R2)) {
				intakeButton.setButtonStatus(intakeButton.getButtonStatus() == POSITIVE ? NEUTRAL : POSITIVE);
			}
			else if (master.get_digital_new_press(DIGITAL_Y)) {
				intakeButton.setButtonStatus(intakeButton.getButtonStatus() == NEGATIVE ? NEUTRAL : NEGATIVE);
			}
		}

		pros::delay(10);
	}
}

// !SECTION
