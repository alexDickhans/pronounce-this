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
pros::ADIDigitalOut brakes(5, false);

// Inertial Measurement Unit
pros::Imu imu(19);

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

TankDrivetrain drivetrain(&frontLeftMotor, &frontRightMotor, &midLeftMotor, &midRightMotor, &backLeftMotor, &backRightMotor, &imu, 8.0);

Pronounce::TankPurePursuit purePursuit(&drivetrain, &odometry, new PID(0.6, 0, 2.0), 20);

Balance balance(&drivetrain, &imu, new BangBang(21, true, -150), new PID(0, 0, 0));

MotorButton liftButton(&master, &lift, DIGITAL_L1, DIGITAL_L2, 200, 0, -200, 0, 0);
MotorButton intakeButton(&master, &intake, DIGITAL_R2, DIGITAL_Y, 200, 0, -200, 0, 0);

SolenoidButton frontGrabberButton(&master, DIGITAL_A);
SolenoidButton frontHookButton(&master, DIGITAL_X);
SolenoidButton backGrabberButton(&master, DIGITAL_R1, DIGITAL_R1);
SolenoidButton backGrabberButton2(&master, DIGITAL_R1, DIGITAL_R1);
SolenoidButton brakesButton(&master, DIGITAL_B);

pros::Vision clawVision(18, VISION_ZERO_CENTER);

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

double goalAngle() {
	if (clawVision.get_object_count() < 1) {
		return 0.0;
	}

	int x = clawVision.get_by_sig(0, 1).x_middle_coord;

	double angle = 0.0033908 * (double)x + 0.064622;

	return angle;
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
		pros::Task::delay(20);
	}
}

void waitForDone() {
	waitForDone(0.5, 15000);
}

void waitForDoneOrientation() {
	pros::Task::delay(100);

	uint32_t startTime = pros::millis();

	pros::Task::delay(300);

	while (!purePursuit.isDoneOrientation(0.1) && pros::millis() - startTime < 5000) {
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

	waitForDone(0.5, 6000);

	purePursuit.setEnabled(false);
	purePursuit.setFollowing(false);

	liftButton.setAutonomousAuthority(1300);

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

	drivetrain.skidSteerVelocity(300, 0);

	pros::Task::delay(300);

	liftButton.setAutonomousAuthority(600);

	pros::Task::delay(1200);

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

	pros::Task::delay(300);

	double angle = goalAngle();

	frontHookButton.setButtonStatus(POSITIVE);

	pros::Task::delay(200);

	drivetrain.skidSteerVelocity(-600, 0);

	pros::Task::delay(150);

	drivetrain.skidSteerVelocity(0, 0);

	frontHookButton.setButtonStatus(NEUTRAL);

	purePursuit.setEnabled(true);
	purePursuit.setFollowing(true);

	purePursuit.setSpeed(60);

	turn(odometry.getPosition()->getTheta() + angle);

	pros::Task::delay(200);

	purePursuit.setEnabled(false);
	purePursuit.setFollowing(false);

	pros::Task::delay(200);

	drivetrain.skidSteerVelocity(350, 0);

	pros::Task::delay(550);

	frontGrabberButton.setButtonStatus(POSITIVE);

	pros::Task::delay(50);

	drivetrain.skidSteerVelocity(0, 0);

	pros::Task::delay(400);
	liftButton.setAutonomousAuthority(200);

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
	brakesButton.setAutonomous(true);

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
	purePursuit.setSpeed(75);

	frontGrabberButton.setButtonStatus(NEUTRAL);

	double oldLookahead = purePursuit.getLookahead();
	double oldAccel = purePursuit.getMaxAcceleration();

	purePursuit.setLookahead(30);
	purePursuit.setMaxAcceleration(oldAccel * 2);

	purePursuit.setUseVoltage(true);

	purePursuit.setEnabled(true);
	purePursuit.setFollowing(true);

	purePursuit.setCurrentPathIndex(leftHomeZoneToLeftNeutralStealIndex);

	waitForDone();

	frontGrabberButton.setButtonStatus(POSITIVE);

	pros::Task::delay(200);

	purePursuit.setCurrentPathIndex(leftNeutralStealToLeftHomeZoneIndex);

	purePursuit.setSpeed(65);

	pros::Task::delay(800);

	double lastY = odometry.getPosition()->getY();

	while (odometry.getPosition()->getY() > 45) {
		double y = odometry.getPosition()->getY();

		if (y - lastY > -0.005) {
			brakesButton.setButtonStatus(POSITIVE);
			purePursuit.setEnabled(false);
			purePursuit.setFollowing(false);
			drivetrain.getLeftMotors().set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			drivetrain.getRightMotors().set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			liftButton.setAutonomousAuthority(0);
		}

		lastY = y;

		pros::Task::delay(60);
	}

	purePursuit.setUseVoltage(false);

	purePursuit.setSpeed(45);

	waitForDone();

	purePursuit.setLookahead(10);
	purePursuit.setMaxAcceleration(oldAccel);

	brakesButton.setButtonStatus(NEUTRAL);

	grabFromHook();
}

void midStealClaw() {
	purePursuit.setSpeed(70);

	frontGrabberButton.setButtonStatus(NEUTRAL);

	double oldLookahead = purePursuit.getLookahead();
	double oldAccel = purePursuit.getMaxAcceleration();

	purePursuit.setLookahead(20);
	purePursuit.setMaxAcceleration(oldAccel * 1.5);

	purePursuit.setUseVoltage(true);

	purePursuit.setEnabled(true);
	purePursuit.setFollowing(true);

	purePursuit.setCurrentPathIndex(rightHomeZoneToMidNeutralIndex);

	waitForDone();

	frontGrabberButton.setButtonStatus(POSITIVE);

	liftButton.setAutonomousAuthority(400);

	purePursuit.setLookahead(oldLookahead);
	purePursuit.setMaxAcceleration(oldAccel);

	pros::Task::delay(200);

	purePursuit.setCurrentPathIndex(midNeutralToRightHomeZoneIndex);

	pros::Task::delay(800);

	double lastY = odometry.getPosition()->getY();

	while (odometry.getPosition()->getY() > 50) {
		double y = odometry.getPosition()->getY();

		if (y - lastY > -0.002) {
			brakesButton.setButtonStatus(POSITIVE);
			purePursuit.setEnabled(false);
			purePursuit.setFollowing(false);
			drivetrain.getLeftMotors().set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			drivetrain.getRightMotors().set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			liftButton.setAutonomousAuthority(0);
		}

		lastY = y;

		pros::Task::delay(60);
	}

	waitForDone(20);

	purePursuit.setSpeed(0);
}

void midSteal() {
	purePursuit.setSpeed(70);

	frontGrabberButton.setButtonStatus(NEUTRAL);
	frontHookButton.setButtonStatus(POSITIVE);

	double oldLookahead = purePursuit.getLookahead();
	double oldAccel = purePursuit.getMaxAcceleration();

	purePursuit.setLookahead(20);
	purePursuit.setMaxAcceleration(oldAccel * 1.5);

	purePursuit.setUseVoltage(true);

	purePursuit.setEnabled(true);
	purePursuit.setFollowing(true);

	purePursuit.setCurrentPathIndex(rightHomeZoneToMidNeutralIndex);

	waitForDone();

	frontHookButton.setButtonStatus(NEUTRAL);

	purePursuit.setLookahead(oldLookahead);
	purePursuit.setMaxAcceleration(oldAccel);

	purePursuit.setCurrentPathIndex(midNeutralToRightHomeZoneIndex);

	purePursuit.setUseVoltage(false);

	purePursuit.setSpeed(50);

	pros::Task::delay(200);

	double lastY = odometry.getPosition()->getY();

	while (odometry.getPosition()->getY() > 50) {
		double y = odometry.getPosition()->getY();

		if (y - lastY > 0.3) {
			brakesButton.setButtonStatus(POSITIVE);
		}

		if (odometry.getPosition()->getY() < 50) {
			purePursuit.setSpeed(40);
		}

		lastY = y;

		pros::Task::delay(60);
	}

	purePursuit.setUseVoltage(false);

	for (int i = 5; i > 2; i--) {
		purePursuit.setSpeed(i * 10);
		pros::Task::delay(300);
	}

	waitForDone();

	purePursuit.setLookahead(oldLookahead);
	purePursuit.setMaxAcceleration(oldAccel);

	brakesButton.setButtonStatus(NEUTRAL);

	grabFromHook();
}

void rightSteal() {
	purePursuit.setSpeed(70);

	frontGrabberButton.setButtonStatus(NEUTRAL);
	frontHookButton.setButtonStatus(POSITIVE);

	double oldLookahead = purePursuit.getLookahead();
	double oldAccel = purePursuit.getMaxAcceleration();

	purePursuit.setLookahead(30);
	purePursuit.setMaxAcceleration(oldAccel * 2);

	purePursuit.setUseVoltage(true);

	purePursuit.setEnabled(true);
	purePursuit.setFollowing(true);

	purePursuit.setCurrentPathIndex(rightHomeZoneToRightNeutralIndex);

	waitForDone(1);

	frontHookButton.setButtonStatus(NEUTRAL);

	waitForDone();

	purePursuit.setCurrentPathIndex(rightNeutralToRightHomeZoneIndex);

	purePursuit.setSpeed(65);

	pros::Task::delay(800);

	double lastY = odometry.getPosition()->getY();

	while (odometry.getPosition()->getY() > 40) {
		double y = odometry.getPosition()->getY();

		if (y - lastY > -0.005) {
			brakesButton.setButtonStatus(POSITIVE);
			purePursuit.setEnabled(false);
			purePursuit.setFollowing(false);
			drivetrain.getLeftMotors().set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			drivetrain.getRightMotors().set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		}

		lastY = y;

		pros::Task::delay(60);
	}

	purePursuit.setUseVoltage(false);

	purePursuit.setSpeed(45);

	waitForDone();

	purePursuit.setLookahead(oldLookahead);
	purePursuit.setMaxAcceleration(oldAccel);

	brakesButton.setButtonStatus(NEUTRAL);

	grabFromHook();
}

void rightStealClaw() {
	purePursuit.setSpeed(70);

	frontGrabberButton.setButtonStatus(NEUTRAL);

	double oldLookahead = purePursuit.getLookahead();
	double oldAccel = purePursuit.getMaxAcceleration();

	purePursuit.setLookahead(30);
	purePursuit.setMaxAcceleration(oldAccel * 2);

	purePursuit.setUseVoltage(true);

	purePursuit.setEnabled(true);
	purePursuit.setFollowing(true);

	purePursuit.setCurrentPathIndex(rightHomeZoneToRightNeutralIndex);

	waitForDone(1);

	frontGrabberButton.setButtonStatus(POSITIVE);

	waitForDone();

	purePursuit.setCurrentPathIndex(rightNeutralToRightHomeZoneIndex);

	purePursuit.setSpeed(65);

	pros::Task::delay(800);

	double lastY = odometry.getPosition()->getY();

	while (odometry.getPosition()->getY() > 45) {
		double y = odometry.getPosition()->getY();

		if (y - lastY > -0.005) {
			brakesButton.setButtonStatus(POSITIVE);
			purePursuit.setEnabled(false);
			purePursuit.setFollowing(false);
			drivetrain.getLeftMotors().set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			drivetrain.getRightMotors().set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			liftButton.setAutonomousAuthority(0);
		}

		lastY = y;

		pros::Task::delay(60);
	}

	purePursuit.setUseVoltage(false);

	purePursuit.setSpeed(0);

	purePursuit.setLookahead(oldLookahead);
	purePursuit.setMaxAcceleration(oldAccel);

	brakesButton.setButtonStatus(NEUTRAL);
}

void rightRightSteal() {
	purePursuit.setSpeed(70);

	frontGrabberButton.setButtonStatus(NEUTRAL);
	frontHookButton.setButtonStatus(POSITIVE);

	double oldLookahead = purePursuit.getLookahead();
	double oldAccel = purePursuit.getMaxAcceleration();

	purePursuit.setLookahead(20);
	purePursuit.setMaxAcceleration(oldAccel * 2);

	purePursuit.setUseVoltage(true);

	purePursuit.setEnabled(true);
	purePursuit.setFollowing(true);

	purePursuit.setCurrentPathIndex(farRightHomeZoneToRightNeutralIndex);

	waitForDone(1);

	frontHookButton.setButtonStatus(NEUTRAL);

	waitForDone();

	purePursuit.setCurrentPathIndex(rightNeutralToFarRightHomeZoneIndex);

	purePursuit.setSpeed(65);

	pros::Task::delay(200);

	double lastY = odometry.getPosition()->getY();

	while (odometry.getPosition()->getY() > 30) {
		double y = odometry.getPosition()->getY();

		if (y - lastY > 0.3) {
			brakesButton.setButtonStatus(POSITIVE);
		}

		lastY = y;

		pros::Task::delay(60);
	}

	purePursuit.setUseVoltage(false);

	purePursuit.setSpeed(45);

	waitForDone();

	purePursuit.setLookahead(oldLookahead);
	purePursuit.setMaxAcceleration(oldAccel);

	brakesButton.setButtonStatus(NEUTRAL);

	grabFromHook();
}

void rightMidSteal() {
	purePursuit.setSpeed(70);

	frontGrabberButton.setButtonStatus(NEUTRAL);
	frontHookButton.setButtonStatus(POSITIVE);

	double oldLookahead = purePursuit.getLookahead();
	double oldAccel = purePursuit.getMaxAcceleration();

	purePursuit.setLookahead(20);
	purePursuit.setMaxAcceleration(oldAccel * 2);

	purePursuit.setUseVoltage(true);

	purePursuit.setEnabled(true);
	purePursuit.setFollowing(true);

	purePursuit.setCurrentPathIndex(rightHomeZoneToMidNeutralIndex);

	waitForDone(1);

	frontHookButton.setButtonStatus(NEUTRAL);

	waitForDone();

	purePursuit.setLookahead(oldLookahead);

	purePursuit.setCurrentPathIndex(midNeutralToRightHomeZoneIndex);

	purePursuit.setSpeed(65);

	purePursuit.setSpeed(65);

	pros::Task::delay(800);

	double lastY = odometry.getPosition()->getY();

	while (odometry.getPosition()->getY() > 45) {
		double y = odometry.getPosition()->getY();

		if (y - lastY > -0.005) {
			brakesButton.setButtonStatus(POSITIVE);
			purePursuit.setEnabled(false);
			purePursuit.setFollowing(false);
			drivetrain.getLeftMotors().set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			drivetrain.getRightMotors().set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			liftButton.setAutonomousAuthority(0);
		}

		lastY = y;

		pros::Task::delay(60);
	}

	purePursuit.setUseVoltage(false);

	purePursuit.setSpeed(45);

	waitForDone();

	purePursuit.setLookahead(oldLookahead);
	purePursuit.setMaxAcceleration(oldAccel);

	brakesButton.setButtonStatus(NEUTRAL);

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
	purePursuit.setCurrentPathIndex(enterRightHomeZoneToRightAllianceIndex);

	waitForDone(20, 2000);

	purePursuit.setSpeed(40);

	liftButton.setAutonomousAuthority(600);

	waitForDone(0.1, 2000);

	backGrabberChange(true);

	pros::Task::delay(100);

	purePursuit.setSpeed(60);

	turn(-M_PI_4 * 0.5);

	purePursuit.setSpeed(30);

	purePursuit.setCurrentPathIndex(rightAllianceToRightRingsIndex);

	liftButton.setAutonomousAuthority(2000);

	pros::Task::delay(300);

	intakeButton.setButtonStatus(POSITIVE);

	waitForDone(0.1, 3000);

	purePursuit.setSpeed(60);

	purePursuit.setCurrentPathIndex(rightRingsToRightHomeZoneIndex);

	waitForDone(5);

	backGrabberChange(false);

	waitForDone();
}

void rightToLeftAWP() {

	purePursuit.setSpeed(65);

	turn(-M_PI_2 * 0.6);

	purePursuit.setSpeed(50);

	purePursuit.setCurrentPathIndex(rightHomeZoneToLeftHomeZoneIndex);

	waitForDone();

	turn(-M_PI_4 * 0.7);

	purePursuit.setCurrentPathIndex(rightHomeZoneToLeftAllianceIndex);

	waitForDone(8);

	purePursuit.setSpeed(20);

	waitForDone();

	backGrabberChange(true);

	pros::Task::delay(800);

	liftButton.setAutonomousAuthority(600);

	pros::Task::delay(100);

	purePursuit.setSpeed(20);

	purePursuit.setCurrentPathIndex(leftAllianceToRightHomeZoneIndex);

	pros::Task::delay(200);

	intakeButton.setButtonStatus(POSITIVE);

	pros::Task::delay(500);

	backGrabberChange(false);
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
	odometry.reset(new Position(27, 18, toRadians(10)));

	pros::Task deployBack(deployBackGrabber);

	leftSteal();

	purePursuit.setSpeed(60);

	turn(-M_PI_4 * 0.5);

	purePursuit.setSpeed(30);

	purePursuit.setCurrentPathIndex(leftHomeZoneToLeftAllianceIndex);

	waitForDone();

	backGrabberChange(true);

	leftAWP();

	pros::Task::delay(1000);

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

	rightStealClaw();

	purePursuit.setSpeed(40);
	turn(-M_PI_2 * 1.3);

	rightAWP();

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

	turn(-M_PI_2 * 1.3);

	rightAWP();

	backGrabberChange(false);

	pros::Task::delay(200);

	return 0;
}

/**
 * @brief Steal the mid goal from the right side and get the Left AWP
 *
 * @return int
 */
int midStealToLeftAWP() {
	odometry.reset(new Position(99.5, 18, toRadians(-30)));

	pros::Task::delay(200);

	pros::Task deployBack(deployBackGrabber);

	midSteal();

	rightToLeftAWP();

	return 0;
}

/**
 * @brief Steal the mid goal from the right side and get the right AWP
 *
 * @return int
 */
int midStealToRightAWP() {
	odometry.reset(new Position(99.5, 18, toRadians(-30)));

	pros::Task deployBack(deployBackGrabber);

	midStealClaw();

	purePursuit.setSpeed(60);

	turn(-M_PI_2 * 1.15);

	rightAWP();

	backGrabberChange(false);

	return 0;
}

int midFakeAWP() {
	odometry.reset(new Position(99.5, 18, toRadians(-30)));

	pros::Task deployBack(deployBackGrabber);

	purePursuit.setEnabled(false);
	purePursuit.setFollowing(false);

	drivetrain.skidSteerVelocity(300, 0);

	pros::Task::delay(200);

	drivetrain.skidSteerVelocity(0, 0);

	purePursuit.setEnabled(true);
	purePursuit.setFollowing(true);

	purePursuit.setSpeed(60);

	turn(-M_PI_2 * 1.5);

	return 0;
}

int rightDoubleSteal() {
	odometry.reset(new Position(99.5, 18, toRadians(-30)));

	pros::Task deployBack(deployBackGrabber);

	purePursuit.setSpeed(70);

	frontGrabberButton.setButtonStatus(NEUTRAL);
	frontHookButton.setButtonStatus(POSITIVE);

	double oldLookahead = purePursuit.getLookahead();
	double oldAccel = purePursuit.getMaxAcceleration();

	purePursuit.setLookahead(20);
	purePursuit.setMaxAcceleration(oldAccel * 1.5);

	purePursuit.setUseVoltage(true);

	purePursuit.setEnabled(true);
	purePursuit.setFollowing(true);

	purePursuit.setCurrentPathIndex(rightHomeZoneToMidNeutralIndex);

	waitForDone();

	frontHookButton.setButtonStatus(NEUTRAL);

	purePursuit.setLookahead(oldLookahead);
	purePursuit.setMaxAcceleration(oldAccel);

	purePursuit.setCurrentPathIndex(midNeutralToRightHomeZoneIndex);

	purePursuit.setUseVoltage(false);

	purePursuit.setSpeed(50);

	pros::Task::delay(200);

	double lastY = odometry.getPosition()->getY();

	while (odometry.getPosition()->getY() > 50) {
		double y = odometry.getPosition()->getY();

		if (y - lastY > -0.05) {
			brakesButton.setButtonStatus(POSITIVE);
		}

		if (odometry.getPosition()->getY() < 50) {
			purePursuit.setSpeed(40);
		}

		lastY = y;

		pros::Task::delay(60);
	}

	purePursuit.setUseVoltage(false);

	for (int i = 5; i > 2; i--) {
		purePursuit.setSpeed(i * 10);
		pros::Task::delay(300);
	}

	waitForDone(5);

	frontHookButton.setButtonStatus(POSITIVE);

	waitForDone();

	frontHookButton.setButtonStatus(NEUTRAL);

	purePursuit.setLookahead(oldLookahead);
	purePursuit.setMaxAcceleration(oldAccel);

	brakesButton.setButtonStatus(NEUTRAL);

	turn(0);

	purePursuit.setSpeed(60);

	purePursuit.setCurrentPathIndex(rightHomeZoneToRightNeutralClawIndex);

	waitForDone();

	frontGrabberButton.setButtonStatus(POSITIVE);

	pros::Task::delay(100);

	purePursuit.setCurrentPathIndex(rightHomeZoneToRightNeutralClawIndex);

	waitForDone();

	turn(-M_PI_2);

	rightAWP();

	return 0;
}

int leftAwpMidSteal() {
	odometry.reset(new Position(26.5, 11, -M_PI_2));

	backGrabberButton.setButtonStatus(ButtonStatus::NEUTRAL);
	frontGrabberButton.setButtonStatus(ButtonStatus::NEUTRAL);

	purePursuit.setMaxAcceleration(65);

	liftButton.setAutonomousAuthority(600);

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

	pros::Task::delay(200);

	drivetrain.skidSteerVelocity(400, 0);

	purePursuit.setFollowing(true);
	purePursuit.setEnabled(true);

	intakeButton.setButtonStatus(POSITIVE);

	purePursuit.setCurrentPathIndex(leftAllianceToRightHomeZoneIndex);

	purePursuit.setSpeed(50);

	pros::Task::delay(1700);

	purePursuit.setSpeed(15);

	while (odometry.getPosition()->getX() < 70) {
		pros::Task::delay(50);
	}

	turn(0);

	backGrabberChange(false);

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

	while (odometry.getPosition()->getX() < 55) {
		pros::Task::delay(50);
	}

	purePursuit.setSpeed(30);

	waitForDone(0.5, 3000);

	purePursuit.setSpeed(60);

	turn(0, 0.1);

	purePursuit.setFollowing(false);

	placeOnPlatform();

	// Far platform 
	purePursuit.setCurrentPathIndex(farPlatformToEnterFarHomeZoneIndex);

	waitForDone();

	liftButton.setAutonomousAuthority(0);

	purePursuit.setSpeed(30);

	turn(M_PI * 0.8);

	purePursuit.setCurrentPathIndex(enterFarHomeZoneToMidGoalIndex);

	intakeButton.setButtonStatus(POSITIVE);

	// Slow down on the way to the tall goal

	purePursuit.setSpeed(30);

	waitForDone(30);

	purePursuit.setSpeed(20);

	waitForDone();

	// Grab the middle goal
	frontGrabberButton.setButtonStatus(POSITIVE);

	pros::Task::delay(300);

	liftButton.setAutonomousAuthority(600);

	pros::Task::delay(500);

	purePursuit.setSpeed(60);

	turn(0);

	turn(0);

	// Place the middle goal on the platform
	purePursuit.setCurrentPathIndex(midGoalToFarPlatformIndex);

	liftButton.setAutonomousAuthority(2000);

	purePursuit.setSpeed(40);

	placeOnPlatform();

	// Move to drop off the alliance goal
	purePursuit.setCurrentPathIndex(farPlatformToRightAllianceDropOffIndex);

	waitForDone();

	liftButton.setAutonomousAuthority(0);

	// Flip goal to front
	{
		backGrabberChange(false);

		pros::Task::delay(400);

		purePursuit.setEnabled(false);
		purePursuit.setFollowing(false);

		drivetrain.skidSteerVelocity(300, 0);

		pros::Task::delay(150);

		purePursuit.setEnabled(true);
		purePursuit.setFollowing(true);

		turn(odometry.getPosition()->getTheta() + M_PI);

		purePursuit.setEnabled(false);
		purePursuit.setFollowing(false);

		pros::Task::delay(2000);

		drivetrain.skidSteerVelocity(300, 0);

		pros::Task::delay(400);

		purePursuit.setEnabled(true);
		purePursuit.setFollowing(true);

		frontGrabberButton.setButtonStatus(POSITIVE);
	}

	liftButton.setAutonomousAuthority(2000);

	turn(0);

	// Place near alliance goal on platform
	purePursuit.setCurrentPathIndex(rightAllianceToFarPlatformIndex);

	waitForDone();

	turn(0);

	placeOnPlatform();

	// Move to the far right alliance goal
	purePursuit.setCurrentPathIndex(platformToEnterFarRightHomeZoneIndex);

	waitForDone();

	turn(M_PI_2);

	purePursuit.setSpeed(40);

	purePursuit.setCurrentPathIndex(farPlatformToFarLeftAllianceGoalIndex);

	waitForDone(20, 4000);

	purePursuit.setSpeed(20);

	waitForDone(0.5, 4000);

	backGrabberChange(true);

	pros::Task::delay(500);

	purePursuit.setSpeed(40);

	purePursuit.setCurrentPathIndex(farLeftAllianceGoalToRightNeutralGoalIndex);

	liftButton.setAutonomousAuthority(600);

	waitForDone(40);

	purePursuit.setSpeed(20);

	liftButton.setAutonomousAuthority(0);

	waitForDone();

	frontGrabberButton.setButtonStatus(POSITIVE);

	pros::Task::delay(150);

	liftButton.setAutonomousAuthority(2000);

	turn(-M_PI_2 - M_PI_4);

	purePursuit.setSpeed(40);

	purePursuit.setCurrentPathIndex(rightNeutralGoalToNearRIghtPlatformIndex);

	waitForDone();

	pros::Task::delay(500);

	turn(M_PI_2);

	liftButton.setAutonomousAuthority(100);

	pros::Task::delay(2000);

	balanceRobot(M_PI_2);

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

	frontGrabberButton.setButtonStatus(POSITIVE);

	pros::Task::delay(200);

	backGrabberChange(true);

	liftButton.setAutonomousAuthority(2000);

	pros::Task::delay(3000);

	liftButton.setAutonomousAuthority(0);

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
	brakesButton.setAutonomous(false);

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

void lvChartAddValue(lv_chart_series_t* chartSeries, int size, int value) {
	for (int i = 0; i < size - 1; i++) {
		chartSeries->points[i] = chartSeries->points[i + 1];
	}

	chartSeries->points[size - 1] = value;
}

void chartInit(lv_chart_series_t* chartSeries, int size) {
	for (int i = 0; i < size; i++) {
		chartSeries->points[i] = 0;
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

	// Intake
	lv_obj_t* intakeTab = lv_tabview_add_tab(tabview, "Intake");
	lv_obj_t* intakeChart = lv_chart_create(intakeTab, NULL);
	lv_obj_set_size(intakeChart, 200, 140);
	lv_chart_set_type(intakeChart, LV_CHART_TYPE_LINE);

	lv_chart_series_t* intakeInputSpeed = lv_chart_add_series(intakeChart, LV_COLOR_BLUE);
	lv_chart_series_t* intakeOutputSpeed = lv_chart_add_series(intakeChart, LV_COLOR_RED);

	// Vision
	lv_obj_t* visionTab = lv_tabview_add_tab(tabview, "Vision");
	lv_obj_t* visionLabel = lv_label_create(visionTab, NULL);
	lv_obj_t* visionResultLabel = lv_label_create(visionTab, NULL);

	//chartInit(intakeInputSpeed, 50);
	//chartInit(intakeOutputSpeed, 50);
//
	//pros::Task::delay(100);

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

		// Intake
		//addValue(intakeInputSpeed, 50, intake.get_target_velocity());
		//addValue(intakeOutputSpeed, 50, intake.get_actual_velocity());

		lv_label_set_text(visionLabel, ("X coordinate:" + std::to_string(clawVision.get_by_sig(0, 1).x_middle_coord) + "\nAngle: " + std::to_string(toDegrees(goalAngle())) + "\nError: " + std::to_string(toDegrees(goalAngle() + odometry.getPosition()->getTheta()))).c_str());

		//lv_chart_refresh(intakeChart);

		pros::Task::delay(100);
	}
}

void initDisplay() {
	pros::Task updateDisplayTask = pros::Task(updateDisplay, "Update dispay");
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
			if (pros::millis() - lastChange > 150) {
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

		brakesButton.update();

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

	brakesButton.setSolenoid(&brakes);
	brakesButton.setSingleToggle(true);

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
	double turningFactor = (((100.35 / 100.0) - 1.0) / 2);
	double tuningFactor = 0.998791278;
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

void initVision() {
	pros::vision_signature_s_t YELLOW_GOAL = pros::Vision::signature_from_utility(1, 2415, 3681, 3048, -3761, -3379, -3570, 3.000, 0);
	clawVision.set_signature(0, &YELLOW_GOAL);
	clawVision.set_auto_white_balance(0);
	clawVision.set_exposure(33);
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
	initVision();
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

#if AUTON == AUTON_STEAL_LEFT_LEFT_AWP
	leftStealLeftAWP();
#elif AUTON == AUTON_STEAL_MID_LEFT_AWP
	midStealToLeftAWP();
#elif AUTON == AUTON_STEAL_MID_RIGHT_AWP
	midStealToRightAWP();
#elif AUTON == AUTON_STEAL_RIGHT_RIGHT_AWP
	rightStealRightAWP();
#elif AUTON == AUTON_STEAL_RIGHT_RIGHT_RIGHT_AWP
	rightRightStealToRightAWP();
#elif AUTON == AUTON_RIGHT_DOUBLE_STEAL
	rightDoubleSteal();
#elif AUTON == AUTON_NONE
	// Do Nothing
#elif AUTON == AUTON_TEST
	testBalanceAuton();
#elif AUTON == AUTON_SKILLS
	skills();
#endif // AUTON

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

#if AUTON != AUTON_SKILLS
	frontGrabberButton.setButtonStatus(POSITIVE);
#else
	frontGrabberButton.setButtonStatus(NEUTRAL);
#endif

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

			if (brakesButton.getButtonStatus() == POSITIVE) {
				drivetrain.skidSteerVelocity(leftY < 0 ? leftY : 0, rightX);
			}
			else {
				drivetrain.skidSteerVelocity(leftY, rightX);
			}
		}
		else if (!balance.isEnabled()) {
			int leftY = filterAxis(master, ANALOG_LEFT_Y);
			int rightY = filterAxis(master, ANALOG_RIGHT_Y);

			if (brakesButton.getButtonStatus() == POSITIVE) {
				drivetrain.tankSteerVelocity(leftY < 0 ? leftY : 0, rightY < 0 ? rightY : 0);
			}
			else {
				drivetrain.tankSteerVelocity(leftY, rightY);
			}
		}

		if (master.get_digital_new_press(DIGITAL_RIGHT)) {
			drivetrain.getLeftMotors().set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			drivetrain.getRightMotors().set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		}
		else if (master.get_digital_new_press(DIGITAL_LEFT)) {
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

			if (partner.get_digital_new_press(DIGITAL_A)) {
				purePursuit.setEnabled(true);
				purePursuit.setFollowing(true);
				turn(odometry.getPosition()->getTheta() + goalAngle());
				purePursuit.setEnabled(false);
				purePursuit.setFollowing(false);
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
