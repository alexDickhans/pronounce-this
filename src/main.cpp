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

TankDrivetrain drivetrain(&frontLeftMotor, &frontRightMotor, &midLeftMotor, &midRightMotor, &backLeftMotor, &backRightMotor, &imu, 10.0);

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

#define DRIFT_MIN 7.0

bool preDriverTasksDone = false;
bool disableIntake = true;

int driverMode = 0;

bool backButtonStatus = false;
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

	purePursuit.setEnabled(true);
	purePursuit.setFollowing(true);

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

void leftSteal() {
	purePursuit.setSpeed(70);

	frontGrabberButton.setButtonStatus(NEUTRAL);
	backGrabberChange(false);
	frontHookButton.setButtonStatus(POSITIVE);

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

	grabFromHook();
}

void rightSteal() {
	purePursuit.setSpeed(70);

	frontGrabberButton.setButtonStatus(NEUTRAL);
	backGrabberChange(false);
	frontHookButton.setButtonStatus(POSITIVE);

	purePursuit.setEnabled(true);
	purePursuit.setFollowing(true);

	purePursuit.setCurrentPathIndex(rightHomeZoneToRightNeutralIndex);

	waitForDone();

	frontHookButton.setButtonStatus(NEUTRAL);

	purePursuit.setCurrentPathIndex(rightNeutralToRightHomeZoneIndex);

	while (odometry.getPosition()->getY() > 40) {
		pros::Task::delay(50);
	}

	purePursuit.setSpeed(45);

	waitForDone();

	grabFromHook();
}

void rightRightSteal() {
	purePursuit.setSpeed(70);

	frontGrabberButton.setButtonStatus(NEUTRAL);
	backGrabberChange(false);
	frontHookButton.setButtonStatus(POSITIVE);

	purePursuit.setEnabled(true);
	purePursuit.setFollowing(true);

	purePursuit.setCurrentPathIndex(farRightHomeZoneToRightNeutralIndex);

	waitForDone();

	frontHookButton.setButtonStatus(NEUTRAL);

	purePursuit.setCurrentPathIndex(rightNeutralToFarRightHomeZoneIndex);

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

	grabFromHook();
}

void leftAWP() {
	liftButton.setAutonomousAuthority(600);

	pros::Task::delay(100);

	purePursuit.setCurrentPathIndex(leftAllianceToRightHomeZoneIndex);

	pros::Task::delay(200);

	intakeButton.setButtonStatus(POSITIVE);

	waitForDone(40);
}

void rightAWP() {
	turn(-M_PI_2 * 0.85);

	purePursuit.setCurrentPathIndex(enterRightHomeZoneToRightAllianceIndex);

	waitForDone(20);

	purePursuit.setSpeed(30);

	waitForDone(8);

	backGrabberChange(true);

	waitForDone(0.1, 3000);

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

	purePursuit.setCurrentPathIndex(rightHomeZoneToRightAllianceIndex);

	waitForDone(8);

	backGrabberChange(true);

	waitForDone(0.1, 3000);

	intakeButton.setButtonStatus(POSITIVE);

	purePursuit.setCurrentPathIndex(rightAllianceToLeftAllianceIndex);

	waitForDone(50);

	intakeButton.setButtonStatus(NEUTRAL);

	turn(-M_PI_4);

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

int leftStealLeftAWP() {
	odometry.reset(new Position(27, 18, 0.17));

	leftSteal();

	purePursuit.setCurrentPathIndex(leftHomeZoneToLeftAllianceIndex);

	waitForDone();

	backGrabberChange(true);

	leftAWP();

	return 0;
}

int leftStealFullAWP() {
	odometry.reset(new Position(27, 18, 0.17));

	leftSteal();

	purePursuit.setCurrentPathIndex(leftHomeZoneToLeftAllianceIndex);

	waitForDone();

	backGrabberChange(true);

	leftAWP();

	waitForDone();

	rightAWP();

	return 0;
}

int rightStealRightAWP() {
	odometry.reset(new Position(105.7, 19.5, 0.0));

	rightSteal();

	rightAWP();

	return 0;
}

int rightStealFullAWP() {
	odometry.reset(new Position(105.7, 19.5, 0.0));

	rightSteal();

	rightAWPToLeftAWP();

	return 0;
}

int rightRightStealToRightAWP() {
	odometry.reset(new Position(115, 19.5, 0.0));

	rightRightSteal();

	rightAWP();

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

	intakeButton.setButtonStatus(POSITIVE);

	purePursuit.setCurrentPathIndex(leftHomeZoneToLeftNeutralGoalIndex);

	liftButton.setAutonomousAuthority(600);

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

	waitForDone();

	purePursuit.setSpeed(60);

	turn(0, 0.2);

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

	purePursuit.setCurrentPathIndex(farRightDropOffToFarLeftAllianceGoalIndex);

	purePursuit.setSpeed(60);

	waitForDone(25);

	liftButton.setAutonomousAuthority(600);

	turn(M_PI_2);

	purePursuit.setSpeed(20);

	waitForDone(0.5, 2000);

	backGrabberChange(true);

	pros::Task::delay(100);

	purePursuit.setSpeed(30);

	turn(M_PI_2 + M_PI_4);

	purePursuit.setCurrentPathIndex(farLeftAllianceToMidGoalIndex);

	intakeButton.setButtonStatus(POSITIVE);

	purePursuit.setSpeed(20);

	waitForDone(15);

	liftButton.setAutonomousAuthority(0);

	waitForDone();

	frontGrabberButton.setButtonStatus(POSITIVE);

	pros::Task::delay(300);

	purePursuit.setSpeed(60);

	turn(0);

	purePursuit.setCurrentPathIndex(midGoalToFarPlatformIndex);

	liftButton.setAutonomousAuthority(2000);

	purePursuit.setSpeed(20);

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

	while (odometry.getPosition()->getY() > 85) {
		pros::Task::delay(50);
	}

	purePursuit.setSpeed(50);

	while (odometry.getPosition()->getY() > 40) {
		pros::Task::delay(50);
	}

	purePursuit.setSpeed(10);

	while (odometry.getPosition()->getY() > 35) {
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
	lv_obj_t* infoLabel = lv_label_create(lv_scr_act(), NULL);
	lv_label_set_text(infoLabel, "drivetrain");

	while (1) {
		uint32_t startTime = pros::millis();
		odometry.update();

		if (purePursuit.isEnabled()) {
			purePursuit.update();
		}
		else if (balance.isEnabled()) {
			balance.update();
		}

		lv_label_set_text(infoLabel, (odometry.getPosition()->to_string() + "\nL: " + std::to_string(leftOdomWheel.getPosition()) + ", R: " + std::to_string(rightOdomWheel.getPosition())).c_str());

		pros::Task::delay_until(&startTime, 20);
	}
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

		if (backButtonStatus) {
			backGrabberButton2.setButtonStatus(POSITIVE);
			if (pros::millis() - lastChange > 100) {
				backGrabberButton.setButtonStatus(POSITIVE);
			}
		}
		else {
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
	double turningFactor = (((101.22 / 101.50) - 1.0) / 2);
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

	printf("LVGL Init");

	// Initialize functions
	initSensors();
	initMotors();
	autoPaths(&purePursuit);
	initDrivetrain();
	initController();
	initLogger();
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

	if (partner.is_connected()) {
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


		if (master.get_digital_new_press(DIGITAL_RIGHT) || partner.get_digital_new_press(DIGITAL_R1)) {
			drivetrain.getLeftMotors().set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			drivetrain.getRightMotors().set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		}
		else if (master.get_digital_new_press(DIGITAL_LEFT) || partner.get_digital_new_press(DIGITAL_R2)) {
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

			if (master.get_digital(DIGITAL_R2) || partner.get_digital(DIGITAL_R2)) {
				intakeButton.setButtonStatus(intakeButton.getButtonStatus() == POSITIVE ? NEUTRAL : NEGATIVE);
			} else if (master.get_digital(DIGITAL_Y) || partner.get_digital(DIGITAL_R1)) {
				intakeButton.setButtonStatus(intakeButton.getButtonStatus() == NEGATIVE ? NEUTRAL : POSITIVE);
			}

			if (master.get_digital_new_press(DIGITAL_B) || partner.get_digital_new_press(DIGITAL_B)) {
				balance.setEnabled(!balance.isEnabled());
				balance.getOrientationController()->setTarget(imu.get_heading());
				drivetrain.getLeftMotors().set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
				drivetrain.getRightMotors().set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
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

			if (master.get_digital_new_press(DIGITAL_B)) {
				balance.setEnabled(!balance.isEnabled());
				balance.getOrientationController()->setTarget(imu.get_heading());
				drivetrain.getLeftMotors().set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
				drivetrain.getRightMotors().set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			}

			if (master.get_digital_new_press(DIGITAL_RIGHT)) {
				drivetrain.getLeftMotors().set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
				drivetrain.getRightMotors().set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			}
			else if (master.get_digital_new_press(DIGITAL_LEFT)) {
				drivetrain.getLeftMotors().set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
				drivetrain.getRightMotors().set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			}
		}

		pros::delay(10);
	}
}

// !SECTION
