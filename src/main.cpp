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
pros::ADIDigitalIn frontGrabberBumperSwitch(4);
pros::ADIDigitalIn backGrabberBumperSwitch(5);

// Inertial Measurement Unit
pros::Imu imu(5);

pros::Rotation leftEncoder(12);
pros::Rotation rightEncoder(14);

// Odom wheels
Pronounce::TrackingWheel leftOdomWheel(&leftEncoder);
Pronounce::TrackingWheel rightOdomWheel(&rightEncoder);
OdomWheel nullOdomWheel;

// GPS sensor
pros::Gps gps(4, 0, 0, 90, 0.2, 0.2);
GpsOdometry gpsOdometry(&gps);

// ThreeWheelOdom odometry(&leftOdomWheel, &rightOdomWheel, &backOdomWheel, &imu);
ThreeWheelOdom odometry(&leftOdomWheel, &rightOdomWheel, &nullOdomWheel, &imu);

TankDrivetrain drivetrain(&frontLeftMotor, &frontRightMotor, &midLeftMotor, &midRightMotor, &backLeftMotor, &backRightMotor, &imu, 11.0);

Pronounce::TankPurePursuit purePursuit(&drivetrain, &odometry, new PID(0.7, 0, 0.05), 20);

Balance balance(&drivetrain, &imu, new BangBang(20, true, -30), new PID(0, 0, 0));

MotorButton liftButton(&master, &lift, DIGITAL_L1, DIGITAL_L2, 200, 0, -200, 0, 0);
MotorButton intakeButton(&master, &intake, DIGITAL_R2, DIGITAL_Y, 200, 0, -100, 0, 0);

SolenoidButton frontGrabberButton(&master, DIGITAL_A);
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
		pros::Task::delay(50);
	}
}

void waitForDone() {
	waitForDone(0.1, 15000);
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
	pros::Task::delay(500);

	liftButton.setAutonomousAuthority(2000);

	waitForDone();

	purePursuit.setEnabled(false);
	purePursuit.setFollowing(false);

	liftButton.setAutonomousAuthority(1300);

	pros::Task::delay(1000);

	frontGrabberButton.setButtonStatus(NEUTRAL);

	pros::Task::delay(800);

	drivetrain.skidSteerVelocity(-50, 0);

	pros::Task::delay(350);

	drivetrain.skidSteerVelocity(-20, 0);

	pros::Task::delay(200);

	liftButton.setAutonomousAuthority(2000);

	drivetrain.skidSteerVelocity(-50, 0);

	pros::Task::delay(400);

	drivetrain.skidSteerVelocity(0, 0);

	pros::Task::delay(300);

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

/**
 * @brief Runs the Right Steal Right Auton
 *
 */
int leftStealLeft() {
	odometry.reset(new Position(25, 16, 0.19));

	purePursuit.setSpeed(80);
	purePursuit.setLookahead(15);

	printf("Left Steal Left\n");

	double oldLookahead = purePursuit.getLookahead();

	backGrabberButton.setButtonStatus(ButtonStatus::NEUTRAL);
	frontGrabberButton.setButtonStatus(ButtonStatus::NEUTRAL);

	purePursuit.setLookahead(20);

	purePursuit.setCurrentPathIndex(leftHomeZoneToLeftNeutralIndex);
	purePursuit.setFollowing(true);

	waitForDone(4);

	// Collect front goal
	frontGrabberButton.setButtonStatus(ButtonStatus::POSITIVE);

	waitForDone(0.5);

	purePursuit.setLookahead(oldLookahead);

	purePursuit.setFollowing(false);

	pros::Task::delay(300);

	liftButton.setAutonomousAuthority(100);

	purePursuit.setFollowing(true);

	printf("Left steal left: Collected front goal\n");

	purePursuit.setCurrentPathIndex(leftNeutralToLeftAllianceGoalIndex);
	purePursuit.setFollowing(true);

	// Change to false if you don't want to let go of the neutral mobile goal after a certain amount of time
	if (false) {
		uint32_t startTime = pros::millis();

		while (odometry.getPosition()->getY() > 40 && pros::millis() - startTime < 3000) {
			pros::Task::delay(50);
		}

		if (odometry.getPosition()->getY() > 40 && pros::millis() - startTime >= 3000) {
			printf("Left steal right: Failed to get to the right alliance goal\n");
			frontGrabberButton.setButtonStatus(ButtonStatus::NEUTRAL);

			// Move back
			// Timed programming, our favorite!
			// This is all the safegaurds I have to bypass
			purePursuit.setFollowing(false);
			purePursuit.setEnabled(false);

			drivetrain.skidSteerVelocity(50, 0);

			pros::Task::delay(250);

			drivetrain.skidSteerVelocity(0, 0);

			purePursuit.setFollowing(true);
			purePursuit.setEnabled(true);

			// Get mid goal if you don't get the main goal
			if (false) {
				purePursuit.setCurrentPathIndex(leftNeutralToEnterLeftHomeZoneIndex);
				purePursuit.setFollowing(true);

				intakeButton.setButtonStatus(POSITIVE);

				waitForDone(1);

				purePursuit.setSpeed(65);

				turn(M_PI_4);

				purePursuit.setCurrentPathIndex(enterLeftHomeZoneToMidGoalIndex);
				purePursuit.setFollowing(true);

				waitForDone(20);

				liftButton.setAutonomousAuthority(0);

				waitForDone(1);

				frontGrabberButton.setButtonStatus(ButtonStatus::POSITIVE);

				waitForDone(0.5);

				purePursuit.setFollowing(false);

				pros::Task::delay(200);

				liftButton.setAutonomousAuthority(600);

				purePursuit.setFollowing(true);

				purePursuit.setCurrentPathIndex(midGoalToEnterLeftHomeZoneIndex);

				waitForDone();

				purePursuit.setCurrentPathIndex(leftNeutralToLeftAllianceGoalIndex);
			}
		}
	}
	else {
		while (odometry.getPosition()->getY() > 40) {
			pros::Task::delay(50);
		}
	}

	purePursuit.setSpeed(30);

	waitForDone(5);

	purePursuit.setSpeed(10);

	waitForDone(1);

	backGrabberButton.setButtonStatus(POSITIVE);

	pros::Task::delay(200);

	intakeButton.setButtonStatus(ButtonStatus::POSITIVE);

	pros::Task::delay(600);

	purePursuit.setCurrentPathIndex(leftAllianceToRightHomeZoneIndex);
	purePursuit.setFollowing(true);

	liftButton.setAutonomousAuthority(600);

	purePursuit.setSpeed(30);

	pros::Task::delay(2000);

	purePursuit.setSpeed(65);

	waitForDone(8);

	backGrabberButton.setButtonStatus(ButtonStatus::NEUTRAL);

	waitForDone();

	pros::Task::delay(500);

	intakeButton.setButtonStatus(ButtonStatus::NEUTRAL);

	purePursuit.setFollowing(false);

	pros::Task::delay(500);

	return 0;
}

int rightStealRight() {
	odometry.reset(new Position(105.7, 16, 0));

	purePursuit.setSpeed(80);
	purePursuit.setLookahead(15);

	printf("Right Steal Right\n");

	backGrabberButton.setButtonStatus(ButtonStatus::NEUTRAL);
	frontGrabberButton.setButtonStatus(ButtonStatus::NEUTRAL);

	double oldLookahead = purePursuit.getLookahead();

	purePursuit.setCurrentPathIndex(rightHomeZoneToRightNeutralIndex);
	purePursuit.setFollowing(true);

	purePursuit.setLookahead(20);

	waitForDone(4);

	// Collect front goal
	frontGrabberButton.setButtonStatus(ButtonStatus::POSITIVE);

	waitForDone(0.5);

	purePursuit.setLookahead(oldLookahead);

	purePursuit.setFollowing(false);

	liftButton.setAutonomousAuthority(0);

	printf("Right steal right: Collected front goal\n");

	purePursuit.setCurrentPathIndex(rightNeutralToRightAllianceGoalIndex);
	purePursuit.setFollowing(true);

	purePursuit.setSpeed(45);

	// Change to false if you don't want to let go of the alliance mobile goal after a certain amount of time
	if (false) {
		uint32_t startTime = pros::millis();

		while (odometry.getPosition()->getY() > 45 && pros::millis() - startTime < 3000) {
			pros::Task::delay(50);
		}

		if (odometry.getPosition()->getY() > 40 && pros::millis() - startTime >= 3000) {
			printf("Left steal right: Failed to get to the right alliance goal\n");
			frontGrabberButton.setButtonStatus(ButtonStatus::NEUTRAL);

			// Move back
			// Timed programming, our favorite!
			// This is all the safegaurds I have to bypass
			purePursuit.setFollowing(false);
			purePursuit.setEnabled(false);

			drivetrain.skidSteerVelocity(50, 0);

			pros::Task::delay(250);

			drivetrain.skidSteerVelocity(0, 0);

			purePursuit.setFollowing(true);
			purePursuit.setEnabled(true);
		}
	}
	else {
		while (odometry.getPosition()->getY() > 45) {
			pros::Task::delay(50);
		}
	}

	liftButton.setAutonomousAuthority(100);

	purePursuit.setSpeed(85);

	liftButton.setAutonomousAuthority(600);

	waitForDone(20);

	purePursuit.setSpeed(15);

	waitForDone(0.1, 3000);

	backGrabberButton.setButtonStatus(POSITIVE);

	pros::Task::delay(200);

	purePursuit.setSpeed(45);

	turn(-M_PI_4);

	purePursuit.setSpeed(15);

	purePursuit.setCurrentPathIndex(rightAllianceGoalToRightRingsIndex);
	purePursuit.setFollowing(true);
	purePursuit.setSpeed(20);

	pros::Task::delay(500);

	intakeButton.setButtonStatus(ButtonStatus::POSITIVE);

	waitForDone(30);

	liftButton.setAutonomousAuthority(1800);

	waitForDone(0.1, 3000);

	purePursuit.setCurrentPathIndex(rightRingsToRightHomeZoneIndex);
	purePursuit.setFollowing(true);
	purePursuit.setSpeed(45);

	pros::Task::delay(2000);

	liftButton.setAutonomousAuthority(600);

	waitForDone();

	drivetrain.skidSteerVelocity(0, 0);

	purePursuit.setFollowing(false);

	backGrabberButton.setButtonStatus(ButtonStatus::NEUTRAL);

	pros::Task::delay(300);

	// Move back
	// Timed programming, our favorite!
	// This is all the safegaurds I have to bypass
	purePursuit.setFollowing(false);
	purePursuit.setEnabled(false);

	drivetrain.skidSteerVelocity(50, 0);

	pros::Task::delay(250);

	drivetrain.skidSteerVelocity(0, 0);

	purePursuit.setFollowing(true);
	purePursuit.setEnabled(true);

	pros::Task::delay(200);

	return 0;
}

int leftAwpRight() {
	odometry.reset(new Position(29.0, 11.4, -M_PI_2));

	backGrabberButton.setButtonStatus(ButtonStatus::NEUTRAL);
	frontGrabberButton.setButtonStatus(ButtonStatus::NEUTRAL);

	// Move back
	// Timed programming, our favorite!
	// This is all the safegaurds I have to bypass
	purePursuit.setFollowing(false);
	purePursuit.setEnabled(false);

	backGrabberButton.setButtonStatus(ButtonStatus::NEUTRAL);

	drivetrain.skidSteerVelocity(-100, 0);

	pros::Task::delay(250);

	drivetrain.skidSteerVelocity(0, 0);

	backGrabberButton.setButtonStatus(ButtonStatus::POSITIVE);

	purePursuit.setFollowing(true);
	purePursuit.setEnabled(true);

	// Run pure pursuit paths, ewwww
	purePursuit.setCurrentPathIndex(leftAllianceToRightAllianceIndex);
	purePursuit.setFollowing(true);

	purePursuit.setSpeed(15);

	liftButton.setAutonomousAuthority(600);

	pros::Task::delay(300);

	intakeButton.setButtonStatus(ButtonStatus::POSITIVE);

	pros::Task::delay(700);

	purePursuit.setSpeed(30);

	pros::Task::delay(1000);

	purePursuit.setSpeed(20);

	waitForDone(35);

	purePursuit.setSpeed(15);

	pros::Task::delay(400);

	backGrabberButton.setButtonStatus(ButtonStatus::NEUTRAL);

	pros::Task::delay(400);

	purePursuit.setSpeed(15);

	intakeButton.setButtonStatus(ButtonStatus::NEUTRAL);

	turn(-M_PI_2);

	intakeButton.setButtonStatus(ButtonStatus::NEGATIVE);

	purePursuit.setSpeed(30);

	waitForDone(0.1, 3000);

	backGrabberButton.setButtonStatus(POSITIVE);

	pros::Task::delay(200);

	purePursuit.setSpeed(15);

	turn(-M_PI_4, 0.2);

	purePursuit.setSpeed(30);

	purePursuit.setCurrentPathIndex(rightAllianceGoalToRightRingsIndex);
	purePursuit.setFollowing(true);

	pros::Task::delay(500);

	intakeButton.setButtonStatus(ButtonStatus::POSITIVE);

	waitForDone(20);

	liftButton.setAutonomousAuthority(1600);

	waitForDone(0.1, 3000);

	purePursuit.setCurrentPathIndex(rightRingsToRightHomeZoneIndex);
	purePursuit.setFollowing(true);
	purePursuit.setSpeed(45);

	pros::Task::delay(2000);

	liftButton.setAutonomousAuthority(600);

	while (odometry.getPosition()->getY() > 48) {
		pros::Task::delay(50);
	}

	backGrabberButton.setButtonStatus(ButtonStatus::NEUTRAL);

	waitForDone();

	purePursuit.setFollowing(false);

	backGrabberButton.setButtonStatus(ButtonStatus::NEUTRAL);

	pros::Task::delay(100);

	intakeButton.setButtonStatus(ButtonStatus::NEUTRAL);

	pros::Task::delay(200);

	return 0;
}

int skills() {
	odometry.reset(new Position(29.0, 11.4, -M_PI_2));

	backGrabberButton.setButtonStatus(ButtonStatus::NEUTRAL);
	frontGrabberButton.setButtonStatus(ButtonStatus::NEUTRAL);
	intakeButton.setButtonStatus(POSITIVE);

	purePursuit.setMaxAcceleration(65);

	// Move back
	// Timed programming, our favorite!
	// This is all the safegaurds I have to bypass
	purePursuit.setFollowing(false);
	purePursuit.setEnabled(false);

	backGrabberChange(false);

	drivetrain.skidSteerVelocity(-50, 0);

	pros::Task::delay(250);

	drivetrain.skidSteerVelocity(0, 0);

	backGrabberChange(true);

	purePursuit.setFollowing(true);
	purePursuit.setEnabled(true);

	purePursuit.setLookahead(12);
	purePursuit.setSpeed(65);

	pros::Task::delay(500);

	purePursuit.setCurrentPathIndex(leftHomeZoneToLeftNeutralGoalIndex);

	liftButton.setAutonomousAuthority(600);

	while (odometry.getPosition()->getY() < 38) {
		pros::Task::delay(50);
	}

	purePursuit.setSpeed(15);

	waitForDone(10);

	liftButton.setAutonomousAuthority(0);

	waitForDone();

	frontGrabberButton.setButtonStatus(POSITIVE);

	purePursuit.setSpeed(65);

	pros::Task::delay(300);

	purePursuit.setCurrentPathIndex(leftNeutralGoalToFarHomeZoneIndex);

	liftButton.setAutonomousAuthority(2000);

	waitForDone();

	purePursuit.setSpeed(15);

	turn(0, 0.2);

	purePursuit.setFollowing(false);

	placeOnPlatform();

	turn(-M_PI_2);

	purePursuit.setCurrentPathIndex(farLeftHomeZoneToFarRightGoalDropOffIndex);

	waitForDone(2);

	backGrabberChange(false);

	pros::Task::delay(600);

	waitForDone();

	purePursuit.setCurrentPathIndex()

	printf("Skills path done\n");

	return 0;
}

/**
 * @brief Test auton
 *
 * @return 0
 */
int testOrientationAuton() {

	printf("Test Auton\n");

	odometry.reset(new Position(0, 0, 0));

	purePursuit.setCurrentPathIndex(0);

	purePursuit.setOrientationControl(true);
	purePursuit.setFollowing(true);

	purePursuit.setTargetOrientation(M_PI);

	waitForDoneOrientation();

	printf("Orientation set to M_PI\n");
	pros::Task::delay(1000);

	purePursuit.setTargetOrientation(0);

	printf("Orientation set to 0\n");

	waitForDoneOrientation();

	purePursuit.setOrientationControl(false);
	purePursuit.setFollowing(false);

	return 0;
}

/**
 * @brief Test auton
 *
 * @return 0
 */
int tuneOdom() {

	printf("Test Auton\n");

	odometry.reset(new Position(20, 0, 0));

	purePursuit.setCurrentPathIndex(0);
	purePursuit.setSpeed(45);

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

	backGrabberButton.setButtonStatus(POSITIVE);

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
		pros::Task::delay_until(&startTime, 10);
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
			if (pros::millis() - lastChange > 500) {
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

	backGrabberButton.setSolenoid(&backTilter);
	backGrabberButton.setSingleToggle(true);

	backGrabberButton2.setSolenoid(&backGrabber);
	backGrabberButton2.setSingleToggle(true);

	intakeButton.setSingleToggle(true);
	intakeButton.setDejam(true);
	intakeButton.setDejamAuthority(-250);
	intakeButton.setDejamSpeed(50);
	intakeButton.setDejamTime(250);

	liftButton.setMultiplier(1.4);

	pros::Task updateButtons(updateMotors, "Update buttons");
}

void initDrivetrain() {
	printf("Init drivetrain\n");

	// odometry.setUseImu(false);
	// Left/Right fraction
	// 1.072124756
	leftOdomWheel.setRadius(2.75 / 2);
	leftOdomWheel.setTuningFactor(1.0252);
	rightOdomWheel.setRadius(2.75 / 2);
	rightOdomWheel.setTuningFactor(leftOdomWheel.getTuningFactor() * 0.9862039);

	leftEncoder.set_reversed(false);
	rightEncoder.set_reversed(true);

	odometry.setLeftOffset(3.9);
	odometry.setRightOffset(3.9);
	odometry.setBackOffset(0);


	/*
		// Used for tuning straight lines with pure pursuit
		odometry.setLeftOffset(2000);
		odometry.setRightOffset(2000);
		odometry.setBackOffset(0);
	*/

	odometry.setMaxMovement(0);

	purePursuit.setOutputMultiplier(600/65);
	purePursuit.setNormalizeDistance(10);
	purePursuit.setSpeed(45);
	purePursuit.setLookahead(15);
	purePursuit.setStopDistance(0.5);
	purePursuit.setMaxAcceleration(400);

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
	autonomousSelector.addAuton(Auton("Left AWP Right", leftAwpRight));
	autonomousSelector.addAuton(Auton("Right steal right", leftStealLeft));
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
	double quadraticFilter = pow(controllerFilter / 127.0, 3) * 600;

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
	// farRightAllianceToPlatformTest();
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

	if (frontGrabberBumperSwitch.get_value() == 1 || !partner.is_connected()) {
		frontGrabberButton.setButtonStatus(POSITIVE);
	}
	else {
		frontGrabberButton.setButtonStatus(NEGATIVE);
	}

	liftButton.setAutonomous(true);
	liftButton.setAutonomousPosition(false);

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

		if (master.get_digital_new_press(DIGITAL_X)) {
			odometry.reset(new Position());
		}

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

			if (master.get_digital_new_press(DIGITAL_B) || partner.get_digital_new_press(DIGITAL_B)) {
				balance.setEnabled(!balance.isEnabled());
				balance.getOrientationController()->setTarget(imu.get_heading());
				drivetrain.getLeftMotors().set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
				drivetrain.getRightMotors().set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			}

			if (master.get_digital_new_press(DIGITAL_RIGHT) || partner.get_digital_new_press(DIGITAL_R1)) {
				drivetrain.getLeftMotors().set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
				drivetrain.getRightMotors().set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			}
			else if (master.get_digital_new_press(DIGITAL_LEFT) || partner.get_digital_new_press(DIGITAL_R2)) {
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
