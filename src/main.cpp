#include "main.h"


// Controllers
Pronounce::Controller master(pros::E_CONTROLLER_MASTER);

// Motors

// Drive Motors
pros::Motor frontLeftMotor(1, true);
pros::Motor frontRightMotor(2, false);
pros::Motor midLeftMotor(15, false);
pros::Motor midRightMotor(16, true);
pros::Motor backLeftMotor(9, true);
pros::Motor backRightMotor(10, false);

pros::Motor lift(3, false);

pros::Motor intake(11, true);

pros::ADIDigitalOut frontGrabber(1, false);
pros::ADIDigitalOut backGrabber(2, false);
pros::ADIDigitalIn frontGrabberBumperSwitch(3);

// Inertial Measurement Unit
pros::Imu imu(5);

pros::Rotation leftEncoder(12);
pros::Rotation rightEncoder(14);
pros::Rotation backEncoder(13);

// Odom wheels
Pronounce::TrackingWheel leftOdomWheel(&leftEncoder);
Pronounce::TrackingWheel rightOdomWheel(&rightEncoder);
Pronounce::TrackingWheel backOdomWheel(&backEncoder);

ThreeWheelOdom odometry(&leftOdomWheel, &rightOdomWheel, &backOdomWheel, &imu);

TankDrivetrain drivetrain(&frontLeftMotor, &frontRightMotor, &midLeftMotor, &midRightMotor, &backLeftMotor, &backRightMotor, &imu, 15.0);

Pronounce::TankPurePursuit purePursuit(&drivetrain, &odometry, new PID(60, 0, 0), 20);

Balance balance(&drivetrain, &imu, new BangBang(15, true, 200), new PID(20, 0, 0));

MotorButton liftButton(&master, &lift, DIGITAL_L1, DIGITAL_L2, 200, 0, -200, 0, 0);
MotorButton intakeButton(&master, &intake, DIGITAL_R2, DIGITAL_Y, 200, 0, -100, 0, 0);

SolenoidButton frontGrabberButton(&master, DIGITAL_A, DIGITAL_B);
SolenoidButton backGrabberButton(&master, DIGITAL_R1, DIGITAL_R1);

// Autonomous Selector
Pronounce::AutonSelector autonomousSelector;

#define DRIFT_MIN 7.0

bool preDriverTasksDone = false;

int driverMode = 0;

// SECTION Auton

void flipOut() {
	pros::Task::delay(500);

	intakeButton.setButtonStatus(ButtonStatus::POSITIVE);
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
	intakeButton.setAutonomous(true);
	liftButton.setAutonomousPosition(true);

	pros::Task flipOutTask(flipOut);

	return 0;
}

/**
 * @brief Runs the Right Steal Right Auton
 *
 */
int rightStealRight() {
	odometry.reset(new Position(0, 16, 0));

	purePursuit.setSpeed(250);
	purePursuit.setLookahead(15);

	printf("Right Steal Right\n");

	backGrabberButton.setButtonStatus(ButtonStatus::NEUTRAL);
	frontGrabberButton.setButtonStatus(ButtonStatus::NEUTRAL);

	purePursuit.setCurrentPathIndex(forwardIndex);
	purePursuit.setFollowing(true);

	// Wait until it is done
	while (!purePursuit.isDone(4)) {
		pros::Task::delay(50);
	}

	purePursuit.setFollowing(false);

	// Collect front goal
	frontGrabberButton.setButtonStatus(ButtonStatus::POSITIVE);
	pros::Task::delay(300);
	liftButton.setAutonomousAuthority(360);

	purePursuit.setFollowing(true);

	printf("Right Steal Right: Collected front goal\n");

	// Move back
	// Timed programming, our favorite!
	// This is all the safegaurds I have to bypass
	purePursuit.setFollowing(false);
	purePursuit.setEnabled(false);

	backGrabberButton.setButtonStatus(ButtonStatus::NEUTRAL);

	drivetrain.skidSteerVelocity(-200, -10);

	pros::Task::delay(750);

	drivetrain.skidSteerVelocity(0, 0);

	backGrabberButton.setButtonStatus(ButtonStatus::POSITIVE);

	purePursuit.setFollowing(true);
	purePursuit.setEnabled(true);

	return 0;
}

int rightAwpRight() {
	odometry.reset(new Position(129.9, 16));

	backGrabberButton.setButtonStatus(ButtonStatus::NEUTRAL);
	frontGrabberButton.setButtonStatus(ButtonStatus::NEUTRAL);

	purePursuit.setCurrentPathIndex(farRightHomeZoneToRightAllianceIndex);
	purePursuit.setFollowing(true);

	// Wait until it is done
	while (!purePursuit.isDone(2)) {
		pros::Task::delay(50);
	}

	// Collect front goal
	frontGrabberButton.setButtonStatus(ButtonStatus::POSITIVE);
	pros::Task::delay(200);
	liftButton.setAutonomousAuthority(360);

	purePursuit.setCurrentPathIndex(rightAllianceToRightHomeZoneIndex);
	purePursuit.setFollowing(true);

	// Wait until it is done
	while (!purePursuit.isDone(2)) {
		pros::Task::delay(50);
	}

	return 0;
}

int leftAwpLeft() {
	odometry.reset(new Position(30, 11.4, -M_PI_2));

	backGrabberButton.setButtonStatus(ButtonStatus::NEUTRAL);
	frontGrabberButton.setButtonStatus(ButtonStatus::NEUTRAL);

	// Move back
	// Timed programming, our favorite!
	// This is all the safegaurds I have to bypass
	purePursuit.setFollowing(false);
	purePursuit.setEnabled(false);

	backGrabberButton.setButtonStatus(ButtonStatus::NEUTRAL);

	drivetrain.skidSteerVelocity(-150, 0);

	pros::Task::delay(250);

	drivetrain.skidSteerVelocity(0, 0);

	backGrabberButton.setButtonStatus(ButtonStatus::POSITIVE);

	purePursuit.setFollowing(true);
	purePursuit.setEnabled(true);

	purePursuit.setCurrentPathIndex(leftAllianceToLeftNeutralIndex);
	purePursuit.setFollowing(true);

	// Wait until it is done
	while (!purePursuit.isDone(2)) {
		pros::Task::delay(50);
	}

	frontGrabberButton.setButtonStatus(ButtonStatus::POSITIVE);
	pros::Task::delay(200);
	liftButton.setAutonomousAuthority(360);

	purePursuit.setCurrentPathIndex(leftNeutralToMidNeutralIndex);
	purePursuit.setFollowing(true);
	purePursuit.setTurnTarget(3.14);

	// Wait until it is done
	while (!purePursuit.isDone(2)) {
		pros::Task::delay(50);
	}

	backGrabberButton.setButtonStatus(ButtonStatus::NEUTRAL);
	pros::Task::delay(500);

	purePursuit.setCurrentPathIndex(midNeutralToMidHomeZoneIndex);
	purePursuit.setFollowing(true);
	purePursuit.setTurnTarget(M_PI_2);

	// Wait until it is done
	while (!purePursuit.isDone(2)) {
		pros::Task::delay(50);
	}

	return 0;
}

int leftAwp() {
	odometry.reset(new Position(29.0, 11.4, -M_PI_2));

	backGrabberButton.setButtonStatus(ButtonStatus::NEUTRAL);
	frontGrabberButton.setButtonStatus(ButtonStatus::NEUTRAL);

	// Move back
	// Timed programming, our favorite!
	// This is all the safegaurds I have to bypass
	purePursuit.setFollowing(false);
	purePursuit.setEnabled(false);

	backGrabberButton.setButtonStatus(ButtonStatus::NEUTRAL);

	drivetrain.skidSteerVelocity(-150, 0);

	pros::Task::delay(250);

	drivetrain.skidSteerVelocity(0, 0);

	backGrabberButton.setButtonStatus(ButtonStatus::POSITIVE);

	purePursuit.setFollowing(true);
	purePursuit.setEnabled(true);

	purePursuit.setSpeed(65);

	purePursuit.setLookahead(8);

	// Run pure pursuit paths, ewwww
	purePursuit.setCurrentPathIndex(leftAllianceToRightHomeZoneIndex);
	purePursuit.setFollowing(true);

	liftButton.setAutonomousAuthority(600);

	// Wait until it is done
	while (!purePursuit.isDone(4)) {
		pros::Task::delay(50);
	}

	purePursuit.setLookahead(10);

	pros::delay(1000);

	purePursuit.setSpeed(75);

	pros::delay(2000);

	liftButton.setAutonomousAuthority(0);

	purePursuit.setCurrentPathIndex(rightHomeZoneToRightAllianceIndex);
	purePursuit.setFollowing(true);

	// Wait until it is done
	while (!purePursuit.isDone(2)) {
		pros::Task::delay(50);
	}

	frontGrabberButton.setButtonStatus(POSITIVE);

	pros::delay(300);

	purePursuit.setCurrentPathIndex(rightAllianceToRightHomeZoneIndex);
	purePursuit.setFollowing(true);

	// Wait until it is done
	while (!purePursuit.isDone(2)) {
		pros::Task::delay(50);
	}

	intakeButton.setButtonStatus(ButtonStatus::NEUTRAL);

	/*

	// Move back
	// Timed programming, our favorite!
	// This is all the safegaurds I have to bypass
	purePursuit.setFollowing(false);
	purePursuit.setEnabled(false);

	drivetrain.skidSteerVelocity(-150, 0);

	pros::Task::delay(250);

	drivetrain.skidSteerVelocity(0, 0);

	purePursuit.setFollowing(true);
	purePursuit.setEnabled(true);

	frontGrabberButton.setButtonStatus(NEUTRAL);
/*
	purePursuit.setCurrentPathIndex(rightAllianceToRightRingsIndex);
	purePursuit.setFollowing(true);
	purePursuit.setTurnTarget(M_PI_2);

	// Wait until it is done
	while (!purePursuit.isDone(2)) {
		pros::Task::delay(50);
	}

	purePursuit.setCurrentPathIndex(rightRingsToRightHomeZoneIndex);
	purePursuit.setFollowing(true);
	purePursuit.setTurnTarget(M_PI_2);

	// Wait until it is done
	while (!purePursuit.isDone(2)) {
		pros::Task::delay(50);
	}
*/
	return 0;
}

int skills() {
	odometry.reset(new Position(29.0, 11.4, -M_PI_2));

	backGrabberButton.setButtonStatus(ButtonStatus::NEUTRAL);
	frontGrabberButton.setButtonStatus(ButtonStatus::NEUTRAL);

	// Move back
	// Timed programming, our favorite!
	// This is all the safegaurds I have to bypass
	purePursuit.setFollowing(false);
	purePursuit.setEnabled(false);

	backGrabberButton.setButtonStatus(ButtonStatus::NEUTRAL);

	drivetrain.skidSteerVelocity(-150, 0);

	pros::Task::delay(250);

	drivetrain.skidSteerVelocity(0, 0);

	backGrabberButton.setButtonStatus(ButtonStatus::POSITIVE);

	purePursuit.setFollowing(true);
	purePursuit.setEnabled(true);

	purePursuit.setCurrentPathIndex(leftAllianceToLeftNeutralIndex);

	liftButton.setAutonomousAuthority(0);
	
	// Wait until it is done
	while (!purePursuit.isDone(2)) {
		pros::Task::delay(50);
	}

	frontGrabberButton.setButtonStatus(POSITIVE);

	pros::Task::delay(500);

	purePursuit.setCurrentPathIndex(leftNeutralGoalToFarHomeZoneIndex);

	liftButton.setAutonomousAuthority(700);

	// Wait until it is done
	while (!purePursuit.isDone(2)) {
		pros::Task::delay(50);
	}

	pros::Task::delay(500);

	liftButton.setAutonomousAuthority(500);

	pros::Task::delay(800);

	frontGrabberButton.setButtonStatus(NEUTRAL);
	liftButton.setAutonomousAuthority(700);

	purePursuit.setCurrentPathIndex(farHomeZoneToMidNeutralGoalIndex);

	liftButton.setAutonomousAuthority(600);


	frontGrabberButton.setButtonStatus(POSITIVE);

	pros::Task::delay(500);

	purePursuit.setCurrentPathIndex(farHomeZoneToMidNeutralGoalIndex);

	liftButton.setAutonomousAuthority(0);

	// Wait until it is done
	while (!purePursuit.isDone(2)) {
		pros::Task::delay(50);
	}

	purePursuit.setOrientationControl(true);
	purePursuit.setTargetOrientation(M_PI);

	while(!purePursuit.isDoneOrientation(0.1)) {
		pros::Task::delay(50);
	}

	purePursuit.setOrientationControl(false);

	frontGrabberButton.setButtonStatus(POSITIVE);

	purePursuit.setOrientationControl(true);
	purePursuit.setTargetOrientation(0);

	while(!purePursuit.isDoneOrientation(0.1)) {
		pros::Task::delay(50);
	}

	purePursuit.setOrientationControl(false);

	liftButton.setAutonomousAuthority(700);

	purePursuit.setCurrentPathIndex(midNeutralGoalToPlatformIndex);

	while (purePursuit.isDone(2)) {
		pros::Task::delay(50);
	}

	pros::Task::delay(500);

	liftButton.setAutonomousAuthority(500);

	pros::Task::delay(800);

	frontGrabberButton.setButtonStatus(NEUTRAL);
	liftButton.setAutonomousAuthority(700);

	purePursuit.setCurrentPathIndex(farHomeZoneToMidNeutralGoalIndex);

	liftButton.setAutonomousAuthority(600);

	// Wait until it is done
	while (!purePursuit.isDone(2)) {
		pros::Task::delay(50);
	}



	return 0;
}

/**
 * @brief Test auton
 *
 * @return 0
 */
int testAuton() {

	printf("Test Auton\n");

	odometry.reset(new Position());

	purePursuit.setCurrentPathIndex(testPathIndex);
	purePursuit.setFollowing(true);

	// Wait until it is done
	while (!purePursuit.isDone(2)) {
		pros::Task::delay(50);
	}

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
		purePursuit.update();
		lv_label_set_text(infoLabel, odometry.getPosition()->to_string().c_str());
		pros::Task::delay_until(&startTime, 15);
	}
}

/**
 * Initialize all sensors
 */
void initSensors() {

	imu.reset();

	// Wait until IMU is calibrated
	// while (imu.is_calibrating()) {
	// 	pros::delay(20);
	// }
}

void updateMotors() {
	while (1) {
		frontGrabberButton.update();
		backGrabberButton.update();
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

	backGrabberButton.setSolenoid(&backGrabber);
	backGrabberButton.setSingleToggle(true);

	intakeButton.setSingleToggle(true);
	intakeButton.setDejam(true);
	intakeButton.setDejamAuthority(-100);
	intakeButton.setDejamSpeed(5);
	intakeButton.setDejamTime(500);

	lift.set_current_limit(25000);

	pros::Task updateButtons(updateMotors, "Update buttons");
}

void initDrivetrain() {
	printf("Init drivetrain\n");

	// odometry.setUseImu(true);
	leftOdomWheel.setRadius(2.75 / 2);
	leftOdomWheel.setTuningFactor(1.097);
	rightOdomWheel.setRadius(2.75 / 2);
	rightOdomWheel.setTuningFactor(1.077);
	backOdomWheel.setRadius(1.25);
	backOdomWheel.setTuningFactor(1.06);

	leftEncoder.set_reversed(true);
	rightEncoder.set_reversed(true);
	backEncoder.set_reversed(false);

	odometry.setLeftOffset(4.5 * 1.05);
	odometry.setRightOffset(4.5 * 1.05);
	odometry.setBackOffset(1.5);

	odometry.setMaxMovement(1);

	purePursuit.setNormalizeDistance(10);
	purePursuit.setSpeed(150);
	purePursuit.setLookahead(15);

	pros::Task purePursuitTask = pros::Task(updateDrivetrain, "Pure Pursuit");

	odometry.reset(new Position());

	printf("Init done");
}

/**
 * Initialize the controller
 */
void initController() {
	master.setDrivetrain(&drivetrain);
	pros::Task renderTask(renderThread);
}

// Run selector as task
void runSelector() {
	autonomousSelector.choose();
}

/**
 * Initialize the Auton Selector
 */
void initSelector() {
	autonomousSelector.addAuton(Auton("Right steal right", rightStealRight));
	autonomousSelector.addAuton(Auton("Test", testAuton));
	autonomousSelector.setDefaultAuton(Auton("Right steal right", rightStealRight));
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
	double quadraticFilter = pow(controllerFilter / 127.0, 3) * 200;

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
	rightStealRight();
	postAuton();
}

// !SECTION

// SECTION Operator Control

/**
 * Runs during operator/teleop control
 */
void opcontrol() {

	printf("OpControl");
	lv_obj_clean(lv_scr_act());

	postAuton();

	//lv_obj_t* infoLabel = lv_label_create(lv_scr_act(), NULL);
	// lv_label_set_text(infoLabel, "");

	// Driver Control Loop
	while (true) {

		if (driverMode > 0) {
			// Filter and calculate magnitudes
			int leftY = filterAxis(master, ANALOG_LEFT_Y);
			int rightX = filterAxis(master, ANALOG_RIGHT_X);

			drivetrain.skidSteerVelocity(leftY, rightX);
		}
		else {
			int leftY = filterAxis(master, ANALOG_LEFT_Y);
			int rightY = filterAxis(master, ANALOG_RIGHT_Y);

			drivetrain.tankSteerVelocity(leftY, rightY);
		}

		if (frontGrabberBumperSwitch.get_new_press()) {
			frontGrabberButton.setButtonStatus(Pronounce::ButtonStatus::POSITIVE);
		}

		if (master.get_digital_new_press(DIGITAL_X)) {
			odometry.reset(new Position());
		}

		if (master.get_digital_new_press(DIGITAL_UP)) {
			driverMode = 0;
		}
		else if (master.get_digital_new_press(DIGITAL_DOWN)) {
			driverMode = 2;
		}

		if (master.get_digital_new_press(DIGITAL_RIGHT)) {
			// drivetrain.getLeftMotors().set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			// drivetrain.getRightMotors().set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		}

		if (master.get_digital_new_press(DIGITAL_LEFT)) {

		}

		pros::delay(10);
	}
}

// !SECTION
