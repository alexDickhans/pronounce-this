#include "main.h"


// Controllers
Pronounce::Controller master(pros::E_CONTROLLER_MASTER);

// Motors

// Drive Motors
pros::Motor frontLeftMotor(1);
pros::Motor frontRightMotor(2, true);
pros::Motor backLeftMotor(9);
pros::Motor backRightMotor(10, true);

pros::Motor rightLift(3, true);
pros::Motor leftLift(4, false);

pros::Motor backGrabber(6);

pros::ADIDigitalOut frontGrabber(1, false);
pros::ADIDigitalIn frontGrabberBumperSwitch(2);

// Inertial Measurement Unit
pros::Imu imu(5);

Pronounce::MotorOdom wheel1(&frontLeftMotor, 2);
Pronounce::MotorOdom wheel2(&frontRightMotor, 2);
Pronounce::MotorOdom wheel3(&backLeftMotor, 2);
Pronounce::MotorOdom wheel4(&backRightMotor, 2);

Pronounce::MecanumOdometry odometry(&wheel1, &wheel2, &wheel3, &wheel4, &imu, 14 / 2, 10.5 / 2);

MecanumDrivetrain drivetrain(&frontLeftMotor, &frontRightMotor, &backLeftMotor, &backRightMotor, &imu, &odometry);

Pronounce::PurePursuit purePursuit(&drivetrain, 10);


MotorButton leftLiftButton(&master, &leftLift, DIGITAL_L1, DIGITAL_L2, 200, 0, -200, 0, 0);
MotorButton rightLiftButton(&master, &rightLift, DIGITAL_L1, DIGITAL_L2, 200, 0, -200, 0, 0);
MotorButton backGrabberButton(&master, &backGrabber, DIGITAL_R1, DIGITAL_R2, 200, 200, 200, 0, 450 * 3);

SolenoidButton frontGrabberButton(&master, DIGITAL_A, DIGITAL_B);

// Autonomous Selector
Pronounce::AutonSelector autonomousSelector;

bool relativeMovement = false;
bool driveOdomEnabled = true;

#define ROLL_AUTHORITY 1.0

#define DRIFT_MIN 7.0

bool preDriverTasksDone = false;

int testPathIndex;
int rightHomeToGoalNeutralIndex;
int rightNeutralToMidNeutralIndex;
int midNeutralToRightAllianceIndex;
int midNeutralToMidHomeZoneIndex;
int rightNeutralToRightHomeIndex;
int farRightHomeZoneToRightAllianceIndex;
int rightAllianceToRightHomeZoneIndex;

/**
 * @brief Runs during auton period before auton
 *
 */
int preAutonRun() {

	while (imu.is_calibrating()) {
		pros::Task::delay(50);
	}

	purePursuit.setEnabled(true);

	frontGrabberButton.setAutonomous(true);
	backGrabberButton.setAutonomous(true);
	leftLiftButton.setAutonomous(true);
	rightLiftButton.setAutonomous(true);

	backGrabberButton.setAutonomousButton(true);

	return 0;
}

/**
 * @brief Runs the Right Steal Right Auton
 *
 */
int rightStealRight() {
	odometry.reset(new Position(105.7, 16));

	backGrabberButton.setButtonStatus(ButtonStatus::POSITIVE);
	frontGrabberButton.setButtonStatus(ButtonStatus::NEUTRAL);

	purePursuit.setCurrentPathIndex(rightHomeToGoalNeutralIndex);
	purePursuit.setFollowing(true);

	// Wait until it is done
	while (!purePursuit.isDone(0.5)) {
		pros::Task::delay(50);
	}

	// Collect front goal
	frontGrabberButton.setButtonStatus(ButtonStatus::POSITIVE);
	pros::Task::delay(200);
	leftLiftButton.setAutonomousAuthority(360);
	rightLiftButton.setAutonomousAuthority(360);

	purePursuit.setCurrentPathIndex(rightNeutralToMidNeutralIndex);
	purePursuit.setFollowing(true);
	purePursuit.setTurnTarget(3.14);

	// Wait until it is done
	while (!purePursuit.isDone(0.5)) {
		pros::Task::delay(50);
	}

	backGrabberButton.setButtonStatus(ButtonStatus::NEUTRAL);
	pros::Task::delay(500);

	purePursuit.setCurrentPathIndex(midNeutralToMidHomeZoneIndex);
	purePursuit.setFollowing(true);
	purePursuit.setTurnTarget(-M_PI_2);

	// Wait until it is done
	while (!purePursuit.isDone(0.5)) {
		pros::Task::delay(50);
	}

	return 0;
}

int rightAwpRight() {
	odometry.reset(new Position(129.9, 16));

	purePursuit.setCurrentPathIndex(farRightHomeZoneToRightAllianceIndex);
	purePursuit.setFollowing(true);

	// Wait until it is done
	while (!purePursuit.isDone(0.5)) {
		pros::Task::delay(50);
	}

	// Collect front goal
	frontGrabberButton.setButtonStatus(ButtonStatus::POSITIVE);
	pros::Task::delay(200);
	leftLiftButton.setAutonomousAuthority(360);
	rightLiftButton.setAutonomousAuthority(360);

	purePursuit.setCurrentPathIndex(rightAllianceToRightHomeZoneIndex);
	purePursuit.setFollowing(true);

	// Wait until it is done
	while (!purePursuit.isDone(0.5)) {
		pros::Task::delay(50);
	}

	return 0;
}

int leftAwpLeft() {
	// Collect front goal
	frontGrabberButton.setButtonStatus(ButtonStatus::POSITIVE);
	pros::Task::delay(1000);
	frontGrabberButton.setButtonStatus(ButtonStatus::NEUTRAL);

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

	pros::Task::delay(10000);

	return 0;
}

int postAuton() {
	purePursuit.setFollowing(false);
	purePursuit.setEnabled(false);
	frontGrabberButton.setAutonomous(false);
	backGrabberButton.setAutonomous(false);
	leftLiftButton.setAutonomous(false);
	rightLiftButton.setAutonomous(false);

	return 0;
}


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
	while (imu.is_calibrating()) {
		pros::delay(20);
	}
}

void updateMotors() {
	while (1) {
		frontGrabberButton.update();
		backGrabberButton.update();
		leftLiftButton.update();
		rightLiftButton.update();

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
	backGrabber.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);
	leftLift.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);
	rightLift.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);

	pros::Task updateButtons(updateMotors, "Update buttons");
}

void initDrivetrain() {
	printf("Init drivetrain");

	odometry.setUseImu(true);

	purePursuit.setNormalizeDistance(10);

	purePursuit.setOdometry(&odometry);

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

void autoPaths() {
	// Default pure pursuit profile
	PurePursuitProfile defaultProfile(new PID(20, 0.0, -1.0), new PID(100, 0.0, 40.0), 10.0);
	purePursuit.getPurePursuitProfileManager().setDefaultProfile(defaultProfile);

	// Test path
	Path testPath = Path();

	testPath.addPoint(0, 0);
	testPath.addPoint(24, 0);
	testPath.addPoint(24, 24);
	testPath.addPoint(24, 48);
	testPath.addPoint(-24, 48);
	testPath.addPoint(0, 24);

	testPathIndex = purePursuit.addPath(testPath);

	Path rightNeutralToRightHomeZone;

	rightNeutralToRightHomeZone.addPoint(105.7, 61);
	rightNeutralToRightHomeZone.addPoint(105.7, 16);

	rightNeutralToRightHomeIndex = purePursuit.addPath(rightNeutralToRightHomeZone);

	// Right Steal Right
	Path rightHomeToGoalNeutral;

	rightHomeToGoalNeutral.addPoint(105.7, 16);
	rightHomeToGoalNeutral.addPoint(105.7, 62);

	rightHomeToGoalNeutralIndex = purePursuit.addPath(rightHomeToGoalNeutral);

	Path rightNeutralToMidNeutral;

	rightNeutralToMidNeutral.addPoint(105.7, 62);
	rightNeutralToMidNeutral.addPoint(75.3, 40);
	rightNeutralToMidNeutral.addPoint(60.3, 65);

	rightNeutralToMidNeutralIndex = purePursuit.addPath(rightNeutralToMidNeutral);

	Path midNeutralToRightAlliance;

	midNeutralToRightAlliance.addPoint(70.3, 65);
	midNeutralToRightAlliance.addPoint(120.1, 36);

	midNeutralToRightAllianceIndex = purePursuit.addPath(midNeutralToRightAlliance);

	Path midNeutralToMidHomeZone;

	midNeutralToMidHomeZone.addPoint(70.3, 70.3);
	midNeutralToMidHomeZone.addPoint(70.3, 35);

	midNeutralToMidHomeZoneIndex = purePursuit.addPath(midNeutralToMidHomeZone);

	Path farRightHomeZoneToRightAlliance;

	farRightHomeZoneToRightAlliance.addPoint(127.9, 16);
	farRightHomeZoneToRightAlliance.addPoint(127.9, 24);

	farRightHomeZoneToRightAllianceIndex = purePursuit.addPath(farRightHomeZoneToRightAlliance);

	Path rightAllianceToRightHomeZone;

	rightAllianceToRightHomeZone.addPoint(127.9, 24);
	rightAllianceToRightHomeZone.addPoint(105.7, 16);

	rightAllianceToRightHomeZoneIndex = purePursuit.addPath(rightAllianceToRightHomeZone);
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
	initDrivetrain();
	autoPaths();
	initController();
	initLogger();
	// initSelector();


	backGrabberButton.setSingleToggle(true);
	backGrabberButton.setGoToImmediately(true);

	frontGrabberButton.setSolenoid(&frontGrabber);
	frontGrabberButton.setSingleToggle(true);
}

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

/**
 * Starts when connected to the field
 */
void competition_initialize() {
	// autonomousSelector.choose();

}

/**
 * Runs during the autonomous. NO user control
 */
void autonomous() {
	// This calls the user selection, all the functions prototypes are in 
	// autonRoutines.hpp and the implementation is autonRoutines.cp
	// autonomousSelector.run();
	preAutonRun();
	leftAwpLeft();
	postAuton();
}


/**
 * Runs during operator/teleop control
 */
void opcontrol() {

	printf("OpControl");
	lv_obj_clean(lv_scr_act());

	postAuton();

	//lv_obj_t* infoLabel = lv_label_create(lv_scr_act(), NULL);
	// lv_label_set_text(infoLabel, "");

	const int runningAverageLength = 1;
	RunningAverage<runningAverageLength> leftXAvg;
	RunningAverage<runningAverageLength> leftYAvg;
	RunningAverage<runningAverageLength> rightXAvg;

	// Driver Control Loop
	while (true) {

		// Filter and calculate magnitudes
		int leftY = filterAxis(master, ANALOG_LEFT_Y);
		int leftX = filterAxis(master, ANALOG_LEFT_X);
		int rightX = filterAxis(master, ANALOG_RIGHT_X);

		leftXAvg.add(leftX);
		leftYAvg.add(leftY);
		rightXAvg.add(rightX);

		leftX = leftXAvg.getAverage();
		leftY = leftYAvg.getAverage();
		rightX = rightXAvg.getAverage();

		Vector driveVector = Vector(new Pronounce::Point(leftX, leftY));
		driveVector.setAngle((driveVector.getAngle()));// + threeWheelOdom.getPosition()->getTheta());

		// Send variables to motors
		drivetrain.setDriveVectorVelocity(driveVector, rightX);

		if (frontGrabberBumperSwitch.get_new_press()) {
			frontGrabberButton.setButtonStatus(Pronounce::ButtonStatus::POSITIVE);
		}

		if (master.get_digital_new_press(DIGITAL_X)) {
			odometry.reset(new Position());
		}

		// Prevent wasted resources
		pros::delay(10);
	}
}
