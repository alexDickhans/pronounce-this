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

Pronounce::MecanumOdometry odometry(&wheel1, &wheel2, &wheel3, &wheel4, &imu, 14/2, 10.5/2);

MecanumDrivetrain drivetrain(&frontLeftMotor, &frontRightMotor, &backLeftMotor, &backRightMotor, &imu, &odometry);

Pronounce::PurePursuit purePursuit(&drivetrain, 10);

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
int rightAllianceToRightRingIndex;
int rightRingToLeftHomeZoneIndex;

/**
 * @brief Runs during auton period before auton
 *
 */
int preAutonRun() {

	while (imu.is_calibrating()) {
		pros::Task::delay(50);
	}

	purePursuit.setEnabled(true);

	return 0;
}

/**
 * @brief Runs the Right Steal Right Auton
 *
 */
int rightStealRight() {
	odometry.reset(new Position(105.7, 8));

	purePursuit.setCurrentPathIndex(rightHomeToGoalNeutralIndex);
	purePursuit.setFollowing(true);

	// Wait until it is done
	while (purePursuit.isDone(0.5)) {
		pros::Task::delay(50);
	}

	// Collect front goal
	frontGrabber.set_value(true);

	purePursuit.setCurrentPathIndex(rightNeutralToMidNeutralIndex);
	purePursuit.setFollowing(true);
	purePursuit.setTurnTarget(3.14);

	while (odometry.getPosition()->getY() < 46.8) {
		pros::Task::delay(50);
	}
	
	// Let go of front goal and get ready to collect again
	frontGrabber.set_value(false);
	purePursuit.setTurnTarget(0);

	// Wait until it is done
	while (purePursuit.isDone(0.5)) {
		pros::Task::delay(50);
	}

	// Collect front goal
	frontGrabber.set_value(true);

	purePursuit.setCurrentPathIndex(midNeutralToRightAllianceIndex);
	purePursuit.setFollowing(true);
	purePursuit.setTurnTarget(2.355);

	// Wait until it is done
	while (purePursuit.isDone(0.5)) {
		pros::Task::delay(50);
	}

	backGrabber.move_velocity(200);

	purePursuit.setTurnTarget(0);
	pros::Task::delay(500);

	purePursuit.setCurrentPathIndex(rightAllianceToRightRingIndex);
	purePursuit.setFollowing(true);
	purePursuit.setTurnTarget(0);

	// Wait until it is done
	while (purePursuit.isDone(0.5)) {
		pros::Task::delay(50);
	}

	purePursuit.setCurrentPathIndex(rightRingToLeftHomeZoneIndex);
	purePursuit.setFollowing(true);
	purePursuit.setTurnTarget(-M_PI_2);

	// Wait until it is done
	while (purePursuit.isDone(0.5)) {
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
	odometry.reset(new Position());

	purePursuit.setCurrentPathIndex(testPathIndex);
	purePursuit.setFollowing(true);

	// Wait until it is done
	while (purePursuit.isDone(0.5)) {
		pros::Task::delay(50);
	}

	return 0;
}

int postAuton() {
	purePursuit.setFollowing(false);
	purePursuit.setEnabled(false);
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
	PurePursuitProfile defaultProfile(new PID(20, 0.0, 0.0), new PID(30, 0.0, 0.0), 10.0);
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

	// Right Steal Right
	Path rightHomeToGoalNeutral;

	rightHomeToGoalNeutral.addPoint(105.7, 8);
	rightHomeToGoalNeutral.addPoint(105.7, 60);

	rightHomeToGoalNeutralIndex = purePursuit.addPath(rightHomeToGoalNeutral);

	Path rightNeutralToMidNeutral;

	rightNeutralToMidNeutral.addPoint(105.7, 60);
	rightNeutralToMidNeutral.addPoint(82.3, 40);
	rightNeutralToMidNeutral.addPoint(70.3, 60);

	rightNeutralToMidNeutralIndex = purePursuit.addPath(rightNeutralToMidNeutral);

	Path midNeutralToRightAlliance;

	midNeutralToRightAlliance.addPoint(70.3, 60);
	midNeutralToRightAlliance.addPoint(120.1, 36);

	midNeutralToRightAllianceIndex = purePursuit.addPath(midNeutralToRightAlliance);

	Path rightAllianceToRightRing;

	rightAllianceToRightRing.addPoint(120.1, 36);
	rightAllianceToRightRing.addPoint(117.5, 46.8);
	rightAllianceToRightRing.addPoint(117.5, 70.3);
	rightAllianceToRightRing.addPoint(117.5, 70.3);

	rightAllianceToRightRingIndex = purePursuit.addPath(rightAllianceToRightRing);

	Path rightRingToLeftHomeZone;

	rightRingToLeftHomeZone.addPoint(117.5, 70.3);
	rightRingToLeftHomeZone.addPoint(105, 35);
	rightRingToLeftHomeZone.addPoint(35, 35);

	rightRingToLeftHomeZoneIndex = purePursuit.addPath(rightRingToLeftHomeZone);
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
	initSelector();
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
}


/**
 * Runs during operator/teleop control
 */
void opcontrol() {

	printf("OpControl");
	lv_obj_clean(lv_scr_act());

	//lv_obj_t* infoLabel = lv_label_create(lv_scr_act(), NULL);
	// lv_label_set_text(infoLabel, "");

	const int runningAverageLength = 1;
	RunningAverage<runningAverageLength> leftXAvg;
	RunningAverage<runningAverageLength> leftYAvg;
	RunningAverage<runningAverageLength> rightXAvg;

	MotorButton leftLiftButton(&master, &leftLift, DIGITAL_L1, DIGITAL_L2, 127, 0, -127, 0, 0);
	MotorButton rightLiftButton(&master, &rightLift, DIGITAL_L1, DIGITAL_L2, 127, 0, -127, 0, 0);
	MotorButton backGrabberButton(&master, &backGrabber, DIGITAL_R1, DIGITAL_R2, 200, 200, 200, 0, 450*3);
	backGrabberButton.setSingleToggle(true);
	backGrabberButton.setGoToImmediately(true);

	SolenoidButton frontGrabberButton(&master, DIGITAL_A, DIGITAL_B);
	frontGrabberButton.setSolenoid(&frontGrabber);
	frontGrabberButton.setSingleToggle(true);

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

		leftLiftButton.update();
		rightLiftButton.update();
		frontGrabberButton.update();
		backGrabberButton.update();
		if (master.get_digital_new_press(DIGITAL_X)) {
			odometry.reset(new Position());
		}

		odometry.update();

		// Prevent wasted resources
		pros::delay(10);
	}
}
