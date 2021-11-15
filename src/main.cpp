#include "main.h"

// Auton Selector object
autonSelector* autonomousSel = nullptr;

// Controllers
Pronounce::Controller master(pros::E_CONTROLLER_MASTER);

// Motors

// Drive Motors
pros::Motor frontLeftMotor(1);
pros::Motor frontRightMotor(2, true);
pros::Motor backLeftMotor(9);
pros::Motor backRightMotor(10, true);

pros::ADIEncoder leftEncoder(2, 1, true);
pros::ADIEncoder rightEncoder(4, 3, true);
pros::ADIEncoder backEncoder(6, 5, false);

Pronounce::AdiOdomWheel leftOdom(&leftEncoder);
Pronounce::AdiOdomWheel rightOdom(&rightEncoder);
Pronounce::AdiOdomWheel backOdom(&backEncoder);

Pronounce::ThreeWheelOdom threeWheelOdom(&leftOdom, &rightOdom, &backOdom);

// Inertial Measurement Unit
pros::Imu imu(3);
MecanumDrivetrain drivetrain(&frontLeftMotor, &frontRightMotor, &backLeftMotor, &backRightMotor, &imu, &threeWheelOdom);

Pronounce::PurePursuit purePursuit(&drivetrain, 15);

Position* startingPosition = new Position(0, 0, 0);

bool relativeMovement = false;
bool driveOdomEnabled = true;

#define ROLL_AUTHORITY 1.0

#define DRIFT_MIN 7.0

bool preDriverTasksDone = false;

int testPathIndex;

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
 * Left AWP Right
 * Scores AWP and 11 rings
 */
int leftAwpRight() {
	return 0;
}

/**
 * @brief Right Awp Left
 *
 * @return Status - needed for AutonSelector
 */
int rightAwpLeft() {
	return 0;
}

int rightStealRight() {
	return 0;
}

/**
 * @brief Test auton
 *
 * @return 0
 */
int testAuton() {
	threeWheelOdom.setPosition(new Position());

	purePursuit.setCurrentPathIndex(testPathIndex);
	purePursuit.setFollowing(true);

	pros::Task::delay(15000);

	return 0;
}

int postAuton() {
	purePursuit.setEnabled(false);
	purePursuit.setFollowing(false);
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
	lv_label_set_text(infoLabel, "opcontrol()");
	while (1) {
		uint32_t startTime = pros::millis();
		purePursuit.update();
		lv_label_set_text(infoLabel, threeWheelOdom.getPosition()->to_string().c_str());
		pros::Task::delay_until(&startTime, 10);
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

/**
 * Initialize all motors
 */
void initMotors() {
	// Motor brake modes
	frontLeftMotor.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);
	frontRightMotor.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);
	backLeftMotor.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);
	backRightMotor.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);
}

/**
 * Initialize the controller
 */
void initController() {
	master.setDrivetrain(&drivetrain);
	pros::Task renderTask(renderThread);
}

/**
 * Initialize the Auton Selector
 */
void initSelector() {
	// Create a button descriptor string array w/ no repeat "\224"
	static char* btnm_map[] = { (char*)"Test",
								(char*)"" };

	autonomousSel = new autonSelector(btnm_map, lv_scr_act());

	// Set pre and post run
	autonomousSel->setPreRun(preAutonRun);
	autonomousSel->setPostAuton(postAuton);

	// Set functions
	autonomousSel->setFunction(0, testAuton);

	autonomousSel->setSelection(0);
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
	Path testPath = Path();
	testPath.addPoint(0, 0);
	testPath.addPoint(24, 0);
	testPath.addPoint(24, 24);
	testPath.addPoint(24, 48);
	testPath.addPoint(-24, 48);
	testPath.addPoint(0, 24);

	purePursuit.addPath(testPath);
	testPathIndex = 0;
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
	initController();
	initSelector();
	initLogger();
	autoPaths();

	leftEncoder.reset();
	leftOdom.setRadius(1.625);
	leftOdom.setTuningFactor(1.003);
	rightEncoder.reset();
	rightOdom.setRadius(1.625);
	rightOdom.setTuningFactor(1.003);
	backEncoder.reset();
	backOdom.setRadius(1.625);
	backOdom.setTuningFactor(1.003);

	//pros::Task::delay(100);

	threeWheelOdom.setBackOffset(3.25);
	threeWheelOdom.setLeftOffset(3.87);
	threeWheelOdom.setRightOffset(3.87);

	purePursuit.setAnglePid(new PID(0, 0, 0));
	purePursuit.setLateralPid(new PID(30, 0, 0));
	purePursuit.setNormalizeDistance(10);
	purePursuit.setLookahead(10);

	purePursuit.setOdometry(&threeWheelOdom);

	pros::Task purePursuitTast = pros::Task(updateDrivetrain, "Pure Pursuit");

	threeWheelOdom.setPosition(new Position());
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

}

/**
 * Runs during the autonomous. NO user control
 */
void autonomous() {
	// This calls the user selection, all the functions prototypes are in 
	// autonRoutines.hpp and the implementation is autonRoutines.cpp
	autonomousSel->runSelection();

}


/**
 * Runs during operator/teleop control
 */
void opcontrol() {


	lv_obj_t* infoLabel = lv_label_create(lv_scr_act(), NULL);
	lv_label_set_text(infoLabel, "");

	const int runningAverageLength = 25;
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
		driveVector.setAngle((driveVector.getAngle()) + threeWheelOdom.getPosition()->getTheta());

		// Send variables to motors
		drivetrain.setDriveVectorVelocity(driveVector, rightX);

		threeWheelOdom.update();

		// Prevent wasted resources
		pros::delay(10);
	}
}
