#include "main.h"

// Auton Selector object
autonSelector* autonomousSel = nullptr;

// Controllers
Pronounce::Controller master(pros::E_CONTROLLER_MASTER);

// Our vision implementation
PronounceTiP::Vision vision(9);

// Motors

// Drive Motors
pros::Motor frontLeftMotor(1);
pros::Motor frontRightMotor(2, true);
pros::Motor backLeftMotor(9);
pros::Motor backRightMotor(10, true);

// Intake
pros::Motor intakeMotor(8);

// Flippers
pros::Motor frontFlipperMotor1(4, MOTOR_GEARSET_36, true);
pros::Motor frontFlipperMotor2(5, MOTOR_GEARSET_36, false);
pros::Motor backFlipperMotor(7, MOTOR_GEARSET_36, true);

Pronounce::MotorOdom frontLeftOdom(&frontLeftMotor, 2);
Pronounce::MotorOdom frontRightOdom(&frontRightMotor, 2);

// Inertial Measurement Unit
pros::Imu imu(3);
Drivetrain drivetrain(&frontLeftMotor, &frontRightMotor, &backLeftMotor, &backRightMotor, &imu);

Pronounce::TankDrivetrain tankDrivetrain(&frontLeftMotor, &frontRightMotor, &backLeftMotor, &backRightMotor, &imu);

Position* startingPosition = new Position(0, 0, 0);

bool relativeMovement = false;
bool driveOdomEnabled = true;

#define ROLL_AUTHORITY 1.0

#define DRIFT_MIN 7.0

bool preDriverTasksDone = false;

/**
 * @brief Runs during auton period before auton
 * 
 */
int preAutonRun() {
	// Drivetrain
	tankDrivetrain.setEnabled(true);

	// Back flipper
	backFlipperMotor.move_absolute(2500, 200);
	return 0;
}

/**
 * Left AWP Right
 * Scores AWP and 11 rings
 */
int leftAwpRight() {
	startingPosition->setX(21);
	startingPosition->setY(9);
	startingPosition->setTheta(90);

	tankDrivetrain.setStartingPosition(startingPosition);

	tankDrivetrain.setEnabled(true);
	
	// Move to left goal
	tankDrivetrain.setTargetPosition(new Position(35, 11.5));
	tankDrivetrain.waitForStop();

	// Pick up left goal
	frontFlipperMotor1.move_absolute(20*6, 200);
	frontFlipperMotor1.move_absolute(20*6, 200);

	// Move to line
	tankDrivetrain.setTargetPosition(new Position(33.5, 33.5, -1));
	tankDrivetrain.waitForStop();

	// Start collecting and scoring rings
	intakeMotor.move(127);

	// Collect rings
	tankDrivetrain.setTargetPosition(new Position(94, 46.8));
	tankDrivetrain.waitForStop();

	// Manually turn
	tankDrivetrain.setAngle(0);
	tankDrivetrain.waitForStop();

	// Set down goal
	frontFlipperMotor1.move_absolute(0, 200);

	// Stop collecting and scoring rings
	intakeMotor.move(0);

	// Drop goal backwards
	tankDrivetrain.setTargetPosition(new Position(2600, 43, -1));
	tankDrivetrain.waitForStop();

	// Move to right goal
	tankDrivetrain.setTargetPosition(new Position(124, 35));
	tankDrivetrain.waitForStop();

	// Pick up goal
	frontFlipperMotor1.move_absolute(20*6, 200);
	frontFlipperMotor1.move_absolute(20*6, 200);

	// Move off AWP
	tankDrivetrain.setTargetPosition(new Position(114, 23.2, -1));

	// Score in goal
	intakeMotor.move(127);

	pros::Task::delay(1000);

	intakeMotor.move(0);

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
	startingPosition->setX(105.7);
	startingPosition->setY(12);
	startingPosition->setTheta(90);

	tankDrivetrain.setStartingPosition(startingPosition);

	// Move to right neutral goal
	tankDrivetrain.setTargetPosition(new Position(105.7, 62));
	tankDrivetrain.waitForStop();

	// Pick up goal
	frontFlipperMotor1.move_absolute(30, 200);
	frontFlipperMotor2.move_absolute(30, 200);

	// Move to other goal
	tankDrivetrain.setAngle(180);
	tankDrivetrain.waitForStop();
	tankDrivetrain.setTargetPosition(new Position(105.7, 73.3, -1));
	tankDrivetrain.waitForStop();

	// Pick up ring
	backFlipperMotor.move_absolute(3700, 200);

	// Move to the target position
	tankDrivetrain.setTargetPosition(new Position(130, 23));
	tankDrivetrain.waitForStop();

	// Get ready for match
	tankDrivetrain.setAngle(45);

	return 0;
}

/**
 * @brief Test auton
 * 
 * @return 0
 */
int testAuton() {
	return 0;
}


int preDriver() {
	preDriverTasksDone = true;
	return 0;
}

int postAuton() {
	preDriver();
	return 0;
}


/**
 * Render thread to update items on the controller
 */
void renderThread() {
	master.renderFunc();
}

/**
 * @brief Tank drive thread
 *
 */
void tankDriveThread() {
	while (true) {
		tankDrivetrain.update();

		uint32_t now = pros::millis();
		pros::Task::delay_until(&now, pros::millis() + 20);
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
	frontLeftMotor.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);
	frontRightMotor.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);
	backLeftMotor.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);
	backRightMotor.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);

	frontFlipperMotor1.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);
	frontFlipperMotor2.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);
	frontFlipperMotor1.set_encoder_units(MOTOR_ENCODER_DEGREES);
	frontFlipperMotor2.set_encoder_units(MOTOR_ENCODER_DEGREES);
	backFlipperMotor.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);
}

/**
 * Initialize the controller
 */
void initController() {
	master.setDrivetrain(&drivetrain);
	pros::Task renderTask(renderThread);
}

void initVision() {
	vision = PronounceTiP::Vision(9);
	vision.clear_led();
	vision.set_wifi_mode(0);

	pros::vision_signature_s_t BLUE_RING =
		pros::Vision::signature_from_utility(1, -2899, -1681, -2290, 8489, 12763, 10626, 3.000, 0);

	pros::vision_signature_s_t YELLOW_GOAL =
		pros::Vision::signature_from_utility(2, -8245, -5707, -6976, -7199, -3865, -5532, 3.000, 0);

	vision.set_signature(1, &BLUE_RING);
	vision.set_signature(2, &YELLOW_GOAL);
}

/**
 * Initialize the Auton Selector
 */
void initSelector() {
	// Create a button descriptor string array w/ no repeat "\224"
	static char* btnm_map[] = { (char*)"Left AWP Right", (char*)"\n",
								(char*)"Right AWP Left", (char*)"\n",
								(char*)"Right Steal Right", (char*)"\n",
								(char*)"Test", (char*)"\n",
								(char*)"" };

	autonomousSel = new autonSelector(btnm_map, lv_scr_act());
	
	// Set pre and post run
	autonomousSel->setPreRun(preAutonRun);
	autonomousSel->setPostAuton(postAuton);

	// Set functions
	autonomousSel->setFunction(0, leftAwpRight);
	autonomousSel->setFunction(2, rightAwpLeft);
	autonomousSel->setFunction(4, rightStealRight);
	autonomousSel->setFunction(6, testAuton);
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

void initDrivetrain() {
	pros::Task tankDriveTask(tankDriveThread);
	
	tankDrivetrain.getTankOdom()->getLeftPivot()->setTuningFactor(1.0);
	tankDrivetrain.getTankOdom()->getRightPivot()->setTuningFactor(1.0);

	tankDrivetrain.getTankOdom()->setTuningFactor(1.0);

	PID* turnPid = new PID(0.0, 0.0, 0.0);
	PID* movePid = new PID(0.0, 0.0, 0.0);
	tankDrivetrain.setTurnPid(turnPid);
	tankDrivetrain.setMovePid(movePid);

	tankDrivetrain.setStartingPosition(startingPosition);
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
	double quadraticFilter = pow(controllerFilter / 127.0, 3) * 127.0;

	// Return solution
	return quadraticFilter;
}


void updateVisionTask() {
	while (1) {
		vision.updateAngles();
		pros::Task::delay(500);
	}
}

void reset() {
}

/**
 * Runs when the robot starts up
 */
void initialize() {

	lv_init();

	// Initialize functions
	//initSensors();
	initMotors();
	initDrivetrain();
	initController();
	initSelector();
	initLogger();
	// initVision();

	// pros::Task visionTask = pros::Task(updateVisionTask, "Vision");
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

	// Show GUI
	autonomousSel->choose();

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
	
	if (!preDriverTasksDone) {
		preDriver();
	}

	// Delete all items on screen
	lv_obj_clean(lv_scr_act());

	// Condensed way to put a few pieces of information on screen
	lv_obj_t* infoLabel = lv_label_create(lv_scr_act(), NULL);

	// Motor buttons
	MotorButton intakeButton(&master, &intakeMotor, DIGITAL_R1, DIGITAL_R2, 85, 0, -127, 0, 0);

	MotorButton frontFlipperButton1(&master, &frontFlipperMotor1, DIGITAL_L1, DIGITAL_L2, 90, 0, -90, 0, 0);
	MotorButton frontFlipperButton2(&master, &frontFlipperMotor2, DIGITAL_L1, DIGITAL_L2, 90, 0, -90, 0, 0);
	MotorButton backFlipperButton(&master, &backFlipperMotor, DIGITAL_X, DIGITAL_A, 100, 0, -200, 0, 3700);
	backFlipperButton.setGoToImmediately(true);

	bool lastButton = false;

	// Driver Control Loop
	while (true) {
		// Filter input
		int leftX = master.get_analog(ANALOG_LEFT_X);
		int leftY = master.get_analog(ANALOG_LEFT_Y);

		// Filter and calculate magnitudes
		int leftWheelMag = filterAxis(master, ANALOG_LEFT_Y);
		int rightWheelMag = filterAxis(master, ANALOG_RIGHT_Y);

		// Send variables to motors
		frontLeftMotor.move(leftWheelMag);
		backLeftMotor.move(leftWheelMag);
		frontRightMotor.move(rightWheelMag);
		backRightMotor.move(rightWheelMag);

		// Used to test odom on the robot currently
		if (driveOdomEnabled) {
			tankDrivetrain.getTankOdom()->update();

			// Used for testing how well the inertial sensor will keep orientation
			lv_label_set_text(infoLabel, tankDrivetrain.getPosition()->to_string().c_str());
		}

		if (master.get_digital_new_press(DIGITAL_Y)) {
			reset();
		}

		// Buttons
		intakeButton.update();
		frontFlipperButton1.update();
		frontFlipperButton2.update();
		backFlipperButton.update();

		// Prevent wasted resources
		pros::delay(10);
	}
}
