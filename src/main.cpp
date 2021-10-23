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


// Flippers
pros::Motor frontLiftLeftMotor(4, MOTOR_GEARSET_36, true);
pros::Motor frontLiftRightMotor(5, MOTOR_GEARSET_36, false);
pros::Motor frontFlipperMotor(6, MOTOR_GEARSET_36, true);
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

	// frontLeftMotor.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
	// frontRightMotor.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
	// backLeftMotor.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
	// backRightMotor.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);

	// Drivetrain
	tankDrivetrain.setEnabled(true);

	// Back flipper
	backFlipperMotor.move_absolute(2000, 200);
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

	// Move to left goal
	tankDrivetrain.setTargetPosition(new Position(35, 11.5));
	tankDrivetrain.waitForStop();

	// Pick up left goal
	frontFlipperMotor.move_absolute(20 * 6, 200);

	// Move to line
	tankDrivetrain.setTargetPosition(new Position(33.5, 33.5, -1));
	pros::Task::delay(500);
	tankDrivetrain.waitForStop();

	// Collect rings
	tankDrivetrain.setTargetPosition(new Position(94, 46.8));
	pros::Task::delay(500);
	tankDrivetrain.waitForStop();

	// Manually turn
	tankDrivetrain.setAngle(0);
	tankDrivetrain.waitForStop();

	// Set down goal
	frontFlipperMotor.move_absolute(0, 200);

	// Drop goal backwards
	tankDrivetrain.setTargetPosition(new Position(2600, 43, -1));
	pros::Task::delay(500);
	tankDrivetrain.waitForStop();

	// Move to right goal
	tankDrivetrain.setTargetPosition(new Position(124, 35));
	pros::Task::delay(500);
	tankDrivetrain.waitForStop();

	// Pick up goal
	frontFlipperMotor.move_absolute(20 * 6, 200);

	// Move off AWP
	tankDrivetrain.setTargetPosition(new Position(114, 23.2, -1));
	pros::Task::delay(500);
	tankDrivetrain.waitForStop();

	pros::Task::delay(1000);

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
	frontFlipperMotor.move_absolute(30, 200);

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

	tankDrivetrain.setEnabled(true);

	startingPosition->setX(0);
	startingPosition->setY(0);
	startingPosition->setTheta(90);

	tankDrivetrain.setStartingPosition(startingPosition);

	tankDrivetrain.setTargetPosition(new Position(24, 24));
	tankDrivetrain.waitForStop();

	pros::Task::delay(4000);

	// tankDrivetrain.setAngle(180);
	// tankDrivetrain.waitForStop();

	// pros::Task::delay(5000);

	// tankDrivetrain.setAngle(0);
	// pros::Task::delay(500);
	// tankDrivetrain.waitForStop();

	// pros::Task::delay(2000);

	// pros::Task::delay(2000);

	tankDrivetrain.setTargetPosition(new Position(0, 0, -1));
	pros::Task::delay(500);
	tankDrivetrain.waitForStop();

	return 0;
}


int preDriver() {
	preDriverTasksDone = true;
	return 0;
}

int postAuton() {
	tankDrivetrain.setEnabled(false);
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
pros::task_fn_t tankDriveThread(void) {
	while (true) {
		tankDrivetrain.update();

		uint32_t now = pros::millis();
		pros::Task::delay_until(&now, 50);
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

	frontFlipperMotor.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);
	frontFlipperMotor.set_encoder_units(MOTOR_ENCODER_DEGREES);
	backFlipperMotor.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);

	frontLiftLeftMotor.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);
	frontLiftRightMotor.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);
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
								(char*)"Test",
								(char*)"" };

	autonomousSel = new autonSelector(btnm_map, lv_scr_act());

	// Set pre and post run
	autonomousSel->setPreRun(nullAutonFunc);
	autonomousSel->setPostAuton(postAuton);

	// Set functions
	autonomousSel->setFunction(0, leftAwpRight);
	autonomousSel->setFunction(2, rightAwpLeft);
	autonomousSel->setFunction(4, rightStealRight);
	autonomousSel->setFunction(6, testAuton);

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

void initDrivetrain() {

	tankDrivetrain.getTankOdom()->getLeftPivot()->setTuningFactor(1.0);
	tankDrivetrain.getTankOdom()->getRightPivot()->setTuningFactor(1.0);

	tankDrivetrain.getTankOdom()->setTuningFactor(1.0);

	PID* turnPid = new PID(4, 0.0, -2);
	PID* movePid = new PID(15.0, 0.0, -10.0);
	tankDrivetrain.setTurnPid(turnPid);
	tankDrivetrain.setMovePid(movePid);

	tankDrivetrain.setStartingPosition(startingPosition);

	pros::Task tankDriveTask(tankDriveThread, "TankDrive");
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


void updateVisionTask() {
	while (1) {
		vision.updateAngles();
		pros::Task::delay(500);
	}
}

void reset() {
	tankDrivetrain.reset();
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
	MotorButton frontFlipperButton(&master, &frontFlipperMotor, DIGITAL_R1, DIGITAL_R2, 90, 0, -90, 0, 0);
	MotorButton frontLiftLeftButton(&master, &frontLiftLeftMotor, DIGITAL_L1, DIGITAL_L2, 90, 0, -90, 0, 0);
	MotorButton frontLiftRightButton(&master, &frontLiftRightMotor, DIGITAL_L1, DIGITAL_L2, 90, 0, -90, 0, 0);
	MotorButton backFlipperButton(&master, &backFlipperMotor, DIGITAL_X, DIGITAL_A, 100, 0, -200, 2000, 3700);
	backFlipperButton.setGoToImmediately(true);

	// Driver Control Loop
	while (true) {

		// Filter and calculate magnitudes
		int leftWheelMag = filterAxis(master, ANALOG_LEFT_Y);
		int rightWheelMag = filterAxis(master, ANALOG_RIGHT_Y);

		// Send variables to motors
		frontLeftMotor.move_velocity(leftWheelMag);
		backLeftMotor.move_velocity(leftWheelMag);
		frontRightMotor.move_velocity(rightWheelMag);
		backRightMotor.move_velocity(rightWheelMag);


		if (leftWheelMag == 0) {
			frontLeftMotor.move_velocity(0.0);
			backLeftMotor.move_velocity(0.0);
		}


		// Used to test odom on the robot currently
		if (driveOdomEnabled) {
			//tankDrivetrain.getTankOdom()->update();

			// Used for testing how well the inertial sensor will keep orientation
			lv_label_set_text(infoLabel, tankDrivetrain.getTankOdom()->to_string().c_str());
		}

		if (master.get_digital_new_press(DIGITAL_Y)) {
			reset();
		}

		if (master.get_digital(DIGITAL_B)) {
			preAutonRun();
			testAuton();
			postAuton();
		} else {
			tankDrivetrain.setEnabled(false);
		}

		// Buttons
		frontFlipperButton.update();
		frontLiftLeftButton.update();
		frontLiftRightButton.update();
		backFlipperButton.update();
		
		tankDrivetrain.update();

		// Prevent wasted resources
		pros::delay(10);
	}
}
