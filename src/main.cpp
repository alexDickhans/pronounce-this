#include "main.h"

// Auton Selector object
autonSelector* autonomousSel = nullptr;

// Controllers
Pronounce::Controller master(pros::E_CONTROLLER_MASTER);

// Our vision implementation
// PronounceTiP::Vision vision(9);

// Motors

// Drive Motors
pros::Motor frontLeftMotor(1);
pros::Motor frontRightMotor(2, true);
pros::Motor backLeftMotor(9);
pros::Motor backRightMotor(10, true);

Pronounce::MotorOdom frontLeftOdom(&frontLeftMotor, 2);
Pronounce::MotorOdom frontRightOdom(&frontRightMotor, 2);

// Inertial Measurement Unit
pros::Imu imu(3);

TankDrivetrain drivetrain(&frontLeftMotor, &frontRightMotor, &backLeftMotor, &backRightMotor, &imu);

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

	while(imu.is_calibrating()) {
		pros::Task::delay(50);
	}

	// Drivetrain
	drivetrain.setEnabled(true);

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
	drivetrain.setEnabled(false);
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
		drivetrain.update();

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
}

/**
 * Initialize the controller
 */
void initController() {
	master.setDrivetrain(&drivetrain);
	pros::Task renderTask(renderThread);
}

// void initVision() {
// 	vision = PronounceTiP::Vision(9);
// 	vision.clear_led();
// 	vision.set_wifi_mode(0);

// 	pros::vision_signature_s_t BLUE_RING =
// 		pros::Vision::signature_from_utility(1, -2899, -1681, -2290, 8489, 12763, 10626, 3.000, 0);

// 	pros::vision_signature_s_t YELLOW_GOAL =
// 		pros::Vision::signature_from_utility(2, -8245, -5707, -6976, -7199, -3865, -5532, 3.000, 0);

// 	vision.set_signature(1, &BLUE_RING);
// 	vision.set_signature(2, &YELLOW_GOAL);
// }

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

	drivetrain.getTankOdom()->getOdomWheel()-> setTuningFactor(1.0);

	drivetrain.getTankOdom()->setTuningFactor(1.0);

	PID* turnPid = new PID(4, 0.0, -2);
	PID* movePid = new PID(10.0, 0.0, 0.0);
	drivetrain.setTurnPid(turnPid);
	drivetrain.setMovePid(movePid);

	drivetrain.setStartingPosition(startingPosition);

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


// void updateVisionTask() {
// 	while (1) {
// 		vision.updateAngles();
// 		pros::Task::delay(500);
// 	}
// }

void reset() {
	drivetrain.reset();
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
	//initDrivetrain();
	initController();
	initSelector();
	//initLogger();
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

}

/**
 * Runs during the autonomous. NO user control
 */
void autonomous() {
	// This calls the user selection, all the functions prototypes are in 
	// autonRoutines.hpp and the implementation is autonRoutines.cpp
	//autonomousSel->runSelection();

}


/**
 * Runs during operator/teleop control
 */
void opcontrol() {

	// Show GUI
	//autonomousSel->choose();

	drivetrain.setEnabled(false);

	if (!preDriverTasksDone) {
		preDriver();
	}

	lv_obj_t* infoLabel = lv_label_create(lv_scr_act(), NULL);
	lv_label_set_text(infoLabel, "Hello");

	// Driver Control Loop
	while (true) {

		// Filter and calculate magnitudes
		int leftY = filterAxis(master, ANALOG_LEFT_Y);
		int leftX = filterAxis(master, ANALOG_LEFT_X);
		int rightX = filterAxis(master, ANALOG_RIGHT_X);
		int leftWheelMag = filterAxis(master, ANALOG_LEFT_Y);
		int rightWheelMag = filterAxis(master, ANALOG_RIGHT_Y);

		// Send variables to motors
		frontLeftMotor.move_velocity(leftY + leftX + rightX);
		backLeftMotor.move_velocity(leftY - leftX + rightX);
		frontRightMotor.move_velocity(leftY - leftX - rightX);
		backRightMotor.move_velocity(leftY + leftX - rightX);

		drivetrain.update();

		// Used to test odom on the robot currently
		if (driveOdomEnabled) {
			// Used for testing how well the inertial sensor will keep orientation
			lv_label_set_text(infoLabel, drivetrain.getTankOdom()->to_string().c_str());
		}

		if (master.get_digital_new_press(DIGITAL_Y)) {
			reset();
		}

		// Prevent wasted resources
		pros::delay(10);
	}
}
