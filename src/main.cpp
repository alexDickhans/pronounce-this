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
pros::Motor frontFlipperMotor1(3);
pros::Motor frontFlipperMotor2(4, true);
pros::Motor backFlipperMotor(7, MOTOR_GEARSET_36, true);

Pronounce::MotorOdom frontLeftOdom(&frontLeftMotor, 50.8);
Pronounce::MotorOdom frontRightOdom(&frontRightMotor, 50.8);

// Inertial Measurement Unit
pros::Imu imu(5);
Drivetrain drivetrain(&frontLeftMotor, &frontRightMotor, &backLeftMotor, &backRightMotor, &imu);

TankOdom tankOdom(&frontLeftOdom, &frontRightOdom, &imu);

bool relativeMovement = false;
bool driveOdomEnabled = true;

#define ROLL_AUTHORITY 1.0

#define DRIFT_MIN 7.0

/**
 * Render thread to update items on the controller
 */
void renderThread() {
	master.renderFunc();
}

/**
 * Initialize all sensors
 */
void initSensors() {

	// Calibrate only if it is not being calibrated.
	if (!imu.is_calibrating()) imu.reset();

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
	pros::Task task(renderThread);
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
	static char* btnm_map[] = { (char*)"Top Left", (char*)"Top Right", (char*)"\n",
									 (char*)"Misc Left", (char*)"Misc Right", (char*)"\n",
									 (char*)"Bottom Left", (char*)"Bottom Left", (char*)"\n",
											(char*)"Skills", (char*)"" };

	autonomousSel = new autonSelector(btnm_map, lv_scr_act());

	// Set functions
	autonomousSel->setFunction(0, topLeft);
	autonomousSel->setFunction(1, topRight);

	autonomousSel->setFunction(3, miscLeft);
	autonomousSel->setFunction(4, miscRight);

	autonomousSel->setFunction(6, bottomLeft);
	autonomousSel->setFunction(7, bottomRight);

	autonomousSel->setFunction(9, skills);
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

/**
 * Runs when the robot starts up
 */
void initialize() {

	lv_init();

	// Initialize functions
	//initSensors();
	initMotors();
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

	// Delete all items on screen
	lv_obj_clean(lv_scr_act());

	// Condensed way to put a few pieces of information on screen
	lv_obj_t* infoLabel = lv_label_create(lv_scr_act(), NULL);

	// Motor buttons
	MotorButton intakeButton(&master, &intakeMotor, DIGITAL_R1, DIGITAL_R2, 85, 0, -127, 0, 0);

	MotorButton frontFlipperButton1(&master, &frontFlipperMotor1, DIGITAL_L1, DIGITAL_L2, 127, 0, -127, 0, 100);
	MotorButton frontFlipperButton2(&master, &frontFlipperMotor2, DIGITAL_L1, DIGITAL_L2, 127, 0, -127, 0, 100);
	MotorButton backFlipperButton(&master, &backFlipperMotor, DIGITAL_X, DIGITAL_A, 100, 0, -200, 0, 3700);
	backFlipperButton.setGoToImmediately(true);

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
			// Used for testing how well the inertial sensor will keep orientation
			lv_label_set_text(infoLabel,  tankOdom.to_string().c_str());
		}

		tankOdom.update();

		// Buttons
		intakeButton.update();
		frontFlipperButton1.update();
		frontFlipperButton2.update();
		backFlipperButton.update();

		// Prevent wasted resources
		pros::delay(10);
	}
}
