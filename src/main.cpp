#include "main.h"

// Auton Selector object
autonSelector* autonomousSel;

// Controllers
pros::Controller master(pros::E_CONTROLLER_MASTER);

// Motors

// Drive Motors
pros::Motor frontLeftMotor(1);
pros::Motor frontRightMotor(3, true);
pros::Motor backLeftMotor(2);
pros::Motor backRightMotor(4, true);

// Inertial Measurement Unit
pros::Imu imu(5);

bool relativeMovement = true;


#define ROLL_AUTHORITY 1.0
#define STOP_ROLL_IF_CALIBRATING true

#define DRIFT_MIN 3

/**
 * Initialize all sensors
 */
void initSensors() {
	
	// Calibrate only if it is not being calibrated.
	if(!imu.is_calibrating()) imu.reset();

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
 * Filter and apply the quadratic function.
 */
double filterAxis(pros::Controller controller, pros::controller_analog_e_t controllerAxis) {
	// Remove drift
	double controllerValue = controller.get_analog(controllerAxis);
	double controllerFilter = abs(controllerValue) < DRIFT_MIN ? 0.0 : controllerValue;
	
	// Apply quadratic function 
	// f(x) = controllerFilter ^ 3 * 0.00009
	double quadraticFilter = pow(controllerFilter, 3) * 0.00009;

	// Return solution
	return quadraticFilter;
}

/**
 * Runs when the robot starts up
 */
void initialize() {
	lv_init();

	// Initialize functions
	initSensors();
	initMotors();
}

/**
 * Runs while the robot is disabled i.e. before and after match, between auton
 * and teleop period
 */
void disabled() {

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

	// Create a button descriptor string array w/ no repeat "\224"
  	static char * btnm_map[] = { "Top Left", "Top Right", "\n",
  									 "Misc Left", "Misc Right", "\n",
                                     "Bottom Left", "Bottom Left", "\n",
									 		"Skills", ""};

	autonomousSel = new autonSelector(btnm_map, lv_scr_act());

	// Set functions
	autonomousSel->setFunction(0, topLeft);
	autonomousSel->setFunction(1, topRight);
	
	autonomousSel->setFunction(3, miscLeft);
	autonomousSel->setFunction(4, miscRight);

	autonomousSel->setFunction(6, bottomLeft);
	autonomousSel->setFunction(7, bottomRight);

	autonomousSel->setFunction(9, skills);

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
	if (!pros::competition::is_connected()) {
		// Choose auton
		// autonomousSel->choose();
	}

	// Delete all items on screen
	lv_obj_clean(lv_scr_act());

	// Condensed way to put a few pieces of information on screen
	lv_obj_t* infoLabel = lv_label_create(lv_scr_act(), NULL);

	// Variable to hold imu data during reset
	double degrees = 0;

	// Driver Control Loop
	while (true) {
		// Filter input
		int leftX = filterAxis(master, ANALOG_LEFT_X);
		int leftY = filterAxis(master, ANALOG_LEFT_Y);
		int roll = STOP_ROLL_IF_CALIBRATING ? 0 : filterAxis(master, ANALOG_RIGHT_X) * ROLL_AUTHORITY;

		// Calculate the controller vector + mixing
		double magnitude = sqrt(pow(leftX, 2) + pow(leftY, 2)); // √leftX^2 + leftY^2
		double leftJoystickAngle = atan2(leftY, leftX) * 180/PI; // atan(leftY, leftX) * 180/PI
		double angle = std::fmod(leftJoystickAngle + imu.get_rotation(), 360.0); 
		// leftJoystickAngle + imu.get_rotation() % 360.0

		// Create a switch between relative movement on and off
		double computedX;
		double computedY;

		// Used to hold degrees while calibrating
		if (!imu.is_calibrating()) {
			degrees = imu.get_rotation();
		}

		// Use a switch
		// I was debating weather to switch modes based on the status of the imu, but I thought 
		// it would be a much more usable experience if the robot did not automatically switch modes.
		if (relativeMovement/* || imu.is_calibrating()*/) { 
			computedX = magnitude * cos(angle * PI/180);
			computedY = magnitude * sin(angle * PI/180);
		} else {
			computedX = leftX;
			computedY = leftY;
		}
		
		// Send parameters to motors
		frontLeftMotor.move(computedY + computedX + roll);
		frontRightMotor.move(computedY - computedX - roll);
		backLeftMotor.move(computedY - computedX + roll);
		backRightMotor.move(computedY + computedX - roll);

		// Used for testing how well the inertial sensor will keep orientation
		lv_label_set_text(infoLabel, std::to_string(imu.get_rotation()).c_str());

		// Buttons
		if (master.get_digital(DIGITAL_Y)) {
			relativeMovement = !relativeMovement;
		}
		if (master.get_digital(DIGITAL_X)) {
			imu.reset();
		}

		// Prevent wasted resources
		pros::delay(20);
	}
}
