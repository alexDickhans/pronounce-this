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
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	lv_init();

	initSensors();	

	frontLeftMotor.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);
	frontRightMotor.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);
	backLeftMotor.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);
	backRightMotor.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);
	
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
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
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
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	// This calls the user selection, all the functions prototypes are in 
	// autonRoutines.hpp and the implementation is autonRoutines.cpp
	autonomousSel->runSelection();

}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
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
		int leftX = pow(master.get_analog(ANALOG_LEFT_X), 3) * 0.00009;
		int leftY = pow(master.get_analog(ANALOG_LEFT_Y), 3) * 0.00009;
		int roll = master.get_analog(ANALOG_RIGHT_X) * ROLL_AUTHORITY;

		// Calculate the controller vector + mixing
		double magnitude = sqrt(pow(leftX, 2) + pow(leftY, 2));
		double leftJoystickAngle = (atan2(leftY, leftX) * 180/PI);
		double angle = std::fmod(leftJoystickAngle + imu.get_rotation(), 360.0);

		// Create a switch between relative movement on and off
		double computedX;
		double computedY;

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
