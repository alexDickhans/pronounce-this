#include "main.h"

autonSelector* autonomousSel;

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::Motor frontLeftMotor(1);
pros::Motor frontRightMotor(3, true);
pros::Motor backLeftMotor(2);
pros::Motor backRightMotor(4, true);

pros::Imu imu(5);

bool relativeMovement = true;

#define ROLL_AUTHORITY 1.0

void initSensors() {
	while (imu.is_calibrating()) {
		pros::delay(20);
	}
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

	

	frontLeftMotor.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);
	frontRightMotor.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);
	backLeftMotor.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);
	backRightMotor.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);
	
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

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
		//autonomousSel->choose();
	}

	// Condensed way to put a few pieces of information on screen
	lv_obj_t* infoLabel = lv_label_create(lv_scr_act(), NULL);

	// Driver Control Loop
	while (true) {
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

		if (relativeMovement) {
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
