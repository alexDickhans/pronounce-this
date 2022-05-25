#include "main.h"

// Controllers
Pronounce::Controller master(pros::E_CONTROLLER_MASTER);
Pronounce::Controller partner(pros::E_CONTROLLER_PARTNER);

// Motors

// Drive Motors
pros::Motor frontRightMotor(3, pros::E_MOTOR_GEARSET_06, true);
pros::Motor backRightMotor(5, pros::E_MOTOR_GEARSET_06, false);
pros::Motor frontLeftMotor(6, pros::E_MOTOR_GEARSET_06, false);
pros::Motor backLeftMotor(8, pros::E_MOTOR_GEARSET_06, true);

pros::Motor intake(1, true);

// Inertial Measurement Unit
pros::Imu imu(19);

pros::Rotation leftEncoder(10);
pros::Rotation rightEncoder(9);
pros::Rotation backEncoder(9);

// Odom wheels
Pronounce::TrackingWheel leftOdomWheel(&leftEncoder);
Pronounce::TrackingWheel rightOdomWheel(&rightEncoder);
Pronounce::TrackingWheel backOdomWheel(&backEncoder);

// GPS sensor
pros::Gps gps(4, 0, 0, 90, 0.2, 0.2);
GpsOdometry gpsOdometry(&gps);

// ThreeWheelOdom odometry(&leftOdomWheel, &rightOdomWheel, &backOdomWheel, &imu);
ThreeWheelOdom odometry(&leftOdomWheel, &rightOdomWheel, &backOdomWheel, &imu);

MotorButton intakeButton(&master, &intake, DIGITAL_R2, DIGITAL_Y, 200, 0, -200, 0, 0);

pros::Vision turretVision(18, VISION_ZERO_CENTER);

grafanalib::GUIManager manager;

// Autonomous Selector
Pronounce::AutonSelector autonomousSelector;

// LVGL
lv_obj_t* tabview;

#define DRIFT_MIN 7.0

// SECTION Auton

/**
 * @brief Runs during auton period before auton
 *
 */
int preAutonRun() {
	return 0;
}

int autonTemplate() {
	odometry.reset(new Position(0, 0, 0));

	pros::Task::delay(500);

	return 0;
}

int postAuton() {
	return 0;
}

// !SECTION

// SECTION INIT
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
	frontLeftMotor.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
	frontRightMotor.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
	backLeftMotor.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
	backRightMotor.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);

	intakeButton.setSingleToggle(true);
	intakeButton.setDejam(true);
	intakeButton.setDejamAuthority(-200);
	intakeButton.setDejamSpeed(25);
	intakeButton.setDejamTime(100);
}

void initDrivetrain() {

	// odometry.setUseImu(false);
	// Left/Right fraction
	// 1.072124756
	// Left 99.57
	// Right 100.57
	double turningFactor = (((100.35 / 100.0) - 1.0) / 2);
	double tuningFactor = 0.998791278;
	leftOdomWheel.setRadius(2.75 / 2);
	leftOdomWheel.setTuningFactor(tuningFactor * (1 - turningFactor));
	rightOdomWheel.setRadius(2.75 / 2);
	rightOdomWheel.setTuningFactor(tuningFactor * (1 + turningFactor));

	leftEncoder.set_reversed(true);
	rightEncoder.set_reversed(false);

	odometry.setLeftOffset(3.303827647);
	odometry.setRightOffset(3.303827647);
	odometry.setBackOffset(0);

	odometry.setMaxMovement(0);

	pros::Task::delay(10);

	odometry.reset(new Position());
}

/**
 * Initialize the controller
 */
void initController() {
	master.setOdometry(&odometry);
	partner.setOdometry(&odometry);
	pros::Task renderTask(renderThread);
}

// Run selector as task
void runSelector() {
	pros::Task::delay(1000);
	autonomousSelector.choose();
}

/**
 * Initialize the logger for data collection after the match
 */
void initLogger() {

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
	double quadraticFilter = pow(controllerFilter / 127.0, 3) * 620;

	// Return solution
	return quadraticFilter;
}

void initGrafanaLib() {
	manager.setRefreshRate(50);

	grafanalib::Variable<Pronounce::Odometry> odometryVar("Odometry", odometry);

	grafanalib::VariableGroup<Pronounce::Odometry> odometryVarGroup({odometryVar});

	odometryVarGroup.add_getter("X", &Pronounce::Odometry::getX);
	odometryVarGroup.add_getter("Y", &Pronounce::Odometry::getY);
	odometryVarGroup.add_getter("Angle", &Pronounce::Odometry::getTheta);

	manager.registerDataHandler(&odometryVarGroup);

	manager.startTask();
}

/**
 * Runs when the robot starts up
 */
void initialize() {

	lv_init();
	tabview = lv_tabview_create(lv_scr_act(), NULL);

	// Initialize functions
	initSensors();
	initMotors();
	initDrivetrain();
	initController();
	initLogger();
	initGrafanaLib();
}

// !SECTION

// SECTION Disabled
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

// !SECTION

// SECTION Competition Initialize

/**
 * Starts when connected to the field
 */
void competition_initialize() {
	// autonomousSelector.choose();

}

// !SECTION

// SECTION Auton

/**
 * Runs during the autonomous. NO user control
 */
void autonomous() {
	// This calls the user selection, all the functions prototypes are in 
	// autonRoutines.hpp and the implementation is autonRoutines.cpp
	// autonomousSelector.run();
	preAutonRun();
	autonTemplate();
	postAuton();

	// autonomousSelector.run();
}

// !SECTION

// SECTION Operator Control

/**
 * Runs during operator/teleop control
 */
void opcontrol() {

	postAuton();
	//lv_obj_t* infoLabel = lv_label_create(lv_scr_act(), NULL);
	// lv_label_set_text(infoLabel, "");

	// Driver Control Loop
	while (true) {

		// Filter and calculate magnitudes
		int leftY = filterAxis(master, ANALOG_LEFT_Y);
		int leftX = filterAxis(master, ANALOG_LEFT_X);
		int rightX = filterAxis(master, ANALOG_RIGHT_X);

		pros::delay(10);
	}
}

// !SECTION
