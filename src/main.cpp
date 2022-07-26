#include "main.h"

// LVGL
lv_obj_t* tabview;

RobotStatus robotStatus;
ModeLogic modeLogic(&robotStatus);
TeleopModeLogic teleopModeLogic(new pros::Controller(CONTROLLER_MASTER), new pros::Controller(CONTROLLER_PARTNER));

// SECTION Auton

/**
 * @brief Runs during auton period before auton
 *
 */
int preAutonRun() {
	teleopController.useDefaultBehavior();
	return 0;
}

int autonTemplate() {
	odometry.reset(new Pose2D(0, 0, 0));

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
 * Initialize the controller
 */
void initController() {
	master.setOdometry(&odometry);
	partner.setOdometry(&odometry);
	pros::Task renderTask(renderThread);
}

/**
 * Initialize the logger for data collection after the match
 */
void initLogger() {

}

void update() {

	uint32_t startTime = 0;
	while (true) {
		// Create stuff for exact delay
		std::cout << "Frame time: " << pros::millis() - startTime << std::endl;
		startTime = pros::millis();

		modeLogic.update();
		odometry.update();

		pros::delay(10 - (pros::millis() - startTime));
	}
}

void updateDisplay() {

	// Odom
	lv_obj_t* odomTab = lv_tabview_add_tab(tabview, "Odom");
	lv_obj_t* odomLabel = lv_label_create(odomTab, NULL);

	// Drivetrain
	lv_obj_t* drivetrainTab = lv_tabview_add_tab(tabview, "Drivetrain");
	lv_obj_t* drivetrainTable = lv_table_create(drivetrainTab, NULL);

	lv_table_set_row_cnt(drivetrainTable, 2);
	lv_table_set_col_cnt(drivetrainTable, 2);

	lv_table_set_col_width(drivetrainTable, 0, 200);
	lv_table_set_col_width(drivetrainTable, 1, 200);

	// Flywheels
	lv_obj_t* flywheelTab = lv_tabview_add_tab(tabview, "Flywheel");
	lv_obj_t* flywheelLabel = lv_label_create(flywheelTab, NULL);

	while (true) {
		// Odometry
		lv_label_set_text(odomLabel, (odometry.getPosition()->to_string()
			+ "\nL: " + std::to_string(leftOdomWheel.getPosition().Convert(inch)) +
			", R: " + std::to_string(rightOdomWheel.getPosition().Convert(inch))).c_str());

		// Drivetrain
		lv_table_set_cell_value(drivetrainTable, 0, 0, (std::to_string(frontLeftMotor.get_temperature()) + " C").c_str());
		lv_table_set_cell_value(drivetrainTable, 1, 0, (std::to_string(backLeftMotor.get_temperature()) + " C").c_str());
		lv_table_set_cell_value(drivetrainTable, 0, 1, (std::to_string(frontRightMotor.get_temperature()) + " C").c_str());
		lv_table_set_cell_value(drivetrainTable, 1, 1, (std::to_string(backRightMotor.get_temperature()) + " C").c_str());

		// Flywheel
		lv_label_set_text(flywheelLabel, ("\nTarget Speed: " + std::to_string(robotStatus.getFlywheelTarget()) +
			"\nCurrent Speed: " + std::to_string(robotStatus.getActualFlywheelRpm()) +
			"\nVoltage: " + std::to_string(flywheel1.get_voltage())).c_str());

		pros::Task::delay(50);
	}
}

void initDisplay() {
	pros::Task display(updateDisplay);
}

/**
 * Runs when the robot starts up
 */
void initialize() {

	lv_init();
	tabview = lv_tabview_create(lv_scr_act(), NULL);

	// Initialize functions
	initSensors();
	initDrivetrain();
	initController();
	initLogger();
	initController();
	initLauncherStates();
	initDisplay();
	initIntake();

	modeLogic.initialize();

	pros::Task modeLogicTask(update);

}

// !SECTION

// SECTION Disabled
/**
 * Runs while the robot is disabled i.e. before and after match, between auton
 * and teleop period
 */
void disabled() {
	teleopController.useDefaultBehavior();

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
	
	teleopController.setCurrentBehavior(&teleopModeLogic);

	// Driver Control Loop
	while (true) {
		pros::delay(10);
	}
}

// !SECTION
