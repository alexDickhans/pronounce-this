#include "main.h"

grafanalib::GUIManager manager;

// LVGL
lv_obj_t* tabview;

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

void initGrafanaLib() {
	manager.setRefreshRate(50);

	grafanalib::Variable<Pronounce::Odometry> odometryVar("Odometry", odometry);

	grafanalib::VariableGroup<Pronounce::Odometry> odometryVarGroup({odometryVar});

	odometryVarGroup.add_getter("X", &Pronounce::Odometry::getX);
	odometryVarGroup.add_getter("Y", &Pronounce::Odometry::getY);
	odometryVarGroup.add_getter("Angle", &Pronounce::Odometry::getTheta);

	manager.registerDataHandler(&odometryVarGroup);

	grafanalib::Variable<Pronounce::Controller> controller1Var("Controller1", master);
	grafanalib::Variable<Pronounce::Controller> controller2Var("Controller2", partner);

	grafanalib::VariableGroup<Pronounce::Controller> controllerVarGroup({controller1Var, controller2Var});

	manager.registerDataHandler(&controllerVarGroup);

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

	// Driver Control Loop
	while (true) {
		pros::delay(10);
	}
}

// !SECTION
