#include "main.h"

// LVGL
std::shared_ptr<lv_obj_t> tabview;

RobotStatus robotStatus;
ModeLogic modeLogic(&robotStatus);
TeleopModeLogic teleopModeLogic(new pros::Controller(CONTROLLER_MASTER), new pros::Controller(CONTROLLER_PARTNER));

pros::Mutex robotMutex;

// SECTION Auton

/**
 * @brief Runs during auton period before auton
 *
 */
int preAutonRun() {
	teleopController.useDefaultBehavior();
	drivetrainStateController.setCurrentBehavior(&drivetrainStopped);
	return 0;
}

int closeFullAWP() {
	threeWheelOdom.setPose(Pose2D(34_in, 13_in, 180_deg));

	// NOTE: First roller

	ptoStateController.setCurrentBehavior(&ptoIntakeStopped);

	pros::Task::delay(20);

	stateExtensionController.setCurrentBehavior(&rollerSequence);

	pros::Task::delay(50);

	while(!stateExtensionController.isDone()) 
		pros::Task::delay(10);

	// NOTE: Move to the middle of the field 

	drivetrainStateController.setCurrentBehavior(&turnTo45);

	pros::Task::delay(1000);

	drivetrainStateController.setCurrentBehavior(&closeToMidField);

	while(!drivetrainStateController.isDone()) 
		pros::Task::delay(10);
	
	drivetrainStateController.setCurrentBehavior(&turnTo315);

	pros::Task::delay(1000);

	// NOTE: Shoot first disc

	drivetrainStateController.setCurrentBehavior(&midFieldToAutonLine);

	while(!drivetrainStateController.isDone()) 
		pros::Task::delay(10);

	ptoStateController.setCurrentBehavior(&ptoIntaking);

	pros::Task::delay(500);
	
	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);

	while(!ptoStateExtensionController.isDone()) 
		pros::Task::delay(10);
	
	// NOTE: Shoot second disc 

	ptoStateController.setCurrentBehavior(&ptoIntaking);
	
	drivetrainStateController.setCurrentBehavior(&autonLineMidField);

	while(!drivetrainStateController.isDone()) 
		pros::Task::delay(10);

	drivetrainStateController.setCurrentBehavior(&midFieldToAutonLine);

	while(!drivetrainStateController.isDone()) 
		pros::Task::delay(10);
	
	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);

	// NOTE: Shoot third disc 

	/*
	while(!ptoStateExtensionController.isDone()) 
		pros::Task::delay(10);
	
	drivetrainStateController.setCurrentBehavior(&autonLineToMidDiscs);

	while(!drivetrainStateController.isDone()) 
		pros::Task::delay(10);

	drivetrainStateController.setCurrentBehavior(&midDiscsToAutonLine);

	while(!drivetrainStateController.isDone()) 
		pros::Task::delay(10);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);

	while(!ptoStateExtensionController.isDone()) 
		pros::Task::delay(10);
	
	*/
	
	// NOTE: Forth disc

	/*
	while(!ptoStateExtensionController.isDone()) 
		pros::Task::delay(10);
	
	drivetrainStateController.setCurrentBehavior(&turnTo325);

	while(!ptoStateExtensionController.isDone()) 
		pros::Task::delay(10);
	
	drivetrainStateController.setCurrentBehavior(&autonLineToMidDiscs);

	while(!drivetrainStateController.isDone()) 
		pros::Task::delay(10);

	drivetrainStateController.setCurrentBehavior(&midDiscsToAutonLine);

	while(!drivetrainStateController.isDone()) 
		pros::Task::delay(10);

	drivetrainStateController.setCurrentBehavior(&turnTo315);

	while(!drivetrainStateController.isDone()) 
		pros::Task::delay(10);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);

	while(!ptoStateExtensionController.isDone()) 
		pros::Task::delay(10);
	*/

	// NOTE: Move to roller

	drivetrainStateController.setCurrentBehavior(&autonLineMidField);

	while(!drivetrainStateController.isDone()) 
		pros::Task::delay(10);
	
	drivetrainStateController.setCurrentBehavior(&turnTo225);

	pros::Task::delay(1000);
	
	drivetrainStateController.setCurrentBehavior(&midFieldToFarField);

	while(!drivetrainStateController.isDone()) 
		pros::Task::delay(10);
	
	drivetrainStateController.setCurrentBehavior(&turnTo90);

	pros::Task::delay(1000);

	// NOTE: Spin roller

	stateExtensionController.setCurrentBehavior(&rollerSequence);

	pros::Task::delay(50);

	while(!stateExtensionController.isDone()) 
		pros::Task::delay(10);

	pros::Task::delay(50);

	return 0;
}

int tunePid() {

	threeWheelOdom.setPose(Pose2D(0_in, 0_in, 0_deg));

	drivetrainStateController.setCurrentBehavior(&turnTo180);

	pros::Task::delay(1000);

	drivetrainStateController.setCurrentBehavior(&turnTo0);

	pros::Task::delay(1000);

	return 0;
}

int drive24inBackward() {

	threeWheelOdom.setPose(Pose2D(0_in, 0_in, 0_deg));

	drivetrainStateController.setCurrentBehavior(&backwards48in);

	while(!drivetrainStateController.isDone()) 
		pros::Task::delay(10);

	pros::Task::delay(1000);
	
	return 0;
}

int postAuton() {
	return 0;
}

// !SECTION

// SECTION INIT

void update() {

	robotMutex.take();
	modeLogic.initialize();
	robotMutex.give();

	uint32_t startTime = pros::millis();
	uint32_t startTimeMicros = pros::micros();

	while (true) {
		// Create stuff for exact delay
		startTime = pros::millis();
		startTimeMicros = pros::micros();

		robotMutex.take();
		odometry.update();
		modeLogic.update();
		robotMutex.give();

		// Wait a maximum of 10 milliseconds
		pros::delay(std::min(10 - (pros::millis() - startTime), (long unsigned int) 10));

		std::cout << "FrameTime: " << pros::micros() - startTimeMicros << std::endl;
	}
}

void updateDisplay() {

	// Odom
	std::shared_ptr<lv_obj_t> odomTab = std::shared_ptr<lv_obj_t>(lv_tabview_add_tab(tabview.get(), "Odom"));
	std::shared_ptr<lv_obj_t> odomLabel = std::shared_ptr<lv_obj_t>(lv_label_create(odomTab.get(), NULL));

	// Drivetrain
	std::shared_ptr<lv_obj_t> drivetrainTab = std::shared_ptr<lv_obj_t>(lv_tabview_add_tab(tabview.get(), "Drivetrain"));
	std::shared_ptr<lv_obj_t> drivetrainTable = std::shared_ptr<lv_obj_t>(lv_table_create(drivetrainTab.get(), NULL));

	lv_table_set_row_cnt(drivetrainTable.get(), 4);
	lv_table_set_col_cnt(drivetrainTable.get(), 2);

	lv_table_set_col_width(drivetrainTable.get(), 0, 200);
	lv_table_set_col_width(drivetrainTable.get(), 1, 200);

	// Flywheels
	std::shared_ptr<lv_obj_t> flywheelTab = std::shared_ptr<lv_obj_t>(lv_tabview_add_tab(tabview.get(), "PTO"));
	std::shared_ptr<lv_obj_t> flywheelLabel = std::shared_ptr<lv_obj_t>(lv_label_create(flywheelTab.get(), NULL));

	while (true) {
		// Odometry
		lv_label_set_text(odomLabel.get(), (odometry.getPosition().to_string()
			+ "\nL: " + std::to_string(leftDrive1Odom.getPosition().Convert(inch)) +
			", R: " + std::to_string(rightDrive1Odom.getPosition().Convert(inch)) +
			", S: " + std::to_string(backOdomWheel.getPosition().Convert(inch))).c_str());

		// Drivetrain
		lv_table_set_cell_value(drivetrainTable.get(), 0, 0, (std::to_string(leftDrive1.get_temperature()) + " C").c_str());
		lv_table_set_cell_value(drivetrainTable.get(), 1, 0, (std::to_string(leftDrive2.get_temperature()) + " C").c_str());
		lv_table_set_cell_value(drivetrainTable.get(), 2, 0, (std::to_string(leftDrive3.get_temperature()) + " C").c_str());
		lv_table_set_cell_value(drivetrainTable.get(), 3, 0, (std::to_string(leftPtoMotor.get_temperature()) + " C").c_str());
		lv_table_set_cell_value(drivetrainTable.get(), 0, 1, (std::to_string(rightDrive1.get_temperature()) + " C").c_str());
		lv_table_set_cell_value(drivetrainTable.get(), 1, 1, (std::to_string(rightDrive2.get_temperature()) + " C").c_str());
		lv_table_set_cell_value(drivetrainTable.get(), 2, 1, (std::to_string(rightDrive3.get_temperature()) + " C").c_str());
		lv_table_set_cell_value(drivetrainTable.get(), 3, 1, (std::to_string(rightPtoMotor.get_temperature()) + " C").c_str());

		// Flywheel
		lv_label_set_text(flywheelLabel.get(), ("Speed: " + std::to_string(leftPtoMotor.get_actual_velocity())).c_str());

		pros::Task::delay(50);
	}
}

void ledUpdate() {
	while(1) {
		leftLedController.update();
		pros::Task::delay(10);
		rightLedController.update();
		pros::Task::delay(10);
	}
}

void initDisplay() {
	pros::Task display(updateDisplay, TASK_PRIORITY_MIN);
	pros::Task leds(ledUpdate, TASK_PRIORITY_MIN);
}

/**
 * Runs when the robot starts up
 */
void initialize() {

	robotMutex.take();

	lv_init();
	tabview = std::shared_ptr<lv_obj_t>(lv_tabview_create(lv_scr_act(), NULL));

	// Initialize functions
	initHardware();
	initDrivetrain();
	initPto();
	initBehaviors();
	initDisplay();

	pros::Task modeLogicTask = pros::Task(update, TASK_PRIORITY_MAX);

	robotMutex.give();
}

// !SECTION

// SECTION Disabled
/**
 * Runs while the robot is disabled i.e. before and after match, between auton
 * and teleop period
 */
void disabled() {
	robotMutex.take();
	teleopController.useDefaultBehavior();
	robotMutex.give();

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
	drive24inBackward();
	postAuton();

	// autonomousSelector.run();
}

// !SECTION

// SECTION Operator Control

/**
 * Runs during operator/teleop control
 */
void opcontrol() {
	robotMutex.take();
	teleopController.setCurrentBehavior(&teleopModeLogic);
	robotMutex.give();
}

// !SECTION
