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
	drivetrainStateController.setDefaultBehavior(&drivetrainStopped);
	leftDriveMotors.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
	rightDriveMotors.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
	return 0;
}

void turnTo(Angle angle, int waitTimeMS) {
	RotationController angleRotation("AngleTurn", drivetrain, odometry, turningPid, angle, drivetrainMutex);

	drivetrainStateController.setCurrentBehavior(&angleRotation);

	pros::Task::delay(waitTimeMS);

	drivetrainStateController.useDefaultBehavior();

	pros::Task::delay(10);
}

void move(QLength distance, ProfileConstraints profileConstraints, QCurvature curvature) {
	TankMotionProfiling motionProfiling("moveDistance", &drivetrain, profileConstraints, distance, &odometry, &distancePid, drivetrainMutex, curvature);

	drivetrainStateController.setCurrentBehavior(&motionProfiling);

	pros::delay(50);

	while(!drivetrainStateController.isDone()) 
		pros::Task::delay(10);

	drivetrainStateController.setCurrentBehavior(&drivetrainStopped);
}

void move(QLength distance, ProfileConstraints profileConstraints, QCurvature curvature, Angle startAngle) {
	TankMotionProfiling motionProfiling("moveDistance", &drivetrain, profileConstraints, distance, &odometry, &distancePid, drivetrainMutex, curvature, startAngle, &movingTurnPid);

	drivetrainStateController.setCurrentBehavior(&motionProfiling);

	pros::delay(50);

	while(!drivetrainStateController.isDone()) 
		pros::Task::delay(10);

	drivetrainStateController.setCurrentBehavior(&drivetrainStopped);
}

int tuneTurnPid() {

	threeWheelOdom.setPose(Pose2D(0_in, 0_in, 0_deg));

	// turnTo(180_deg, 1000);

	// move(50_in, defaultProfileConstraints, 0.0, 5_deg);

	// pros::Task::delay(20);

	turnTo(-90_deg, 1500);

	turnTo(90_deg, 1500);

	turnTo(270_deg, 1500);

	turnTo(-90_deg, 1500);

	// pros::Task::delay(200);

	// move(50_in, defaultProfileConstraints, 0.0, 180_deg);

	// pros::delay(50);

	return 0;
}

int tuneDistancePid() {

	threeWheelOdom.setPose(Pose2D(0_in, 0_in, 0_deg));

	move(24_in, defaultProfileConstraints, 0.0, 0_deg);

	// turnTo(90_deg, 1000);

	// move(24_in, defaultProfileConstraints, 0.0, 90_deg);
	// move(-24_in, defaultProfileConstraints, 0.0, 90_deg);

	// turnTo(0_deg, 1000);

	// move(-30_in, defaultProfileConstraints, 0.0, 0_deg);

	pros::Task::delay(50);

	return 0;
}

int testRamseteAuton() {

	threeWheelOdom.setPose(Pose2D(0_in, 0_in, 0_deg));
	ptoStateController.setCurrentBehavior(&ptoIntakeStopped);

	drivetrainStateController.setCurrentBehavior(&testRamsete);

	pros::Task::delay(10000);

	drivetrainStateController.setCurrentBehavior(&drivetrainStopped);

	return 0;
}

int close8Disc() {
	threeWheelOdom.reset(Pose2D(34_in, 12_in, 180_deg));

	ptoStateController.setCurrentBehavior(&ptoIntakeStopped);

	move(6_in, defaultProfileConstraints, 0.0, 0.0);

	ptoStateController.setCurrentBehavior(&ptoIntaking);
	pistonBoostStateController.setCurrentBehavior(&pistonBoostBoth);

	ptoStateController.setCurrentBehavior(&ptoIntaking);

	turnTo(180_deg, 200);

	ptoStateController.setCurrentBehavior(&ptoIntakeStopped);

	move(-15_in, defaultProfileConstraints, (130_deg/-28_in));

	turnTo(347_deg, 1000);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);

	intakeSolenoid.set_value(true);

	pros::delay(500);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostNone);

	turnTo(270_deg, 800);

	move(10_in, defaultProfileConstraints, 0.0);

	intakeSolenoid.set_value(false);

	pros::delay(500);

	pros::Task::delay(1000);

	move(-7_in, defaultProfileConstraints, 0.0);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostBoosting);

	turnTo(328_deg, 400);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);

	pros::Task::delay(400);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostNone);

	turnTo(450_deg, 800);

	move(24_in, defaultProfileConstraints, 0.0);

	move(10_in, intakeProfileConstraints, 0.0);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostBoosting);

	turnTo(320_deg, 800);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);

	pros::Task::delay(400);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostNone);

	return 0;
}

int closeFullAWP() {
	threeWheelOdom.reset(Pose2D(34_in, 12_in, 180_deg));

	ptoStateController.setCurrentBehavior(&ptoIntakeStopped);

	move(6_in, defaultProfileConstraints, 0.0, 0.0);

	ptoStateController.setCurrentBehavior(&ptoIntaking);
	pistonBoostStateController.setCurrentBehavior(&pistonBoostBoth);

	ptoStateController.setCurrentBehavior(&ptoIntaking);

	turnTo(180_deg, 200);

	ptoStateController.setCurrentBehavior(&ptoIntakeStopped);

	move(-15_in, defaultProfileConstraints, (130_deg/-28_in));

	turnTo(347_deg, 1000);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);

	turnTo(450_deg, 800);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostNone);

	move(15_in, defaultProfileConstraints, 0.0);

	move(10_in, intakeProfileConstraints, 0.0);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostBoosting);

	turnTo(320_deg, 800);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);

	turnTo(390_deg, 800);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostNone);

	move(24_in, defaultProfileConstraints, 0.0, 390_deg);
	
	move(20_in, defaultProfileConstraints, 15_deg/20_in, 390_deg);

	move(50_in, defaultProfileConstraints, 0.0);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostBoosting);

	turnTo(270_deg, 800);
	
	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);

	turnTo(405_deg, 800);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostNone);

	move(30_in, defaultProfileConstraints, 45_deg/30_in, 405_deg);
	
	ptoStateExtensionController.setCurrentBehavior(&ptoIntakeStopped);

	move(4_in, defaultProfileConstraints, 0.0, 0.0, 450_deg);

	ptoStateExtensionController.setCurrentBehavior(&ptoIntaking);

	turnTo(450_deg, 200);
	
	ptoStateExtensionController.setCurrentBehavior(&ptoIntakeStopped);

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

	std::shared_ptr<lv_obj_t> portsTab = std::shared_ptr<lv_obj_t>(lv_tabview_add_tab(tabview.get(), "Ports"));
	std::shared_ptr<lv_obj_t> portsPage = std::shared_ptr<lv_obj_t>(lv_page_create(portsTab.get(), NULL));
	std::shared_ptr<lv_obj_t> portsLabel = std::shared_ptr<lv_obj_t>(lv_label_create(portsTab.get(), NULL));
	std::shared_ptr<lv_obj_t> portsTable = std::shared_ptr<lv_obj_t>(lv_table_create(portsPage.get(), NULL));
	
	lv_obj_set_width(portsPage.get(), 400);
	lv_obj_set_height(portsPage.get(), 100);

	int count = checkPorts(portsTable.get());

	std::cout << count << std::endl;

	if (count > 0) {
		lv_label_set_text(portsLabel.get(), "SOMETHING IS MISSING");
		// lv_label_set_style(portsLabel.get(), )
	} else {
		lv_label_set_text(portsLabel.get(), "All Good");
	}

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
			", R: " + std::to_string(rightDrive1Odom.getPosition().Convert(inch))).c_str());

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

	std::cout << "INIT" << std::endl;

	robotMutex.take();

	lv_init();
	tabview = std::shared_ptr<lv_obj_t>(lv_tabview_create(lv_scr_act(), NULL));

	// Initialize functions
	initHardware();
	initDrivetrain();
	initBoost();
	initPto();
	initEndgame();
	initBehaviors();
	initDisplay();

	robotMutex.give();

	pros::Task modeLogicTask = pros::Task(update, TASK_PRIORITY_MAX);
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
	preAutonRun();

	close8Disc();

	// #if AUTON == 0
	// 	closeFullAWPMax();
	// #endif // !1
	// #if AUTON == 1
	// 	close5DiscAuton();
	// #endif // !1
	// #if AUTON == 2
	// 	disc5RightAuton();
	// #endif // !1
	// #if AUTON == 3
	// 	disc8RightAuton();
	// #endif // !1
	// #if AUTON == 4
	// 	skillsMax();
	// #endif // !1

	postAuton();
}

// !SECTION

// SECTION Operator Control

/**
 * Runs during operator/teleop control
 */
void opcontrol() {
	robotMutex.take();
	teleopController.setCurrentBehavior(&teleopModeLogic);
	drivetrainStateController.setDefaultBehavior(&normalJoystick);
	robotMutex.give();
}

// !SECTION
