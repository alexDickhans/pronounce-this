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
	leftDriveMotors.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
	rightDriveMotors.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
	return 0;
}

void turnTo(Angle angle, int waitTimeMS) {
	RotationController angleRotation("AngleTurn", drivetrain, odometry, turningPid, angle, drivetrainMutex);

	drivetrainStateController.setCurrentBehavior(&angleRotation);

	pros::Task::delay(waitTimeMS);

	drivetrainStateController.setCurrentBehavior(&drivetrainStopped);
}

void move(QLength distance, ProfileConstraints profileConstraints, QCurvature curvature) {
	TankMotionProfiling motionProfiling("moveDistance", &drivetrain, profileConstraints, distance, &odometry, drivetrainMutex, curvature);

	drivetrainStateController.setCurrentBehavior(&motionProfiling);

	while(!drivetrainStateController.isDone()) 
		pros::Task::delay(10);

	drivetrainStateController.setCurrentBehavior(&drivetrainStopped);
}

int tunePid() {

	threeWheelOdom.setPose(Pose2D(0_in, 0_in, 0_deg));

	turnTo(180_deg, 1500);

	turnTo(0_deg, 1500);

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

int testCurvatureAuton() {

	threeWheelOdom.setPose(Pose2D(0_in, 0_in, 0_deg));
	// ptoStateController.setCurrentBehavior(&ptoIntakeStopped);

	drivetrainStateController.setCurrentBehavior(&testCurvatureDrive);

	pros::Task::delay(50);

	while(!drivetrainStateController.isDone()) 
		pros::Task::delay(10);

	drivetrainStateController.setCurrentBehavior(&drivetrainStopped);

	return 0;
}

int skills() {

	threeWheelOdom.reset(Pose2D(34_in, 12_in, 180_deg));
	robotStatus.setDiscCount(1);

	ptoStateController.setCurrentBehavior(&ptoIntakeStopped);

	drivetrainStateController.setCurrentBehavior(&intoCloseRoller2);

	pros::Task::delay(50);

	while(!drivetrainStateController.isDone()) 
		pros::Task::delay(10);

	ptoStateController.setCurrentBehavior(&ptoIntaking);

	pros::Task::delay(700);

	ptoStateController.setCurrentBehavior(&ptoIntakeStopped);

	drivetrainStateController.setCurrentBehavior(&turnTo180);

	pros::Task::delay(500);

	drivetrainStateController.setCurrentBehavior(&fromCloseRoller2);

	pros::Task::delay(50);

	while(!drivetrainStateController.isDone()) 
		pros::Task::delay(10);

	drivetrainStateController.setCurrentBehavior(&turnTo270);

	pros::Task::delay(700);

	ptoStateController.setCurrentBehavior(&ptoIntaking);

	drivetrainStateController.setCurrentBehavior(&closeRollerToLeftRoller);

	pros::Task::delay(50);

	while(!drivetrainStateController.isDone()) 
		pros::Task::delay(10);
	
	ptoStateController.setCurrentBehavior(&ptoIntaking);

	pros::Task::delay(300);

	ptoStateController.setCurrentBehavior(&ptoIntakeStopped);

	turnTo(270_deg, 300);

	drivetrainStateController.setCurrentBehavior(&fromLeftRoller);

	pros::Task::delay(50);

	while(!drivetrainStateController.isDone()) 
		pros::Task::delay(10);

	turnTo(1_deg, 700);

	drivetrainStateController.setCurrentBehavior(&leftRollerToGoal);

	pros::Task::delay(50);

	while(!drivetrainStateController.isDone()) 
		pros::Task::delay(10);

	drivetrainStateController.setCurrentBehavior(&targetingJoystick);

	pros::Task::delay(300);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);

	pros::Task::delay(300);

	drivetrainStateController.setCurrentBehavior(&turnTo90);

	pros::Task::delay(750);

	drivetrainStateController.setCurrentBehavior(&intakeFarBarrierCloseDiscs);

	pros::Task::delay(50);

	while(!drivetrainStateController.isDone()) 
		pros::Task::delay(10);

	drivetrainStateController.setCurrentBehavior(&backIntakeFarBarrierCloseDiscs);

	pros::Task::delay(50);

	while(!drivetrainStateController.isDone()) 
		pros::Task::delay(10);

	drivetrainStateController.setCurrentBehavior(&turnTo10);

	pros::Task::delay(600);

	drivetrainStateController.setCurrentBehavior(&targetingJoystick);

	pros::Task::delay(400);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);

	pros::Task::delay(400);

	turnTo(145_deg, 800);

	drivetrainStateController.setCurrentBehavior(&toMiddleLineDiscs);

	pros::Task::delay(50);

	while(!drivetrainStateController.isDone()) 
		pros::Task::delay(10);

	turnTo(42_deg, 800);

	drivetrainStateController.setCurrentBehavior(&intakeMiddleLineDiscs);

	pros::Task::delay(50);

	while(!drivetrainStateController.isDone()) 
		pros::Task::delay(10);

	turnTo(310_deg, 400);

	drivetrainStateController.setCurrentBehavior(&targetingJoystick);

	pros::Task::delay(400);

	drivetrainStateController.setCurrentBehavior(&toFarBarrier);

	pros::Task::delay(50);

	while(!drivetrainStateController.isDone()) 
		pros::Task::delay(10);
	
	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);

	pros::Task::delay(400);

	drivetrainStateController.setCurrentBehavior(&turnTo0);

	pros::Task::delay(800);

	drivetrainStateController.setCurrentBehavior(&intakeFarBarrierFarDiscs);

	pros::Task::delay(50);

	while(!drivetrainStateController.isDone()) 
		pros::Task::delay(10);
	
	drivetrainStateController.setCurrentBehavior(&turnTo270);

	pros::Task::delay(400);

	drivetrainStateController.setCurrentBehavior(&targetingJoystick);

	pros::Task::delay(400);

	drivetrainStateController.setCurrentBehavior(&drivetrainStopped);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);

	pros::Task::delay(400);

	return 0;
}

int skillsMax() {

	threeWheelOdom.reset(Pose2D(34_in, 12_in, 180_deg));
	robotStatus.setDiscCount(1);

	ptoStateController.setCurrentBehavior(&ptoIntakeStopped);

	drivetrainStateController.setCurrentBehavior(&intoCloseRoller2);

	pros::Task::delay(50);

	while(!drivetrainStateController.isDone()) 
		pros::Task::delay(10);

	ptoStateController.setCurrentBehavior(&ptoIntaking);

	drivetrainStateController.setCurrentBehavior(&turnTo180);

	pros::Task::delay(600);

	ptoStateController.setCurrentBehavior(&ptoIntakeStopped);

	drivetrainStateController.setCurrentBehavior(&fromCloseRoller2);

	pros::Task::delay(50);

	while(!drivetrainStateController.isDone()) 
		pros::Task::delay(10);

	turnTo(265_deg, 300);

	ptoStateController.setCurrentBehavior(&ptoIntaking);

	drivetrainStateController.setCurrentBehavior(&closeRollerToLeftRoller);

	pros::Task::delay(50);

	while(!drivetrainStateController.isDone()) 
		pros::Task::delay(10);
	
	ptoStateController.setCurrentBehavior(&ptoIntaking);

	turnTo(270_deg, 400);

	ptoStateController.setCurrentBehavior(&ptoIntakeStopped);

	move(-8_in, defaultProfileConstraints, 0.0);

	pros::Task::delay(50);

	while(!drivetrainStateController.isDone()) 
		pros::Task::delay(10);

	ptoStateController.setCurrentBehavior(&ptoIntaking);

	turnTo(1_deg, 700);

	move(55_in, defaultProfileConstraints, (30_deg/85_in));

	pros::Task::delay(50);

	while(!drivetrainStateController.isDone()) 
		pros::Task::delay(10);

	drivetrainStateController.setCurrentBehavior(&targetingJoystick);

	pros::Task::delay(300);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);

	pros::Task::delay(500);

	turnTo(135_deg, 800);

	move(27_in, defaultProfileConstraints, 0.0);

	turnTo(45_deg, 800);

	move(37_in, { 45_in / second, 200_in / second / second, 0.0 }, 0.0);

	turnTo(305_deg, 500);

	move(5_in, defaultProfileConstraints, 0.0);

	drivetrainStateController.setCurrentBehavior(&targetingJoystick);

	pros::Task::delay(300);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);

	pros::Task::delay(500);

	turnTo(275_deg, 800);

	move(30_in, intakeProfileConstraints, 0.0);

	move(-29_in, defaultProfileConstraints, 0.0);

	turnTo(315_deg, 500);

	drivetrainStateController.setCurrentBehavior(&targetingJoystick);

	pros::Task::delay(300);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);

	pros::Task::delay(500);

	turnTo(357_deg, 800);

	move(30_in, intakeProfileConstraints, 0.0);

	move(-30_in, defaultProfileConstraints, 0.0);

	turnTo(315_deg, 500);

	drivetrainStateController.setCurrentBehavior(&targetingJoystick);

	pros::Task::delay(300);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);

	pros::Task::delay(500);
	
	turnTo(60_deg, 800);

	move(30_in, defaultProfileConstraints, 0.0);

	move(22_in, intakeProfileConstraints, 0.0);

	move(20_in, defaultProfileConstraints, -90_deg/20_in);
	
	move(5_in, defaultProfileConstraints, 0.0);

	ptoStateController.setCurrentBehavior(&ptoIntaking);

	turnTo(0_deg, 600);

	ptoStateController.setCurrentBehavior(&ptoIntakeStopped);

	move(-8_in, defaultProfileConstraints, 0.0);

	turnTo(270_deg, 800);

	move(30_in, defaultProfileConstraints, 0.0);

	drivetrainStateController.setCurrentBehavior(&targetingJoystick);

	pros::Task::delay(300);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);

	pros::Task::delay(500);

	turnTo(135_deg, 800);

	move(30_in, defaultProfileConstraints, 0.0);

	turnTo(90_deg, 800);

	move(35_in, intakeProfileConstraints, 0.0);

	ptoStateController.setCurrentBehavior(&ptoIntakeStopped);

	move(5_in, defaultProfileConstraints, 0.0);

	ptoStateController.setCurrentBehavior(&ptoIntaking);

	turnTo(90_deg, 300);

	ptoStateController.setCurrentBehavior(&ptoIntakeStopped);

	move(-10_in, defaultProfileConstraints, 0.0);

	turnTo(180_deg, 800);

	move(45_in, defaultProfileConstraints, 0.0);

	drivetrainStateController.setCurrentBehavior(&targetingJoystick);

	pros::Task::delay(300);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);

	pros::Task::delay(500);

	turnTo(315_deg, 800);

	move(35_in, defaultProfileConstraints, 0.0);

	turnTo(225_deg, 800);

	move(45_in, defaultProfileConstraints, 0.0);

	turnTo(135_deg, 800);

	drivetrainStateController.setCurrentBehavior(&targetingJoystick);

	pros::Task::delay(300);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);

	pros::Task::delay(500);

	turnTo(90_deg, 800);

	move(30_in, intakeProfileConstraints, 0.0);

	move(-30_in, defaultProfileConstraints, 0.0);

	turnTo(135_deg, 500);

	drivetrainStateController.setCurrentBehavior(&targetingJoystick);

	pros::Task::delay(300);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);

	pros::Task::delay(500);

	turnTo(180_deg, 800);
	
	move(30_in, intakeProfileConstraints, 0.0);

	move(-30_in, defaultProfileConstraints, 0.0);

	turnTo(135_deg, 500);

	drivetrainStateController.setCurrentBehavior(&targetingJoystick);

	pros::Task::delay(300);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);

	pros::Task::delay(500);
	
	turnTo(240_deg, 800);

	move(30_in, defaultProfileConstraints, 0.0);

	move(20_in, intakeProfileConstraints, 0.0);

	turnTo(115_deg, 800);
	
	move(40_in, defaultProfileConstraints, 0.0);

	drivetrainStateController.setCurrentBehavior(&targetingJoystick);

	pros::Task::delay(300);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);

	pros::Task::delay(500);

	turnTo(75_deg, 800);

	move(-80_in, defaultProfileConstraints, 0.0);

	turnTo(45_deg, 800);

	endgameStateController.setCurrentBehavior(&endgameEnabled);

	pros::Task::delay(1000);

	return 0;
}

int closeFullAWP() {
	threeWheelOdom.reset(Pose2D(34_in, 12_in, 180_deg));
	robotStatus.setDiscCount(1);

	ptoStateController.setCurrentBehavior(&ptoIntakeStopped);

	move(4_in, defaultProfileConstraints, 0.0);

	ptoStateController.setCurrentBehavior(&ptoIntaking);

	drivetrainStateController.setCurrentBehavior(&turnTo180);

	pros::Task::delay(300);

	ptoStateController.setCurrentBehavior(&ptoIntakeStopped);

	move(-5_in, defaultProfileConstraints, 0.0);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostBoosting);

	turnTo(345_deg, 800);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);

	pros::Task::delay(300);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostNone);

	turnTo(51_deg, 800);

	move(20_in, defaultProfileConstraints, 0.0);

	move(30_in, intakeProfileConstraints, 0.0);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostBoosting);

	turnTo(320_deg, 800);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);

	pros::Task::delay(300);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostNone);

	turnTo(50_deg, 800);

	move(75_in, defaultProfileConstraints, 0.0);

	move(15_in, defaultProfileConstraints, 90_deg/15_in);

	ptoStateController.setCurrentBehavior(&ptoIntaking);

	drivetrainStateController.setCurrentBehavior(&turnTo90);

	pros::Task::delay(300);

	ptoStateController.setCurrentBehavior(&ptoIntakeStopped);

	pros::Task::delay(1000);

	return 0;
}

int closeFullAWPMax() {
	threeWheelOdom.reset(Pose2D(34_in, 12_in, 180_deg));
	robotStatus.setDiscCount(1);

	ptoStateController.setCurrentBehavior(&ptoIntakeStopped);

	move(4_in, defaultProfileConstraints, 0.0);

	ptoStateController.setCurrentBehavior(&ptoIntaking);

	drivetrainStateController.setCurrentBehavior(&turnTo180);

	pros::Task::delay(300);

	ptoStateController.setCurrentBehavior(&ptoIntakeStopped);

	move(-5_in, defaultProfileConstraints, 0.0);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostBoosting);

	turnTo(345_deg, 800);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);

	pros::Task::delay(300);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostNone);

	turnTo(51_deg, 800);

	move(20_in, defaultProfileConstraints, 0.0);

	move(30_in, intakeProfileConstraints, 0.0);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostBoosting);

	turnTo(320_deg, 800);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);

	pros::Task::delay(300);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostNone);

	turnTo(50_deg, 800);

	move(55_in, defaultProfileConstraints, 0.0);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostBoosting);

	turnTo(285_deg, 800);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);

	pros::Task::delay(300);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostNone);

	turnTo(50_deg, 800);

	move(20_in, defaultProfileConstraints, 0.0);

	move(15_in, defaultProfileConstraints, 90_deg/15_in);

	ptoStateController.setCurrentBehavior(&ptoIntaking);

	drivetrainStateController.setCurrentBehavior(&turnTo90);

	pros::Task::delay(300);

	ptoStateController.setCurrentBehavior(&ptoIntakeStopped);

	pros::Task::delay(1000);

	return 0;
}

int disc5RightAuton() {

	threeWheelOdom.reset(Pose2D(132_in, 84_in, 112_deg));
	robotStatus.setDiscCount(2);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostBoosting);

	pros::Task::delay(200);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);

	pros::Task::delay(300);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostNone);
	
	move(25_in, defaultProfileConstraints, 165_deg/25_in);

	ptoStateController.setCurrentBehavior(&ptoIntaking);

	drivetrainStateController.setCurrentBehavior(&turnTo90);

	pros::Task::delay(300);

	ptoStateController.setCurrentBehavior(&ptoIntakeStopped);

	move(-5_in, defaultProfileConstraints, 0.0);

	turnTo(225_deg, 800);

	move(60_in, defaultProfileConstraints, 0.0);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostBoosting);

	turnTo(315_deg, 800);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);

	pros::Task::delay(300);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostNone);

	pros::Task::delay(50);

	return 0;
}

int disc8RightAuton() {

	threeWheelOdom.reset(Pose2D(132_in, 84_in, 292_deg));
	robotStatus.setDiscCount(2);

	ptoStateController.setCurrentBehavior(&ptoIntakeStopped);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostBoosting);

	pros::Task::delay(200);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);

	pros::Task::delay(300);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostNone);
	
	ptoStateController.setCurrentBehavior(&ptoIntakeStopped);

	move(42_in, defaultProfileConstraints, 220_deg/30_in);

	move(10_in, defaultProfileConstraints, 0.0);

	ptoStateController.setCurrentBehavior(&ptoIntaking);

	drivetrainStateController.setCurrentBehavior(&turnTo90);

	pros::Task::delay(100);

	ptoStateController.setCurrentBehavior(&ptoIntakeStopped);

	move(-8_in, defaultProfileConstraints, 0.0);

	ptoStateController.setCurrentBehavior(&ptoIntaking);

	turnTo(229_deg, 800);

	move(65_in, {30_in / second, 150_in / second / second, 0.0}, 0.0);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostBoosting);

	turnTo(323_deg, 800);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);

	pros::Task::delay(300);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostNone);

	pros::Task::delay(50);

	turnTo(95_deg, 900);

	move(40_in, { 20_in / second, 200_in / second / second, 0.0 }, 0.0);

	move(-40_in, defaultProfileConstraints, 0.0);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostBoosting);

	turnTo(315_deg, 800);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);

	pros::Task::delay(300);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostNone);

	pros::Task::delay(50);

	return 0;
}

int testPurePursuit() {

	drivetrainStateController.setCurrentBehavior(&testPathPurePursuit);

	pros::Task::delay(50);

	while(!drivetrainStateController.isDone()) 
		pros::Task::delay(10);

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
	std::cout << "INIT" << std::endl;
	initHardware();
	std::cout << "INIT" << std::endl;
	initDrivetrain();
	std::cout << "INIT" << std::endl;
	initBoost();
	initPto();

	std::cout << "INIT" << std::endl;
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
	skillsMax();
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
	robotMutex.give();
}

// !SECTION
