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

	pros::Task::delay(30);
}

void move(QLength distance, ProfileConstraints profileConstraints, QCurvature curvature, QSpeed initialSpeed = 0.0, QSpeed endSpeed = 0.0) {
	TankMotionProfiling motionProfiling("moveDistance", &drivetrain, profileConstraints, distance, &odometry, &distancePid, drivetrainMutex, curvature, initialSpeed, endSpeed);

	drivetrainStateController.setCurrentBehavior(&motionProfiling);

	pros::delay(50);

	while(!drivetrainStateController.isDone()) 
		pros::Task::delay(10);

	drivetrainStateController.setCurrentBehavior(&drivetrainStopped);
}

void move(QLength distance, ProfileConstraints profileConstraints, QCurvature curvature, Angle startAngle, QSpeed initialSpeed = 0.0, QSpeed endSpeed = 0.0) {
	TankMotionProfiling motionProfiling("moveDistance", &drivetrain, profileConstraints, distance, &odometry, &distancePid, drivetrainMutex, curvature, startAngle, &movingTurnPid, initialSpeed, endSpeed);

	drivetrainStateController.setCurrentBehavior(&motionProfiling);

	pros::delay(50);

	while(!drivetrainStateController.isDone()) 
		pros::Task::delay(10);

	drivetrainStateController.setCurrentBehavior(&drivetrainStopped);
}

void shootWhileMoving(QLength distance, QSpeed speed, Angle angle, double waitTime = 100, bool off = false) {

	TankMotionProfiling motionProfiling("moveDistance", &drivetrain, { speed, 200_in / second / second, 0.0 }, distance, &odometry, &distancePid, drivetrainMutex, 0.0, angle, &movingTurnPid);

	drivetrainStateController.setCurrentBehavior(&motionProfiling);

	pros::delay(50);

	pros::Task::delay(waitTime);

	ptoStateExtensionController.setCurrentBehavior(off ? &ptoCatapultLaunchOff : &ptoCatapultLaunch);

	while(!drivetrainStateController.isDone()) 
		pros::Task::delay(10);

	drivetrainStateController.setCurrentBehavior(&drivetrainStopped);
}

int spinRoller(Angle angle, QLength backupDistance = -5_in) {
	QLength distanceToRoller = frontDistanceSensor.get()*1_mm - 95_mm;

	ptoStateController.setCurrentBehavior(&ptoIntakeStopped);

	move(distanceToRoller, defaultProfileConstraints, 0.0, angle, 0.0, 0.0);

	ptoStateController.setCurrentBehavior(&ptoIntaking);

	drivetrain.tankSteerVoltage(2000, 2000);

	pros::Task::delay(350);

	drivetrain.tankSteerVoltage(0, 0);

	ptoStateController.setCurrentBehavior(&ptoIntakeStopped);

	// move(-5_in, defaultProfileConstraints, 0.0, 0_deg);

	return 0;
}

void matchLoad(Angle angle, Angle goalAngle) {

	QLength distanceToMatch = frontDistanceSensor.get()*1_mm - 66_in;

	move(distanceToMatch, defaultProfileConstraints, 0.0, goalAngle, 0.0, 0.0);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunchOff);
	intakeStopperOverride = true;

	turnTo(goalAngle, 300);

	turnTo(angle, 500);

	distanceToMatch = backDistanceSensor.get()*1_mm - 30_mm;

	move(-distanceToMatch, defaultProfileConstraints, 0.0, angle);

	pros::Task::delay(300);

	move(5_in, defaultProfileConstraints, 0.0, angle);

	turnTo(goalAngle, 600);
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

int skillsMatchLoad() {
	threeWheelOdom.reset(Pose2D(34_in, 12_in, 0_deg));

	ptoStateController.setCurrentBehavior(&ptoIntakeStopped);

	intakeStopperOverride = true;

	pros::Task::delay(900);

	move(5_in, defaultProfileConstraints, 0.0, 0_deg);

	turnTo(88_deg, 500);

	matchLoad(0_deg, 88_deg);

	matchLoad(0_deg, 88_deg);

	return 0;
}

int skills() {
	threeWheelOdom.reset(Pose2D(34_in, 12_in, 0_deg));

	ptoStateController.setCurrentBehavior(&ptoIntakeStopped);

	intakeStopperOverride = true;

	pros::Task::delay(900);

	move(5_in, defaultProfileConstraints, 0.0, 0_deg);

	turnTo(88_deg, 500);

	matchLoad(0_deg, 88_deg);

	matchLoad(0_deg, 88_deg);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);

	turnTo(88_deg, 300);

	intakeStopperOverride = false;

	// Finish match loads
	// Shoot close left stack

	// intakeSolenoid.set_value(true);

	turnTo(-61_deg, 700);

	move(30_in, { 50_in / second, 100_in / second / second, 0.0 }, 0.0, -61_deg, 0_in/second, 10_in/second);

	move(22_in, { 10_in / second, 100_in / second / second, 0.0 }, 0.0, -61_deg, 10_in/second, 0_in/second);

	// intakeSolenoid.set_value(false);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostBoosting);

	turnTo(-6_deg, 450);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunchOff);

	turnTo(-6_deg, 250);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostNone);

	// Spin close rollers

	turnTo(-180_deg, 600);

	spinRoller(-180_deg);

	move(frontDistanceSensor.get() * 1_mm - 800_mm, defaultProfileConstraints, 0.0, -180_deg);

	turnTo(-90_deg, 550);

	spinRoller(-90_deg);

	move(frontDistanceSensor.get() * 1_mm - 10_in, defaultProfileConstraints, 0.0, -90_deg);

	ptoStateController.setCurrentBehavior(&ptoIntaking);

	turnTo(45_deg, 600);

	move(62_in, intakeProfileConstraints, 0.0, 45_deg);

	turnTo(-45_deg, 500);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);

	turnTo(-45_deg, 500);

	// First Barrier

	turnTo(-15_deg, 400);

	move(35_in, intakeBarrierProfileConstraints, 0.0);

	move(frontDistanceSensor.get() * 1_mm - 1330_mm, defaultProfileConstraints, 0.0, 0_deg);

	turnTo(-45_deg, 500);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);

	turnTo(-45_deg, 600);

	turnTo(-75_deg, 400);

	move(35_in, intakeBarrierProfileConstraints, 0.0);

	move(frontDistanceSensor.get() * 1_mm - 1350_mm, defaultProfileConstraints, 0.0, -90_deg);

	turnTo(-45_deg, 500);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunchOff);

	turnTo(-45_deg, 250);

	move(-14_in, defaultProfileConstraints, 0.0, -45_deg);

	turnTo(-180_deg, 500);

	// Second match loader

	intakeStopperOverride = true;

	move(-backDistanceSensor.get() * 1_mm + 30_mm, { 70_in / second, 80_in / second / second, 0.0 }, 0.0, -180_deg);

	move(5_in, defaultProfileConstraints, 0.0, -180_deg);

	turnTo(-92_deg, 450);

	matchLoad(-180_deg, -92_deg);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);

	turnTo(-93_deg, 300);

	intakeStopperOverride = false;

	// REPEAT

	turnTo(-244_deg, 800);

	move(30_in, { 50_in / second, 100_in / second / second, 0.0 }, 0.0, -244_deg, 0_in/second, 10_in/second);

	move(19_in, { 10_in / second, 100_in / second / second, 0.0 }, 0.0, -244_deg, 10_in/second, 0_in/second);

	// intakeSolenoid.set_value(false);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostBoosting);

	turnTo(-186_deg, 450);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunchOff);

	turnTo(-186_deg, 250);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostNone);

	// Spin far rollers

	turnTo(-360_deg, 600);

	spinRoller(-360_deg);

	move(frontDistanceSensor.get() * 1_mm - 800_mm, defaultProfileConstraints, 0.0, -360_deg);

	turnTo(-270_deg, 550);

	spinRoller(-270_deg);

	move(frontDistanceSensor.get() * 1_mm - 10_in, defaultProfileConstraints, 0.0, -270_deg);

	ptoStateController.setCurrentBehavior(&ptoIntaking);

	turnTo(-135_deg, 600);

	move(62_in, intakeProfileConstraints, 0.0, -135_deg);

	turnTo(-225_deg, 450);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);

	turnTo(-225_deg, 300);

	// Second Barrier

	turnTo(-195_deg, 400);

	move(35_in, intakeBarrierProfileConstraints, 0.0);

	turnTo(-275_deg, 550);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunchOff);

	turnTo(-275_deg, 300);

	move(-60_in, defaultProfileConstraints, 0.0, -270_deg);

	turnTo(-135_deg, 500);

	endgameStateController.setCurrentBehavior(&endgameEnabled);

	pros::Task::delay(500);

	return 0;
}

int closeFullAWP() {
	threeWheelOdom.reset(Pose2D(34_in, 12_in, 180_deg));

	ptoStateController.setCurrentBehavior(&ptoIntakeStopped);

	move(3_in, defaultProfileConstraints, 0.0, 180_deg);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostBoosting);

	ptoStateController.setCurrentBehavior(&ptoIntaking);

	turnTo(175_deg, 80);

	ptoStateController.setCurrentBehavior(&ptoIntakeStopped);

	move(-3_in, defaultProfileConstraints, 0.0, 180_deg);

	turnTo(347_deg, 800);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);

	turnTo(347_deg, 400);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostNone);

	turnTo(405_deg, 600);

	move(48_in, { 40_in / second, 100_in / second / second, 0.0 }, 0.0, 405_deg);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostBoosting);

	turnTo(323_deg, 700);

	shootWhileMoving(5_in, 10_in/second, 323_deg, 100);

	move(-5_in, defaultProfileConstraints, 0.0, 323_deg);

	turnTo(323_deg, 200);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostNone);

	turnTo(405_deg, 600);

	move(55_in, { 50_in / second, 75_in / second / second, 0.0 }, 0.0, 405_deg);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostBoosting);

	turnTo(290_deg, 600);
	
	shootWhileMoving(5_in, 50_in/second, 290_deg, 100, true);

	pros::Task::delay(200);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostNone);

	turnTo(405_deg, 600);

	move(40_in, defaultProfileConstraints, 45_deg/40_in, 405_deg);

	ptoStateExtensionController.setCurrentBehavior(new Behavior());

	ptoStateController.setCurrentBehavior(&ptoIntaking);

	turnTo(456_deg, 70);
	
	ptoStateController.setCurrentBehavior(&ptoIntakeStopped);

	return 0;
}

int close9Disc() {
	threeWheelOdom.reset(Pose2D(34_in, 12_in, 45_deg));

	// intake auton line discs
	// Disc rush

	// rezero

	// momentum shot

	// roller

	// intake close stack

	// Momentum shot

	// move to back of barrier

	// intake barrier discs

	// back up

	// Momentum shot

	return 0;
}

int right9Disc() {
	threeWheelOdom.reset(Pose2D(34_in, 12_in, -45_deg));

	// intake auton line discs
	// Disc rush

	// rezero

	// momentum shot

	// roller

	// intake line of discs

	// shoot
	
	// intake barrier
	
	// back up to auton line

	// Shoot

	return 0;
}

int testRollers() {
	threeWheelOdom.reset(Pose2D(34_in, 12_in, -180_deg));

	turnTo(-180_deg, 450);

	spinRoller(-180_deg);

	move(frontDistanceSensor.get() * 1_mm - 800_mm, defaultProfileConstraints, 0.0, -180_deg);

	turnTo(-90_deg, 550);

	spinRoller(-90_deg);

	move(frontDistanceSensor.get() * 1_mm - 10_in, defaultProfileConstraints, 0.0, -90_deg);
}

int testLongShot() {
	threeWheelOdom.reset(Pose2D(0_in, 0_in, 0_deg));

	pistonBoostStateController.setCurrentBehavior(&pistonBoostBoosting);

	TankMotionProfiling motionProfiling("moveDistance", &drivetrain, {30_in/second, 150_in/second/second, 150_in/second/second/second}, 10_in, &odometry, &distancePid, drivetrainMutex, 0.0, 0_deg, &movingTurnPid);

	drivetrainStateController.setCurrentBehavior(&motionProfiling);

	pros::delay(300);

	ptoStateExtensionController.setCurrentBehavior(&ptoCatapultLaunch);
	
	pros::delay(1000);

	pistonBoostStateController.setCurrentBehavior(&pistonBoostNone);

	drivetrainStateController.setCurrentBehavior(&drivetrainStopped);

	return 0;
}

int testMatchLoad() {

	threeWheelOdom.reset(Pose2D(0_in, 0_in, 90_deg));
	pros::Task::delay(50);

	while (1) {
		pros::Task::delay(20);

		matchLoad(0_deg, 90_deg);
	}

	pros::Task::delay(500);
}

int testMotionProfiling() {
	drivetrain.reset();
	drivetrain.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

	threeWheelOdom.reset(Pose2D(0_in, 0_in, 0_deg));

	move(50_in, { 60_in / second, 130_in / second / second, 0.0 }, 0.0, 0.0, 0_in/second, 20_in/second);

	move(10_in, { 20_in / second, 130_in / second / second, 0.0 }, 0.0, 0.0, 20_in/second, 0_in/second);

	return 0;
}

int testSpinRoller() {

	threeWheelOdom.reset(Pose2D(0_in, 0_in, 0_deg));

	spinRoller(0_deg);

	return 0;
}

int postAuton() {
	pros::Task::delay(100);

	drivetrainStateController.setCurrentBehavior(&drivetrainStopped);

	return 0;
}

// !SECTION

// SECTION INIT

void update() {

	pros::Task::delay(200);
	
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

	#if AUTON == 0
		skills();
	#endif // !1
	#if AUTON == 1
		close9Disc();
	#endif // !1
	#if AUTON == 2
		close8Disc();
	#endif // !1
	#if AUTON == 3
		right8disc();
	#endif // !1
	#if AUTON == 4
		skills();
	#endif // !1

	postAuton();
}

// !SECTION

// SECTION Operator Control

/**
 * Runs during operator/teleop control
 */
void opcontrol() {

	#if AUTON == 4
		preAutonRun();

		pros::Task skillsTask = pros::Task(skillsMatchLoad);

		while (true) {
			if (master->get_digital(E_CONTROLLER_DIGITAL_X)) {
				skillsTask.suspend();
				postAuton();
				robotMutex.take();
				teleopController.setCurrentBehavior(&teleopModeLogic);
				drivetrainStateController.setDefaultBehavior(&normalJoystick);
				robotMutex.give();
			}

			pros::Task::delay(10);
		}
	#else
		robotMutex.take();
		teleopController.setCurrentBehavior(&teleopModeLogic);
		drivetrainStateController.setDefaultBehavior(&normalJoystick);
		robotMutex.give();
	#endif // DEBUG
}

// !SECTION
