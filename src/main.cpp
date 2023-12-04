#include "main.h"

// LVGL
std::shared_ptr<lv_obj_t> tabview;

RobotStatus robotStatus;
ModeLogic modeLogic(&robotStatus);
TeleopModeLogic teleopModeLogic(new RobotJoystick(E_CONTROLLER_MASTER), new RobotJoystick(E_CONTROLLER_PARTNER));

pros::Mutex robotMutex;

bool skillsDone = false;

// SECTION Auton

/**
 * @brief Runs during auton period before auton
 *
 */
int preAutonRun() {
	teleopController.useDefaultBehavior();
	drivetrainStateController.setDefaultBehavior(&drivetrainStopped);
	drivetrainStateController.useDefaultBehavior();
	leftDriveMotors.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
	rightDriveMotors.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
	return 0;
}

void turnTo(Angle angle, QTime waitTimeMS) {
	RotationController angleRotation("AngleTurn", drivetrain, odometry, turningPid, angle, drivetrainMutex);

	drivetrainStateController.setCurrentBehavior(&angleRotation);

	pros::Task::delay(waitTimeMS.Convert(millisecond));

	drivetrainStateController.useDefaultBehavior();
	pros::Task::delay(10);
}

void move(QLength distance, ProfileConstraints profileConstraints, QCurvature curvature, QSpeed initialSpeed = 0.0, QSpeed endSpeed = 0.0) {
	drivetrainStateController.setCurrentBehavior(new TankMotionProfiling("moveDistance", &drivetrain, profileConstraints, distance, &odometry, &distancePid, drivetrainMutex, curvature, initialSpeed, endSpeed));

	pros::delay(50);

	while(!drivetrainStateController.isDone())
		pros::Task::delay(10);

	drivetrainStateController.setCurrentBehavior(&drivetrainStopped);
}

void move(QLength distance, ProfileConstraints profileConstraints, QCurvature curvature, Angle startAngle, QSpeed initialSpeed = 0.0, QSpeed endSpeed = 0.0) {

	drivetrainStateController.setCurrentBehavior(new TankMotionProfiling("moveDistance", &drivetrain, profileConstraints, distance, &odometry, &distancePid, drivetrainMutex, curvature, startAngle, &movingTurnPid, initialSpeed, endSpeed));

	pros::delay(50);

	while(!drivetrainStateController.isDone())
		pros::Task::delay(10);

	drivetrainStateController.setCurrentBehavior(&drivetrainStopped);
}

int closeAWP() {

	threeWheelOdom.reset(Pose2D(130_in, 22_in, -30_deg));

	catapultStateController.setCurrentBehavior(catapultFire.wait(800_ms));
	catapultStateController.waitUntilDone()();

	drivetrainStateController.setCurrentBehavior(
			new PathPlanner::PathFollower(
					"TestPath",
					defaultProfileConstraints,
					drivetrain,
					[ObjectPtr = &odometry] { return ObjectPtr->getAngle(); },
					movingTurnPid,
					distancePid,
					8000.0/64.0,
					65_in/second,
					CloseAWP1,
					{
							{0.2, [] () -> void {
								intakeStateController.setCurrentBehavior(&intakeIntaking);
							}},
							{2.2, [] () -> void {
								intakeStateController.setCurrentBehavior(&intakeEject);
							}},
							{4.2, [] () -> void {
								intakeStateController.setCurrentBehavior(&intakeIntaking);
							}},
							{6.2, [] () -> void {
								intakeStateController.setCurrentBehavior(&intakeEject);
							}},
							{8.4, [] () -> void {
								intakeStateController.useDefaultBehavior();
								leftWingStateController.setCurrentBehavior(&leftWingOut);
							}},
							{9.0, [] () -> void {
								leftWingStateController.useDefaultBehavior();
								rightWingStateController.useDefaultBehavior();
							}},
							{9.95, [] () -> void {
								rightWingStateController.setCurrentBehavior(&rightWingOut);
							}}
					}));

	drivetrainStateController.waitUntilDone()();

	drivetrainStateController.setCurrentBehavior(
			new PathPlanner::PathFollower(
					"TestPath",
					defaultProfileConstraints,
					drivetrain,
					[ObjectPtr = &odometry] { return ObjectPtr->getAngle(); },
					movingTurnPid,
					distancePid,
					8000.0/64.0,
					65_in/second,
					CloseAWP2,
					{
							{0.4, [] () -> void {
								intakeStateController.useDefaultBehavior();
								leftWingStateController.setCurrentBehavior(&leftWingOut);
							}},
							{1.0, [] () -> void {
								leftWingStateController.useDefaultBehavior();
								rightWingStateController.useDefaultBehavior();
							}},
							{2.95, [] () -> void {
								rightWingStateController.setCurrentBehavior(&rightWingOut);
							}}
					}));

	drivetrainStateController.waitUntilDone()();

	rightWingStateController.setCurrentBehavior(&rightWingOut);

	drivetrainStateController.setCurrentBehavior(new RotationController("MatchloadRotationController", drivetrain, odometry, turningPid, -170_deg, drivetrainMutex, -3000));

	pros::Task::delay(7000);

	return 0;
}

int far6BallFullAWP() {
	threeWheelOdom.reset(Pose2D(0_in, 0_in, 180_deg));

	drivetrainStateController.setCurrentBehavior(
			new PathPlanner::PathFollower(
					"TestPath",
					{64_in/second, 200_in/second/second, 0.0},
					drivetrain,
					[ObjectPtr = &odometry] { return ObjectPtr->getAngle(); },
					movingTurnPid,
					distancePid,
					8000.0/64.0,
					61_in/second,
					Auto6BallPath1,
					{
							{0.1, [] () -> void {
								intakeStateController.setCurrentBehavior(&intakeIntaking);
							}},
							{1.2, [] () -> void {
								leftWingStateController.setCurrentBehavior(&leftWingOut);
								rightWingStateController.setCurrentBehavior(&rightWingOut);
							}},
							{1.5, [] () -> void {
								leftWingStateController.useDefaultBehavior();
								rightWingStateController.useDefaultBehavior();
							}}
					}));

	drivetrainStateController.waitUntilDone()();

	turnTo(270_deg, 500_ms);

	intakeStateController.setCurrentBehavior(&intakeEject);

	move(20_in, {61_in/second, 200_in/second/second, 0.0}, 0.0, 270_deg);
	move(-14.5_in, {61_in/second, 200_in/second/second, 0.0}, 0.0, 270_deg);

	turnTo(210_deg, 400_ms);

	drivetrainStateController.setCurrentBehavior(
			new PathPlanner::PathFollower(
					"TestPath",
					{64_in/second, 170_in/second/second, 0.0},
					drivetrain,
					[ObjectPtr = &odometry] { return ObjectPtr->getAngle(); },
					movingTurnPid,
					distancePid,
					8000.0/64.0,
					61_in/second,
					Auto6BallPath2,
					{
							{0.6, [] () -> void {
								intakeStateController.setCurrentBehavior(&intakeIntaking);
							}},
					}));

	drivetrainStateController.waitUntilDone()();

	turnTo(340_deg, 500_ms);

	drivetrainStateController.setCurrentBehavior(
			new PathPlanner::PathFollower(
					"TestPath",
					{64_in/second, 200_in/second/second, 0.0},
					drivetrain,
					[ObjectPtr = &odometry] { return ObjectPtr->getAngle(); },
					movingTurnPid,
					distancePid,
					8000.0/64.0,
					61_in/second,
					Auto6BallPath3,
					{
							{0.3, [] () -> void {
								intakeStateController.setCurrentBehavior(&intakeEject);
							}},
							{1.2, [] () -> void {
								intakeStateController.useDefaultBehavior();
							}},
							{2.2, [] () -> void {
								intakeStateController.setCurrentBehavior(&intakeIntaking);
							}}
					}));

	drivetrainStateController.waitUntilDone()();

	turnTo(360_deg, 500_ms);

	drivetrainStateController.setCurrentBehavior(
			new PathPlanner::PathFollower(
					"TestPath",
					{65_in/second, 200_in/second/second, 0.0},
					drivetrain,
					[ObjectPtr = &odometry] { return ObjectPtr->getAngle(); },
					movingTurnPid,
					distancePid,
					8000.0/64.0,
					61_in/second,
					Auto6BallPath4,
					{
							{0.1, [] () -> void {
								intakeStateController.setCurrentBehavior(&intakeEject);
							}},
							{1.6, [] () -> void {
								intakeStateController.useDefaultBehavior();
								leftWingStateController.setCurrentBehavior(&leftWingOut);
								rightWingStateController.setCurrentBehavior(&rightWingOut);
							}}
					}));

	drivetrainStateController.waitUntilDone()();

	drivetrain.tankSteerVoltage(-4000, 0);

	pros::Task::delay(2000);

	return 0;
}

int far6BallRush() {
	threeWheelOdom.reset(Pose2D(0_in, 0_in, 180_deg));

	drivetrainStateController.setCurrentBehavior(
			new PathPlanner::PathFollower(
					"TestPath",
					{64_in/second, 200_in/second/second, 0.0},
					drivetrain,
					[ObjectPtr = &odometry] { return ObjectPtr->getAngle(); },
					movingTurnPid,
					distancePid,
					8000.0/64.0,
					65_in/second,
					Auto6BallPath1,
					{
							{0.1, [] () -> void {
								intakeStateController.setCurrentBehavior(&intakeIntaking);
							}},
							{1.2, [] () -> void {
								leftWingStateController.setCurrentBehavior(&leftWingOut);
								rightWingStateController.setCurrentBehavior(&rightWingOut);
							}},
							{1.5, [] () -> void {
								leftWingStateController.useDefaultBehavior();
								rightWingStateController.useDefaultBehavior();
							}}
					}));

	drivetrainStateController.waitUntilDone()();

	turnTo(280_deg, 500_ms);

	intakeStateController.setCurrentBehavior(&intakeEject);

	move(20_in, {65_in/second, 200_in/second/second, 0.0}, 0.0, 280_deg);
	move(-14.5_in, {65_in/second, 200_in/second/second, 0.0}, 0.0, 280_deg);

	turnTo(200_deg, 400_ms);

	drivetrainStateController.setCurrentBehavior(
			new PathPlanner::PathFollower(
					"TestPath",
					{64_in/second, 170_in/second/second, 0.0},
					drivetrain,
					[ObjectPtr = &odometry] { return ObjectPtr->getAngle(); },
					movingTurnPid,
					distancePid,
					8000.0/64.0,
					65_in/second,
					{
							{PathPlanner::BezierSegment(PathPlanner::Point(108_in, 128_in), PathPlanner::Point(100_in, 100_in), PathPlanner::Point(88_in, 100_in), PathPlanner::Point(88_in, 75_in), false),
									nullptr},
							{PathPlanner::BezierSegment(PathPlanner::Point(93_in, 70_in), PathPlanner::Point(93_in, 80_in), PathPlanner::Point(90_in, 80_in), PathPlanner::Point(90_in, 87_in), true),
									nullptr}
					},
					{
							{0.6, [] () -> void {
								intakeStateController.setCurrentBehavior(&intakeIntaking);
							}},
					}));

	drivetrainStateController.waitUntilDone()();

	turnTo(340_deg, 500_ms);

	drivetrainStateController.setCurrentBehavior(
			new PathPlanner::PathFollower(
					"TestPath",
					{64_in/second, 200_in/second/second, 0.0},
					drivetrain,
					[ObjectPtr = &odometry] { return ObjectPtr->getAngle(); },
					movingTurnPid,
					distancePid,
					8000.0/64.0,
					65_in/second,
					Auto6BallPath3,
					{
							{0.3, [] () -> void {
								intakeStateController.setCurrentBehavior(&intakeEject);
							}},
							{1.2, [] () -> void {
								intakeStateController.useDefaultBehavior();
							}},
							{2.2, [] () -> void {
								intakeStateController.setCurrentBehavior(&intakeIntaking);
							}}
					}));

	drivetrainStateController.waitUntilDone()();

	turnTo(360_deg, 500_ms);

	drivetrainStateController.setCurrentBehavior(
			new PathPlanner::PathFollower(
					"TestPath",
					{65_in/second, 200_in/second/second, 0.0},
					drivetrain,
					[ObjectPtr = &odometry] { return ObjectPtr->getAngle(); },
					movingTurnPid,
					distancePid,
					8000.0/64.0,
					65_in/second,
					{
							{PathPlanner::BezierSegment(PathPlanner::Point(70_in, 80_in), PathPlanner::Point(70_in, 90_in), PathPlanner::Point(70_in, 100_in), PathPlanner::Point(70_in, 112_in), false),
									nullptr},
							{PathPlanner::BezierSegment(PathPlanner::Point(70_in, 115_in), PathPlanner::Point(70_in, 85_in), PathPlanner::Point(80_in, 84_in), PathPlanner::Point(125_in, 79_in), true),
									nullptr}
					},
					{
							{0.1, [] () -> void {
								intakeStateController.setCurrentBehavior(&intakeEject);
							}},
							{1.6, [] () -> void {
								intakeStateController.useDefaultBehavior();
								leftWingStateController.setCurrentBehavior(&leftWingOut);
								rightWingStateController.setCurrentBehavior(&rightWingOut);
							}}
					}));

	drivetrainStateController.waitUntilDone()();

	drivetrain.tankSteerVoltage(-4000, -4000);

	pros::Task::delay(2000);

	return 0;
}

int skills() {
	threeWheelOdom.reset(Pose2D(0_in, 0_in, -90_deg));

	intakeStateController.setCurrentBehavior(&intakeHold);

	leftWingStateController.setCurrentBehavior(&leftWingOut);

	drivetrainStateController.setCurrentBehavior(
			new PathPlanner::PathFollower(
					"TestPath",
					{61_in/second, 180_in/second/second, 0.0},
					drivetrain,
					[ObjectPtr = &odometry] { return ObjectPtr->getAngle(); },
					movingTurnPid,
					distancePid,
					8000.0/64.0,
					61_in/second,
					Skills1,
					{
					}));

	drivetrainStateController.waitUntilDone()();

	drivetrainStateController.setCurrentBehavior(new RotationController("MatchloadRotationController", drivetrain, odometry, turningPid, 22.5_deg, drivetrainMutex, -1200));

	// Wait until the catapult triballs shot has increased to 46 triballs
	while (modeLogic.getTriballCount() < 44 && catapultStateController.getDuration() < 3_s) {
		// Wait 0.01s (10 ms * (second / 1000ms) = 0.01s / 100Hz)
		pros::Task::delay(10);
	}

	pros::Task::delay(100);

	leftWingStateController.useDefaultBehavior();
	rightWingStateController.useDefaultBehavior();

	turnTo(90_deg, 300_ms);

	intakeStateController.setCurrentBehavior(&intakeEject);

	move(25_in, defaultProfileConstraints, 0.0, 90_deg);

	pros::Task::delay(50);
	leftWingStateController.setCurrentBehavior(&leftWingOut);

	drivetrainStateController.setCurrentBehavior(
			new PathPlanner::PathFollower(
					"TestPath",
					defaultProfileConstraints,
					drivetrain,
					[ObjectPtr = &odometry] { return ObjectPtr->getAngle(); },
					movingTurnPid,
					distancePid,
					180.0,
					61_in/second,
					Skills2,
					{
							{0.35, [] () -> void {
								rightWingStateController.setCurrentBehavior(&rightWingOut);
							}},
							{0.75, [] () -> void {
								rightWingStateController.setCurrentBehavior(&rightWingIn);
							}},
							{3.15, [] () -> void {
								rightWingStateController.setCurrentBehavior(&rightWingOut);
							}},
							{3.48, [] () -> void {
								rightWingStateController.setCurrentBehavior(&rightWingIn);
							}},
							{5.0, [] () -> void {
								leftWingStateController.setCurrentBehavior(&leftWingIn);
							}},
					}));

	drivetrainStateController.waitUntilDone()();

	turnTo(188_deg, 600_ms);

	leftWingStateController.setCurrentBehavior(&leftWingOut);
	rightWingStateController.setCurrentBehavior(&rightWingOut);

	drivetrainStateController.setCurrentBehavior(
			new PathPlanner::PathFollower(
					"TestPath",
					defaultProfileConstraints,
					drivetrain,
					[ObjectPtr = &odometry] { return ObjectPtr->getAngle(); },
					movingTurnPid,
					distancePid,
					180.0,
					61_in/second,
					Skills3,
					{
							{1.0, [] () -> void {
								rightWingStateController.setCurrentBehavior(&rightWingIn);
								leftWingStateController.setCurrentBehavior(&leftWingIn);
							}},
					}));

	drivetrainStateController.waitUntilDone()();

	turnTo(175_deg, 600_ms);

	leftWingStateController.setCurrentBehavior(&leftWingOut);
	rightWingStateController.setCurrentBehavior(&rightWingOut);

	drivetrainStateController.setCurrentBehavior(
			new PathPlanner::PathFollower(
					"TestPath",
					defaultProfileConstraints,
					drivetrain,
					[ObjectPtr = &odometry] { return ObjectPtr->getAngle(); },
					movingTurnPid,
					distancePid,
					180.0,
					61_in/second,
					Skills4,
					{
							{1.0, [] () -> void {
								rightWingStateController.setCurrentBehavior(&rightWingIn);
								leftWingStateController.setCurrentBehavior(&leftWingIn);
							}},
							{2.0, [] () -> void {
								leftWingStateController.setCurrentBehavior(&leftWingOut);
								rightWingStateController.setCurrentBehavior(&rightWingOut);
							}},
					}));

	drivetrainStateController.waitUntilDone()();

	return 0;
}

int safeCloseAWP() {
	threeWheelOdom.reset(Pose2D(0_in, 0_in, -135_deg));

	move(10_in, defaultProfileConstraints, 0.0);

	leftWingStateController.setCurrentBehavior(&leftWingOut);

	drivetrainStateController.setCurrentBehavior(
			new PathPlanner::PathFollower(
					"TestPath",
					pushingProfileConstraints,
					drivetrain,
					[ObjectPtr = &odometry] { return ObjectPtr->getAngle(); },
					movingTurnPid,
					distancePid,
					180.0,
					61_in/second,
					SafeCloseAWP1,
					{
							{0.2, [] () -> void {
								leftWingStateController.setCurrentBehavior(&leftWingIn);
							}},
					}));

	drivetrainStateController.waitUntilDone()();

	rightWingStateController.setCurrentBehavior(&rightWingOut);

	drivetrainStateController.setCurrentBehavior(new RotationController("MatchloadRotationController", drivetrain, odometry, turningPid, -173_deg, drivetrainMutex, -3000));

	pros::Task::delay(20000);

	return 0;
}

int postAuton() {
	pros::Task::delay(10);

	drivetrainStateController.useDefaultBehavior();
	intakeStateController.useDefaultBehavior();
	catapultStateController.useDefaultBehavior();
	leftWingStateController.useDefaultBehavior();
	rightWingStateController.useDefaultBehavior();

	return 0;
}

// !SECTION

// SECTION INIT

[[noreturn]] void update() {

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

[[noreturn]] void updateDisplay() {

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

	// std::cout << count << std::endl;

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
		lv_table_set_cell_value(drivetrainTable.get(), 3, 0, (std::to_string(intakeMotor.get_temperature()) + " C").c_str());
		lv_table_set_cell_value(drivetrainTable.get(), 0, 1, (std::to_string(rightDrive1.get_temperature()) + " C").c_str());
		lv_table_set_cell_value(drivetrainTable.get(), 1, 1, (std::to_string(rightDrive2.get_temperature()) + " C").c_str());
		lv_table_set_cell_value(drivetrainTable.get(), 2, 1, (std::to_string(rightDrive3.get_temperature()) + " C").c_str());

		// Flywheel
		lv_label_set_text(flywheelLabel.get(), ("Triball count: " + std::to_string(modeLogic.getTriballCount())).c_str());

		pros::Task::delay(50);
	}
}

/**
 * Runs when the robot starts up
 */
void initialize() {

	// std::cout << "INIT" << std::endl;

	robotMutex.take();

	lv_init();
	tabview = std::shared_ptr<lv_obj_t>(lv_tabview_create(lv_scr_act(), NULL));

	// Initialize functions
	initHardware();
	initIntake();
	initCatapult();
	initWings();
	initBehaviors();

	robotMutex.give();

	pros::Task display(updateDisplay, TASK_PRIORITY_MIN);
	pros::Task modeLogicTask = pros::Task(update, TASK_PRIORITY_MAX);
}

// !SECTION

// SECTION Disabled
/**
 * Runs while the robot is disabled i.e. before and after match, between auton
 * and teleop period
 */
[[noreturn]] void disabled() {
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
	far6BallFullAWP();
	#elif AUTON == 1
	closeAWP();
	#elif AUTON == 2
	skills();
	#elif AUTON == 3
	safeCloseAWP();
	#elif AUTON == 4
	testBezier();
	#endif // !1

	postAuton();
}

// !SECTION

// SECTION Operator Control

/**
 * Runs during operator/teleop control
 */
void opcontrol() {

//	pros::delay(5000);

	// Causes the programming skills code to only run during skills
#if AUTON == 2
	{
		preAutonRun();
		pros::Task auton(skills);

		while (!skillsDone) {
			if (skillsDone) {
				break;
			}
			if (master->get_digital(Pronounce::E_CONTROLLER_DIGITAL_A)) {
				auton.suspend();
				auton.remove();
				break;
			}
		}
	}
#endif

	robotMutex.take();
	postAuton();
	teleopController.setCurrentBehavior(&teleopModeLogic);
	robotMutex.give();
}

// !SECTION