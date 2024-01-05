#include "main.h"

// LVGL
std::shared_ptr<lv_obj_t> tabview;

bool skillsDone = false;

// SECTION Auton

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

void closeAWP(void* args) {

	threeWheelOdom.reset(Pose2D(130_in, 22_in, -30_deg));

	catapultStateController.setCurrentBehavior(catapultFire.wait(800_ms));
	catapultStateController.waitUntilDone()();

	drivetrainStateController.setCurrentBehavior(pathFollower.changePath(defaultProfileConstraints, CloseAWP1,{
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

	drivetrainStateController.setCurrentBehavior(pathFollower.changePath(defaultProfileConstraints, CloseAWP2,
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
}

void far6BallFullAWP(void* args) {
	threeWheelOdom.reset(Pose2D(0_in, 0_in, 180_deg));

	drivetrainStateController.setCurrentBehavior(pathFollower.changePath({(64_in/second).getValue(), (200_in/second/second).getValue(), 0.0}, Auto6BallPath1, {
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

	drivetrainStateController.setCurrentBehavior(pathFollower.changePath(defaultProfileConstraints, Auto6BallPath2, {
							{0.6, [] () -> void {
								intakeStateController.setCurrentBehavior(&intakeIntaking);
							}},
					}));

	drivetrainStateController.waitUntilDone()();

	turnTo(340_deg, 500_ms);

	drivetrainStateController.setCurrentBehavior(pathFollower.changePath({(64_in/second).getValue(), (200_in/second/second).getValue(), 0.0}, Auto6BallPath3,
					{
							{0.1, [] () -> void {
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

	drivetrainStateController.setCurrentBehavior(pathFollower.changePath({(61_in/second).getValue(), (200_in/second/second).getValue(), 0.0}, Auto6BallPath4,
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
}

void far6BallRush(void* args) {
	threeWheelOdom.reset(Pose2D(0_in, 0_in, 180_deg));

	drivetrainStateController.setCurrentBehavior(pathFollower.changePath(defaultProfileConstraints, TestPath,
					{
							{0.1, [] () -> void {
								intakeStateController.setCurrentBehavior(&intakeIntaking);
							}},
					}));

	drivetrainStateController.waitUntilDone()();

	turnTo(280_deg, 500_ms);

	pros::Task::delay(2000);
}

void skills(void* args) {
	threeWheelOdom.reset(Pose2D(0_in, 0_in, 90_deg));
	std::cout << "SKILLS START" << std::endl;

//	intakeStateController.setCurrentBehavior(&intakeHold);
//
//	leftWingStateController.setCurrentBehavior(&leftWingOut);
//
//	auton.resetTriballs();
//
//	drivetrainStateController.setCurrentBehavior(pathFollower.changePath(defaultProfileConstraints, Skills1));
//
//	drivetrainStateController.waitUntilDone()();
//
//	drivetrainStateController.setCurrentBehavior(new RotationController("MatchloadRotationController", drivetrain, odometry, turningPid, 22.5_deg, drivetrainMutex, -1200));
//
//	// Wait until the catapult triballs shot has increased to 46 triballs
//	while (auton.getTriballCount() < 44 && catapultStateController.getDuration() < 1.6_s) {
//		// Wait 0.01s (10 ms * (second / 1000ms) = 0.01s / 100Hz)
//		pros::Task::delay(10);
//	}
//
//	pros::Task::delay(200);
//
//	leftWingStateController.useDefaultBehavior();
//	rightWingStateController.useDefaultBehavior();
//
//	turnTo(90_deg, 300_ms);
//
//	intakeStateController.setCurrentBehavior(&intakeEject);
//
//	move(25_in, oldDefaultProfileConstraints, 0.0, 90_deg);
//
//	pros::Task::delay(50);
//	leftWingStateController.setCurrentBehavior(&leftWingOut);

	drivetrainStateController.setCurrentBehavior(pathFollower.changePath(defaultProfileConstraints, Skills2, {
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
							{6.2, [] () -> void {
								leftWingStateController.setCurrentBehavior(&leftWingIn);
							}},
					}));

	drivetrainStateController.waitUntilDone()();

	pros::Task::delay(50);

	return;

	turnTo(197_deg, 400_ms);

	leftWingStateController.setCurrentBehavior(&leftWingOut);
	rightWingStateController.setCurrentBehavior(&rightWingOut);

	drivetrainStateController.setCurrentBehavior(pathFollower.changePath(defaultProfileConstraints, Skills3,
					{
							{1.0, [] () -> void {
								rightWingStateController.setCurrentBehavior(&rightWingIn);
								leftWingStateController.setCurrentBehavior(&leftWingIn);
							}},
					}));

	drivetrainStateController.waitUntilDone()();

	turnTo(175_deg, 450_ms);

	leftWingStateController.setCurrentBehavior(&leftWingOut);
	rightWingStateController.setCurrentBehavior(&rightWingOut);

	turnTo(175_deg, 450_ms);

	drivetrainStateController.setCurrentBehavior(pathFollower.changePath(defaultProfileConstraints,
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

	leftWingStateController.setCurrentBehavior(&leftWingIn);
	rightWingStateController.setCurrentBehavior(&rightWingIn);

	drivetrainStateController.setCurrentBehavior(pathFollower.changePath(defaultProfileConstraints,
					DriverSkills5,
					{
							{0.0, [] () -> void {
								intakeStateController.setCurrentBehavior(&intakeEject);
							}},
					}));

	drivetrainStateController.waitUntilDone()();

	move(-10_in, oldDefaultProfileConstraints, 0.0);

	turnTo(90_deg, 500_ms);

	rightWingStateController.setCurrentBehavior(&rightWingOut);

	drivetrainStateController.setCurrentBehavior(pathFollower.changePath(defaultProfileConstraints,
					DriverSkills6,
					{
							{4.5, [] () -> void {
								rightWingStateController.setCurrentBehavior(&rightWingOut);
								leftWingStateController.setCurrentBehavior(&leftWingOut);
							}},
							{4.8, [] () -> void {
								leftWingStateController.setCurrentBehavior(&leftWingIn);
							}},
							{7.0, [] () -> void {
								leftWingStateController.setCurrentBehavior(&leftWingOut);
							}},
							{7.3, [] () -> void {
								leftWingStateController.setCurrentBehavior(&leftWingIn);
							}},
					}));

	drivetrainStateController.waitUntilDone()();

	move(5_in, oldDefaultProfileConstraints, 0.0, -270_deg);
	move(-10_in, oldDefaultProfileConstraints, 0.0, -270_deg);
	move(5_in, oldDefaultProfileConstraints, 0.0, -270_deg);
	move(-10_in, oldDefaultProfileConstraints, 0.0, -270_deg);

	drivetrainStateController.setCurrentBehavior(pathFollower.changePath(defaultProfileConstraints,
					Skills6,
					{
							{1.0, [] () -> void {
								rightWingStateController.setCurrentBehavior(&rightWingOut);
								leftWingStateController.setCurrentBehavior(&leftWingOut);
							}},
							{2.0, [] () -> void {
								rightWingStateController.setCurrentBehavior(&rightWingOut);
								leftWingStateController.setCurrentBehavior(&leftWingOut);
							}},
					}));

	drivetrainStateController.waitUntilDone()();
}

void safeCloseAWP(void* args) {
	threeWheelOdom.reset(Pose2D(0_in, 0_in, -135_deg));

	move(10_in, oldDefaultProfileConstraints, 0.0);

	intakeStateController.setCurrentBehavior(&intakeEject);

	leftWingStateController.setCurrentBehavior(&leftWingOut);

	drivetrainStateController.setCurrentBehavior(pathFollower.changePath(defaultProfileConstraints,
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
}

// !SECTION

// SECTION INIT

[[noreturn]] void update() {

	std::cout << "Init: update" << std::endl;

	competitionController.initialize();

	robotMutex.give();

	uint32_t startTime;
	uint32_t startTimeMicros;

	while (true) {
		// Create stuff for exact delay
		startTime = pros::millis();
		startTimeMicros = pros::micros();

		robotMutex.take(TIMEOUT_MAX);
		odometry.update();
		competitionController.update();
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
		lv_label_set_text(flywheelLabel.get(), ("Triball count: " + std::to_string(teleop.getTriballCount())).c_str());

		pros::Task::delay(50);
	}
}

/**
 * Runs when the robot starts up
 */
void initialize() {

//	robotMutex.take(TIMEOUT_MAX);
	std::cout << "Init: initialize" << std::endl;

	lv_init();
	tabview = std::shared_ptr<lv_obj_t>(lv_tabview_create(lv_scr_act(), NULL));

	// Initialize functions
	initHardware();
	initIntake();
	initCatapult();
	initWings();
	initBehaviors();

	pros::Task modeLogicTask(update, TASK_PRIORITY_MAX);
	pros::Task display(updateDisplay, TASK_PRIORITY_MIN);

	pros::Task::delay(10);
}

// !SECTION

// SECTION Disabled
/**
 * Runs while the robot is disabled i.e. before and after match, between auton
 * and teleop period
 */
void disabled() {
	std::cout << "Init: disabled" << std::endl;
	competitionController.useDefaultBehavior();

	// Create a label
	lv_obj_t* disabledLabel = lv_label_create(lv_scr_act(), NULL);
	lv_obj_align(disabledLabel, NULL, LV_ALIGN_CENTER, 0, 0);
	lv_label_set_text(disabledLabel, "Robot Disabled.");
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

	std::cout << "Init: Auton" << std::endl;

	#if AUTON == 0
	auton.setAuton(far6BallFullAWP);
	#elif AUTON == 1
	auton.setAuton(closeAWP);
	#elif AUTON == 2
	auton.setAuton(skills);
	#elif AUTON == 3
	auton.setAuton(safeCloseAWP)
	#endif // !1

	std::cout << "HIIPROS auton starting" << std::endl;

	competitionController.setCurrentBehavior(&auton);
}

// !SECTION

// SECTION Operator Control

/**
 * Runs during operator/teleop control
 */
void opcontrol() {

	std::cout << "Init: OP control" << std::endl;

	// Causes the programming skills code to only run during skills
#if AUTON == 2
	robotMutex.take(TIMEOUT_MAX);
	competitionController.setCurrentBehavior(auton.setAuton(skills));
	robotMutex.give();
	while (!master->get_digital(Pronounce::E_CONTROLLER_DIGITAL_A)){// || !competitionController.isDone()) {
		pros::Task::delay(10);
	}
#endif

	std::cout << "HII2: a pressed" << std::endl;

	robotMutex.take(TIMEOUT_MAX);
	competitionController.setCurrentBehavior(&teleop);
	robotMutex.give();
}

// !SECTION