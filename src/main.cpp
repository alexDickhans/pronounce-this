#include "main.h"

// LVGL
std::shared_ptr<lv_obj_t> tabview;

// SECTION Auton

ASSET(close_mid_rush_json);
ASSET(mid_6_ball_1_json);
ASSET(mid_6_ball_2_json);
ASSET(mid_6_ball_awp_json);
ASSET(safe_close_awp_json);
ASSET(safe_close_awp_2_json);

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

void far5BallRushMid(void* args) {

	threeWheelOdom.reset(Pose2D(0_in, 0_in, 80.7_deg));

	intakeStateController(&intakeIntaking);
	rightWingStateController(rightWingOut.wait(200_ms));

	move(50_in, speedProfileConstraints, 0.0, 80.7_deg);

	drivetrainStateController(pathFollower.changePath(mid_6_ball_1_json))->wait();

	turnTo(2_deg, 550_ms);

	intakeStateController(&intakeIntaking);

	move(19_in, speedProfileConstraints, 0.0, 2_deg, 0.0, 0.0);
	move(-16_in, speedProfileConstraints, 0.0, 2_deg, 0.0, 0.0);

//	move(-4_in, speedProfileConstraints, 0.0, 2_deg);
	turnTo(-170_deg, 700_ms);

	drivetrainStateController(pathFollower.changePath(mid_6_ball_2_json))->wait();

	move(-8_in, speedProfileConstraints, 0.0);

	turnTo(-245_deg, 300_ms);
	move(18_in, speedProfileConstraints, 0.0, -245_deg, 0.0, -60_in/second);
	move (-8_in, speedProfileConstraints, 0.0, -270_deg);
	leftWingStateController(&leftWingIn);
	turnTo(-335_deg, 200_ms);
	intakeStateController(&intakeIntaking);
	move(48_in, speedProfileConstraints, 0.0, -335_deg);

	turnTo(-213_deg, 550_ms);
	intakeExtensionStateController(&outtakeSequence);
	move(38_in, speedProfileConstraints, 0.0, -213_deg);
}

void far6BallRushMid(void* args) {
	far5BallRushMid(args);

	turnTo(-357_deg, 550_ms);

	intakeStateController(&intakeIntaking);

	move(28_in, defaultProfileConstraints, 0.0, -357_deg);

	turnTo(-180_deg, 550_ms);
	intakeExtensionStateController(&outtakeSequence);
	leftWingStateController(&leftWingOut);
	rightWingStateController(&rightWingOut);
	move(35_in, speedProfileConstraints, 0.0, -180_deg);
	move(-10_in, speedProfileConstraints, 0.0, -180_deg);
	turnTo(-360_deg, 3_s);
}

void far5BallAWP(void* args) {
	far5BallRushMid(args);

	drivetrainStateController.setCurrentBehavior(pathFollower.changePath(mid_6_ball_awp_json));

	drivetrainStateController.waitUntilDone();

	drivetrainStateController.setCurrentBehavior(new RotationController("MatchloadRotationController", drivetrain, odometry, turningPid, -300_deg, drivetrainMutex, -3000));

	pros::Task::delay(5000);
}

void skills(void* args) {
	threeWheelOdom.reset(Pose2D(0_in, 0_in, -90_deg));
	std::cout << "SKILLS START" << std::endl;

	intakeStateController.setCurrentBehavior(&intakeHold);

	leftWingStateController.setCurrentBehavior(&leftWingOut);

	auton.resetTriballs();

//	drivetrainStateController.setCurrentBehavior(pathFollower.changePath(speedProfileConstraints, Skills1));
//	drivetrainStateController.waitUntilDone()();

	drivetrainStateController.setCurrentBehavior(new RotationController("MatchloadRotationController", drivetrain, odometry, turningPid, 18.5_deg, drivetrainMutex, -1200));

	// Wait until the catapult triballs shot has increased to 46 triballs
	while (auton.getTriballCount() < 44 && catapultStateController.getDuration() < 1.6_s) {
		// Wait 0.01s (10 ms * (second / 1000ms) = 0.01s / 100Hz)
		pros::Task::delay(10);
	}

	pros::Task::delay(200);
}

void safeCloseAWP(void* args) {
	threeWheelOdom.reset(Pose2D(0_in, 0_in, 45_deg));

	move(-10_in, defaultProfileConstraints, 0.0);

	intakeStateController.setCurrentBehavior(&intakeIntaking);

	drivetrainStateController(pathFollower.changePath(safe_close_awp_json))->wait();

	move(-8_in, defaultProfileConstraints, 0.0, -20_deg);

	drivetrainStateController(pathFollower.changePath(safe_close_awp_2_json))->wait();

	pros::Task::delay(15000);
}

void closeRushMid(void* args) {
	threeWheelOdom.reset(Pose2D(0_in, 0_in, -75.7_deg));

	leftWingStateController(leftWingOut.wait(300_ms));

	intakeStateController(&intakeIntaking);

	move(47_in, speedProfileConstraints, 0.0, -75.7_deg);

	move(-23_in, speedProfileConstraints, 0.0, -75.7_deg);

	turnTo(104.3_deg, 800_ms);

	intakeStateController(&intakeEject);

	turnTo(104.3_deg, 200_ms);

	turnTo(-35.3_deg, 600_ms);

	drivetrainStateController.setCurrentBehavior(pathFollower.changePath(close_mid_rush_json));

	drivetrainStateController.waitUntilDone();

	turnTo(15_deg, 800_ms);

	intakeStateController(&intakeEject);

	move(26_in, speedProfileConstraints, 0.0, 15_deg);
}

void closeRushMidElim(void* args) {
	closeRushMid(args);

	move(-8_in, speedProfileConstraints, 0.0, -355_deg);

	turnTo(-180_deg, 800_ms);

	intakeStateController(&intakeIntaking);

	drivetrainStateController(pathFollower.changePath(close_mid_rush_json));

	drivetrainStateController.waitUntilDone();
}

void tuneTurnPid(void* args) {
	threeWheelOdom.reset(Pose2D(0_in, 0_in, 0.0_deg));
	while(1) {
		turnTo(180_deg, 2_s);
		turnTo(0.0_deg, 2_s);
	}
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

	std::shared_ptr<lv_obj_t> flywheelTab = std::shared_ptr<lv_obj_t>(lv_tabview_add_tab(tabview.get(), "PTO"));
	std::shared_ptr<lv_obj_t> flywheelLabel = std::shared_ptr<lv_obj_t>(lv_label_create(flywheelTab.get(), NULL));

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

		auto gpsStatus = gps.get_status();

		telemetryRadio.transmit("[" + std::to_string(static_cast<int>((gpsStatus.x * 350.0 / 1.8 + 350))) + "," + std::to_string(static_cast<int>((gpsStatus.y * 350.0 / 1.8 + 350))) + "]\n");

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

	pathFollower.addCommandMapping("intake", [&]() -> void {
		intakeStateController(&intakeIntaking);
	});

	pathFollower.addCommandMapping("intakeStopped", [&]() -> void {
		intakeStateController();
	});

	pathFollower.addCommandMapping("outtake", [&]() -> void {
		intakeStateController(&intakeEject);
	});

	pathFollower.addCommandMapping("leftWingOut", [&]() -> void {
		leftWingStateController(&leftWingOut);
	});

	pathFollower.addCommandMapping("leftWingIn", [&]() -> void {
		leftWingStateController(&leftWingIn);
	});

	pathFollower.addCommandMapping("rightWingOut", [&]() -> void {
		rightWingStateController(&rightWingOut);
	});

	pathFollower.addCommandMapping("rightWingIn", [&]() -> void {
		rightWingStateController(&rightWingIn);
	});

	pathFollower.addCommandMapping("awpOut", [&]() -> void {
		awpStateController(&awpOut);
	});

	pathFollower.addCommandMapping("awpIn", [&]() -> void {
		awpStateController(&awpIn);
	});

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
//#define ELIM
/**
 * Runs during the autonomous. NO user control
 */
void autonomous() {

	std::cout << "Init: Auton" << AUTON << std::endl;

	#if AUTON == 0
	auton.setAuton(far6BallRushMid);
	#elif AUTON == 1
	auton.setAuton(far5BallAWP);
	#elif AUTON == 4
	auton.setAuton(safeCloseAWP);
	#elif AUTON == 5
	auton.setAuton(closeRushMidAWP);
	#elif AUTON == 7
	auton.setAuton(skills);
	#endif // !1

//	auton.setAuton(tuneTurnPid);

	competitionController.setCurrentBehavior(&auton);
}

// !SECTION

// SECTION Operator Control

/**
 * Runs during operator/teleop control
 */
void opcontrol() {

	// Causes the programming skills code to only run during skills
#if AUTON == 7
	robotMutex.take(TIMEOUT_MAX);
	competitionController.setCurrentBehavior(auton.setAuton(skills));
	robotMutex.give();
	while (!master->get_digital(Pronounce::E_CONTROLLER_DIGITAL_A) && !competitionController.isDone()) {
		pros::Task::delay(10);
	}
#endif

	robotMutex.take(TIMEOUT_MAX);
	competitionController.setCurrentBehavior(&teleop);
	robotMutex.give();
}

// !SECTION