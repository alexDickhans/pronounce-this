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
ASSET(skills_1_json);
ASSET(skills_2_json);
ASSET(skills_3_json);
ASSET(skills_4_json);
ASSET(skills_5_json);
ASSET(skills_6_json);
ASSET(skills_6_5_json);
ASSET(skills_7_json);
ASSET(skills_7_5_json);
ASSET(skills_8_json);
ASSET(skills_9_json);
ASSET(skills_10_json);
ASSET(close_mid_rush_elim_json);
ASSET(close_rush_mid_2_json);

void turnTo(Angle angle, QTime waitTimeMS) {
	auto angleRotation = std::make_shared<RotationController>("AngleTurn", drivetrain, odometry, turningPid, angle,
	                                                          drivetrainMutex);

	drivetrainStateController->sb(angleRotation);

	pros::Task::delay(waitTimeMS.Convert(millisecond));

	drivetrainStateController->ud();
	pros::Task::delay(10);
}

void move(QLength distance, ProfileConstraints profileConstraints, QCurvature curvature, QSpeed initialSpeed = 0.0,
          QSpeed endSpeed = 0.0) {
	drivetrainStateController->sb(
			std::make_shared<TankMotionProfiling>("moveDistance", &drivetrain, profileConstraints, distance, &odometry,
			                                      &distancePid,
			                                      drivetrainMutex, curvature, initialSpeed, endSpeed)).wait();
}

void move(QLength distance, ProfileConstraints profileConstraints, QCurvature curvature, Angle startAngle,
          QSpeed initialSpeed = 0.0, QSpeed endSpeed = 0.0) {

	drivetrainStateController->sb(
			std::make_shared<TankMotionProfiling>("moveDistance", &drivetrain, profileConstraints, distance, &odometry,
			                                      &distancePid,
			                                      drivetrainMutex, curvature, startAngle, &movingTurnPid, initialSpeed,
			                                      endSpeed)).wait();
}

void far5BallRushMid(void *args) {

	threeWheelOdom.reset(Pose2D(0_in, 0_in, 80.7_deg));

	intakeExtensionStateController->sb(deploySequence);
	rightWingStateController->sb(std::make_shared<Wait>(rightWingOut, 200_ms));

	move(50_in, speedProfileConstraints, 0.0, 80.7_deg);

	intakeExtensionStateController->ud();
	intakeStateController->sb(intakeIntaking);

	pathFollower->changePath(mid_6_ball_1_json);
	drivetrainStateController->sb(pathFollower).wait();

	turnTo(-2_deg, 550_ms);

	intakeStateController->sb(intakeIntaking);

	move(19_in, speedProfileConstraints, 0.0, 2_deg, 0.0, 0.0);
	move(-16_in, speedProfileConstraints, 0.0, 2_deg, 0.0, 0.0);

	intakeStateController->sb(intakeHold);
//	move(-4_in, speedProfileConstraints, 0.0, 2_deg);
	turnTo(170_deg, 700_ms);

	pathFollower->changePath(mid_6_ball_2_json);
	drivetrainStateController->sb(pathFollower).wait();

	move(-12_in, speedProfileConstraints, 0.0, 110_deg);

	leftWingStateController->ud();
	turnTo(120_deg, 300_ms);
	leftWingStateController->sb(leftWingOut);
	drivetrain.tankSteerVoltage(12000, 12000);
	pros::Task::delay(800);
	drivetrain.tankSteerVoltage(0.0, 0.0);
	leftWingStateController->ud();
	move(-9_in, speedProfileConstraints, 0.0, 90_deg);
	turnTo(25_deg, 200_ms);
	intakeStateController->sb(intakeIntaking);
	move(48_in, speedProfileConstraints, 0.0, 25_deg);

	turnTo(150_deg, 550_ms);
	intakeExtensionStateController->sb(outtakeSequence);
	move(38_in, speedProfileConstraints, 0.0, 150_deg);
}

void far6BallRushMid(void *args) {
	far5BallRushMid(args);

	turnTo(3_deg, 550_ms);

	intakeStateController->sb(intakeIntaking);

	move(23_in, defaultProfileConstraints, 0.0, 3_deg);

	turnTo(180_deg, 550_ms);
	intakeExtensionStateController->ud();
	intakeStateController->sb(intakeEject);
	leftWingStateController->sb(leftWingOut);
	rightWingStateController->sb(rightWingOut);
	move(35_in, speedProfileConstraints, 0.0, 180_deg);
	move(-10_in, speedProfileConstraints, 0.0, 180_deg);
	turnTo(0_deg, 3_s);
}

void far5BallAWP(void *args) {
	far5BallRushMid(args);

	move(-5_in, speedProfileConstraints, 0.0, 0_deg);

	turnTo(-90_deg, 600_ms);

	pathFollower->changePath(mid_6_ball_awp_json);
	drivetrainStateController->sb(pathFollower).wait();

	drivetrain.tankSteerVoltage(3000, 2000);
	pros::Task::delay(5000);
}

void skills(void *args) {

	threeWheelOdom.reset(Pose2D(0_in, 0_in, 135_deg));

	pathFollower->changePath(skills_1_json);
	drivetrainStateController->sb(pathFollower).wait();

	drivetrainStateController->sb(
			std::make_shared<RotationController>("MatchloadRotationController", drivetrain, odometry, turningPid,
			                                     21.1_deg,
			                                     drivetrainMutex, -800.0));
	auton->resetTriballs();
	pros::Task::delay(500);

	// Wait until the catapult triballs shot has increased to 44 triballs
	while (auton->getTriballCount() < 44 && catapultStateController->getDuration() < 2.0_s) {
		// Wait 0.01s (10 ms * (second / 1000ms) = 0.01s / 100Hz)
		pros::Task::delay(10);
	}

	pros::Task::delay(200);

	pathFollower->changePath(skills_2_json);
	drivetrainStateController->sb(pathFollower).wait();

	rightWingStateController->sb(rightWingIn);
	turnTo(180_deg, 300_ms);

	pathFollower->changePath(skills_3_json);
	drivetrainStateController->sb(pathFollower).wait();

	move(-15_in, speedProfileConstraints, 0.0, -70_deg);
//
//	drivetrainStateController(pathFollower.changePath(skills_4_json))->wait();
//
//	move(-15_in, speedProfileConstraints, 0.0, -75_deg);

	pathFollower->changePath(skills_4_json);
	drivetrainStateController->sb(pathFollower).wait();
	move(-5_in, speedProfileConstraints, 0.0, -75_deg);

	turnTo(-160_deg, 200_ms);

	pathFollower->changePath(skills_5_json);
	drivetrainStateController->sb(pathFollower).wait();

	move(-5_in, speedProfileConstraints, 0.0);

	move(8_in, speedProfileConstraints, 0.0, 0.0_deg);

	pathFollower->changePath(skills_6_json);
	drivetrainStateController->sb(pathFollower).wait();

	move(-3_in, speedProfileConstraints, 0.0);

	move(8_in, speedProfileConstraints, 0.0, 0.0_deg);

	pathFollower->changePath(skills_6_5_json);
	drivetrainStateController->sb(pathFollower).wait();

	move(-3_in, speedProfileConstraints, 0.0);

	move(8_in, speedProfileConstraints, 0.0, 0.0_deg);

	pathFollower->changePath(skills_7_json);
	drivetrainStateController->sb(pathFollower).wait();

	move(-3_in, speedProfileConstraints, 0.0);

	move(8_in, speedProfileConstraints, 0.0, 0.0_deg);

	pathFollower->changePath(skills_7_5_json);
	drivetrainStateController->sb(pathFollower).wait();

	drivetrainStateController->sb(
			std::make_shared<RotationController>("MatchloadRotationController", drivetrain, odometry, turningPid, 0_deg,
			                                     drivetrainMutex));

	QLength wallDistance = getDistanceSensorMedian(distanceSensor, 3) * 1_mm;

	pathFollower->changePath(pushingProfileConstraints, {{
			                                                     PathPlanner::BezierSegment(
					                                                     PathPlanner::Point(
							                                                     wallDistance, 76_in),
					                                                     PathPlanner::Point(
							                                                     wallDistance.getValue() * 0.74,
							                                                     68_in),
					                                                     PathPlanner::Point(
							                                                     19_in, 55_in),
					                                                     PathPlanner::Point(
							                                                     20_in, 20_in), true),
			                                                     nullptr}});
	drivetrainStateController->sb(pathFollower).wait();

	pathFollower->changePath(skills_8_json);
	drivetrainStateController->sb(pathFollower).wait();

	move(-15_in, speedProfileConstraints, 0.0, 70_deg);

	pathFollower->changePath(skills_9_json);
	drivetrainStateController->sb(pathFollower).wait();

	move(-15_in, speedProfileConstraints, 0.0, 80_deg);

	pathFollower->changePath(skills_9_json);
	drivetrainStateController->sb(pathFollower).wait();

	rightWingStateController->ud();

	move(-15_in, speedProfileConstraints, 0.0, 80_deg);

	pathFollower->changePath(skills_10_json);
	drivetrainStateController->sb(pathFollower).wait();

	pros::Task::delay(500);

	hangStateController->ud();
}

void safeCloseAWP(void *args) {
	threeWheelOdom.reset(Pose2D(0_in, 0_in, 45_deg));

	intakeExtensionStateController->sb(deploySequence);

	move(-10_in, defaultProfileConstraints, 0.0);

	intakeExtensionStateController->ud();
	intakeStateController->sb(intakeIntaking);

	pathFollower->changePath(safe_close_awp_json);
	drivetrainStateController->sb(pathFollower).wait();

	move(-8_in, defaultProfileConstraints, 0.0, 35_deg);

	turnTo(45_deg, 300_ms);

	pathFollower->changePath(safe_close_awp_2_json);
	drivetrainStateController->sb(pathFollower).wait();

	pros::Task::delay(15000);
}

void closeRushMid(void *args) {
	threeWheelOdom.reset(Pose2D(0_in, 0_in, -75.7_deg));

	leftWingStateController->sb(std::make_shared<Wait>(leftWingOut, 300_ms));

	intakeExtensionStateController->sb(deploySequence);

	move(41_in, speedProfileConstraints, 0.0, -75.7_deg);

	intakeExtensionStateController->ud();
	intakeStateController->sb(intakeIntaking);

	move(-13_in, speedProfileConstraints, 0.0, -75.7_deg);

	turnTo(104.3_deg, 600_ms);

	intakeStateController->sb(intakeEject);

	turnTo(104.3_deg, 200_ms);

	turnTo(-35.3_deg, 600_ms);

	pathFollower->changePath(close_mid_rush_json);
	drivetrainStateController->sb(pathFollower).wait();

	move(-10_in, speedProfileConstraints, 0.0, 56_deg);

	intakeStateController->sb(intakeEject);

	pathFollower->changePath(close_rush_mid_2_json);
	drivetrainStateController->sb(pathFollower).wait();
}

void closeRushMidElim(void *args) {
	closeRushMid(args);

	turnTo(-180_deg, 800_ms);

	intakeStateController->sb(intakeIntaking);

	pathFollower->changePath(close_mid_rush_elim_json);
	drivetrainStateController->sb(pathFollower).wait();
}

void tuneTurnPid(void *args) {
	threeWheelOdom.reset(Pose2D(0_in, 0_in, 0.0_deg));
	while (1) {
		turnTo(180_deg, 2_s);
		turnTo(0.0_deg, 2_s);
	}
}
// !SECTION

// SECTION INIT

[[noreturn]] void update() {

	std::cout << "Init: update" << std::endl;

	competitionController->initialize();

	robotMutex.give();

	uint32_t startTime;
	uint32_t startTimeMicros;

	while (true) {
		// Create stuff for exact delay
		startTime = pros::millis();
		startTimeMicros = pros::micros();

		robotMutex.take(TIMEOUT_MAX);
		odometry.update();
		competitionController->update();
		robotMutex.give();

		// Wait a maximum of 10 milliseconds
		pros::delay(std::min(10 - (pros::millis() - startTime), (long unsigned int) 10));

		std::cout << "FrameTime: " << pros::micros() - startTimeMicros << std::endl;
	}
}

[[noreturn]] void updateDisplay() {

	Log("Init");

	// Odom
	std::shared_ptr<lv_obj_t> odomTab = std::shared_ptr<lv_obj_t>(lv_tabview_add_tab(tabview.get(), "Odom"));
	std::shared_ptr<lv_obj_t> odomLabel = std::shared_ptr<lv_obj_t>(lv_label_create(odomTab.get(), NULL));

	std::shared_ptr<lv_obj_t> flywheelTab = std::shared_ptr<lv_obj_t>(lv_tabview_add_tab(tabview.get(), "PTO"));
	std::shared_ptr<lv_obj_t> flywheelLabel = std::shared_ptr<lv_obj_t>(lv_label_create(flywheelTab.get(), NULL));

	std::shared_ptr<lv_obj_t> portsTab = std::shared_ptr<lv_obj_t>(lv_tabview_add_tab(tabview.get(), "Ports"));
	std::shared_ptr<lv_obj_t> portsPage = std::shared_ptr<lv_obj_t>(lv_page_create(portsTab.get(), NULL));
	std::shared_ptr<lv_obj_t> portsLabel = std::shared_ptr<lv_obj_t>(lv_label_create(portsTab.get(), NULL));
	std::shared_ptr<lv_obj_t> portsTable = std::shared_ptr<lv_obj_t>(lv_table_create(portsPage.get(), NULL));

	lv_obj_set_width(portsPage.get(), 400);
	lv_obj_set_height(portsPage.get(), 100);

	// Drivetrain
	std::shared_ptr<lv_obj_t> drivetrainTab = std::shared_ptr<lv_obj_t>(
			lv_tabview_add_tab(tabview.get(), "Drivetrain"));
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
		                                    ", R: " +
		                                    std::to_string(rightDrive1Odom.getPosition().Convert(inch))).c_str());

		// Drivetrain
		lv_table_set_cell_value(drivetrainTable.get(), 0, 0,
		                        (std::to_string(leftDrive1.get_temperature()) + " C").c_str());
		lv_table_set_cell_value(drivetrainTable.get(), 1, 0,
		                        (std::to_string(leftDrive2.get_temperature()) + " C").c_str());
		lv_table_set_cell_value(drivetrainTable.get(), 2, 0,
		                        (std::to_string(leftDrive3.get_temperature()) + " C").c_str());
		lv_table_set_cell_value(drivetrainTable.get(), 3, 0,
		                        (std::to_string(intakeMotor.get_temperature()) + " C").c_str());
		lv_table_set_cell_value(drivetrainTable.get(), 0, 1,
		                        (std::to_string(rightDrive1.get_temperature()) + " C").c_str());
		lv_table_set_cell_value(drivetrainTable.get(), 1, 1,
		                        (std::to_string(rightDrive2.get_temperature()) + " C").c_str());
		lv_table_set_cell_value(drivetrainTable.get(), 2, 1,
		                        (std::to_string(rightDrive3.get_temperature()) + " C").c_str());

		pros::Task::delay(50);
	}
}

/**
 * Runs when the robot starts up
 */
void initialize() {

	logger = Logger::getInstance();
	Log("Initialize");

	lv_init();
	tabview = std::shared_ptr<lv_obj_t>(lv_tabview_create(lv_scr_act(), NULL));

	// Initialize functions
	initHardware();
	initIntake();
	initWings();
	initBehaviors();
	initCatapult();

	pathFollower->addCommandMapping("intake", [&]() -> void {
		intakeStateController->sb(intakeIntaking);
	});

	pathFollower->addCommandMapping("intakeHold", [&]() -> void {
		intakeStateController->sb(intakeHold);
	});

	pathFollower->addCommandMapping("intakeStopped", [&]() -> void {
		intakeStateController->ud();
	});

	pathFollower->addCommandMapping("outtake", [&]() -> void {
		intakeStateController->sb(intakeEject);
	});

	pathFollower->addCommandMapping("leftWingOut", [&]() -> void {
		leftWingStateController->sb(leftWingOut);
	});

	pathFollower->addCommandMapping("leftWingIn", [&]() -> void {
		leftWingStateController->sb(leftWingIn);
	});

	pathFollower->addCommandMapping("rightWingOut", [&]() -> void {
		rightWingStateController->sb(rightWingOut);
	});

	pathFollower->addCommandMapping("rightWingIn", [&]() -> void {
		rightWingStateController->sb(rightWingIn);
	});

	pathFollower->addCommandMapping("awpOut", [&]() -> void {
		awpStateController->sb(awpOut);
	});

	pathFollower->addCommandMapping("awpIn", [&]() -> void {
		awpStateController->sb(awpIn);
	});

	pathFollower->addCommandMapping("hang", [&]() -> void {
		hangStateController->sb(hangOut);
	});

	pathFollower->addCommandMapping("wingsOut", [&]() -> void {
		leftWingStateController->sb(leftWingOut);
		rightWingStateController->sb(rightWingOut);
	});

	pathFollower->addCommandMapping("wingsIn", [&]() -> void {
		leftWingStateController->sb(leftWingIn);
		rightWingStateController->sb(rightWingIn);
	});

	pathFollower->addCommandMapping("catapult", [&]() -> void {
		catapultStateController->sb(catapultFire);
	});

	pathFollower->addCommandMapping("catapultStop", [&]() -> void {
		catapultStateController->ud();
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
	competitionController->ud();

	// Create a label
	lv_obj_t *disabledLabel = lv_label_create(lv_scr_act(), NULL);
	lv_obj_align(disabledLabel, NULL, LV_ALIGN_CENTER, 0, 0);
	lv_label_set_text(disabledLabel, "Robot Disabled.");
}

// !SECTION

// SECTION Competition Initialize

/**
 * Starts when connected to the field
 */
void competition_initialize() {
//	Log("Competition Initialize");
}

// !SECTION

// SECTION Auton
//#define ELIM
/**
 * Runs during the autonomous. NO user control
 */
void autonomous() {

//	Log("Auton Init");

#if AUTON == 0
	auton->setAuton(far6BallRushMid);
#elif AUTON == 1
	auton->setAuton(far5BallAWP);
#elif AUTON == 2
	auton->setAuton(safeCloseAWP);
#elif AUTON == 3
	auton->setAuton(closeRushMidElim);
#elif AUTON == 4
	auton->setAuton(closeRushMid);
#elif AUTON == 5
	auton->setAuton(skills);
#endif // !1

	competitionController->sb(auton);
}

// !SECTION

// SECTION Operator Control

/**
 * Runs during operator/teleop control
 */
void opcontrol() {

	// Causes the programming skills code to only run during skills
#if AUTON == 5
	robotMutex.take(TIMEOUT_MAX);
	auton->setAuton(skills);
	competitionController->sb(std::make_shared<Until>(auton, [=]() -> auto {
		return master->get_digital(Pronounce::E_CONTROLLER_DIGITAL_A);
	}));
	robotMutex.give();
	competitionController->wait();
#endif

	robotMutex.take(TIMEOUT_MAX);
	competitionController->sb(teleop);
	robotMutex.give();
}

// !SECTION