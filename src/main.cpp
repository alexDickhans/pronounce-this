#include "main.h"

// LVGL
std::shared_ptr<lv_obj_t> tabview;

// SECTION Auton
ASSET(close_mid_rush_json);
ASSET(mid_6_ball_1_json);
ASSET(mid_6_ball_2_json);
ASSET(mid_6_ball_awp_json);
SMOOTH_SPLINE_PATH_ASSET(safe_close_awp);
//SMOOTH_SPLINE_PATH_ASSET(skills_1);
//SMOOTH_SPLINE_PATH_ASSET(skills_2);
//SMOOTH_SPLINE_PATH_ASSET(skills_3);
//SMOOTH_SPLINE_PATH_ASSET(skills_4);
//SMOOTH_SPLINE_PATH_ASSET(skills_5);
//SMOOTH_SPLINE_PATH_ASSET(skills_6);
//SMOOTH_SPLINE_PATH_ASSET(skills_6_5);
//SMOOTH_SPLINE_PATH_ASSET(skills_7);
//SMOOTH_SPLINE_PATH_ASSET(skills_7_5);
//SMOOTH_SPLINE_PATH_ASSET(skills_8);
//SMOOTH_SPLINE_PATH_ASSET(skills_9);
//SMOOTH_SPLINE_PATH_ASSET(skills_10);
ASSET(close_mid_rush_elim_json);
ASSET(close_rush_mid_2_json);

void turnTo(Angle angle, QTime waitTimeMS) {
	auto angleRotation = std::make_shared<RotationController>("AngleTurn", drivetrain,
	                                                          [&]() -> Angle { return imuOrientation.getAngle(); },
	                                                          turningPid, angle);

	drivetrainStateController->sb(angleRotation);

	pros::Task::delay(waitTimeMS.Convert(millisecond));

	drivetrainStateController->ud();
	pros::Task::delay(10);
}

void move(QLength distance, ProfileConstraints profileConstraints, QCurvature curvature, QVelocity initialSpeed = 0.0,
          QVelocity endSpeed = 0.0) {
	drivetrainStateController->sb(
			std::make_shared<TankMotionProfiling>("moveDistance", drivetrain, profileConstraints, distance,
			                                      [&]() -> Angle { return imuOrientation.getAngle(); },
			                                      &distancePid, curvature, drivetrainFeedforward, initialSpeed,
			                                      endSpeed))->wait();
}

void move(QLength distance, ProfileConstraints profileConstraints, QCurvature curvature, Angle startAngle,
          QVelocity initialSpeed = 0.0, QVelocity endSpeed = 0.0) {

	drivetrainStateController->sb(
			std::make_shared<TankMotionProfiling>("moveDistance", drivetrain, profileConstraints, distance,
			                                      [&]() -> Angle { return imuOrientation.getAngle(); },
			                                      &distancePid, curvature, drivetrainFeedforward, startAngle,
			                                      &movingTurnPid, initialSpeed,
			                                      endSpeed))->wait();
}

void far5BallRushMid(void *args) {

	Log("5 ball start");

	imuOrientation.setRotation(80.7_deg);

	intakeExtensionStateController->sb(deploySequence);
	frontRightWingStateController->sb(std::make_shared<Wait>(frontRightWingOut, 200_ms));

	move(50_in, speedProfileConstraints, 0.0, 80.7_deg);

	intakeExtensionStateController->ud();
	intakeStateController->sb(intakeIntaking);

	pathFollower->setMotionProfile(PathPlanner::SmoothSplineProfile::build(mid_6_ball_1_json));
	drivetrainStateController->sb(pathFollower)->wait();

	turnTo(-2_deg, 550_ms);

	intakeStateController->sb(intakeIntaking);

	move(19_in, speedProfileConstraints, 0.0, 2_deg, 0.0, 0.0);
	move(-16_in, speedProfileConstraints, 0.0, 2_deg, 0.0, 0.0);

	intakeStateController->sb(intakeHold);
//	move(-4_in, speedProfileConstraints, 0.0, 2_deg);
	turnTo(170_deg, 700_ms);

	pathFollower->setMotionProfile(PathPlanner::SmoothSplineProfile::build(mid_6_ball_2_json));
	drivetrainStateController->sb(pathFollower)->wait();

	move(-12_in, speedProfileConstraints, 0.0, 110_deg);

	frontLeftWingStateController->ud();
	turnTo(120_deg, 300_ms);
	frontLeftWingStateController->sb(frontLeftWingOut);
	drivetrain.tankSteerVoltage(12000, 12000);
	pros::Task::delay(800);
	drivetrain.tankSteerVoltage(0.0, 0.0);
	frontLeftWingStateController->ud();
	move(-9_in, speedProfileConstraints, 0.0, 90_deg);
	turnTo(25_deg, 200_ms);
	intakeStateController->sb(intakeIntaking);
	move(48_in, speedProfileConstraints, 0.0, 25_deg);

	turnTo(150_deg, 550_ms);
	intakeExtensionStateController->sb(outtakeSequence);
	move(38_in, speedProfileConstraints, 0.0, 150_deg);
}

void far6BallRushMid(void *args) {
	Log("6 ball");

	far5BallRushMid(args);

	turnTo(3_deg, 550_ms);

	intakeStateController->sb(intakeIntaking);

	move(23_in, defaultProfileConstraints, 0.0, 3_deg);

	turnTo(180_deg, 550_ms);
	intakeExtensionStateController->ud();
	intakeStateController->sb(intakeEject);
	frontLeftWingStateController->sb(frontLeftWingOut);
	frontRightWingStateController->sb(frontRightWingOut);
	move(35_in, speedProfileConstraints, 0.0, 180_deg);
	move(-10_in, speedProfileConstraints, 0.0, 180_deg);
	turnTo(0_deg, 3_s);
}

void far5BallAWP(void *args) {
	far5BallRushMid(args);

	move(-5_in, speedProfileConstraints, 0.0, 0_deg);

	turnTo(-90_deg, 600_ms);

	pathFollower->setMotionProfile(PathPlanner::SmoothSplineProfile::build(mid_6_ball_awp_json));
	drivetrainStateController->sb(pathFollower)->wait();

	drivetrain.tankSteerVoltage(3000, 2000);
	pros::Task::delay(5000);
}
//
//void skills(void *args) {
//
//	threeWheelOdom.reset(Pose2D(0_in, 0_in, 135_deg));
//
//	pathFollower->setMotionProfile(skills_1);
//	drivetrainStateController->sb(pathFollower)->wait();
//
//	drivetrainStateController->sb(
//			std::make_shared<RotationController>("MatchloadRotationController", drivetrain, odometry, turningPid,
//			                                     21.1_deg,
//			                                     drivetrainMutex, -800.0));
//	auton->resetTriballs();
//	pros::Task::delay(500);
//
//	// Wait until the catapult triballs shot has increased to 44 triballs
//	while (auton->getTriballCount() < 44 && catapultStateController->getDuration() < 2.0_s) {
//		// Wait 0.01s (10 ms * (second / 1000ms) = 0.01s / 100Hz)
//		pros::Task::delay(10);
//	}
//
//	pros::Task::delay(200);
//
//	pathFollower->setMotionProfile(skills_2);
//	drivetrainStateController->sb(pathFollower)->wait();
//
//	frontRightWingStateController->sb(frontRightWingIn);
//	turnTo(180_deg, 300_ms);
//
//	pathFollower->setMotionProfile(skills_3);
//	drivetrainStateController->sb(pathFollower)->wait();
//
//	move(-15_in, speedProfileConstraints, 0.0, -70_deg);
////
////	drivetrainStateController(pathFollower.changePath(skills_4_json))->wait();
////
////	move(-15_in, speedProfileConstraints, 0.0, -75_deg);
//
//	pathFollower->setMotionProfile(skills_4);
//	drivetrainStateController->sb(pathFollower)->wait();
//	move(-5_in, speedProfileConstraints, 0.0, -75_deg);
//
//	turnTo(-160_deg, 200_ms);
//
//	pathFollower->setMotionProfile(skills_5);
//	drivetrainStateController->sb(pathFollower)->wait();
//
//	move(-5_in, speedProfileConstraints, 0.0);
//
//	move(8_in, speedProfileConstraints, 0.0, 0.0_deg);
//
//	pathFollower->setMotionProfile(skills_6);
//	drivetrainStateController->sb(pathFollower)->wait();
//
//	move(-3_in, speedProfileConstraints, 0.0);
//
//	move(8_in, speedProfileConstraints, 0.0, 0.0_deg);
//
//	pathFollower->setMotionProfile(skills_6_5);
//	drivetrainStateController->sb(pathFollower)->wait();
//
//	move(-3_in, speedProfileConstraints, 0.0);
//
//	move(8_in, speedProfileConstraints, 0.0, 0.0_deg);
//
//	pathFollower->setMotionProfile(skills_7);
//	drivetrainStateController->sb(pathFollower)->wait();
//
//	move(-3_in, speedProfileConstraints, 0.0);
//
//	move(8_in, speedProfileConstraints, 0.0, 0.0_deg);
//
//	pathFollower->setMotionProfile(skills_7_5);
//	drivetrainStateController->sb(pathFollower)->wait();
//
//	drivetrainStateController->sb(
//			std::make_shared<RotationController>("MatchloadRotationController", drivetrain, odometry, turningPid, 0_deg,
//			                                     drivetrainMutex));
//
//	QLength wallDistance = getDistanceSensorMedian(distanceSensor, 3) * 1_mm;
//
//	pathFollower->setMotionProfile(
//			PathPlanner::SmoothSplineProfile::build(
//					{PathPlanner::SmoothBezierSegment(PathPlanner::Point(
//							                                  wallDistance, 76_in),
//					                                  PathPlanner::Point(
//							                                  wallDistance.getValue() * 0.74,
//							                                  68_in),
//					                                  PathPlanner::Point(
//							                                  19_in, 55_in),
//					                                  PathPlanner::Point(
//							                                  20_in, 20_in), true, true,
//					                                  pushingProfileConstraints)}));
//
//	drivetrainStateController->sb(pathFollower)->wait();
//
//	pathFollower->setMotionProfile(skills_8);
//	drivetrainStateController->sb(pathFollower)->wait();
//
//	move(-15_in, speedProfileConstraints, 0.0, 70_deg);
//
//	pathFollower->setMotionProfile(skills_9);
//	drivetrainStateController->sb(pathFollower)->wait();
//
//	move(-15_in, speedProfileConstraints, 0.0, 80_deg);
//
//	pathFollower->setMotionProfile(skills_9);
//	drivetrainStateController->sb(pathFollower)->wait();
//
//	frontRightWingStateController->ud();
//
//	move(-15_in, speedProfileConstraints, 0.0, 80_deg);
//
//	pathFollower->setMotionProfile(skills_10);
//	drivetrainStateController->sb(pathFollower)->wait();
//
//	pros::Task::delay(500);
//
//	backLeftWingStateController->ud();
//}

void safeCloseAWP(void *args) {
	imuOrientation.setRotation(135_deg);

	intakeExtensionStateController->sb(deploySequence);

	move(10_in, defaultProfileConstraints, 0.0);

	intakeExtensionStateController->ud();
	intakeStateController->sb(intakeIntaking);
	Log("NextMove");
	pathFollower->setMotionProfile(safe_close_awp);
	drivetrainStateController->sb(pathFollower)->wait();

	Log("NextMove");
	pros::Task::delay(15000);
}

void closeRushMid(void *args) {
	imuOrientation.setRotation(-75.7_deg);

	frontLeftWingStateController->sb(std::make_shared<Wait>(frontLeftWingOut, 300_ms));

	intakeExtensionStateController->sb(deploySequence);

	move(41_in, speedProfileConstraints, 0.0, -75.7_deg);

	intakeExtensionStateController->ud();
	intakeStateController->sb(intakeIntaking);

	move(-13_in, speedProfileConstraints, 0.0, -75.7_deg);

	turnTo(104.3_deg, 600_ms);

	intakeStateController->sb(intakeEject);

	turnTo(104.3_deg, 200_ms);

	turnTo(-35.3_deg, 600_ms);

	pathFollower->setMotionProfile(PathPlanner::SmoothSplineProfile::build(close_mid_rush_json));
	drivetrainStateController->sb(pathFollower)->wait();

	move(-10_in, speedProfileConstraints, 0.0, 56_deg);

	intakeStateController->sb(intakeEject);

	pathFollower->setMotionProfile(PathPlanner::SmoothSplineProfile::build(close_rush_mid_2_json));
	drivetrainStateController->sb(pathFollower)->wait();
}

void closeRushMidElim(void *args) {
	closeRushMid(args);

	turnTo(-180_deg, 800_ms);

	intakeStateController->sb(intakeIntaking);

	pathFollower->setMotionProfile(PathPlanner::SmoothSplineProfile::build(close_mid_rush_elim_json));
	drivetrainStateController->sb(pathFollower)->wait();
}

void tuneTurnPid(void *args) {
	imuOrientation.setRotation(0.0_deg);
	for (int i = 0; i < 5; ++i) {
		turnTo(180_deg, 2_s);
		turnTo(0.0_deg, 2_s);
	}
}
// !SECTION

// SECTION INIT

[[noreturn]] void update() {

	Log("Init");

	competitionController->initialize();

	robotMutex.give();

	uint32_t startTime;
	uint32_t startTimeMicros;

	while (true) {
		// Create stuff for exact delay
		startTime = pros::millis();
		startTimeMicros = pros::micros();

		Log(string_format("Competition Status: %s",
		                  !pros::competition::is_connected() ? "Not Connected" : pros::competition::is_disabled()
		                                                                         ? "Disabled"
		                                                                         : pros::competition::is_autonomous()
		                                                                           ? "Autonomous" : "Driver"));

		robotMutex.lock();
		imuOrientation.update();
		competitionController->update();
		robotMutex.unlock();

		Log("Good");

		// Wait a maximum of 10 milliseconds
		pros::delay(std::min(10 - (pros::millis() - startTime), (long unsigned int) 10));

		Log(string_format("Frame time: %s", std::to_string(pros::micros() - startTimeMicros).c_str()));
	}
}

[[noreturn]] void updateDisplay() {

	Log("Init");

	lv_theme_t *th = lv_theme_default_init(lv_disp_get_default(),  /*Use the DPI, size, etc from this display*/
	                                       lv_color_hex(0xff7d26),
	                                       lv_color_hex(0x303236),   /*Primary and secondary palette*/
	                                       true,    /*Light or dark mode*/
	                                       &lv_font_montserrat_14); /*Small, normal, large fonts*/

	lv_disp_set_theme(lv_disp_get_default(), th); /*Assign the theme to the display*/

	// Odom
	std::shared_ptr<lv_obj_t> odomTab = std::shared_ptr<lv_obj_t>(lv_tabview_add_tab(tabview.get(), "Odom"));
	auto odomLabel = std::shared_ptr<lv_obj_t>(lv_label_create(odomTab.get()));

	std::shared_ptr<lv_obj_t> flywheelTab = std::shared_ptr<lv_obj_t>(lv_tabview_add_tab(tabview.get(), "PTO"));
	auto flywheelLabel = std::shared_ptr<lv_obj_t>(lv_label_create(flywheelTab.get()));

	// Drivetrain
	std::shared_ptr<lv_obj_t> drivetrainTab = std::shared_ptr<lv_obj_t>(
			lv_tabview_add_tab(tabview.get(), "Drivetrain"));
	std::shared_ptr<lv_obj_t> drivetrainTable = std::shared_ptr<lv_obj_t>(lv_table_create(drivetrainTab.get()));

	lv_table_set_row_cnt(drivetrainTable.get(), 4);
	lv_table_set_col_cnt(drivetrainTable.get(), 2);

	lv_table_set_col_width(drivetrainTable.get(), 0, 200);
	lv_table_set_col_width(drivetrainTable.get(), 1, 200);

	// Flywheels

	while (true) {
		Log("Start");
		// Odometry
		lv_label_set_text(odomLabel.get(), (std::to_string(imuOrientation.getAngle().Convert(degree)) + "\n" +
		                                    std::to_string(drivetrain.getDistanceSinceReset().Convert(inch))).c_str());

		auto leftDriveTemps = leftDriveMotors.get_temperature_all();
		auto rightDriveTemps = rightDriveMotors.get_temperature_all();

		// Drivetrain
		for (int i = 0; i < 4; i++) {
			lv_table_set_cell_value(drivetrainTable.get(), i, 0,
			                        (std::to_string(leftDriveTemps[i]) + " C").c_str());
		}

		for (int i = 0; i < 4; i++) {
			lv_table_set_cell_value(drivetrainTable.get(), i, 1,
			                        (std::to_string(rightDriveTemps[i]) + " C").c_str());
		}
		Log("End");

		pros::Task::delay(50);
	}
}

/**
 * Runs when the robot starts up
 */
void initialize() {

	Log("Initialize");

	lv_init();
	tabview = std::shared_ptr<lv_obj_t>(lv_tabview_create(lv_scr_act(), LV_DIR_TOP, 50));

	pros::Task display(updateDisplay, TASK_PRIORITY_MIN + 1, TASK_STACK_DEPTH_DEFAULT, "updateDisplay");

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
//		auton->setAuton(skills);
#endif // !1

	// Initialize functions
	initHardware();
	initIntake();
	initWings();
	initBehaviors();
	initCatapult();
	initDrivetrain();
	initAutonomousMappings();
	initWinch();

	pros::Task modeLogicTask(update, TASK_PRIORITY_MAX, TASK_STACK_DEPTH_DEFAULT * 2, "modeLogicUpdate");

	pros::Task::delay(10);
}

// !SECTION

// SECTION Disabled
/**
 * Runs while the robot is disabled i.e. before and after match, between auton
 * and teleop period
 */
void disabled() {
	Log("Init");
	competitionController->ud();

	// Create a label
	auto disabledLabel = std::shared_ptr<lv_obj_t>(lv_label_create(lv_scr_act()));
	lv_obj_align(disabledLabel.get(), LV_ALIGN_CENTER, 0, 0);
	lv_label_set_text(disabledLabel.get(), "Robot Disabled.");
}

// !SECTION

// SECTION Competition Initialize

/**
 * Starts when connected to the field
 */
void competition_initialize() {
	Log("Competition Initialize");
}

// !SECTION

// SECTION Auton
//#define ELIM
/**
 * Runs during the autonomous. NO user control
 */
void autonomous() {

	Log(string_format("Auton Init: %d", AUTON));
	competitionController->sb(auton);

	pros::Task::delay(60000);
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
//	auton->setAuton(skills);
	competitionController->sb(std::make_shared<Until>(auton, [=]() -> auto {
		return master.get_digital(Pronounce::E_CONTROLLER_DIGITAL_A);
	}));
	robotMutex.give();
	competitionController->wait();
#endif

	robotMutex.take(TIMEOUT_MAX);
	competitionController->sb(teleop);
	robotMutex.give();
}

// !SECTION