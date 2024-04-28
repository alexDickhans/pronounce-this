#include "main.h"

// LVGL
std::shared_ptr<lv_obj_t> tabview;

// SECTION Auton
SMOOTH_SPLINE_PATH_ASSET(close_rush_mid_awp)
SMOOTH_SPLINE_PATH_ASSET(close_rush_mid_triball)
SMOOTH_SPLINE_PATH_ASSET(close_rush_mid_no_triball)
SMOOTH_SPLINE_PATH_ASSET(safe_close_awp)
SMOOTH_SPLINE_PATH_ASSET(far_6_1)
SMOOTH_SPLINE_PATH_ASSET(far_6_2)
SMOOTH_SPLINE_PATH_ASSET(far_6_3)
SMOOTH_SPLINE_PATH_ASSET(far_6_4)
SMOOTH_SPLINE_PATH_ASSET(far_6_5)
SMOOTH_SPLINE_PATH_ASSET(mid_6_ball_1)
SMOOTH_SPLINE_PATH_ASSET(mid_6_ball_2)
SMOOTH_SPLINE_PATH_ASSET(mid_6_ball_awp)
SMOOTH_SPLINE_PATH_ASSET(safe_6)
SMOOTH_SPLINE_PATH_ASSET(skills_1)
SMOOTH_SPLINE_PATH_ASSET(skills_2)
SMOOTH_SPLINE_PATH_ASSET(skills_3)
SMOOTH_SPLINE_PATH_ASSET(skills_5)
SMOOTH_SPLINE_PATH_ASSET(skills_6)
SMOOTH_SPLINE_PATH_ASSET(skills_6_5)
SMOOTH_SPLINE_PATH_ASSET(skills_7)
SMOOTH_SPLINE_PATH_ASSET(skills_7_5)
SMOOTH_SPLINE_PATH_ASSET(skills_8)
SMOOTH_SPLINE_PATH_ASSET(skills_9)

void turnTo(Angle angle, QTime waitTimeMS, RotationOptimizer rotationOptimizer = none, double idleSpeed = 0.0) {
	auto angleRotation = std::make_shared<RotationController>("AngleTurn", drivetrain,
	                                                          [&]() -> Angle { return imuOrientation.getAngle(); },
	                                                          turningPid, angle, idleSpeed, rotationOptimizer);

	drivetrainStateController->sb(angleRotation);

	pros::Task::delay(static_cast<uint32_t>(waitTimeMS.Convert(millisecond)));

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

void far6Ball(void* args) {
	imuOrientation.setRotation(-120.5_deg);

	intakeExtensionStateController->sb(deploySequence);

	move(55_in, speedProfileConstraints, 0.0, -120.5_deg);

	pros::Task::delay(200);

	intakeExtensionStateController->ud();

	pathFollower->setMotionProfile(far_6_1);
	drivetrainStateController->sb(pathFollower)->wait();

	turnTo(-180_deg, 0.4_s);
	intakeStateController->sb(intakeIntaking);
	move(5_in, speedProfileConstraints, 0.0, -180_deg);

	pathFollower->setMotionProfile(far_6_2);
	drivetrainStateController->sb(pathFollower)->wait();

	turnTo(-45_deg, 0.6_s, counterclockwise);

	leftWingStateController->sb(leftWingOut);

	intakeStateController->sb(intakeEject);

	pathFollower->setMotionProfile(far_6_3);
	drivetrainStateController->sb(pathFollower)->wait();
	move(-15_in, defaultProfileConstraints, 0.0, -430_deg);
	pathFollower->setMotionProfile(far_6_3);
	drivetrainStateController->sb(pathFollower)->wait();
}

void far6BallElim(void* args) {
	far6Ball(args);

	move(-15_in, defaultProfileConstraints, 0.0, -430_deg);
	pathFollower->setMotionProfile(far_6_4);
	drivetrainStateController->sb(pathFollower)->wait();
	move(25_in, speedProfileConstraints, 0.0, -135_deg);
}

void far5BallRushMid(void *args) {

	imuOrientation.setRotation(80.7_deg);

	intakeExtensionStateController->sb(deploySequence);
	rightWingStateController->sb(std::make_shared<Wait>(rightWingOut, 200_ms));

	move(50_in, speedProfileConstraints, 0.0, 80.7_deg);

	intakeExtensionStateController->ud();
	intakeStateController->sb(intakeIntaking);

	pathFollower->setMotionProfile(mid_6_ball_1);

	drivetrainStateController->sb(pathFollower)->wait();

	turnTo(-2_deg, 550_ms);

	intakeStateController->sb(intakeIntaking);

	move(19_in, speedProfileConstraints, 0.0, 2_deg, 0.0, 0.0);
	move(-16_in, speedProfileConstraints, 0.0, 2_deg, 0.0, 0.0);

	intakeStateController->sb(intakeHold);
//	move(-4_in, speedProfileConstraints, 0.0, 2_deg);
	turnTo(170_deg, 700_ms);

	pathFollower->setMotionProfile(mid_6_ball_2);

	drivetrainStateController->sb(pathFollower)->wait();

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

void safe6Ball(void* args) {
	imuOrientation.setRotation(180_deg);

	intakeStateController->sb(intakeEject);

	pros::Task::delay(200);

	intakeStateController->sb(intakeIntaking);

	move(10_in, defaultProfileConstraints, 0.0, 180_deg);

	pathFollower->setMotionProfile(far_6_2);
	drivetrainStateController->sb(pathFollower)->wait();

	turnTo(-45_deg, 0.6_s, counterclockwise);

	leftWingStateController->sb(leftWingOut);

	intakeStateController->sb(intakeEject);

	pathFollower->setMotionProfile(far_6_3);
	drivetrainStateController->sb(pathFollower)->wait();
	move(-15_in, defaultProfileConstraints, 0.0, -430_deg);

	pathFollower->setMotionProfile(far_6_3);
	drivetrainStateController->sb(pathFollower)->wait();
	move(-15_in, defaultProfileConstraints, 0.0, -430_deg);

	leftWingStateController->ud();

	turnTo(-165_deg, 0.6_s);

	intakeStateController->sb(intakeIntaking);

	move(55_in, defaultProfileConstraints, 0.0, -165_deg);

	move(-10_in, defaultProfileConstraints, 0.0, -165_deg);

	turnTo(-20_deg, 0.4_s, closest);

	intakeStateController->sb(intakeEject);

	turnTo(-20_deg, 0.4_s, closest);

	turnTo(-120_deg, 0.4_s);

	intakeStateController->sb(intakeIntaking);

	move(24_in, defaultProfileConstraints, 0.0, -120_deg);

	turnTo(0_deg, 0.6_s);

	intakeStateController->sb(intakeEject);
	rightWingStateController->sb(rightWingOut);

	move(40_in, speedProfileConstraints, 0.0, 0_deg);
	rightWingStateController->ud();

	turnTo(-80_deg, 0.3_s);

	pathFollower->setMotionProfile(safe_6);

	drivetrainStateController->sb(pathFollower)->wait();

	turnTo(180_deg, 15_s, closest);

}

void safeCloseAWP(void *args) {
	imuOrientation.setRotation(-135_deg);

	pathFollower->setMotionProfile(safe_close_awp);
	drivetrainStateController->sb(pathFollower)->wait();

	turnTo(0_deg, 15_s, closest);
}

void safeCloseAWPDelay(void *args) {
	imuOrientation.setRotation(-135_deg);

	pros::Task::delay(11000);

	pathFollower->setMotionProfile(safe_close_awp);
	drivetrainStateController->sb(pathFollower)->wait();
	turnTo(0_deg, 15_s, closest);

}

void closeRushMidAwp(void *args) {
	imuOrientation.setRotation(-75.7_deg);

	leftWingStateController->sb(std::make_shared<Wait>(leftWingOut, 300_ms));

	intakeExtensionStateController->sb(deploySequence);

	move(44_in, speedProfileConstraints, 0.0, -75.7_deg);

	pathFollower->setMotionProfile(close_rush_mid_awp);
	drivetrainStateController->sb(pathFollower)->wait();

	turnTo(0_deg, 15_s, closest);
}

void closeRushMidElim(void *args) {
	imuOrientation.setRotation(-75.7_deg);

	leftWingStateController->sb(std::make_shared<Wait>(leftWingOut, 300_ms));

	intakeExtensionStateController->sb(deploySequence);

	move(44_in, speedProfileConstraints, 0.0, -75.7_deg);

	move(-8_in, speedProfileConstraints, 0.0, -75.7_deg);

	if (true) {
		// Has triball in the intake
		pathFollower->setMotionProfile(close_rush_mid_triball);
		turnTo(50_deg, 0.5_s);
		drivetrainStateController->sb(pathFollower)->wait();
	} else {
		// Doesn't have a triball in the intake
		pathFollower->setMotionProfile(close_rush_mid_no_triball);
		turnTo(0_deg, 0.4_s);
		drivetrainStateController->sb(pathFollower)->wait();
	}

	move(-30_in, speedProfileConstraints, 0.0, 0_deg);
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

		Log("Loop done");

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

#if AUTON == 0
	auton->setAuton(far6BallElim);
#elif AUTON == 1
	auton->setAuton(safe6Ball);
#elif AUTON == 2
	auton->setAuton(far6BallRushMid);
#elif AUTON == 3
	auton->setAuton(safeCloseAWPDelay);
#elif AUTON == 4
	auton->setAuton(closeRushMidElim);
#elif AUTON == 5
	auton->setAuton(closeRushMidAwp);
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

	pros::Task display(updateDisplay, TASK_PRIORITY_MIN + 1, TASK_STACK_DEPTH_DEFAULT, "updateDisplay");
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

	while (1) {
		pros::Task::delay(50);
	}
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

	pros::Task::delay(80000);
}

// !SECTION

// SECTION Operator Control

/**
 * Runs during operator/teleop control
 */
void opcontrol() {

	// Causes the programming skills code to only run during skills
#if AUTON == 6
	robotMutex.lock();
	auton->setAuton(skills);
	competitionController->sb(std::make_shared<Until>(auton, [=]() -> auto {
		return master.get_digital(Pronounce::E_CONTROLLER_DIGITAL_A);
	}));
	robotMutex.unlock();
	competitionController->wait(80000);
#endif

	competitionController->sb(teleop);

	pros::Task::delay(120000);
}

// !SECTION
