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

	move(56_in, speedProfileConstraints, 0.0, -120.5_deg);

	pros::Task::delay(200);

	intakeExtensionStateController->ud();

	pathFollower->setMotionProfile(far_6_1);
	drivetrainStateController->sb(pathFollower)->wait();

	turnTo(-180_deg, 0.4_s);
	intakeStateController->sb(intakeIntaking);
	move(7_in, speedProfileConstraints, 0.0, -180_deg);

	pathFollower->setMotionProfile(far_6_2);
	drivetrainStateController->sb(pathFollower)->wait();

	backRightWingStateController->ud();
	move(15_in, speedProfileConstraints, 0.0, -250_deg);
	turnTo(-45_deg, 0.6_s, counterclockwise);

	frontLeftWingStateController->sb(frontLeftWingOut);

	intakeStateController->sb(intakeEject);

	pathFollower->setMotionProfile(far_6_3);
	drivetrainStateController->sb(pathFollower)->wait();

	move(-15_in, defaultProfileConstraints, 0.0, -430_deg);

//	frontLeftWingStateController->ud();

}

void far6BallElim(void* args) {
	far6Ball(args);

	pathFollower->setMotionProfile(far_6_3);
	drivetrainStateController->sb(pathFollower)->wait();
	pathFollower->setMotionProfile(far_6_4);
	drivetrainStateController->sb(pathFollower)->wait();
}

void far6BallAWP(void* args) {
	far6Ball(args);
	pathFollower->setMotionProfile(far_6_4);
	drivetrainStateController->sb(pathFollower)->wait();
	pathFollower->setMotionProfile(far_6_5);
	drivetrainStateController->sb(pathFollower)->wait();
}

void skills(void *args) {

	imuOrientation.setRotation(135_deg);

	pathFollower->setMotionProfile(skills_1);
	drivetrainStateController->sb(pathFollower)->wait();

	drivetrainStateController->sb(
			std::make_shared<RotationController>("MatchloadRotationController", drivetrain, [&]() -> auto { return imuOrientation.getAngle(); }, turningPid,
			                                     21.0_deg, -800.0));
	auton->resetTriballs();
	pros::Task::delay(100);
	backLeftWingStateController->sb(backLeftWingOut);
	pros::Task::delay(900);

	// Wait until the catapult triballs shot has increased to 44 triballs
	while (auton->getTriballCount() < 44 && catapultStateController->getDuration() < 2.0_s) {
		// Wait 0.01s (10 ms * (second / 1000ms) = 0.01s / 100Hz)
		pros::Task::delay(10);
	}

	pros::Task::delay(200);

	pathFollower->setMotionProfile(skills_2);
	drivetrainStateController->sb(pathFollower)->wait();

	frontRightWingStateController->sb(frontRightWingIn);
	frontLeftWingStateController->sb(frontLeftWingIn);

	turnTo(180.0_deg, 600_ms, clockwise);

	catapultStateController->sb(catapultHoldHigh);

	pathFollower->setMotionProfile(skills_3);
	drivetrainStateController->sb(pathFollower)->wait();

	move(20_in, speedProfileConstraints, 0.0, -80_deg);

	turnTo(-70_deg, 1.0_s, closest, -12000);

	move(20_in, speedProfileConstraints, 0.0, -80_deg);
	backRightWingStateController->ud();
	backLeftWingStateController->ud();

	turnTo(-70_deg, 1.0_s, closest, -12000);

	turnTo(-170_deg, 0.4_s, closest);

	pathFollower->setMotionProfile(skills_5);
	drivetrainStateController->sb(pathFollower)->wait();

	move(-8_in, speedProfileConstraints, 0.0);

	move(12_in, speedProfileConstraints, 0.0, 0.0_deg);

	pathFollower->setMotionProfile(skills_6);
	drivetrainStateController->sb(pathFollower)->wait();

	pathFollower->setMotionProfile(skills_6_5);
	drivetrainStateController->sb(pathFollower)->wait();

	move(-8_in, speedProfileConstraints, 0.0);

	move(12_in, speedProfileConstraints, 0.0, 0.0_deg);

	pathFollower->setMotionProfile(skills_7);
	drivetrainStateController->sb(pathFollower)->wait();

	move(-13_in, speedProfileConstraints, 0.0);

	move(18_in, speedProfileConstraints, 0.0, 0.0_deg);

	pathFollower->setMotionProfile(skills_7_5);
	drivetrainStateController->sb(pathFollower)->wait();

	drivetrainStateController->sb(
			std::make_shared<RotationController>("MatchloadRotationController", drivetrain, [&]() -> auto { return imuOrientation.getAngle(); }, turningPid, 0_deg, 0.0, closest));

	QLength wallDistance = getDistanceSensorMedian(wallDistanceSensor, 3, (70_in).Convert(millimetre)) * 1_mm;

	turnTo(55_deg, 0.4_s, closest);

	pathFollower->setMotionProfile(
			PathPlanner::SmoothSplineProfile::build(
					{PathPlanner::BezierSegment(PathPlanner::Point(
							                                  wallDistance, 76_in),
					                                  PathPlanner::Point(
							                                  wallDistance - 14_in,
							                                  66_in),
					                                  PathPlanner::Point(
							                                  15_in, 52_in),
					                                  PathPlanner::Point(
							                                  16_in, 32_in), true, true,
					                                  pushingProfileConstraints)}));
	drivetrainStateController->sb(pathFollower)->wait();

	turnTo(-55_deg, 0.4_s, closest);

	pathFollower->setMotionProfile(skills_8);
	drivetrainStateController->sb(pathFollower)->wait();

	move(20_in, speedProfileConstraints, 0.0, 90_deg);

	turnTo(80_deg, 1.0_s, closest, -12000);

	winchStateController->sb(winchUp);

	move(20_in, speedProfileConstraints, 0.0, 90_deg);

	backLeftWingStateController->ud();

	turnTo(80_deg, 1.0_s, closest, -12000);

	pathFollower->setMotionProfile(skills_9);
	drivetrainStateController->sb(pathFollower)->wait();

	pros::Task::delay(3000);

	backLeftWingStateController->ud();
}

void safeCloseAWP(void *args) {
	imuOrientation.setRotation(-135_deg);

	pathFollower->setMotionProfile(safe_close_awp);
	drivetrainStateController->sb(pathFollower)->wait();

	pros::Task::delay(15000);
}

void safeCloseAWPDelay(void *args) {
	imuOrientation.setRotation(-135_deg);

	pros::Task::delay(11000);

	pathFollower->setMotionProfile(safe_close_awp);
	drivetrainStateController->sb(pathFollower)->wait();

}

void closeRushMidAwp(void *args) {
	imuOrientation.setRotation(-75.7_deg);

	frontLeftWingStateController->sb(std::make_shared<Wait>(frontLeftWingOut, 300_ms));

	intakeExtensionStateController->sb(deploySequence);

	move(41_in, speedProfileConstraints, 0.0, -75.7_deg);

	pathFollower->setMotionProfile(close_rush_mid_awp);
	drivetrainStateController->sb(pathFollower)->wait();

	pros::Task::delay(15000);
}

void closeRushMidElim(void *args) {
	imuOrientation.setRotation(-75.7_deg);

	frontLeftWingStateController->sb(std::make_shared<Wait>(frontLeftWingOut, 300_ms));

	intakeExtensionStateController->sb(deploySequence);

	move(41_in, speedProfileConstraints, 0.0, -75.7_deg);

	move(-10_in, speedProfileConstraints, 0.0, -75.7_deg);

	if (hopperDistanceSensor.get() * 1_mm < 160_mm) {
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

	pros::Task::delay(15000);
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

	pros::Task display(updateDisplay, TASK_PRIORITY_MIN + 1, TASK_STACK_DEPTH_DEFAULT, "updateDisplay");

#if AUTON == 0
	auton->setAuton(far6Ball);
#elif AUTON == 1
	auton->setAuton(far6BallAWP);
#elif AUTON == 2
	auton->setAuton(safeCloseAWP);
#elif AUTON == 3
	auton->setAuton(safeCloseAWPDelay);
#elif AUTON == 4
	auton->setAuton(closeRushMidElim);
#elif AUTON == 5
	auton->setAuton(closeRushMidAwp);
#elif AUTON == 6
	auton->setAuton(skills);
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
	robotMutex.lock();
	auton->setAuton(skills);
	competitionController->sb(std::make_shared<Until>(auton, [=]() -> auto {
		return master.get_digital(Pronounce::E_CONTROLLER_DIGITAL_A);
	}));
	robotMutex.unlock();
	competitionController->wait(60000);
#endif

	robotMutex.lock();
	competitionController->sb(teleop);
	robotMutex.unlock();
}

// !SECTION