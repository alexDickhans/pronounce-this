#include "main.h"

// LVGL
std::shared_ptr<lv_obj_t> tabview;

RobotStatus robotStatus;
ModeLogic modeLogic(&robotStatus);
TeleopModeLogic teleopModeLogic(new RobotJoystick(E_CONTROLLER_MASTER), new RobotJoystick(E_CONTROLLER_PARTNER));

pros::Mutex robotMutex;

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

	drivetrainStateController.setCurrentBehavior(RotationController("AngleTurn", drivetrain, odometry, turningPid, angle, drivetrainMutex).wait(waitTimeMS));

	drivetrainStateController.waitUntilDone()();
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

int tuneTurnPid() {

	threeWheelOdom.setPose(Pose2D(0_in, 0_in, 0_deg));

	// turnTo(180_deg, 1000);

	// move(50_in, defaultProfileConstraints, 0.0, 5_deg);

	// pros::Task::delay(20);

	turnTo(-90_deg, 1500_ms);

	turnTo(90_deg, 1500_ms);

	turnTo(270_deg, 1500_ms);

	turnTo(-90_deg, 1500_ms);

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

int testMove() {

	threeWheelOdom.reset(Pose2D(0_in, 0_in, 0_deg));

	drivetrainStateController.setCurrentBehavior(getMPInstance(
			CombinedPath({{30_in, 0.0}, {10_in, 0.0}}),
			defaultProfileConstraints,
			0.0));

	drivetrainStateController.waitUntilDone()();

	return 0;
}

int safeCloseAWP() {



	return 0;
}

int farAWP() {

	threeWheelOdom.reset(Pose2D(0_in, 0_in, -90_deg));

	drivetrainStateController.setCurrentBehavior(getMPInstance(
			CombinedPath({
						{63_in, 0.0},
						{24_in, 135_deg/24_in},
						{92_in, -90_deg/92_in},
						{24_in, 135_deg/24_in},
						{63_in, 0.0},
						{24_in, 135_deg/24_in},
						{92_in, -90_deg/92_in},
						{24_in, 135_deg/24_in},
						{63_in, 0.0},
						{24_in, 135_deg/24_in},
						{92_in, -90_deg/92_in},
						{24_in, 135_deg/24_in},
						{63_in, 0.0},
						{24_in, 135_deg/24_in},
						{92_in, -90_deg/92_in},
						{24_in, 135_deg/24_in},
						{63_in, 0.0},
						{24_in, 135_deg/24_in},
						{92_in, -90_deg/92_in},
						{24_in, 135_deg/24_in},
						{63_in, 0.0},
						{24_in, 135_deg/24_in},
						{92_in, -90_deg/92_in},
						{24_in, 135_deg/24_in}}),
			defaultProfileConstraints,
			-90.0_deg));

	drivetrainStateController.waitUntilDone()();

//	drivetrainStateController.setCurrentBehavior(getMPInstance(
//			CombinedPath({{-55_in, 0.0}}),
//			defaultProfileConstraints,
//			-90.0_deg));
//
//	drivetrainStateController.waitUntilDone()();

	return 0;
}

int testBezier() {
	threeWheelOdom.reset(Pose2D(0_in, 0_in, 180_deg));

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
					{
							{PathPlanner::BezierSegment(
									PathPlanner::Point(12_in, 12_in),
									PathPlanner::Point(12_in, 36_in),
									PathPlanner::Point(36_in, 12_in),
									PathPlanner::Point(36_in, 36_in),
									true
							),
							 nullptr}
							 ,
							 {PathPlanner::BezierSegment(PathPlanner::Point(36_in, 36_in), PathPlanner::Point(36_in, 60_in), PathPlanner::Point(12_in, 36_in), PathPlanner::Point(12_in, 60_in), true),
							  nullptr}
					},
					{
							{0.5, [] () -> void {
								wingsStateController.setCurrentBehavior(&wingsOut);
							}},
							{1.5, [] () -> void {
								wingsStateController.useDefaultBehavior();
							}}
					}));

	drivetrainStateController.waitUntilDone()();

	return 0;
}

int closeAWP() {

	threeWheelOdom.reset(Pose2D(130_in, 22_in, 150_deg));

	catapultMotors.set_zero_position(1.25 * 20.0/15.0);

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
					{
							{PathPlanner::BezierSegment(PathPlanner::Point(130_in, 22_in), PathPlanner::Point(123_in, 32_in), PathPlanner::Point(108_in, 50_in), PathPlanner::Point(108_in, 28_in), true),
									nullptr},
							{PathPlanner::BezierSegment(PathPlanner::Point(108_in, 30_in), PathPlanner::Point(108_in, 36_in), PathPlanner::Point(100_in, 38_in), PathPlanner::Point(82_in, 38_in), false),
									nullptr},
							{PathPlanner::BezierSegment(PathPlanner::Point(90_in, 44_in), PathPlanner::Point(98_in, 44_in), PathPlanner::Point(108_in, 39_in), PathPlanner::Point(108_in, 34_in), true),
									nullptr},
							{PathPlanner::BezierSegment(PathPlanner::Point(90_in, 36_in), PathPlanner::Point(90_in, 45_in), PathPlanner::Point(90_in, 45_in), PathPlanner::Point(90_in, 59_in), false),
									nullptr},
							{PathPlanner::BezierSegment(PathPlanner::Point(102_in, 70_in), PathPlanner::Point(104_in, 60_in), PathPlanner::Point(115_in, 54_in), PathPlanner::Point(115_in, 38_in), true),
							 nullptr},
							{PathPlanner::BezierSegment(PathPlanner::Point(110_in, 38_in), PathPlanner::Point(115_in, 22_in), PathPlanner::Point(127_in, 20_in), PathPlanner::Point(130_in, 45_in), true),
							 nullptr},
							{PathPlanner::BezierSegment(PathPlanner::Point(130_in, 45_in), PathPlanner::Point(132_in, 60_in), PathPlanner::Point(121_in, 72_in), PathPlanner::Point(121_in, 83_in), true),
							 nullptr}
					},
					{
							{0.01, [] () -> void {
								catapultStateController.setCurrentBehavior(catapultFire.wait(400_ms));
							}},
							{1.2, [] () -> void {
								intakeStateController.setCurrentBehavior(&intakeIntaking);
							}},
							{2.5, [] () -> void {
								intakeStateController.useDefaultBehavior();
							}},
							{3.5, [] () -> void {
								intakeStateController.setCurrentBehavior(&intakeEject);
							}},
							{5.6, [] () -> void {
								intakeStateController.useDefaultBehavior();
								wingsStateController.setCurrentBehavior(&wingsOut);
								catapultStateController.setCurrentBehavior(&catapultHang);
							}},
							{6.1, [] () -> void {
								wingsStateController.setCurrentBehavior(&wingsIn);
							}},
							{6.9, [] () -> void {
								wingsStateController.setCurrentBehavior(&wingsOut);
							}}
					}));

	drivetrainStateController.waitUntilDone()();

	drivetrain.tankSteerVoltage(-2000, -2000);

	pros::Task::delay(500);

	wingsStateController.setCurrentBehavior(&wingsOut);

	return 0;
}

int far6BallFullAWP() {
	threeWheelOdom.reset(Pose2D(0_in, 0_in, 180_deg));

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
					{
							{PathPlanner::BezierSegment(PathPlanner::Point(132_in, 74_in), PathPlanner::Point(132_in, 80_in), PathPlanner::Point(132_in, 74_in), PathPlanner::Point(132_in, 80_in), false),
									nullptr},
							{PathPlanner::BezierSegment(PathPlanner::Point(132_in, 80_in), PathPlanner::Point(132_in, 140_in), PathPlanner::Point(110_in, 128_in), PathPlanner::Point(90_in, 128_in), true),
									nullptr},
							{PathPlanner::BezierSegment(PathPlanner::Point(90_in, 128_in), PathPlanner::Point(95_in, 128_in), PathPlanner::Point(95_in, 128_in), PathPlanner::Point(100_in, 128_in), false),
									nullptr}
					},
					{
							{0.1, [] () -> void {
								intakeStateController.setCurrentBehavior(&intakeIntaking);
							}},
							{1.1, [] () -> void {
								intakeStateController.useDefaultBehavior();
							}},
							{1.6, [] () -> void {
								wingsStateController.setCurrentBehavior(&wingsOut);
							}},
							{1.9, [] () -> void {
								wingsStateController.useDefaultBehavior();
							}}
					}));

	drivetrainStateController.waitUntilDone()();

	turnTo(270_deg, 600_ms);

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
					{
							{PathPlanner::BezierSegment(PathPlanner::Point(100_in, 128_in), PathPlanner::Point(95_in, 128_in), PathPlanner::Point(95_in, 128_in), PathPlanner::Point(90_in, 128_in), false),
									nullptr},
							{PathPlanner::BezierSegment(PathPlanner::Point(90_in, 128_in), PathPlanner::Point(95_in, 128_in), PathPlanner::Point(95_in, 128_in), PathPlanner::Point(108_in, 128_in), false),
									nullptr}
					},
					{
							{0.3, [] () -> void {
								intakeStateController.setCurrentBehavior(&intakeEject);
							}}
					}));

	drivetrainStateController.waitUntilDone()();

	turnTo(200_deg, 500_ms);

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
					{
							{PathPlanner::BezierSegment(PathPlanner::Point(108_in, 128_in), PathPlanner::Point(100_in, 100_in), PathPlanner::Point(93_in, 100_in), PathPlanner::Point(93_in, 70_in), false),
									nullptr},
							{PathPlanner::BezierSegment(PathPlanner::Point(93_in, 70_in), PathPlanner::Point(93_in, 75_in), PathPlanner::Point(93_in, 85_in), PathPlanner::Point(93_in, 90_in), true),
									nullptr}
					},
					{
							{0.6, [] () -> void {
								intakeStateController.setCurrentBehavior(&intakeIntaking);
							}},
							{1.6, [] () -> void {
								intakeStateController.useDefaultBehavior();
							}}
					}));

	drivetrainStateController.waitUntilDone()();

	turnTo(370_deg, 500_ms);

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
					{
							{PathPlanner::BezierSegment(PathPlanner::Point(93_in, 90_in), PathPlanner::Point(93_in, 100_in), PathPlanner::Point(85_in, 100_in), PathPlanner::Point(85_in, 115_in), false),
									nullptr},
							{PathPlanner::BezierSegment(PathPlanner::Point(85_in, 115_in), PathPlanner::Point(85_in, 105_in), PathPlanner::Point(90_in, 105_in), PathPlanner::Point(95_in, 105_in), true),
									nullptr},
							{PathPlanner::BezierSegment(PathPlanner::Point(95_in, 105_in), PathPlanner::Point(85_in, 105_in), PathPlanner::Point(70_in, 100_in), PathPlanner::Point(70_in, 80_in), false),
									nullptr}
					},
					{
							{0.5, [] () -> void {
								intakeStateController.setCurrentBehavior(&intakeEject);
							}},
							{1.2, [] () -> void {
								intakeStateController.useDefaultBehavior();
							}}
					}));

	drivetrainStateController.waitUntilDone()();

	turnTo(360_deg, 700_ms);

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
					{
							{PathPlanner::BezierSegment(PathPlanner::Point(70_in, 80_in), PathPlanner::Point(70_in, 95_in), PathPlanner::Point(70_in, 95_in), PathPlanner::Point(70_in, 115_in), false),
									nullptr},
							{PathPlanner::BezierSegment(PathPlanner::Point(70_in, 115_in), PathPlanner::Point(70_in, 85_in), PathPlanner::Point(80_in, 70_in), PathPlanner::Point(115_in, 70_in), true),
									nullptr}
					},
					{
							{0.5, [] () -> void {
								intakeStateController.setCurrentBehavior(&intakeEject);
							}},
							{1.8, [] () -> void {
								intakeStateController.useDefaultBehavior();
								wingsStateController.setCurrentBehavior(&wingsOut);
							}}
					}));

	drivetrainStateController.waitUntilDone()();

	drivetrain.tankSteerVoltage(-3000, 3000);

	pros::Task::delay(2000);

	return 0;
}

int closeAWPBad() {
	threeWheelOdom.reset(Pose2D(0_in, 0_in, 135_deg));

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
					{
							{PathPlanner::BezierSegment(PathPlanner::Point(8_in, 24_in), PathPlanner::Point(16_in, 16_in), PathPlanner::Point(24_in, 18_in), PathPlanner::Point(48_in, 18_in), false),
									new SinusoidalVelocityProfile(0.0, {30_in/second, 180_in/second/second, 0.0})},
							{PathPlanner::BezierSegment(PathPlanner::Point(48_in, 18_in), PathPlanner::Point(32_in, 18_in), PathPlanner::Point(30_in, 30_in), PathPlanner::Point(20_in, 20_in), true),
									nullptr},
							{PathPlanner::BezierSegment(PathPlanner::Point(20_in, 20_in), PathPlanner::Point(30_in, 30_in), PathPlanner::Point(48_in, 20_in), PathPlanner::Point(48_in, 70_in), false),
									nullptr},
							{PathPlanner::BezierSegment(PathPlanner::Point(48_in, 70_in), PathPlanner::Point(48_in, 48_in), PathPlanner::Point(60_in, 58_in), PathPlanner::Point(57_in, 43_in), true),
									nullptr},
							{PathPlanner::BezierSegment(PathPlanner::Point(57_in, 43_in), PathPlanner::Point(57_in, 54_in), PathPlanner::Point(71_in, 50_in), PathPlanner::Point(68_in, 70_in), false),
									nullptr},
							{PathPlanner::BezierSegment(PathPlanner::Point(75_in, 66_in), PathPlanner::Point(67_in, 45_in), PathPlanner::Point(43_in, 84_in), PathPlanner::Point(18_in, 84_in), true),
									nullptr}
					},
					{
							{0.1, [] () -> void {
								wingsStateController.setCurrentBehavior(&wingsOut);
							}},
							{0.5, [] () -> void {
								wingsStateController.useDefaultBehavior();
							}},
							{0.5, [] () -> void {
								intakeStateController.setCurrentBehavior(&intakeEject);
							}},
							{1.2, [] () -> void {
								intakeStateController.useDefaultBehavior();
							}},
							{2.5, [] () -> void {
								intakeStateController.setCurrentBehavior(&intakeEject);
							}},
							{5.8, [] () -> void {
								intakeStateController.useDefaultBehavior();
								wingsStateController.setCurrentBehavior(&wingsOut);
							}}
					}));

	drivetrainStateController.waitUntilDone()();

	return 0;
}

int skills() {
	threeWheelOdom.reset(Pose2D(0_in, 0_in, 135_deg));

	drivetrainStateController.setCurrentBehavior(
			new PathPlanner::PathFollower(
					"SkillsPath1",
					defaultProfileConstraints,
					drivetrain,
					[ObjectPtr = &odometry] { return ObjectPtr->getAngle(); },
					movingTurnPid,
					distancePid,
					8000.0/64.0,
					65_in/second,
					{
							{PathPlanner::BezierSegment(PathPlanner::Point(12_in, 36_in), PathPlanner::Point(28_in, 28_in), PathPlanner::Point(24_in, 36_in), PathPlanner::Point(18_in, 8_in), false),
							 nullptr}
					},
					{

					}));

	drivetrainStateController.waitUntilDone()();

	drivetrainStateController.setCurrentBehavior(new RotationController("MatchloadRotationController", drivetrain, odometry, turningPid, 195_deg, drivetrainMutex, 800));

	catapultStateController.setCurrentBehavior(catapultSkills.wait(8.0_s));

	catapultStateController.waitUntilDone()();

	drivetrainStateController.setCurrentBehavior(
			new PathPlanner::PathFollower(
					"SkillsPath2",
					defaultProfileConstraints,
					drivetrain,
					[ObjectPtr = &odometry] { return ObjectPtr->getAngle(); },
					movingTurnPid,
					distancePid,
					8000.0/64.0,
					65_in/second,
					{
							{PathPlanner::BezierSegment(PathPlanner::Point(24_in, 18_in), PathPlanner::Point(30_in, 30_in), PathPlanner::Point(0_in, 30_in), PathPlanner::Point(18_in, 75_in), true),
							 nullptr},
							{PathPlanner::BezierSegment(PathPlanner::Point(18_in, 75_in), PathPlanner::Point(18_in, 110_in), PathPlanner::Point(15_in, 130_in), PathPlanner::Point(55_in, 120_in), true),
							 nullptr}
					},
					{
							{1.2, [] () -> void {
								wingsStateController.setCurrentBehavior(&wingsOut);
							}}
					}));

	drivetrainStateController.waitUntilDone()();
}

int postAuton() {
	pros::Task::delay(100);

	drivetrainStateController.setCurrentBehavior(&drivetrainStopped);

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
		lv_label_set_text(flywheelLabel.get(), ("Speed: " + std::to_string(intakeMotor.get_actual_velocity())).c_str());

		pros::Task::delay(50);
	}
}

void initDisplay() {
	pros::Task display(updateDisplay, TASK_PRIORITY_MIN);
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
	initDrivetrain();
	initIntake();
	initCatapult();
	initWings();
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
	closeAWP();
	#endif // !1

	postAuton();
}

// !SECTION

// SECTION Operator Control

/**
 * Runs during operator/teleop control
 */
void opcontrol() {
	postAuton();
	robotMutex.take();
	teleopController.setCurrentBehavior(&teleopModeLogic);
	robotMutex.give();
}

// !SECTION