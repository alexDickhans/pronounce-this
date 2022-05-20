#include "main.h"

// Controllers
Pronounce::Controller master(pros::E_CONTROLLER_MASTER);
Pronounce::Controller partner(pros::E_CONTROLLER_PARTNER);

// Motors

// Drive Motors
pros::Motor frontRightMotor(3, pros::E_MOTOR_GEARSET_06, true);
pros::Motor backRightMotor(5, pros::E_MOTOR_GEARSET_06, false);
pros::Motor frontLeftMotor(6, pros::E_MOTOR_GEARSET_06, false);
pros::Motor backLeftMotor(8, pros::E_MOTOR_GEARSET_06, true);

pros::Motor intake(1, true);

// Inertial Measurement Unit
pros::Imu imu(19);

pros::Rotation leftEncoder(10);
pros::Rotation rightEncoder(9);
pros::Rotation backEncoder(9);

// Odom wheels
Pronounce::TrackingWheel leftOdomWheel(&leftEncoder);
Pronounce::TrackingWheel rightOdomWheel(&rightEncoder);
Pronounce::TrackingWheel backOdomWheel(&backEncoder);

// GPS sensor
pros::Gps gps(4, 0, 0, 90, 0.2, 0.2);
GpsOdometry gpsOdometry(&gps);

// ThreeWheelOdom odometry(&leftOdomWheel, &rightOdomWheel, &backOdomWheel, &imu);
ThreeWheelOdom odometry(&leftOdomWheel, &rightOdomWheel, &backOdomWheel, &imu);

Pronounce::TankPurePursuit purePursuit(&drivetrain, &odometry, new PID(0.6, 0, 2.0), 20);

MotorButton liftButton(&master, &lift, DIGITAL_L1, DIGITAL_L2, 200, 0, -200, 0, 0);
MotorButton intakeButton(&master, &intake, DIGITAL_R2, DIGITAL_Y, 200, 0, -200, 0, 0);

pros::Vision turretVision(18, VISION_ZERO_CENTER);

// Autonomous Selector
Pronounce::AutonSelector autonomousSelector;

// LVGL
lv_obj_t* tabview;

#define DRIFT_MIN 7.0

bool preDriverTasksDone = false;
bool disableIntake = true;

int driverMode = 0;

bool backButtonStatus = false;
bool backGrabberManual = false;
uint32_t lastChange = 0;

// SECTION Auton

void waitForDone(double distance, double timeout) {
	uint32_t startTime = pros::millis();

	// Wait for done
	while (!purePursuit.isDone(distance) && pros::millis() - startTime < timeout && purePursuit.isFollowing()) {
		pros::Task::delay(50);
	}
}

void waitForDone(double distance) {
	// Wait for done
	while (!purePursuit.isDone(distance)) {
		pros::Task::delay(20);
	}
}

void waitForDone() {
	waitForDone(0.5, 15000);
}

void waitForDoneOrientation() {
	pros::Task::delay(100);

	uint32_t startTime = pros::millis();

	pros::Task::delay(300);

	while (!purePursuit.isDoneOrientation(0.1) && pros::millis() - startTime < 5000) {
		pros::Task::delay(50);
	}

	pros::Task::delay(500);
}

void waitForDoneOrientation(double margin) {
	pros::Task::delay(100);

	uint32_t startTime = pros::millis();

	pros::Task::delay(300);

	while (!purePursuit.isDoneOrientation(margin) && pros::millis() - startTime < 3000) {
		pros::Task::delay(50);
	}

	pros::Task::delay(500);
}

void turn(double angle) {
	purePursuit.setFollowing(true);
	purePursuit.setOrientationControl(true);

	purePursuit.setTargetOrientation(angle);

	waitForDoneOrientation();

	purePursuit.setOrientationControl(false);
}

void turn(double angle, double sensitivity) {
	purePursuit.setFollowing(true);
	purePursuit.setOrientationControl(true);

	purePursuit.setTargetOrientation(angle);

	waitForDoneOrientation(sensitivity);

	purePursuit.setOrientationControl(false);
}

void grabFromHook() {
	purePursuit.setEnabled(false);
	purePursuit.setFollowing(false);

	pros::Task::delay(300);

	double angle = goalAngle();

	frontHookButton.setButtonStatus(POSITIVE);

	pros::Task::delay(200);

	drivetrain.skidSteerVelocity(-600, 0);

	pros::Task::delay(150);

	drivetrain.skidSteerVelocity(0, 0);

	frontHookButton.setButtonStatus(NEUTRAL);

	purePursuit.setEnabled(true);
	purePursuit.setFollowing(true);

	purePursuit.setSpeed(60);

	turn(odometry.getPosition()->getTheta() + angle);

	pros::Task::delay(200);

	purePursuit.setEnabled(false);
	purePursuit.setFollowing(false);

	pros::Task::delay(200);

	drivetrain.skidSteerVelocity(350, 0);

	pros::Task::delay(550);

	frontGrabberButton.setButtonStatus(POSITIVE);

	pros::Task::delay(50);

	drivetrain.skidSteerVelocity(0, 0);

	pros::Task::delay(400);
	liftButton.setAutonomousAuthority(200);

	purePursuit.setEnabled(true);
	purePursuit.setFollowing(true);
}

/**
 * @brief Runs during auton period before auton
 *
 */
int preAutonRun() {

	purePursuit.setEnabled(true);

	frontGrabberButton.setAutonomous(true);
	backGrabberButton.setAutonomous(true);
	liftButton.setAutonomous(true);
	liftButton.setAutonomousPosition(true);
	intakeButton.setAutonomous(true);
	brakesButton.setAutonomous(true);

	drivetrain.getLeftMotors().set_brake_mode(MOTOR_BRAKE_HOLD);
	drivetrain.getRightMotors().set_brake_mode(MOTOR_BRAKE_HOLD);

	return 0;
}

/**
 * @brief Test auton
 *
 * @return 0
 */
int testOrientationAuton() {

	printf("Test Orientation Auton\n");

	bool grabFront = true;
	bool grabBack = true;

	purePursuit.setCurrentPathIndex(zeroPathIndex);

	turn(M_PI);

	printf("Done with 180\n");

	turn(0);

	printf("Done with 0\n");

	pros::Task::delay(1000);

	return 0;
}

/**
 * @brief Test auton
 *
 * @return 0
 */
int tuneOdom() {

	printf("Test Auton\n");

	odometry.reset(new Position(0, 0, 0));

	purePursuit.setCurrentPathIndex(0);
	purePursuit.setSpeed(60);

	//purePursuit.setCurrentPathIndex(wiggleIndex);
	purePursuit.setCurrentPathIndex(testPathIndex);
	purePursuit.setFollowing(true);

	waitForDone();

	purePursuit.setFollowing(false);

	pros::Task::delay(500);

	return 0;
}

int autonTemplate() {
	odometry.reset(new Position(0, 0, 0));

	purePursuit.setSpeed(30);

	pros::Task::delay(500);

	return 0;
}

int postAuton() {
	purePursuit.setFollowing(false);
	purePursuit.setEnabled(false);

	frontGrabberButton.setAutonomous(false);
	backGrabberButton.setAutonomous(false);
	liftButton.setAutonomous(false);
	intakeButton.setAutonomous(false);
	brakesButton.setAutonomous(false);

	balance.setEnabled(false);

	drivetrain.getLeftMotors().set_brake_mode(MOTOR_BRAKE_COAST);
	drivetrain.getRightMotors().set_brake_mode(MOTOR_BRAKE_COAST);

	return 0;
}

// !SECTION

// SECTION INIT
/**
 * Render thread to update items on the controller
 */
void renderThread() {
	master.renderFunc();
}

void updateDrivetrain() {

	while (1) {
		uint32_t startTime = pros::millis();
		odometry.update();

		purePursuit.update();
		balance.update();

		// Has to be in increments of 10 because of the VEX sensor update time
		pros::Task::delay_until(&startTime, 20);
	}
}

void lvChartAddValue(lv_chart_series_t* chartSeries, int size, int value) {
	for (int i = 0; i < size - 1; i++) {
		chartSeries->points[i] = chartSeries->points[i + 1];
	}

	chartSeries->points[size - 1] = value;
}

void chartInit(lv_chart_series_t* chartSeries, int size) {
	for (int i = 0; i < size; i++) {
		chartSeries->points[i] = 0;
	}
}

void updateDisplay() {

	// Odom
	lv_obj_t* odomTab = lv_tabview_add_tab(tabview, "Odom");
	lv_obj_t* odomLabel = lv_label_create(odomTab, NULL);

	// PurePursuit
	lv_obj_t* purePursuitTab = lv_tabview_add_tab(tabview, "Pure pursuit");
	lv_obj_t* purePursuitLabel = lv_label_create(purePursuitTab, NULL);

	// Drivetrain
	lv_obj_t* drivetrainTab = lv_tabview_add_tab(tabview, "Drivetrain");
	lv_obj_t* drivetrainTable = lv_table_create(drivetrainTab, NULL);

	lv_table_set_row_cnt(drivetrainTable, 2);
	lv_table_set_col_cnt(drivetrainTable, 2);

	lv_table_set_col_width(drivetrainTable, 0, 200);
	lv_table_set_col_width(drivetrainTable, 1, 200);

	// Intake
	lv_obj_t* intakeTab = lv_tabview_add_tab(tabview, "Intake");
	lv_obj_t* intakeChart = lv_chart_create(intakeTab, NULL);
	lv_obj_set_size(intakeChart, 200, 140);
	lv_chart_set_type(intakeChart, LV_CHART_TYPE_LINE);

	lv_chart_series_t* intakeInputSpeed = lv_chart_add_series(intakeChart, LV_COLOR_BLUE);
	lv_chart_series_t* intakeOutputSpeed = lv_chart_add_series(intakeChart, LV_COLOR_RED);

	// Vision
	lv_obj_t* visionTab = lv_tabview_add_tab(tabview, "Vision");
	lv_obj_t* visionLabel = lv_label_create(visionTab, NULL);
	lv_obj_t* visionResultLabel = lv_label_create(visionTab, NULL);

	//chartInit(intakeInputSpeed, 50);
	//chartInit(intakeOutputSpeed, 50);
//
	//pros::Task::delay(100);

	while (1) {
		// Odometry
		lv_label_set_text(odomLabel, (odometry.getPosition()->to_string()
			+ "\nL: " + std::to_string(leftOdomWheel.getPosition()) +
			", R: " + std::to_string(rightOdomWheel.getPosition())).c_str());

		// Balance
		lv_label_set_text(balanceLabel, ("Angle: " + std::to_string(balance.getLinearController()->getLastInput()) + "\n").c_str());

		// Drivetrain
		lv_table_set_cell_value(drivetrainTable, 0, 0, (std::to_string(frontLeftMotor.get_temperature()) + " C").c_str());
		lv_table_set_cell_value(drivetrainTable, 2, 0, (std::to_string(backLeftMotor.get_temperature()) + " C").c_str());
		lv_table_set_cell_value(drivetrainTable, 0, 1, (std::to_string(frontRightMotor.get_temperature()) + " C").c_str());
		lv_table_set_cell_value(drivetrainTable, 2, 1, (std::to_string(backRightMotor.get_temperature()) + " C").c_str());

		// Pure pursuit
		lv_label_set_text(purePursuitLabel, ("Distance Remaining on path: " + std::to_string(purePursuit.getPointData().distanceFromEnd)).c_str());

		// Intake
		//addValue(intakeInputSpeed, 50, intake.get_target_velocity());
		//addValue(intakeOutputSpeed, 50, intake.get_actual_velocity());

		lv_label_set_text(visionLabel, ("X coordinate:" + std::to_string(clawVision.get_by_sig(0, 1).x_middle_coord) + "\nAngle: " + std::to_string(toDegrees(goalAngle())) + "\nError: " + std::to_string(toDegrees(goalAngle() + odometry.getPosition()->getTheta()))).c_str());

		//lv_chart_refresh(intakeChart);

		pros::Task::delay(100);
	}
}

void initDisplay() {
	pros::Task updateDisplayTask = pros::Task(updateDisplay, "Update dispay");
}

/**
 * Initialize all sensors
 */
void initSensors() {
	printf("Initializing sensors\n");

	imu.reset();

	// Wait until IMU is calibrated
	// while (imu.is_calibrating()) {
	// 	pros::delay(20);
	// }

	//	printf("IMU calibrated\n");

	printf("Sensors initialized\n");
}

void updateMotors() {

	while (1) {

		pros::Task::delay(20);
	}
}

/**
 * Initialize all motors
 */
void initMotors() {
	// Motor brake modes
	frontLeftMotor.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
	frontRightMotor.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
	backLeftMotor.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
	backRightMotor.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
	lift.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);

	frontGrabberButton.setSolenoid(&frontGrabber);
	frontGrabberButton.setSingleToggle(true);
	frontGrabberButton.setButtonStatus(POSITIVE);

	frontHookButton.setSolenoid(&frontHook);
	frontHookButton.setSingleToggle(true);

	backGrabberButton.setSolenoid(&backTilter);
	backGrabberButton.setSingleToggle(true);

	backGrabberButton2.setSolenoid(&backGrabber);
	backGrabberButton2.setSingleToggle(true);

	brakesButton.setSolenoid(&brakes);
	brakesButton.setSingleToggle(true);

	intakeButton.setSingleToggle(true);
	intakeButton.setDejam(true);
	intakeButton.setDejamAuthority(-200);
	intakeButton.setDejamSpeed(25);
	intakeButton.setDejamTime(100);

	liftButton.setMultiplier(1.4);

	pros::Task updateButtons(updateMotors, "Update buttons");
}

void initDrivetrain() {
	printf("Init drivetrain\n");

	// odometry.setUseImu(false);
	// Left/Right fraction
	// 1.072124756
	// Left 99.57
	// Right 100.57
	double turningFactor = (((100.35 / 100.0) - 1.0) / 2);
	double tuningFactor = 0.998791278;
	leftOdomWheel.setRadius(2.75 / 2);
	leftOdomWheel.setTuningFactor(tuningFactor * (1 - turningFactor));
	rightOdomWheel.setRadius(2.75 / 2);
	rightOdomWheel.setTuningFactor(tuningFactor * (1 + turningFactor));

	leftEncoder.set_reversed(true);
	rightEncoder.set_reversed(false);

	odometry.setLeftOffset(3.303827647);
	odometry.setRightOffset(3.303827647);
	odometry.setBackOffset(0);

	odometry.setMaxMovement(0);

	purePursuit.setOutputMultiplier(600.0 / 65.0);
	purePursuit.setNormalizeDistance(10);
	purePursuit.setSpeed(45);
	purePursuit.setLookahead(15);
	purePursuit.setStopDistance(0.5);
	purePursuit.setMaxAcceleration(400);

	pros::Task::delay(10);

	pros::Task purePursuitTask = pros::Task(updateDrivetrain, "Pure Pursuit");

	odometry.reset(new Position());

	printf("Drivetrain Init done\n");
}

/**
 * Initialize the controller
 */
void initController() {
	master.setDrivetrain(&drivetrain);
	master.setOdometry(&odometry);
	partner.setDrivetrain(&drivetrain);
	partner.setOdometry(&odometry);
	pros::Task renderTask(renderThread);
}

// Run selector as task
void runSelector() {
	pros::Task::delay(1000);
	autonomousSelector.choose();
}

/**
 * Initialize the Auton Selector
 */
void initSelector() {
	autonomousSelector.addAuton(Auton("Test Orientation", testOrientationAuton));
	autonomousSelector.addAuton(Auton("Test Balance", testBalanceAuton));
	autonomousSelector.setDefaultAuton(0);
	autonomousSelector.setPreAuton(Auton("Pre auton", preAutonRun));
	autonomousSelector.setPreAuton(Auton("Post auton", postAuton));

	pros::Task selectorTask(runSelector, "Auton Selector");
}

/**
 * Initialize the logger for data collection after the match
 */
void initLogger() {

	Logger::setDefaultLogger(
		std::make_shared<Logger>(
			TimeUtilFactory::createDefault().getTimer(),
			"/usd/log.txt",
			Logger::LogLevel::debug // Show everything
			)
	);
	Logger::getDefaultLogger()->debug<std::string>("LOGGER: Logger initialized");
}

void initVision() {
	clawVision.set_auto_white_balance(0);
	clawVision.set_exposure(33);
}

/**
 * Filter and apply the quadratic function.
 */
double filterAxis(pros::Controller controller, pros::controller_analog_e_t controllerAxis) {
	// Remove drift
	double controllerValue = controller.get_analog(controllerAxis);
	double controllerFilter = abs(controllerValue) < DRIFT_MIN ? 0.0 : controllerValue;

	// Apply quadratic function 
	// f(x) = controllerFilter / 127.0 ^ 3 * 127.0
	double quadraticFilter = pow(controllerFilter / 127.0, 3) * 620;

	// Return solution
	return quadraticFilter;
}

/**
 * Runs when the robot starts up
 */
void initialize() {

	printf("Initialize");

	lv_init();
	tabview = lv_tabview_create(lv_scr_act(), NULL);

	printf("LVGL Init");

	// Initialize functions
	initSensors();
	initMotors();
	autoPaths(&purePursuit);
	initDrivetrain();
	initController();
	initLogger();
	initDisplay();
	initVision();
	// initSelector();

	printf("Init done\n");
}

// !SECTION

// SECTION Disabled
/**
 * Runs while the robot is disabled i.e. before and after match, between auton
 * and teleop period
 */
void disabled() {

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
	// autonomousSelector.choose();

}

// !SECTION

// SECTION Auton

/**
 * Runs during the autonomous. NO user control
 */
void autonomous() {
	// This calls the user selection, all the functions prototypes are in 
	// autonRoutines.hpp and the implementation is autonRoutines.cpp
	// autonomousSelector.run();
	preAutonRun();
	postAuton();

	// autonomousSelector.run();
}

// !SECTION

// SECTION Operator Control

/**
 * Runs during operator/teleop control
 */
void opcontrol() {

	printf("OpControl");

	postAuton();
	//lv_obj_t* infoLabel = lv_label_create(lv_scr_act(), NULL);
	// lv_label_set_text(infoLabel, "");

	// Driver Control Loop
	while (true) {

		// Filter and calculate magnitudes
		int leftY = filterAxis(master, ANALOG_LEFT_Y);
		int leftX = filterAxis(master, ANALOG_LEFT_X);
		int rightX = filterAxis(master, ANALOG_RIGHT_X);
		
		pros::delay(10);
	}
}

// !SECTION
