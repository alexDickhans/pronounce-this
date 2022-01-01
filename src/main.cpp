#include "main.h"


// Controllers
Pronounce::Controller master(pros::E_CONTROLLER_MASTER);

// Motors

// Drive Motors
pros::Motor frontLeftMotor(1);
pros::Motor frontRightMotor(2, true);
pros::Motor backLeftMotor(9);
pros::Motor backRightMotor(10, true);

pros::Motor rightLift(3, true);
pros::Motor leftLift(4, false);

pros::Motor intake(11);

pros::Motor backGrabber(6);

pros::ADIDigitalOut frontGrabber(1, false);
pros::ADIDigitalIn frontGrabberBumperSwitch(2);

// Inertial Measurement Unit
pros::Imu imu(5);

pros::Rotation leftEncoder(12);
pros::Rotation rightEncoder(14);
pros::Rotation backEncoder(13);

// Odom wheels
Pronounce::TrackingWheel leftOdomWheel(&leftEncoder);
Pronounce::TrackingWheel rightOdomWheel(&rightEncoder);
Pronounce::TrackingWheel backOdomWheel(&backEncoder);

ThreeWheelOdom odometry(&leftOdomWheel, &rightOdomWheel, &backOdomWheel);

MecanumDrivetrain drivetrain(&frontLeftMotor, &frontRightMotor, &backLeftMotor, &backRightMotor, &imu, &odometry);

Pronounce::PurePursuit purePursuit(&drivetrain, 10);

MotorButton leftLiftButton(&master, &leftLift, DIGITAL_L1, DIGITAL_L2, 200, 0, -200, 0, 0);
MotorButton rightLiftButton(&master, &rightLift, DIGITAL_L1, DIGITAL_L2, 200, 0, -200, 0, 0);
MotorButton backGrabberButton(&master, &backGrabber, DIGITAL_R1, DIGITAL_R1, 200, 200, 200, 0, 450 * 3);
MotorButton intakeButton(&master, &intake, DIGITAL_R2, DIGITAL_R2, 200, 0, 0, 0, 0);

SolenoidButton frontGrabberButton(&master, DIGITAL_A, DIGITAL_B);

// Autonomous Selector
Pronounce::AutonSelector autonomousSelector;

bool relativeMovement = false;
bool driveOdomEnabled = true;

#define ROLL_AUTHORITY 1.0

#define DRIFT_MIN 7.0

bool preDriverTasksDone = false;

int driverMode = 0;

// Test path
int testPathIndex;

// Right steal right
int rightHomeToGoalNeutralIndex;
int rightNeutralToMidNeutralIndex;
int midNeutralToRightAllianceIndex;
int midNeutralToMidHomeZoneIndex;
int rightNeutralToRightHomeIndex;

// Right awp right
int farRightHomeZoneToRightAllianceIndex;
int rightAllianceToRightHomeZoneIndex;

// Left steal left
int leftAllianceToLeftNeutralIndex;
int leftNeutralToMidNeutralIndex;
int midNeutralToLeftHomeZoneIndex;

// Skills
int rightNeutralToFarPlatformIndex;
int farPlatformToNearPlatformIndex;
int nearPlatformViaLeftNeutralToFarPlatformIndex;
int nearPlatformToMidIndex;

/**
 * @brief Runs during auton period before auton
 *
 */
int preAutonRun() {

	while (imu.is_calibrating()) {
		pros::Task::delay(50);
	}

	purePursuit.setEnabled(true);

	frontGrabberButton.setAutonomous(true);
	backGrabberButton.setAutonomous(true);
	leftLiftButton.setAutonomous(true);
	rightLiftButton.setAutonomous(true);
	backGrabberButton.setAutonomousButton(true);
	intakeButton.setAutonomousButton(true);

	return 0;
}

/**
 * @brief Runs the Right Steal Right Auton
 *
 */
int rightStealRight() {
	odometry.reset(new Position(105.7, 16));

	backGrabberButton.setButtonStatus(ButtonStatus::POSITIVE);
	frontGrabberButton.setButtonStatus(ButtonStatus::NEUTRAL);

	purePursuit.setCurrentPathIndex(rightHomeToGoalNeutralIndex);
	purePursuit.setFollowing(true);

	// Wait until it is done
	while (!purePursuit.isDone(0.5)) {
		pros::Task::delay(50);
	}

	// Collect front goal
	frontGrabberButton.setButtonStatus(ButtonStatus::POSITIVE);
	pros::Task::delay(200);
	leftLiftButton.setAutonomousAuthority(360);
	rightLiftButton.setAutonomousAuthority(360);

	purePursuit.setCurrentPathIndex(rightNeutralToMidNeutralIndex);
	purePursuit.setFollowing(true);
	purePursuit.setTurnTarget(3.14);

	// Wait until it is done
	while (!purePursuit.isDone(0.5)) {
		pros::Task::delay(50);
	}

	backGrabberButton.setButtonStatus(ButtonStatus::NEUTRAL);
	pros::Task::delay(500);

	purePursuit.setCurrentPathIndex(midNeutralToMidHomeZoneIndex);
	purePursuit.setFollowing(true);
	purePursuit.setTurnTarget(-M_PI_2);

	// Wait until it is done
	while (!purePursuit.isDone(0.5)) {
		pros::Task::delay(50);
	}

	return 0;
}

int rightAwpRight() {
	odometry.reset(new Position(129.9, 16));

	purePursuit.setCurrentPathIndex(farRightHomeZoneToRightAllianceIndex);
	purePursuit.setFollowing(true);

	// Wait until it is done
	while (!purePursuit.isDone(0.5)) {
		pros::Task::delay(50);
	}

	// Collect front goal
	frontGrabberButton.setButtonStatus(ButtonStatus::POSITIVE);
	pros::Task::delay(200);
	leftLiftButton.setAutonomousAuthority(360);
	rightLiftButton.setAutonomousAuthority(360);

	purePursuit.setCurrentPathIndex(rightAllianceToRightHomeZoneIndex);
	purePursuit.setFollowing(true);

	// Wait until it is done
	while (!purePursuit.isDone(0.5)) {
		pros::Task::delay(50);
	}

	return 0;
}

int leftAwpLeft() {
	odometry.reset(new Position(20.5, 16));

	backGrabberButton.setButtonStatus(ButtonStatus::POSITIVE);
	frontGrabberButton.setButtonStatus(ButtonStatus::NEUTRAL);

	purePursuit.setCurrentPathIndex(leftAllianceToLeftNeutralIndex);
	purePursuit.setFollowing(true);
	purePursuit.setTurnTarget(0);

	// Wait until it is done
	while (!purePursuit.isDone(0.5)) {
		pros::Task::delay(50);
	}

	frontGrabberButton.setButtonStatus(ButtonStatus::POSITIVE);
	pros::Task::delay(200);
	leftLiftButton.setAutonomousAuthority(360);
	rightLiftButton.setAutonomousAuthority(360);

	purePursuit.setCurrentPathIndex(leftNeutralToMidNeutralIndex);
	purePursuit.setFollowing(true);
	purePursuit.setTurnTarget(3.14);

	// Wait until it is done
	while (!purePursuit.isDone(0.5)) {
		pros::Task::delay(50);
	}

	backGrabberButton.setButtonStatus(ButtonStatus::NEUTRAL);
	pros::Task::delay(500);

	purePursuit.setCurrentPathIndex(midNeutralToMidHomeZoneIndex);
	purePursuit.setFollowing(true);
	purePursuit.setTurnTarget(M_PI_2);

	// Wait until it is done
	while (!purePursuit.isDone(0.5)) {
		pros::Task::delay(50);
	}

	return 0;
}

int skills() {
	odometry.reset(new Position(105.7, 16));

	backGrabberButton.setButtonStatus(ButtonStatus::POSITIVE);
	frontGrabberButton.setButtonStatus(ButtonStatus::NEUTRAL);

	purePursuit.setCurrentPathIndex(rightHomeToGoalNeutralIndex);
	purePursuit.setFollowing(true);
	purePursuit.setTurnTarget(0);

	// Wait until it is done
	while (!purePursuit.isDone(0.5)) {
		pros::Task::delay(50);
	}

	// Collect front goal
	frontGrabberButton.setButtonStatus(ButtonStatus::POSITIVE);

	purePursuit.setCurrentPathIndex(rightNeutralToFarPlatformIndex);
	purePursuit.setFollowing(true);
	purePursuit.setTurnTarget(0);

	leftLiftButton.setAutonomousAuthority(1500);
	rightLiftButton.setAutonomousAuthority(1500);

	// Wait until it is done
	while (!purePursuit.isDone(0.5)) {
		pros::Task::delay(50);
	}

	frontGrabberButton.setButtonStatus(ButtonStatus::NEUTRAL);

	leftLiftButton.setAutonomousAuthority(0);
	rightLiftButton.setAutonomousAuthority(0);

	purePursuit.setCurrentPathIndex(farPlatformToNearPlatformIndex);
	purePursuit.setFollowing(true);
	purePursuit.setTurnTarget(M_PI);

	// Wait until gets to goal
	while (odometry.getPosition()->getY() > 80) {
		pros::Task::delay(50);
	}

	// Collect front goal
	frontGrabberButton.setButtonStatus(ButtonStatus::POSITIVE);

	leftLiftButton.setAutonomousAuthority(1500);
	rightLiftButton.setAutonomousAuthority(1500);

	// Wait until done
	while (!purePursuit.isDone(0.5)) {
		pros::Task::delay(50);
	}

	frontGrabberButton.setButtonStatus(ButtonStatus::NEUTRAL);

	leftLiftButton.setAutonomousAuthority(0);
	rightLiftButton.setAutonomousAuthority(0);

	purePursuit.setCurrentPathIndex(nearPlatformViaLeftNeutralToFarPlatformIndex);
	purePursuit.setFollowing(true);
	purePursuit.setTurnTarget(0);

	// Wait until gets to goal
	while (odometry.getPosition()->getY() < 61) {
		pros::Task::delay(50);
	}

	// Collect front goal
	frontGrabberButton.setButtonStatus(ButtonStatus::POSITIVE);

	leftLiftButton.setAutonomousAuthority(1500);
	rightLiftButton.setAutonomousAuthority(1500);

	// Wait until done
	while (!purePursuit.isDone(0.5)) {
		pros::Task::delay(50);
	}

	frontGrabberButton.setButtonStatus(ButtonStatus::NEUTRAL);

	leftLiftButton.setAutonomousAuthority(0);
	rightLiftButton.setAutonomousAuthority(0);

	purePursuit.setCurrentPathIndex(nearPlatformToMidIndex);
	purePursuit.setFollowing(true);

	// Wait until it is done
	while (!purePursuit.isDone(0.5)) {
		pros::Task::delay(50);
	}

	return 0;
}

/**
 * @brief Test auton
 *
 * @return 0
 */
int testAuton() {

	printf("Test Auton\n");

	odometry.reset(new Position());

	purePursuit.setCurrentPathIndex(testPathIndex);
	purePursuit.setFollowing(true);

	pros::Task::delay(10000);

	return 0;
}

int postAuton() {
	purePursuit.setFollowing(false);
	purePursuit.setEnabled(false);
	frontGrabberButton.setAutonomous(false);
	backGrabberButton.setAutonomous(false);
	leftLiftButton.setAutonomous(false);
	rightLiftButton.setAutonomous(false);
	intakeButton.setAutonomous(false);

	return 0;
}


/**
 * Render thread to update items on the controller
 */
void renderThread() {
	master.renderFunc();
}

void updateDrivetrain() {
	lv_obj_t* infoLabel = lv_label_create(lv_scr_act(), NULL);
	lv_label_set_text(infoLabel, "drivetrain");
	while (1) {
		uint32_t startTime = pros::millis();
		odometry.update();
		purePursuit.update();
		lv_label_set_text(infoLabel, odometry.getPosition()->to_string().c_str());
		pros::Task::delay_until(&startTime, 15);
	}
}

/**
 * Initialize all sensors
 */
void initSensors() {

	imu.reset();

	// Wait until IMU is calibrated
	// while (imu.is_calibrating()) {
	// 	pros::delay(20);
	// }
}

void updateMotors() {
	while (1) {
		frontGrabberButton.update();
		backGrabberButton.update();
		leftLiftButton.update();
		rightLiftButton.update();
		intakeButton.update();

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
	backGrabber.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);
	leftLift.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);
	rightLift.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);

	backGrabberButton.setSingleToggle(true);
	backGrabberButton.setGoToImmediately(true);

	frontGrabberButton.setSolenoid(&frontGrabber);
	frontGrabberButton.setSingleToggle(true);

	intakeButton.setSingleToggle(true);

	pros::Task updateButtons(updateMotors, "Update buttons");
}

void initDrivetrain() {
	printf("Init drivetrain");

	// odometry.setUseImu(true);

	leftOdomWheel.setRadius(3.25/2);
	leftOdomWheel.setTuningFactor(1);
	rightOdomWheel.setRadius(3.25/2);
	rightOdomWheel.setTuningFactor(1);
	backOdomWheel.setRadius(3.25/2);
	backOdomWheel.setTuningFactor(1);

	leftEncoder.set_reversed(true);
	rightEncoder.set_reversed(false);
	backEncoder.set_reversed(false);

	odometry.setLeftOffset(3.25);
	odometry.setRightOffset(3.25);
	odometry.setBackOffset(2);

	purePursuit.setNormalizeDistance(10);

	purePursuit.setOdometry(&odometry);

	pros::Task purePursuitTask = pros::Task(updateDrivetrain, "Pure Pursuit");

	// delay to let time for settling
	pros::Task::delay(200);

	odometry.reset(new Position());

	printf("Init done");
}

/**
 * Initialize the controller
 */
void initController() {
	master.setDrivetrain(&drivetrain);
	pros::Task renderTask(renderThread);
}

// Run selector as task
void runSelector() {
	autonomousSelector.choose();
}

/**
 * Initialize the Auton Selector
 */
void initSelector() {
	autonomousSelector.addAuton(Auton("Right steal right", rightStealRight));
	autonomousSelector.addAuton(Auton("Test", testAuton));
	autonomousSelector.setDefaultAuton(Auton("Right steal right", rightStealRight));
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

void autoPaths() {
	// Default pure pursuit profile
	PurePursuitProfile defaultProfile(new PID(20, 0.0, 2.0), new PID(60.0, 0.0, 5.0), 10.0);
	purePursuit.getPurePursuitProfileManager().setDefaultProfile(defaultProfile);

	// Test path
	Path testPath = Path();

	testPath.addPoint(0, 0);
	testPath.addPoint(24, 0);
	testPath.addPoint(24, 24);
	testPath.addPoint(24, 48);
	testPath.addPoint(-24, 48);
	testPath.addPoint(0, 24);

	testPathIndex = purePursuit.addPath(testPath);

	Path rightNeutralToRightHomeZone;

	rightNeutralToRightHomeZone.addPoint(105.7, 60);
	rightNeutralToRightHomeZone.addPoint(105.7, 16);

	rightNeutralToRightHomeIndex = purePursuit.addPath(rightNeutralToRightHomeZone);

	// Right Steal Right
	Path rightHomeToGoalNeutral;

	rightHomeToGoalNeutral.addPoint(105.7, 16);
	rightHomeToGoalNeutral.addPoint(105.7, 61);

	rightHomeToGoalNeutralIndex = purePursuit.addPath(rightHomeToGoalNeutral);

	Path rightNeutralToMidNeutral;

	rightNeutralToMidNeutral.addPoint(105.7, 62);
	rightNeutralToMidNeutral.addPoint(75.3, 40);
	rightNeutralToMidNeutral.addPoint(60.3, 65);

	rightNeutralToMidNeutralIndex = purePursuit.addPath(rightNeutralToMidNeutral);

	Path midNeutralToRightAlliance;

	midNeutralToRightAlliance.addPoint(70.3, 65);
	midNeutralToRightAlliance.addPoint(120.1, 28);

	midNeutralToRightAllianceIndex = purePursuit.addPath(midNeutralToRightAlliance);

	Path midNeutralToMidHomeZone;

	midNeutralToMidHomeZone.addPoint(70.3, 70.3);
	midNeutralToMidHomeZone.addPoint(70.3, 36);

	midNeutralToMidHomeZoneIndex = purePursuit.addPath(midNeutralToMidHomeZone);

	Path farRightHomeZoneToRightAlliance;

	farRightHomeZoneToRightAlliance.addPoint(127.9, 16);
	farRightHomeZoneToRightAlliance.addPoint(127.9, 24);

	farRightHomeZoneToRightAllianceIndex = purePursuit.addPath(farRightHomeZoneToRightAlliance);

	Path rightAllianceToRightHomeZone;

	rightAllianceToRightHomeZone.addPoint(127.9, 24);
	rightAllianceToRightHomeZone.addPoint(105.7, 16);

	rightAllianceToRightHomeZoneIndex = purePursuit.addPath(rightAllianceToRightHomeZone);

	Path leftAllianceToLeftNeutral;

	leftAllianceToLeftNeutral.addPoint(29, 11.4);
	leftAllianceToLeftNeutral.addPoint(32, 67);

	leftAllianceToLeftNeutralIndex = purePursuit.addPath(leftAllianceToLeftNeutral);

	Path leftNeutralToMidNeutral;

	leftNeutralToMidNeutral.addPoint(32, 67);
	leftNeutralToMidNeutral.addPoint(65.3, 40);
	leftNeutralToMidNeutral.addPoint(70.3, 65);

	leftNeutralToMidNeutralIndex = purePursuit.addPath(leftNeutralToMidNeutral);

	// mid neutral to mid home zone

	Path rightNeutralToFarPlatform;

	rightNeutralToFarPlatform.addPoint(105.7, 61);
	rightNeutralToFarPlatform.addPoint(75, 76.5);
	rightNeutralToFarPlatform.addPoint(75, 100);
	rightNeutralToFarPlatform.addPoint(60.3, 115);

	rightNeutralToFarPlatformIndex = purePursuit.addPath(rightNeutralToFarPlatform);

	Path farPlatformToNearPlatform;

	farPlatformToNearPlatform.addPoint(70.3, 107);
	farPlatformToNearPlatform.addPoint(60, 70.3);
	farPlatformToNearPlatform.addPoint(58.6, 64.1);
	farPlatformToNearPlatform.addPoint(58.6, 45);
	farPlatformToNearPlatform.addPoint(70.3, 30.7);

	farPlatformToNearPlatformIndex = purePursuit.addPath(farPlatformToNearPlatform);

	Path nearPlatformViaLeftNeutralToFarPlatform;

	nearPlatformViaLeftNeutralToFarPlatform.addPoint(70.3, 30.7);
	nearPlatformViaLeftNeutralToFarPlatform.addPoint(35, 61);
	nearPlatformViaLeftNeutralToFarPlatform.addPoint(70.3, 115);

	nearPlatformViaLeftNeutralToFarPlatformIndex = purePursuit.addPath(nearPlatformViaLeftNeutralToFarPlatform);

	Path nearPlatformToMid;

	nearPlatformToMid.addPoint(70.3, 115);
	nearPlatformToMid.addPoint(70.3, 70.3);

	nearPlatformToMidIndex = purePursuit.addPath(nearPlatformToMid);

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
	double quadraticFilter = pow(controllerFilter / 127.0, 3) * 200;

	// Return solution
	return quadraticFilter;
}

/**
 * Runs when the robot starts up
 */
void initialize() {

	printf("Initialize");

	lv_init();

	printf("LVGL Init");

	// Initialize functions
	initSensors();
	initMotors();
	initDrivetrain();
	autoPaths();
	initController();
	initLogger();
	// initSelector();
}

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

/**
 * Starts when connected to the field
 */
void competition_initialize() {
	// autonomousSelector.choose();

}

/**
 * Runs during the autonomous. NO user control
 */
void autonomous() {
	// This calls the user selection, all the functions prototypes are in 
	// autonRoutines.hpp and the implementation is autonRoutines.cp
	// autonomousSelector.run();
	preAutonRun();
	leftAwpLeft();
	postAuton();
}


/**
 * Runs during operator/teleop control
 */
void opcontrol() {

	printf("OpControl");
	lv_obj_clean(lv_scr_act());

	postAuton();

	//lv_obj_t* infoLabel = lv_label_create(lv_scr_act(), NULL);
	// lv_label_set_text(infoLabel, "");

	const int runningAverageLength = 1;
	RunningAverage<runningAverageLength> leftXAvg;
	RunningAverage<runningAverageLength> leftYAvg;
	RunningAverage<runningAverageLength> rightXAvg;

	// Driver Control Loop
	while (true) {

		if (driverMode > 0) {
			// Filter and calculate magnitudes
			int leftY = filterAxis(master, ANALOG_LEFT_Y);
			int leftX = filterAxis(master, ANALOG_LEFT_X);
			int rightX = filterAxis(master, ANALOG_RIGHT_X);

			leftXAvg.add(leftX);
			leftYAvg.add(leftY);
			rightXAvg.add(rightX);

			leftX = leftXAvg.getAverage();
			leftY = leftYAvg.getAverage();
			rightX = rightXAvg.getAverage();

			Vector driveVector = Vector(new Pronounce::Point(leftX, leftY));
			if (driverMode == 1) {
				driveVector.setAngle(driveVector.getAngle());
			}
			else {
				driveVector.setAngle(driveVector.getAngle() + toRadians(imu.get_rotation()));
			}

			// Send variables to motors
			drivetrain.setDriveVectorVelocity(driveVector, rightX);
		}
		else {
			int leftX = filterAxis(master, ANALOG_LEFT_X);
			int leftY = filterAxis(master, ANALOG_LEFT_Y);
			int rightY = filterAxis(master, ANALOG_RIGHT_Y);

			drivetrain.setDriveVectorVelocity(Vector(new Pronounce::Point(leftX, (leftY + rightY) / 2)), leftY - rightY);
		}

		if (frontGrabberBumperSwitch.get_new_press()) {
			frontGrabberButton.setButtonStatus(Pronounce::ButtonStatus::POSITIVE);
		}

		if (master.get_digital_new_press(DIGITAL_X)) {
			odometry.reset(new Position());
		}

		if (master.get_digital_new_press(DIGITAL_UP)) {
			driverMode = 0;
		}
		else if (master.get_digital_new_press(DIGITAL_DOWN)) {
			driverMode = 1;
		}
		else if (master.get_digital_new_press(DIGITAL_LEFT)) {
			driverMode = 2;
		}

		// Prevent wasted resources
		pros::delay(10);
	}
}
