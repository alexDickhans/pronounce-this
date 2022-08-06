#pragma once

#include "api.h"
#include "intake.hpp"
#include "stateMachine/behavior.hpp"
#include "stateMachine/stateController.hpp"
#include "utils/motorGroup.hpp"

// TODO: Clean up
// TODO: move declarations to another place
// TODO: Add comments

namespace Pronounce {

	pros::Motor intake(4, false);
	pros::Motor intake2(12, true);
	pros::Motor intake3(13, false);

	MotorGroup intakes;	

	Intake intakeIntaking(&intakes, 1.0);
	Intake intakeStopped(&intakes, 0.0);
	Intake intakeEjecting(&intakes, -0.5);

	StateController intakeStateController(&intakeIntaking);

	void initIntake() {
		intakes.addMotor(&intake);
		intakes.addMotor(&intake2);
		intakes.addMotor(&intake3);
	}
}