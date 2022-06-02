#pragma once

#include "api.h"
#include "intake.hpp"
#include "stateMachine/behavior.hpp"
#include "stateMachine/stateController.hpp"

namespace Pronounce {

	pros::Motor intake(4, false);

	Intake intakeIntaking(&intake, 1.0);
	Intake intakeStopped(&intake, 0.0);
	Intake intakeEjecting(&intake, -0.5);

	StateController intakeStateController(&intakeIntaking);

	void initIntake() {
		
	}
}