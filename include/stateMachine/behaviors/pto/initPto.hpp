#pragma once

#include "api.h"
#include "intake.hpp"
#include "drive.hpp"
#include "catapult.hpp"
#include "stateMachine/behavior.hpp"
#include "stateMachine/stateController.hpp"
#include "stateMachine/wait.hpp"
#include "stateMachine/sequence.hpp"

// TODO: Add comments

namespace Pronounce {

	PtoIntake ptoIntaking("PtoIntaking", ptoPiston, false, leftPtoMotor, rightPtoMotor, 600);
	PtoCatapult ptoCatapult("PtoCatapult", ptoPiston, false, leftPtoMotor, rightPtoMotor, catapultLimitSwitch, -600);
	PtoDrive ptoDrive("PtoDrivetrain", ptoPiston, true, leftPtoMotor, leftDrive1, rightPtoMotor, rightDrive1);

	StateController ptoStateController("PtoStateController", &ptoIntaking);

	void initPto() {

	}
}