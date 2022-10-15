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
	PtoDrive ptoDrive("PtoDrivetrain", ptoPiston, true, leftPtoMotor, leftDrive2, rightPtoMotor, rightDrive2);

	StateController ptoStateController("PtoStateController", &ptoIntaking);
	StateController ptoStateExtensionController("PtoStateExtensionController", new Behavior());

	Wait ptoCatapultLaunch1(&ptoCatapult, 500_ms);

	Sequence ptoCatapultLaunch("PtoCatapultLaunch");

	void initPto() {
		ptoCatapultLaunch.addState(&ptoStateController, &ptoCatapultLaunch1);
		ptoCatapultLaunch.addState(&ptoStateController, &ptoCatapult);
	}
}