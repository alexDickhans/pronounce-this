#pragma once

#include "api.h"
#include "intake.hpp"
#include "catapult.hpp"
#include "stateMachine/behavior.hpp"
#include "stateMachine/stateController.hpp"
#include "stateMachine/wait.hpp"
#include "stateMachine/sequence.hpp"

// TODO: Add comments

namespace Pronounce {

	PtoIntake ptoIntaking("PtoIntaking", leftPtoMotor, rightPtoMotor, .8);
	PtoIntake ptoIntakeStopped("PtoIntakeStopped", leftPtoMotor, rightPtoMotor, 0);
	PtoCatapult ptoCatapult("PtoCatapult", leftPtoMotor, rightPtoMotor, catapultLimitSwitch, -1);

	StateController ptoStateController("PtoStateController", &ptoIntaking);
	StateController ptoStateExtensionController("PtoStateExtensionController", new Behavior());

	Wait ptoCatapultLaunch1(&ptoCatapult, 1000_ms);

	Sequence ptoCatapultLaunch("PtoCatapultLaunch");

	void initPto() {
		ptoCatapultLaunch.addState(&ptoStateController, &ptoCatapultLaunch1);
		ptoCatapultLaunch.addState(&ptoStateController, &ptoCatapult);
	}
}