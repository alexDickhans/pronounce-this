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

	PtoIntake ptoIntaking("PtoIntaking", leftPtoMotor, rightPtoMotor, intakeStopper, 1.0);
	PtoIntake ptoIntakeStopped("PtoIntakeStopped", leftPtoMotor, rightPtoMotor, intakeStopper, 0.0);
	PtoCatapult ptoCatapult("PtoCatapult", leftPtoMotor, rightPtoMotor, intakeStopper, catapultLimitSwitch, -1);

	StateController ptoStateController("PtoStateController", &ptoIntaking);
	StateController ptoStateExtensionController("PtoStateExtensionController", new Behavior());

	Wait ptoCatapultLaunch1(&ptoCatapult, 300_ms);

	Sequence ptoCatapultLaunch("PtoCatapultLaunch");
	Sequence ptoCatapultLaunchOff("PtoCatapultLaunch");

	void initPto() {
		ptoCatapultLaunch.addState(&ptoStateController, &ptoCatapultLaunch1);
		ptoCatapultLaunch.addState(&ptoStateController, &ptoCatapult);

		ptoCatapultLaunchOff.addState(&ptoStateController, &ptoCatapultLaunch1);
		ptoCatapultLaunchOff.addState(&ptoStateController, &ptoCatapult);
		ptoCatapultLaunchOff.addState(&ptoStateController, &ptoIntakeStopped);
	}
}