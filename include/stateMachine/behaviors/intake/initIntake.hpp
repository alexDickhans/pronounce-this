#pragma once

#include "api.h"
#include "hardware/hardware.hpp"
#include "stateMachine/behavior.hpp"
#include "stateMachine/stateController.hpp"
#include "intake.hpp"

namespace Pronounce {
    auto intakeStopped = std::make_shared<Intake>("IntakeStopped", intakeMotors, 0.0);
	auto intakeIntaking = std::make_shared<Intake>("IntakeIntaking", intakeMotors, 1.0);
	auto intakeHold = std::make_shared<Intake>("IntakeHold", intakeMotors, 0.65);
	auto intakeEject = std::make_shared<Intake>("IntakeEject", intakeMotors, -1.0);

    auto intakeStateController = std::make_shared<StateController>("IntakeStateController", intakeStopped);
	auto intakeExtensionStateController = std::make_shared<StateController>("IntakeStateExtensionController", std::make_shared<Behavior>());

    auto intakeSequence = std::make_shared<Sequence>("IntakeSequence");
	auto outtakeSequence = std::make_shared<Sequence>("OuttakeSequence");
	auto deploySequence = std::make_shared<Sequence>("StartSequence");

    void initIntake() {
	    Log("Intake Init");

		outtakeSequence->addState(intakeStateController, std::make_shared<Wait>(intakeHold, 170_ms));
		outtakeSequence->addState(intakeStateController, std::make_shared<Wait>(intakeEject, 500_ms));

		deploySequence->addState(intakeStateController, std::make_shared<Wait>(intakeEject, 300_ms));
		deploySequence->addState(intakeStateController, intakeIntaking);

	    Log("Intake Init Done");
    }
} // namespace Pronounce
