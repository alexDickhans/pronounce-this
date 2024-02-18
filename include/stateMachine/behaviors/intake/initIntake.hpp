#pragma once

#include "api.h"
#include "hardware/hardware.hpp"
#include "stateMachine/behavior.hpp"
#include "stateMachine/stateController.hpp"
#include "intake.hpp"

namespace Pronounce {
    Intake intakeStopped("IntakeStopped", intakeMotors, 0.0, false);
	Intake intakeIntaking("IntakeIntaking", intakeMotors, 1.0, true);
	Intake intakeHold("IntakeHold", intakeMotors, 0.5, false);
	Intake intakeEject("IntakeEject", intakeMotors, -1.0, false);

    StateController intakeStateController("IntakeStateController", &intakeStopped);
	StateController intakeExtensionStateController("IntakeStateExtensionController", new Behavior());

    Sequence intakeSequence("IntakeSequence");
	Sequence outtakeSequence("OuttakeSequence");
	Sequence deploySequence("StartSequence");

    void initIntake() {
		intakeSequence.addState(&intakeStateController, intakeIntaking.until([=]() -> bool {return !master->get_digital(E_CONTROLLER_DIGITAL_R1);}));
		intakeSequence.addState(&intakeStateController, &intakeHold);

		outtakeSequence.addState(&intakeStateController, intakeIntaking.wait(170_ms));
		outtakeSequence.addState(&intakeStateController, intakeEject.wait(500_ms));

		deploySequence.addState(&intakeStateController, intakeEject.wait(300_ms));
		deploySequence.addState(&intakeStateController, &intakeIntaking);
    }
} // namespace Pronounce
