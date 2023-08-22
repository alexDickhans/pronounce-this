#pragma once

#include "api.h"
#include "hardware/hardware.hpp"
#include "stateMachine/behavior.hpp"
#include "stateMachine/stateController.hpp"
#include "intake.hpp"

namespace Pronounce {
    Intake intakeStopped("IntakeStopped", intakeMotors, 0.0, false);
    Intake intakeIntaking("IntakeIntaking", intakeMotors, 1.0, true);
    Intake intakeHold("IntakeHold", intakeMotors, 0.2, false);
    Intake intakeEject("IntakeEject", intakeMotors, -1.0, false);

    StateController intakeStateController("IntakeStateController", &intakeStopped);
    StateController intakeExtensionStateController("IntakeStateController", nullptr);

    Sequence intakeSequence("IntakeSequence");

    void initIntake() {
    }
} // namespace Pronounce
