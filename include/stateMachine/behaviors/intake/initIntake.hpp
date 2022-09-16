#pragma once

#include "api.h"
#include "intake.hpp"
#include "stateMachine/behavior.hpp"
#include "stateMachine/stateController.hpp"
#include "utils/motorGroup.hpp"
#include "stateMachine/wait.hpp"
#include "stateMachine/sequence.hpp"

// TODO: Clean up
// TODO: move declarations to another place
// TODO: Add comments

namespace Pronounce {

	pros::Motor bottomIntakeMotor(20, true);
	pros::Motor topIntakeMotor(12, false);

	MotorGroup topIntake;
	MotorGroup bottomIntake;

	Intake intakeIntaking(&bottomIntake, &topIntake, 1.0, 1.0);
	Intake intakeStopped(&bottomIntake, &topIntake, 0.0, 0.0);
	Intake intakeEjecting(&bottomIntake, &topIntake, -1.0, -1.0);
	Intake intakeDejam(&bottomIntake, &topIntake, -1.0, 1.0);

	StateController intakeStateController(&intakeIntaking);
	StateController intakeStateExtensionController(new Behavior());

	Wait intakeDejam1(&intakeDejam, 500);
	Wait intakeDejam2(&intakeIntaking, 500);

	Sequence intakeDejamSequence;

	void initIntake() {
		bottomIntake.addMotor(&bottomIntakeMotor);
		topIntake.addMotor(&topIntakeMotor);

		intakeDejamSequence.addState(&intakeStateController, &intakeDejam1);
		intakeDejamSequence.addState(&intakeStateController, &intakeDejam2);
	}
}