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

	pros::Motor bottomIntakeMotor(12, false);
	pros::Motor topIntakeMotor(20, true);

	MotorGroup topIntake;
	MotorGroup bottomIntake;

	Intake intakeIntaking("IntakeIntaking", &bottomIntake, &topIntake, 1.0, 1.0);
	Intake intakeStopped("IntakeStopped", &bottomIntake, &topIntake, 0.0, 0.0);
	Intake intakeEjecting("IntakeEjecting", &bottomIntake, &topIntake, -1.0, -1.0);
	Intake intakeDejam("IntakeDejam", &bottomIntake, &topIntake, -1.0, 1.0);

	StateController intakeStateController("IntakeStateController", &intakeIntaking);
	StateController intakeStateExtensionController("IntakeStateExtensionController", new Behavior());

	Wait intakeDejam1(&intakeDejam, 500);
	Wait intakeDejam2(&intakeIntaking, 500);

	Wait intakeRoller(&intakeEjecting, 2000);

	Sequence intakeDejamSequence("IntakeDejamSequence");

	void initIntake() {
		bottomIntake.addMotor(&bottomIntakeMotor);
		topIntake.addMotor(&topIntakeMotor);

		intakeDejamSequence.addState(&intakeStateController, &intakeDejam1);
		intakeDejamSequence.addState(&intakeStateController, &intakeDejam2);
	}
}