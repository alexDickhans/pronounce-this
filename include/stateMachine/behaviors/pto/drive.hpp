#pragma once

#include "api.h"
#include "stateMachine/behavior.hpp"

namespace Pronounce {
	class PtoDrive : public Behavior {
	private:
		pros::ADIDigitalOut& ptoPiston;
		pros::Motor& leftPtoMotor;
		pros::Motor& leftDriveMotor;
		pros::Motor& rightPtoMotor;
		pros::Motor& rightDriveMotor;
		bool pistonState;
	public:
		PtoDrive(pros::ADIDigitalOut& ptoPiston, bool pistonState, pros::Motor& leftPtoMotor, pros::Motor& leftDriveMotor, pros::Motor& rightPtoMotor, pros::Motor& rightDriveMotor);

		void initialize() {
			this->ptoPiston.set_value(pistonState);
		}

		void update() {
			this->ptoPiston.set_value(pistonState);

			this->leftPtoMotor.move_velocity(leftDriveMotor.get_target_velocity());
			this->rightDriveMotor.move_velocity(rightDriveMotor.get_target_velocity());
		}

		void exit() {

		}

		bool isDone() {
			return false;
		}

		~PtoDrive();
	};
	
	PtoDrive::PtoDrive(pros::ADIDigitalOut& ptoPiston, bool pistonState, pros::Motor& leftPtoMotor, pros::Motor& leftDriveMotor, pros::Motor& rightPtoMotor, pros::Motor& rightDriveMotor) : ptoPiston(ptoPiston), pistonState(pistonState), leftPtoMotor(leftPtoMotor), leftDriveMotor(leftDriveMotor), rightPtoMotor(rightPtoMotor), rightDriveMotor(rightDriveMotor) {
	}
	
	PtoDrive::~PtoDrive() {
	}
} // namespace Pronounce
