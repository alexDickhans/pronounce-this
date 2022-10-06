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
		PtoDrive(std::string name, pros::ADIDigitalOut& ptoPiston, bool pistonState, pros::Motor& leftPtoMotor, pros::Motor& leftDriveMotor, pros::Motor& rightPtoMotor, pros::Motor& rightDriveMotor);

		void initialize() {
			ptoMutex.take();

			this->ptoPiston.set_value(pistonState);
			
			ptoMutex.give();
		}

		void update() {
			ptoMutex.take();

			this->ptoPiston.set_value(pistonState);

			this->leftPtoMotor.move_velocity(leftDriveMotor.get_target_velocity());
			this->rightDriveMotor.move_velocity(rightDriveMotor.get_target_velocity());

			ptoMutex.take();
		}

		void exit() {

		}

		bool isDone() {
			return false;
		}

		~PtoDrive();
	};
	
	PtoDrive::PtoDrive(std::string name, pros::ADIDigitalOut& ptoPiston, bool pistonState, pros::Motor& leftPtoMotor, pros::Motor& leftDriveMotor, pros::Motor& rightPtoMotor, pros::Motor& rightDriveMotor) : Behavior(name), ptoPiston(ptoPiston), pistonState(pistonState), leftPtoMotor(leftPtoMotor), leftDriveMotor(leftDriveMotor), rightPtoMotor(rightPtoMotor), rightDriveMotor(rightDriveMotor) {
	}
	
	PtoDrive::~PtoDrive() {
	}
} // namespace Pronounce
