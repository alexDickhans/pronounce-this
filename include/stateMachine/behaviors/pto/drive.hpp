#pragma once

#include "api.h"
#include "stateMachine/behavior.hpp"
#include "hardware/hardware.hpp"

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
			leftLedController.setColors(blueColors);
			rightLedController.setColors(blueColors);
			ptoMutex.take();

			this->ptoPiston.set_value(pistonState);
			
			ptoMutex.give();
		}

		void update() {
			ptoMutex.take();

			this->ptoPiston.set_value(pistonState);

			std::cout << "DrivetrainLeftVoltage: " << rightDriveMotor.get_voltage() << std::endl;

			this->leftPtoMotor.move_voltage(leftVoltage);
			this->rightPtoMotor.move_voltage(rightVoltage);

			ptoMutex.give();
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
