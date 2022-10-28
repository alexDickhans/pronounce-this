#pragma once

#include "api.h"
#include "stateMachine/behavior.hpp"
#include "hardware/hardware.hpp"

namespace Pronounce {
	class PtoIntake : public Behavior {
	private:
		pros::ADIDigitalOut& ptoPiston;
		pros::Motor& leftPtoMotor;
		pros::Motor& rightPtoMotor;
		bool pistonState;
		double speed;
	public:
		PtoIntake(std::string name, pros::ADIDigitalOut& ptoPiston, bool pistonState, pros::Motor& leftPtoMotor, pros::Motor& rightPtoMotor, double speed);

		void initialize() {
			leftLedController.setColors(greenColors);
			rightLedController.setColors(greenColors);

			ptoMutex.take();

			ptoPiston.set_value(pistonState);
			leftPtoMotor.move_voltage(speed * 12000);
			rightPtoMotor.move_voltage(speed * 12000);

			ptoMutex.give();
		}

		void update() {
			ptoMutex.take();

			ptoPiston.set_value(pistonState);
			leftPtoMotor.move_voltage(speed * 12000);
			rightPtoMotor.move_voltage(speed * 12000);

			ptoMutex.give();
		}

		void exit() {

		}

		bool isDone() {
			return false;
		}

		~PtoIntake();
	};
	
	PtoIntake::PtoIntake(std::string name, pros::ADIDigitalOut& ptoPiston, bool pistonState, pros::Motor& leftPtoMotor, pros::Motor& rightPtoMotor, double speed) : Behavior(name), ptoPiston(ptoPiston), pistonState(pistonState), leftPtoMotor(leftPtoMotor), rightPtoMotor(rightPtoMotor), speed(speed) {
	}
	
	PtoIntake::~PtoIntake()
	{
	}
	
} // namespace Pronounce
