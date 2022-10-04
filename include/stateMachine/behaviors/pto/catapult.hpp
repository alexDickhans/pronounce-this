#pragma once

#include "api.h"
#include "stateMachine/behavior.hpp"

namespace Pronounce {
	class PtoCatapult : public Behavior{
	private:
		pros::ADIDigitalOut& ptoPiston;
		pros::ADIDigitalIn& catapultLimitSwitch;
		pros::Motor& leftPtoMotor;
		pros::Motor& rightPtoMotor;
		bool pistonState;
		double speed;
	public:
		PtoCatapult(std::string name, pros::ADIDigitalOut& ptoPiston, bool pistonState, pros::Motor& leftPtoMotor, pros::Motor& rightPtoMotor, pros::ADIDigitalIn& catapultLimitSwitch, double speed);

		void initialize() {
			ptoPiston.set_value(pistonState);
			leftPtoMotor.move_velocity(speed*600);
			rightPtoMotor.move_velocity(speed*600);
		}

		void update() {
			ptoPiston.set_value(pistonState);
			leftPtoMotor.move_velocity(speed*600);
			rightPtoMotor.move_velocity(speed*600);
		}

		void exit() {
			
		}

		bool isDone() {
			return catapultLimitSwitch.get_value();
		}

		~PtoCatapult();
	};
	
	PtoCatapult::PtoCatapult(std::string name, pros::ADIDigitalOut& ptoPiston, bool pistonState, pros::Motor& leftPtoMotor, pros::Motor& rightPtoMotor, pros::ADIDigitalIn& catapultLimitSwitch, double speed) : Behavior(name), ptoPiston(ptoPiston), pistonState(pistonState), leftPtoMotor(leftPtoMotor), rightPtoMotor(rightPtoMotor), catapultLimitSwitch(catapultLimitSwitch), speed(speed) {
	}
	
	PtoCatapult::~PtoCatapult()
	{
	}
	
} // namespace Pronounce
