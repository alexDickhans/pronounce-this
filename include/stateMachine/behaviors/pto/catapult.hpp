#pragma once

#include "api.h"
#include "stateMachine/behavior.hpp"

namespace Pronounce {
	class PtoCatapult : public Behavior {
	private:
		pros::Rotation& catapultLimitSwitch;
		pros::Motor& leftPtoMotor;
		pros::Motor& rightPtoMotor;
		double speed;
	public:
		PtoCatapult(std::string name, pros::Motor& leftPtoMotor, pros::Motor& rightPtoMotor, pros::Rotation& catapultLimitSwitch, double speed);

		void initialize() {
			leftLedController.setColors(orangeColors);
			rightLedController.setColors(orangeColors);
			
			ptoMutex.take();

			leftPtoMotor.move_voltage(speed * 12000);
			rightPtoMotor.move_voltage(speed * 12000);

			ptoMutex.give();
		}

		void update() {
			ptoMutex.take();

			leftPtoMotor.move_voltage(speed * 12000);
			rightPtoMotor.move_voltage(speed * 12000);

			ptoMutex.give();
		}

		void exit() {

		}

		bool isDone() {
			return angleDifference((catapultLimitSwitch.get_angle() * degree / 100.0).getValue(), 0) < 0;
		}

		~PtoCatapult();
	};

	PtoCatapult::PtoCatapult(std::string name, pros::Motor& leftPtoMotor, pros::Motor& rightPtoMotor, pros::Rotation& catapultLimitSwitch, double speed) : Behavior(name), leftPtoMotor(leftPtoMotor), rightPtoMotor(rightPtoMotor), catapultLimitSwitch(catapultLimitSwitch), speed(speed) {
	}

	PtoCatapult::~PtoCatapult()
	{
	}

} // namespace Pronounce
