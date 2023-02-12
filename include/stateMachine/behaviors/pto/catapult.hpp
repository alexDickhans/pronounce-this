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

			leftPtoMotor.move_voltage(speed * 12000.0);
			rightPtoMotor.move_voltage(speed * 12000.0);

			ptoMutex.give();
		}

		void update() {
			ptoMutex.take();

			double multiplier = catapultLimitSwitch.get_angle() > 28000 ? 0.8 : 1.0;

			leftPtoMotor.move_voltage(speed * 12000.0 * multiplier);
			rightPtoMotor.move_voltage(speed * 12000.0 * multiplier);

			std::cout << "CommandedIntakeVoltage: " << speed * 12000.0 << std::endl;

			ptoMutex.give();
		}

		void exit() {

		}

		bool isDone() {
			return (catapultLimitSwitch.get_angle() > 35500);
		}

		~PtoCatapult();
	};

	PtoCatapult::PtoCatapult(std::string name, pros::Motor& leftPtoMotor, pros::Motor& rightPtoMotor, pros::Rotation& catapultLimitSwitch, double speed) : Behavior(name), leftPtoMotor(leftPtoMotor), rightPtoMotor(rightPtoMotor), catapultLimitSwitch(catapultLimitSwitch), speed(speed) {
	}

	PtoCatapult::~PtoCatapult()
	{
	}

} // namespace Pronounce
