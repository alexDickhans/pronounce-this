#pragma once

#include "api.h"
#include "stateMachine/behavior.hpp"

namespace Pronounce {
	class PtoCatapult : public Behavior {
	private:
		pros::Rotation& catapultLimitSwitch;
		pros::Motor& leftPtoMotor;
		pros::Motor& rightPtoMotor;

		pros::ADIDigitalOut& intakeStopper;
		double speed;
	public:
		PtoCatapult(std::string name, pros::Motor& leftPtoMotor, pros::Motor& rightPtoMotor, pros::ADIDigitalOut& intakeStopper, pros::Rotation& catapultLimitSwitch, double speed);

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

			double multiplier = catapultLimitSwitch.get_angle() > 30000 ? 0.6 : 1.0;

			leftPtoMotor.move_voltage(speed * 12000.0 * multiplier);
			rightPtoMotor.move_voltage(speed * 12000.0 * multiplier);

			std::cout << "CommandedIntakeVoltage: " << speed * 12000.0 * multiplier << std::endl;
			intakeStopper.set_value(true);

			ptoMutex.give();
		}

		void exit() {
			leftPtoMotor.move_voltage(0.0);
			rightPtoMotor.move_voltage(0.0);
		}

		bool isDone() {
			return (catapultLimitSwitch.get_angle() > 35650) && (catapultLimitSwitch.get_angle() > 1);
		}

		~PtoCatapult();
	};

	PtoCatapult::PtoCatapult(std::string name, pros::Motor& leftPtoMotor, pros::Motor& rightPtoMotor, pros::ADIDigitalOut& intakeStopper, pros::Rotation& catapultLimitSwitch, double speed) : Behavior(name), leftPtoMotor(leftPtoMotor), rightPtoMotor(rightPtoMotor), intakeStopper(intakeStopper), catapultLimitSwitch(catapultLimitSwitch), speed(speed) {
	}

	PtoCatapult::~PtoCatapult()
	{
	}

} // namespace Pronounce
