#pragma once

#include "api.h"
#include "stateMachine/behavior.hpp"
#include "hardware/hardware.hpp"

namespace Pronounce {
	class PtoIntake : public Behavior {
	private:
		pros::Motor& leftPtoMotor;
		pros::Motor& rightPtoMotor;
		pros::ADIDigitalOut& intakeStopper;
		double speed;
	public:
		PtoIntake(std::string name, pros::Motor& leftPtoMotor, pros::Motor& rightPtoMotor, pros::ADIDigitalOut& intakeStopper, double speed);

		void initialize() {
			leftLedController.setColors(greenColors);
			rightLedController.setColors(greenColors);

			ptoMutex.take();

			leftPtoMotor.move_voltage(speed * 12000);
			rightPtoMotor.move_voltage(speed * 12000);
			
			aimingVisionSensor.set_led(COLOR_BROWN);

			intakeStopper.set_value(false || intakeStopperOverride);

			ptoMutex.give();
		}

		void update() {
			ptoMutex.take();

			leftPtoMotor.move_voltage(speed * 12000);
			rightPtoMotor.move_voltage(speed * 12000);

			intakeStopper.set_value(false || intakeStopperOverride);

			std::cout << "CommandedIntakeVoltage: " << speed * 12000.0 << std::endl;

			ptoMutex.give();
		}

		void exit() {
			aimingVisionSensor.clear_led();
		}

		bool isDone() {
			return false;
		}

		~PtoIntake();
	};
	
	PtoIntake::PtoIntake(std::string name, pros::Motor& leftPtoMotor, pros::Motor& rightPtoMotor, pros::ADIDigitalOut& intakeStopper, double speed) : Behavior(name), leftPtoMotor(leftPtoMotor), rightPtoMotor(rightPtoMotor), intakeStopper(intakeStopper), speed(speed) {
	}
	
	PtoIntake::~PtoIntake()
	{
	}
	
} // namespace Pronounce
