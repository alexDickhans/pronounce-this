#pragma once

#include "stateMachine/behavior.hpp"
#include "pros/abstract_motor.hpp"

namespace Pronounce {
	class WinchVoltage : public Behavior {
	private:
		pros::AbstractMotor& abstractMotor;
		int32_t voltage;
	public:
		WinchVoltage(pros::AbstractMotor &abstractMotor, int32_t voltage) : abstractMotor(abstractMotor),
		                                                                    voltage(voltage) {}

		void initialize() override {
			abstractMotor.move_voltage(voltage);
		}

		bool isDone() override {
			return false;
		}

		void exit() override {
			abstractMotor.move_voltage(0);
		}

		[[nodiscard]] int32_t getVoltage() const {
			return voltage;
		}

		void setVoltage(int32_t voltage) {
			WinchVoltage::voltage = voltage;
		}
	};
}