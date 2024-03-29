#pragma once

#include "stateMachine/behavior.hpp"

namespace Pronounce {
	class VoltageDrivetrain : public Behavior {
	private:
		int32_t leftVoltage, rightVoltage;
		AbstractTankDrivetrain& drivetrain;
	public:
		VoltageDrivetrain(int32_t leftVoltage, int32_t rightVoltage, AbstractTankDrivetrain &drivetrain) : leftVoltage(
				leftVoltage), rightVoltage(rightVoltage), drivetrain(drivetrain) {}

		void initialize() override {
			drivetrain.tankSteerVoltage(leftVoltage, rightVoltage);
		}

		void update() override {
			drivetrain.tankSteerVoltage(leftVoltage, rightVoltage);
		}

		bool isDone() override {
			return false;
		}

		void exit() override {
			drivetrain.tankSteerVoltage(0, 0);
		}
	};
}