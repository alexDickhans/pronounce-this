#pragma once

#include "stateMachine/behavior.hpp"

namespace Pronounce {
	class VoltageDrivetrain : public Behavior {
	private:
		int32_t leftVoltage, rightVoltage;
		TankDrivetrain& drivetrain;
	public:
		VoltageDrivetrain(int32_t leftVoltage, int32_t rightVoltage, TankDrivetrain &drivetrain) : leftVoltage(
				leftVoltage), rightVoltage(rightVoltage), drivetrain(drivetrain), Behavior("VoltageDrivetrain") {}

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