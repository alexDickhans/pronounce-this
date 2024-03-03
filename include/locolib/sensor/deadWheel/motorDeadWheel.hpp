#pragma once

#include "deadWheel.hpp"
#include "pros/motors.hpp"
#include <vector>
#include <numeric>
#include "units/units.hpp"

namespace Loco {
	class MotorDeadWheel : public DeadWheel {
	private:
		pros::Motor_Group &motorGroup;
		double gearRatio;
	public:
		MotorDeadWheel(pros::Motor_Group &motorGroup1, double gearRatio, QLength radius) : motorGroup(motorGroup1),
		                                                                                   DeadWheel(radius),
		                                                                                   gearRatio(gearRatio) {
			motorGroup.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
		}

		void update() override {
			double total = 0.0;

			for (const auto &position: motorGroup.get_positions()) {
				total += position;
			}

			total /= motorGroup.size();

			return DeadWheel::updateReading(
					total*degree/gearRatio);
		}

		QLength getTickLength() const override {
			return 0.1_in;
		}
	};
}