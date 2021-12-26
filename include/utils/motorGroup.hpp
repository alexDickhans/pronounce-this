#pragma once

#include <vector>
#include "api.h"

namespace Pronounce {
	class MotorGroup {
	private:
		std::vector<pros::Motor*> motors;
	public:
		MotorGroup();
		MotorGroup(std::vector<pros::Motor*> motors);
		MotorGroup(pros::Motor* motors ...);

		void operator=(std::int32_t power) {
			for (pros::Motor* motor : motors) {
				motor->move_voltage(power);
			}
		}

		void move(double power) {
			for (pros::Motor* motor : motors) {
				motor->move(power);
			}
		}

		void move_absolute(double position, std::int32_t velocity) {
			for (pros::Motor* motor : motors) {
				motor->move_absolute(position, velocity);
			}
		}

		void move_relative(double position, std::int32_t velocity) {
			for (pros::Motor* motor : motors) {
				motor->move_relative(position, velocity);
			}
		}

		void move_velocity(double velocity) {
			for (pros::Motor* motor : motors) {
				motor->move_velocity(velocity);
			}
		}

		void move_voltage(std::int32_t voltage) {
			for (pros::Motor* motor : motors) {
				motor->move_voltage(voltage);
			}
		}

		void modify_profiled_velocity(double voltage) {
			for (pros::Motor* motor : motors) {
				motor->modify_profiled_velocity(voltage);
			}
		}

		double get_target_position() {
			double targetPosition = 0;
			for (pros::Motor* motor : motors) {
				targetPosition += motor->get_target_position();
			}
			return targetPosition / motors.size();
		}

		double get_target_velocity() {
			double targetVelocity = 0;
			for (pros::Motor* motor : motors) {
				targetVelocity += motor->get_target_velocity();
			}
			return targetVelocity / motors.size();
		}

		double get_actual_velocity() {
			double actualVelocity = 0;
			for (pros::Motor* motor : motors) {
				actualVelocity += motor->get_actual_velocity();
			}
			return actualVelocity / motors.size();
		}

		double get_current_draw() {
			double currentDraw = 0;
			for (pros::Motor* motor : motors) {
				currentDraw += motor->get_current_draw();
			}
			return currentDraw / motors.size();
		}

		std::int32_t get_direction() {
			std::int32_t direction = 0;
			for (pros::Motor* motor : motors) {
				direction += motor->get_direction();
			}
			return direction / motors.size();
		}

		double get_efficiency() {
			double efficiency = 0;
			for (pros::Motor* motor : motors) {
				efficiency += motor->get_efficiency();
			}
			return efficiency / motors.size();
		}

		double is_stopped() {
			double stopped = 0;
			for (pros::Motor* motor : motors) {
				stopped += motor->is_stopped();
			}
			return stopped / motors.size();
		}

		double get_temperature() {
			double stopped = 0;
			for (pros::Motor* motor : motors) {
				stopped += motor->get_temperature();
			}
			return stopped / motors.size();
		}

		

		~MotorGroup();
	};	
} // namespace Pronounce
