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

		void operator=(std::int32_t power) {
			for (int i = 0; i < motors.size(); i++) {
				pros::Motor* motor = this->motors.at(i);
				motor->move_voltage(power);
			}
		}

		void addMotor(pros::Motor* motor) {
			this->motors.emplace_back(motor);
		} 

		void move(double power) {
			for (int i = 0; i < motors.size(); i++) {
				pros::Motor* motor = this->motors.at(i);
				motor->move(power);
			}
		}

		void move_absolute(double position, std::int32_t velocity) {
			for (int i = 0; i < motors.size(); i++) {
				pros::Motor* motor = this->motors.at(i);
				motor->move_absolute(position, velocity);
			}
		}

		void move_relative(double position, std::int32_t velocity) {
			for (int i = 0; i < motors.size(); i++) {
				pros::Motor* motor = this->motors.at(i);
				motor->move_relative(position, velocity);
			}
		}

		void move_velocity(double velocity) {
			for (int i = 0; i < motors.size(); i++) {
				pros::Motor* motor = this->motors.at(i);
				motor->move_velocity(velocity);
			}
		}

		void move_voltage(std::int32_t voltage) {
			for (int i = 0; i < motors.size(); i++) {
				pros::Motor* motor = this->motors.at(i);
				motor->move_voltage(voltage);
			}
		}

		void modify_profiled_velocity(double voltage) {
			for (int i = 0; i < motors.size(); i++) {
				pros::Motor* motor = this->motors.at(i);
				motor->modify_profiled_velocity(voltage);
			}
		}

		double get_target_position() {
			double targetPosition = 0;
			for (int i = 0; i < motors.size(); i++) {
				pros::Motor* motor = this->motors.at(i);
				targetPosition += motor->get_target_position();
			}
			return targetPosition / motors.size();
		}

		double get_target_velocity() {
			double targetVelocity = 0;
			for (int i = 0; i < motors.size(); i++) {
				pros::Motor* motor = this->motors.at(i);
				targetVelocity += motor->get_target_velocity();
			}
			return targetVelocity / motors.size();
		}

		double get_actual_velocity() {
			double actualVelocity = 0;
			for (int i = 0; i < motors.size(); i++) {
				pros::Motor* motor = this->motors.at(i);
				actualVelocity += motor->get_actual_velocity();
			}
			return actualVelocity / motors.size();
		}

		double get_current_draw() {
			double currentDraw = 0;
			for (int i = 0; i < motors.size(); i++) {
				pros::Motor* motor = this->motors.at(i);
				currentDraw += motor->get_current_draw();
			}
			return currentDraw / motors.size();
		}

		std::int32_t get_direction() {
			std::int32_t direction = 0;
			for (int i = 0; i < motors.size(); i++) {
				pros::Motor* motor = this->motors.at(i);
				direction += motor->get_direction();
			}
			return direction / motors.size();
		}

		double get_efficiency() {
			double efficiency = 0;
			for (int i = 0; i < motors.size(); i++) {
				pros::Motor* motor = this->motors.at(i);
				efficiency += motor->get_efficiency();
			}
			return efficiency / motors.size();
		}

		double is_stopped() {
			double stopped = 0;
			for (int i = 0; i < motors.size(); i++) {
				pros::Motor* motor = this->motors.at(i);
				stopped += motor->is_stopped();
			}
			return stopped / motors.size();
		}

		double get_temperature() {
			double stopped = 0;
			for (int i = 0; i < motors.size(); i++) {
				pros::Motor* motor = this->motors.at(i);
				stopped += motor->get_temperature();
			}
			return stopped / motors.size();
		}

		void set_brake_mode(pros::motor_brake_mode_e_t mode) {
			for (int i = 0; i < motors.size(); i++) {
				pros::Motor* motor = this->motors.at(i);
				motor->set_brake_mode(mode);
			}
		}

		~MotorGroup();
	};	
} // namespace Pronounce
