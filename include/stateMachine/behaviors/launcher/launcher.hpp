#pragma once

#include "api.h"
#include "stateMachine/behavior.hpp"
#include "feedbackControllers/flywheelPID.hpp"
#include "utils/motorGroup.hpp"
#include <iostream>

namespace Pronounce {
	class Launcher : public Behavior {
	private:
		/**
		 * @brief The flywheel speed multiplier is a configuration that will change per state for different use cases
		 * 
		 */
		double flywheelSpeedMultiplier;
		/**
		 * @brief Staticish variable that should stay the same between all states. Converts the rpm of the motor to the actual rpm of the wheel
		 * 
		 */
		double flywheelOutputMultiplier = 0.0;
		/**
		 * @brief Boolean to determine if the 
		 * 
		 */
		bool pneumaticEngaged;

		// double distanceFromCenter;
		double flywheelSpeed = 0.0;

		double turretAngle = 0.0;
		double turretOutputMultiplier = 1.0;

		MotorGroup* flywheelMotor;
		pros::ADIDigitalOut* indexer;

		pros::Motor* turretMotor;

		FlywheelPID* flywheelPID;

	public:
		Launcher(double flywheelSpeedMultiplier, double flywheelOutputMultiplier, bool pneumaticEngaged, MotorGroup* flywheelMotor, pros::Motor* turretMotor, pros::ADIDigitalOut* indexer, FlywheelPID* flywheelPID) {
			this->flywheelSpeedMultiplier = flywheelSpeedMultiplier;
			this->flywheelOutputMultiplier = flywheelOutputMultiplier;
			this->pneumaticEngaged = pneumaticEngaged;
			this->flywheelMotor = flywheelMotor;
			this->turretMotor = turretMotor;
			this->indexer = indexer;
			this->flywheelPID = flywheelPID;
		}

		void initialize() {
			indexer->set_value(pneumaticEngaged);
		}

		void update() {
			if (flywheelSpeedMultiplier > 0.0) {
				flywheelPID->setPosition(flywheelSpeed * flywheelSpeedMultiplier);
				double flywheelVoltage = flywheelPID->update(flywheelMotor->get_actual_velocity() * flywheelOutputMultiplier);
				flywheelMotor->move_voltage(flywheelVoltage);
				std::cout << "Flywheel speed: " << flywheelMotor->get_actual_velocity() * flywheelOutputMultiplier << std::endl;
				std::cout << "Flywheel voltage: " << flywheelVoltage << std::endl;
			}

			indexer->set_value(pneumaticEngaged);

			turretMotor->move_absolute(turretAngle*turretOutputMultiplier, 200);
		}

		void exit() {
			flywheelMotor->move_voltage(0.0);
		}

		void setFlywheelSpeed(double flywheelSpeed) {
			this->flywheelSpeed = flywheelSpeed;
		}

		void setTurretAngle(double value) {
			this->turretAngle = value;
		} 

		~Launcher() {}
	};
} // namespace Pronounce
