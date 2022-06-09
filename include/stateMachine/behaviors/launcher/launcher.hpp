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
		double turretOutputMultiplier = 3.7;

		MotorGroup* flywheelMotor;
		pros::ADIDigitalOut* indexer;

		pros::Motor* turretMotor;

		FlywheelPID* flywheelPID;
		PID* turretPID;

		bool useIsDone = false;

	public:
		Launcher(double flywheelSpeedMultiplier, double flywheelOutputMultiplier, bool pneumaticEngaged, bool useIsDone, MotorGroup* flywheelMotor, pros::Motor* turretMotor, pros::ADIDigitalOut* indexer, FlywheelPID* flywheelPID, PID* turretPID) {
			this->flywheelSpeedMultiplier = flywheelSpeedMultiplier;
			this->flywheelOutputMultiplier = flywheelOutputMultiplier;
			this->pneumaticEngaged = pneumaticEngaged;
			this->useIsDone = useIsDone;
			this->flywheelMotor = flywheelMotor;
			this->turretMotor = turretMotor;
			this->indexer = indexer;
			this->flywheelPID = flywheelPID;
			this->turretPID = turretPID;
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

			turretPID->setTarget(turretAngle * turretOutputMultiplier);
			double turretPower = turretPID->update(turretMotor->get_position() / turretOutputMultiplier);
			turretMotor->move_voltage(turretPower);
		}

		void exit() {
			flywheelMotor->move_voltage(0.0);
		}

		bool isDone() {
			if (useIsDone) {
				return abs(this->flywheelSpeed - this->getFlywheelSpeed()) < 50;
			} else {
				return false;
			}
		}

		void setFlywheelSpeed(double flywheelSpeed) {
			this->flywheelSpeed = flywheelSpeed;
		}

		void setTurretAngle(double value) {
			this->turretAngle = value;
		} 

		double getFlywheelSpeed() {
			return flywheelMotor->get_actual_velocity() * flywheelOutputMultiplier;
		}

		~Launcher() {}
	};
} // namespace Pronounce
