#pragma once

#include "api.h"
#include "stateMachine/behavior.hpp"
#include "feedbackControllers/flywheelPID.hpp"
#include "utils/motorGroup.hpp"
#include <iostream>
#include "units/units.hpp"

// TODO: Add comments

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
		 * @brief Boolean to determine if the indexer is engaged or not
		 * 
		 */
		bool indexerEngaged;

		double flywheelSpeed = 0.0;

		double turretAngle = 0.0;
		double turretOutputMultiplier = 3.7;

		MotorGroup* flywheelMotor;
		pros::ADIDigitalOut* indexer;

		pros::Motor* turretMotor;
		pros::Rotation& rotationSensor;

		FlywheelPID* flywheelPID;
		PID* turretPID;

		bool useIsDone = false;

	public:
		Launcher(std::string name, double flywheelSpeedMultiplier, double flywheelOutputMultiplier, bool indexerEngaged, bool useIsDone, MotorGroup* flywheelMotor, pros::Motor* turretMotor, pros::ADIDigitalOut* indexer, FlywheelPID* flywheelPID, PID* turretPID, pros::Rotation& turretRotation) : rotationSensor(turretRotation), Behavior(name) {
			this->flywheelSpeedMultiplier = flywheelSpeedMultiplier;
			this->flywheelOutputMultiplier = flywheelOutputMultiplier;
			this->indexerEngaged = indexerEngaged;
			this->useIsDone = useIsDone;
			this->flywheelMotor = flywheelMotor;
			this->turretMotor = turretMotor;
			this->indexer = indexer;
			this->flywheelPID = flywheelPID;
			this->turretPID = turretPID;
			turretPID->setTurnPid(true);
		}

		void initialize() {
			indexer->set_value(indexerEngaged);
		}

		void update() {
			std::cout << "ActualFlywheelSpeed: " << flywheelMotor->get_actual_velocity() * flywheelOutputMultiplier << std::endl;
			if (flywheelSpeedMultiplier > 0.0) {
				flywheelPID->setPosition(flywheelSpeed * flywheelSpeedMultiplier);
				double flywheelVoltage = flywheelPID->update(flywheelMotor->get_actual_velocity() * flywheelOutputMultiplier);
				flywheelMotor->move_voltage(flywheelVoltage);
				// std::cout << "InputFlywheelVoltage: " << flywheelVoltage << std::endl;
			}
			indexer->set_value(indexerEngaged);

			turretPID->setTarget(turretAngle);
			double turretPower = turretPID->update(toRadians(rotationSensor.get_angle() / 100.0));

			turretMotor->move_voltage(turretPower);
		}

		void exit() {
			flywheelMotor->move_voltage(0.0);
		}

		bool isDone() {
			if (useIsDone) {
				return fabs(this->flywheelSpeed - this->getFlywheelSpeed()) < 20.0;
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
