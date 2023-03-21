#pragma once

#include "stateMachine/behavior.hpp"
#include "hardware/hardware.hpp"
#include "stateMachine/behaviors/drivetrain/initDrivetrain.hpp"
#include <iostream>



namespace Pronounce {
	/**
	 * @brief Logs data from the robot that is not logged in the individual files
	 * 
	 */
	class LoggerService : public Behavior {
	private:
	public:
		LoggerService() {}

		void initialize() {
			
		}
		
		void update() {
			std::cout << "OutputDrivetrainSpeed: " << drivetrain.getSpeed().getValue() << std::endl;
			std::cout << "PtoSpeed: " << leftPtoMotor.get_actual_velocity() << std::endl;
			std::cout << "PtoCurrent: " << leftPtoMotor.get_current_draw() << std::endl;
			std::cout << "PtoOutputVoltage: " << leftPtoMotor.get_voltage() << std::endl;
			std::cout << "FrontDistanceSensorDistance: " << frontDistanceSensor.get() << std::endl;
			std::cout << "FrontDistanceSensorConfidence: " << frontDistanceSensor.get_confidence() << std::endl;
			std::cout << "FrontDistanceSensorSize: " << frontDistanceSensor.get_object_velocity() << std::endl;
			std::cout << "FrontDistanceSensorVelocity: " << frontDistanceSensor.get_object_size() << std::endl;
			std::cout << "CatapultLimitSwitch: " << catapultLimitSwitch.get_angle() << std::endl;
		}

		void exit() {

		}

		~LoggerService() {}
	};
} // namespace Pronounce
