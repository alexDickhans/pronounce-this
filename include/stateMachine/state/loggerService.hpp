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
		}

		void exit() {

		}

		~LoggerService() {}
	};
} // namespace Pronounce
