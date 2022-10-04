#pragma once

#include "stateMachine/behavior.hpp"
#include "hardware/hardware.hpp"
#include <iostream>

namespace Pronounce {
	/**
	 * @brief Logs data from the robot that is not logged in the individual files
	 * 
	 */
	class LoggerService : public Behavior {
	private:
	public:
		LoggerService();

		void initialize() {
			
		}

		void update() {

		}

		void exit() {
			
		}

		~LoggerService();
	}	
} // namespace Pronounce
