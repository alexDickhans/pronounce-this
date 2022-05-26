#pragma once

#include "../include/stateMachine/behavior.hpp"
#include <string>
#include <iostream>

namespace Pronounce
{	
	/**
	 * @brief A behavior to test the state machine systems
	 * 
	 */
	class TestBehavior : public Behavior {
	private:
		/**
		 * @brief Manually inputted name to differentiate objects
		 * 
		 */
		std::string name;

		/**
		 * @brief Be able to manually trigger the states stop
		 * 
		 */
		bool done = false;
	public:
		/**
		 * @brief Construct a new Test Behavior object
		 * 
		 * @param name - Name for this object
		 */
		TestBehavior(std::string name) {
			this->name = name;
		}

		/**
		 * @brief Print out Init: <name>
		 * 
		 */
		void initialize() {
			std::cout << "Init: " << name << std::endl;
		}

		/**
		 * @brief Print out Init: <name>
		 * 
		 */
		void update() {
			std::cout << "Update: " << name << std::endl;
		}

		/**
		 * @brief Print out Init: <name>
		 * 
		 */
		void exit() {
			std::cout << "Exit: " << name << std::endl;
		}

		/**
		 * @brief Print out Done: <name> if it is done
		 * 
		 * @return done the done variable
		 */
		bool isDone() {
			if (done) {
				std::cout << "Done: " << name << std::endl;
			}
			return done;
		}

		/**
		 * @brief Set the Done object, can manually tell the task to end
		 * 
		 * @param done 
		 */
		void setDone(bool done) {
			this->done = done;
		}

		~TestBehavior() {}
	};
} // namespace Pronounce
