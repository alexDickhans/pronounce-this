#pragma once

#include <string>
#include "callbackHelper.hpp"

namespace Pronounce {
	/**
	 * @brief Parent class that provides a template class for all behaviors. Inherit this class to create a new behavior
	 * 
	 * @author @ad101-lab - Alex Dickhans
	 */
	class Behavior {
	private:
		std::string name = "none";
	public:
		/**
		 * @brief Basic initializer
		 * 
		 */
		Behavior() {}

		/**
		 * @brief Basic initializer
		 * 
		 */
		Behavior(std::string name) : name(name) {}

		/**
		 * @brief Abstract function to initialize the state. Called before the state starts running
		 * 
		 */
		virtual void initialize() {}

		/**
		 * @brief Abstract function to update the state. Called every frame when the state is running
		 * 
		 */
		virtual void update() {}

		/**
		 * @brief Abstract isDone behavior. Returns true when the state is done running
		 * 
		 * @return true Return true when the state is done being run
		 * @return false Return false when the state isn't done
		 */
		virtual bool isDone() { return false; }

		/**
		 * @brief Abstract function to exit the state. Called after the state stops running. !NOT GUARANTEED TO RUN!
		 * 
		 */
		virtual void exit() {}

		Behavior* until(BooleanCallback booleanCallback);
        Behavior* continueBehavior();

		std::string getName() {
			return name;
		}

		void setName(std::string newName) {
			this->name = std::move(newName);
		}

		~Behavior() = default;
	};	
} // namespace Pronounce
