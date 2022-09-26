#pragma once

#include "behavior.hpp"
#include <string>
#include <unordered_map>
#include <iostream>

namespace Pronounce {
	/**
	 * @brief State controller controls a subsystem so that only one state will ever be running at a time. Returns to default once a state finishes
	 * 
	 * @author Alex Dickhans
	 */
	class StateController : public Behavior {
	private:
		/**
		 * @brief Default behavior that the stateController returns to when the current state isn't running
		 * 
		 */
		Behavior* defaultBehavior;
		
		/**
		 * @brief The current behavior that runs if it exists.
		 * 
		 */
		Behavior* currentBehavior = nullptr;
	public:
		/**
		 * @brief Construct a new State Controller object
		 * 
		 * @param defaultBehavior 
		 */
		StateController(std::string name, Behavior* defaultBehavior) : Behavior(name) {
			this->defaultBehavior = defaultBehavior;
		}

		/**
		 * @brief Initialize the behavior. If currentbehavior exists that one gets initialized
		 * 
		 */
		void initialize() {
			if (currentBehavior != nullptr) {
				currentBehavior->initialize();
			}
			else {
				defaultBehavior->initialize();
			}
		}

		/**
		 * @brief Update the functions and check if you need to Transition
		 * 
		 */
		void update() {
			std::cout << this->toString() << std::endl;

			// If currentBehavior exists run it or transition
			if (currentBehavior != nullptr) {
				if (currentBehavior->isDone()) {
					currentBehavior->exit();
					currentBehavior = nullptr;
					defaultBehavior->initialize();
					defaultBehavior->update();
				}
				else {
					currentBehavior->update();
				}
			}
			else {
				// Run the default behavior when that one doesn't exist
				defaultBehavior->update();
			}
		}

		/**
		 * @brief Return that the object is done when the current behavior isn't running
		 * 
		 * @return true Current behavior is not running
		 * @return false Default behavior is not running
		 */
		bool isDone() {
			return currentBehavior == nullptr;
		}

		/**
		 * @brief Exit the behavior depending on which one is running
		 * 
		 */
		void exit() {
			// Exit the right behavior depending on which one is running
			if (currentBehavior != nullptr) {
				currentBehavior->exit();
				currentBehavior = nullptr;
			}
			else {
				defaultBehavior->exit();
			}
		}

		/**
		 * @brief Set the Default Behavior object
		 * 
		 * @param behavior 
		 */
		void setDefaultBehavior(Behavior* behavior) {
			defaultBehavior = behavior;
		}

		/**
		 * @brief Set the Current Behavior object depending on what is running and what it equals
		 * 
		 * @param behavior 
		 */
		void setCurrentBehavior(Behavior* behavior) {
			// If the defaultBehavior and current behavior are equal then switch to default behavior
			if (defaultBehavior == behavior) {
				this->useDefaultBehavior();
			}
			// If the currentBehavior exists and transition from that if it does
			else if (currentBehavior != nullptr) {
				currentBehavior->exit();
				currentBehavior = behavior;
				currentBehavior->initialize();
				currentBehavior->update();
			}
			// Transition from default behavior if it doesn't
			else {
				defaultBehavior->exit();
				currentBehavior = behavior;
				currentBehavior->initialize();
				currentBehavior->update();
			}
		}
		
		/**
		 * @brief Transition to the default behavior
		 * 
		 */
		void useDefaultBehavior() {
			// If we are currently running the current behavior transition
			if (currentBehavior != nullptr) {
				currentBehavior->exit();
				currentBehavior = nullptr;
				defaultBehavior->initialize();
			}
			// If not no transition needed.
		}

		Behavior* getCurrentBehavior() {
			return currentBehavior;
		}

		std::string toString() {
			return this->getName() + ": " + (this->isDone() ? defaultBehavior->getName() : currentBehavior->getName());
		}

		~StateController() {}
	};
} // namespace Pronounce
