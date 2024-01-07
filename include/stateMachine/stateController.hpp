#pragma once

#include "behavior.hpp"
#include "time/time.hpp"
#include "time/robotTime.hpp"
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

		QTime startTime = 0.0;
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
			startTime = currentTime();
			if (currentBehavior != nullptr) {
				try {
					currentBehavior->initialize();
				} catch(...) {
					std::cerr << "ERROR: In " << this->getName() << " \"" << currentBehavior->getName() << "\" failed to initialize, returning to default state" << std::endl;
					this->useDefaultBehavior();
				}
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
					try {
						currentBehavior->exit();
					} catch (std::exception &e) {
						std::cerr << "ERROR: In " << this->getName() << ", \"" << currentBehavior->getName() << "\" failed to exit. what: " << e.what() << ", returning to default state" << std::endl;
					}

					currentBehavior = nullptr;
					defaultBehavior->initialize();
					defaultBehavior->update();
					startTime = currentTime();
				}
				else {
					try {
						currentBehavior->update();
					} catch (std::exception &e) {
						std::cerr << "ERROR: In " << this->getName() << ", \"" << currentBehavior->getName() << "\" failed to update. what: " << e.what() << ", returning to default state" << std::endl;
						this->useDefaultBehavior();
					}
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

		std::function<void ()> waitUntilDone() {
			return [this] () -> void {
				while(!this->isDone()) {
					pros::Task::delay(10);
				}
				return;
			};
		}

		/**
		 * @brief Exit the behavior depending on which one is running
		 * 
		 */
		void exit() override {
			// Exit the right behavior depending on which one is running
			if (currentBehavior != nullptr) {
				try {
					currentBehavior->exit();
				} catch (std::exception &e) {
					std::cerr << "ERROR: In " << this->getName() << ", \"" << currentBehavior->getName() << "\" failed to exit. what: " << e.what() << ", returning to default state" << std::endl;
				}
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
				try {
					currentBehavior->exit();
				} catch (std::exception &e) {
					std::cerr << "ERROR: In " << this->getName() << ", \"" << currentBehavior->getName() << "\" failed to exit. what: " << e.what() << ", returning to default state" << std::endl;
				}
				try {
					currentBehavior = behavior;
					currentBehavior->initialize();
				} catch (std::exception &e) {
					std::cerr << "ERROR: In " << this->getName() << ", \"" << currentBehavior->getName() << "\" failed to initialize. what: " << e.what() << ", returning to default state" << std::endl;
					this->useDefaultBehavior();
				}
				startTime = currentTime();
			}
			// Transition from default behavior if it doesn't
			else {
				defaultBehavior->exit();
				try {
					currentBehavior = behavior;
					currentBehavior->initialize();
				} catch (std::exception &e) {
					std::cerr << "ERROR: In " << this->getName() << ", \"" << currentBehavior->getName() << "\" failed to initialize. what: " << e.what() << ", returning to default state" << std::endl;
					this->useDefaultBehavior();
				}
				startTime = currentTime();
			}
		}

		void operator()(Behavior* behavior) {
			this->setCurrentBehavior(behavior);
		}

		void operator()() {
			this->useDefaultBehavior();
		}
		
		/**
		 * @brief Transition to the default behavior
		 * 
		 */
		void useDefaultBehavior() {
			// If we are currently running the current behavior transition
			if (currentBehavior != nullptr) {
				try {
					currentBehavior->exit();
				} catch (std::exception &e) {
					std::cerr << "ERROR: In " << this->getName() << ", \"" << currentBehavior->getName() << "\" failed to exit. what: " << e.what() << ", returning to default state" << std::endl;
				}

				currentBehavior = nullptr;
				defaultBehavior->initialize();
				startTime = currentTime();
			}
			// If not no transition needed.
		}

		Behavior* getCurrentBehavior() {
			return currentBehavior;
		}

		std::string toString() {
			return this->getName() + ": " + (this->isDone() ? defaultBehavior->getName() : currentBehavior->getName());
		}

		QTime getDuration() {
			return currentTime() - startTime;
		}

		~StateController() = default;
	};
} // namespace Pronounce
