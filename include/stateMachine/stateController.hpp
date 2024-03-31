#pragma once

#include "behavior.hpp"
#include "time/time.hpp"
#include "time/robotTime.hpp"
#include <string>
#include <unordered_map>
#include <iostream>
#include <utility>

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
		std::shared_ptr<Behavior> defaultBehavior;
		
		/**
		 * @brief The current behavior that runs if it exists.
		 * 
		 */
		std::shared_ptr<Behavior> currentBehavior = nullptr;

		QTime startTime = 0.0;
	public:
		/**
		 * @brief Construct a new State Controller object
		 * 
		 * @param defaultBehavior 
		 */
		StateController(std::string name, std::shared_ptr<Behavior> defaultBehavior) : Behavior(std::move(name)) {
			this->defaultBehavior = defaultBehavior;
		}

		/**
		 * @brief Initialize the behavior. If currentbehavior exists that one gets initialized
		 * 
		 */
		void initialize() override {
			Log_Desc(Behavior::getName() + ":" + this->getName(), "Initialize");
			startTime = currentTime();
			if (currentBehavior != nullptr) {
				try {
					currentBehavior->initialize();
				} catch(...) {
					Log_Desc(this->getName(), "ERROR " + currentBehavior->getName() + "\" failed to initialize, returning to default state");
					this->useDefaultBehavior();
					defaultBehavior->initialize();
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
			Log_Desc(Behavior::getName() + ":" + this->getName(), "Update");

			// If currentBehavior exists run it or transition
			if (currentBehavior != nullptr) {
				if (currentBehavior->isDone()) {
					try {
						currentBehavior->exit();
					} catch (std::exception &e) {
						Log_Desc(this->getName(), "ERROR " + currentBehavior->getName() + "\" failed to initialize, returning to default state");
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
						Log_Desc(this->getName(), "ERROR: In " + this->getName() + ", \"" + currentBehavior->getName() + "\" failed to update. what: " + e.what() + ", returning to default state");
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
		bool isDone() override {
			return currentBehavior == nullptr;
		}

		void waitUntilDone(uint32_t timeout = 15000) {
			auto timeoutStartTime = pros::millis();
			while(!this->isDone() && pros::millis() - timeoutStartTime <= timeout) {
				pros::Task::delay(10);
			}
		}

		void wait(uint32_t timeout = 15000) {
			waitUntilDone(timeout);
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
					Log_Desc(this->getName(), "ERROR: In " + this->getName() + ", \"" + currentBehavior->getName() + "\" failed to exit. what: " + e.what() + ", returning to default state");
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
		void setDefaultBehavior(std::shared_ptr<Behavior> behavior) {
			defaultBehavior = behavior;
		}

		/**
		 * @brief Set the Current Behavior object depending on what is running and what it equals
		 * 
		 * @param behavior 
		 */
		void setCurrentBehavior(const std::shared_ptr<Behavior>& behavior) {
			Log_Desc(this->getName(), "INFO: transitioning to behavior (" + behavior->getName() + ")");
			// If the defaultBehavior and current behavior are equal then switch to default behavior
			if (defaultBehavior == behavior) {
				this->useDefaultBehavior();
			}
			// If the currentBehavior exists and transition from that if it does
			else if (currentBehavior != nullptr) {
				try {
					currentBehavior->exit();
				} catch (std::exception &e) {
					Log_Desc(this->getName(), "ERROR: In " + this->getName() + ", \"" + currentBehavior->getName() + "\" failed to exit. what: " + e.what() + ", returning to default state");
				}
				try {
					currentBehavior = behavior;
					currentBehavior->initialize();
				} catch (std::exception &e) {
					Log_Desc(this->getName(), "ERROR: In " + this->getName() + ", \"" + currentBehavior->getName() + "\" failed to initialize. what: " + e.what() + ", returning to default state");
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
					Log_Desc(this->getName(), "ERROR: In " + this->getName() + ", \"" + currentBehavior->getName() + "\" failed to initialize. what: " + e.what() + ", returning to default state");
					this->useDefaultBehavior();
				}
				startTime = currentTime();
			}
		}

		StateController* sb(std::shared_ptr<Behavior> behavior) {
			this->setCurrentBehavior(behavior);
			return this;
		}

		StateController* ud() {
			this->useDefaultBehavior();
			return this;
		}

		StateController* operator()(std::shared_ptr<Behavior> behavior) {
			this->setCurrentBehavior(behavior);
			return this;
		}

		StateController* operator()() {
			this->useDefaultBehavior();
			return this;
		}
		
		/**
		 * @brief Transition to the default behavior
		 * 
		 */
		void useDefaultBehavior() {
			Log_Desc(this->getName(), "INFO: transitioning to default (" + this->defaultBehavior->getName() + ")");
			// If we are currently running the current behavior transition
			if (currentBehavior != nullptr) {
				try {
					currentBehavior->exit();
				} catch (std::exception &e) {
					Log_Desc(this->getName(), "ERROR: In " + this->getName() + ", \"" + currentBehavior->getName() + "\" failed on exit. what: " + e.what() + ", returning to default state");
				}

				currentBehavior = nullptr;
				defaultBehavior->initialize();
				startTime = currentTime();
			}
		}

		std::shared_ptr<Behavior> getCurrentBehavior() {
			return currentBehavior;
		}

		std::string toString() {
			return this->getName() + ": " + (this->isDone() ? defaultBehavior->getName() : currentBehavior->getName());
		}

		QTime getDuration() {
			return currentTime() - startTime;
		}

		std::string getName() override {
			if (this->currentBehavior != nullptr) {
				return this->currentBehavior->getName();
			} else {
				return this->defaultBehavior->getName();
			}
		}

		~StateController() = default;
	};
} // namespace Pronounce
