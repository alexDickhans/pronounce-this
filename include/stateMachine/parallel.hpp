#pragma once

#include "behavior.hpp"
#include "stateController.hpp"
#include <unordered_map>

namespace Pronounce {
	/**
	 * @brief A class to run several classes in parallel
	 * 
	 */
	class Parallel : Behavior {
	private:
		std::unordered_map<StateController*, Behavior*> behaviors;
	public:
		/**
		 * @brief Construct a new Parallel object
		 * 
		 */
		Parallel();

		/**
		 * @brief Start all the states
		 * 
		 */
		void initialize() {
			for (auto& behavior : behaviors) {
				behavior.first->setCurrentBehavior(behavior.second);
			}
		}

		/**
		 * @brief Update, doesn't do anything yet
		 * 
		 */
		void update() {
			// Don't do anything
		}

		/**
		 * @brief Exit all the behaviors
		 * 
		 */
		void exit() {
			for (auto& behavior : behaviors) {
				behavior.first->useDefaultBehavior();
			}
		}

		/**
		 * @brief Add a state to the parallel
		 * 
		 * @param stateController 
		 * @param behavior 
		 */
		void addBehavior(StateController* stateController, Behavior* behavior) {
			behaviors[stateController] = behavior;
		}

		/**
		 * @brief Remove a behavior by stateController
		 * 
		 * @param stateController 
		 */
		void removeBehavior(StateController* stateController) {
			behaviors.erase(stateController);
		}

		/**
		 * @brief Get the Behaviors object
		 * 
		 * @return std::unordered_map <StateController*, Behavior*> 
		 */
		std::unordered_map <StateController*, Behavior*> getBehaviors() {
			return behaviors;
		}

		/**
		 * @brief Reset the list
		 * 
		 */
		void reset() {
			behaviors.clear();
		}

		/**
		 * @brief Return true if all the states are complete
		 * 
		 * @return true Everything is complete
		 * @return false Not everything is complete
		 */
		bool isDone() {
			// If any behavior is not done return not done
			bool done = true;
			for (auto& behavior : behaviors) {
				done = done && behavior.first->isDone();
			}
			return done;
		}

		~Parallel();
	};
} // namespace Pronounce
