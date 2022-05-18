#pragma once

#include "behavior.hpp"
#include "stateController.hpp"
#include <unordered_map>

namespace Pronounce {
	class Parallel : Behavior {
	private:
		std::unordered_map<StateController*, Behavior*> behaviors;
	public:
		Parallel();

		void initialize() {
			for (auto& behavior : behaviors) {
				behavior.first->setCurrentBehavior(behavior.second);
			}
		}

		void update() {
			// Don't do anything
		}

		void exit() {
			for (auto& behavior : behaviors) {
				behavior.first->useDefaultBehavior();
			}
		}

		void addBehavior(StateController* stateController, Behavior* behavior) {
			behaviors[stateController] = behavior;
		}

		void removeBehavior(StateController* stateController) {
			behaviors.erase(stateController);
		}

		std::unordered_map <StateController*, Behavior*> getBehaviors() {
			return behaviors;
		}

		void reset() {
			behaviors.clear();
		}

		~Parallel();
	};
} // namespace Pronounce

