#pragma once

#include "behavior.hpp"
#include <string>
#include <unordered_map>
#include <mutex>

namespace Pronounce {
	class StateController : Behavior {
	private:
		Behavior* defaultBehavior;
		Behavior* currentBehavior;
	public:
		StateController();

		void initialize() {
			if (currentBehavior != nullptr) {
				currentBehavior->initialize();
			} else {
				if (defaultBehavior != nullptr) {
					defaultBehavior->initialize();
				}
			}
		}

		void update() {
			if (currentBehavior != nullptr) {
				if (currentBehavior->isDone()) {
					currentBehavior->exit();
					currentBehavior = nullptr;
					defaultBehavior->initialize();
				}
				else {
					currentBehavior->update();
				}
			}
			else {
				defaultBehavior->update();
			}
		}

		bool isDone() {
			return currentBehavior == nullptr;
		}

		void exit() {
			if (currentBehavior != nullptr) {
				currentBehavior->exit();
				currentBehavior = nullptr;
			} else {
				defaultBehavior->exit();
			}
		}

		void setDefaultBehavior(Behavior* behavior) {
			defaultBehavior = behavior;
		}

		void setCurrentBehavior(Behavior* behavior) {
			currentBehavior = behavior;
			defaultBehavior->exit();
			currentBehavior->initialize();
		}

		void useDefaultBehavior() {
			currentBehavior->exit();
			currentBehavior = nullptr;
			defaultBehavior->initialize();
		}

		~StateController();
	};
} // namespace Pronounce
