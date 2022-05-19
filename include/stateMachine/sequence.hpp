#pragma once

#include "behavior.hpp"
#include "stateController.hpp"
#include <vector>

namespace Pronounce {
	class Sequence : public Behavior {
	private:
		std::vector<StateController*> stateControllers;
		std::vector<Behavior*> behaviors;
		int currentIndex = 0;
	public:
		Sequence();

		void initialize() {
			if (behaviors.size() > 0) {
				stateControllers.at(0)->initialize();
			}
		}

		void update() {
			if (stateControllers.at(currentIndex)->isDone()) {
				if (currentIndex < stateControllers.size() - 1) {
					stateControllers.at(currentIndex)->useDefaultBehavior();
					currentIndex++;
					stateControllers.at(currentIndex)->setCurrentBehavior(behaviors.at(currentIndex));
				}
				else {
					stateControllers.at(currentIndex)->useDefaultBehavior();
					// Done
				}
			}
		}

		void exit() {
			stateControllers.at(currentIndex)->useDefaultBehavior();
			currentIndex = stateControllers.size()-1;
		}

		bool isDone() {
			return currentIndex >= stateControllers.size()-1;
		}

		~Sequence();
	};
} // namespace Pronounce
