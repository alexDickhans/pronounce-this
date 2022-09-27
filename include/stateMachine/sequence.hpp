#pragma once

#ifndef DEBUG
#include "behavior.hpp"
#include "stateController.hpp"
#endif // !DEBUG

// TODO: add docstrings

#include <vector>
#include <iostream>
namespace Pronounce {
	class Sequence : public Behavior {
	private:
		std::vector<StateController*> stateControllers;
		std::vector<Behavior*> behaviors;

		int currentIndex = 0;
	public:
		Sequence(std::string name) : Behavior(name) {}

		void initialize() {
			currentIndex = 0;
			if (behaviors.size() > 0) {
				stateControllers.at(currentIndex)->setCurrentBehavior(behaviors.at(currentIndex));
			}
		}

		void update() {
			
			if (behaviors.at(currentIndex)->isDone()) {
				if (currentIndex < stateControllers.size() - 1) {
					stateControllers.at(currentIndex)->useDefaultBehavior();
					currentIndex++;
					stateControllers.at(currentIndex)->setCurrentBehavior(behaviors.at(currentIndex));
				}
				else {
					std::cout << this->getname() << "Ending" << std::endl;
					stateControllers.at(currentIndex)->useDefaultBehavior();
					currentIndex++;
					// Done
				}
			}
		}

		void exit() {
			if (!this->isDone()) {
				stateControllers.at(currentIndex)->useDefaultBehavior();
				currentIndex = stateControllers.size() - 1;
			}
		}

		bool isDone() {
			return !(currentIndex < stateControllers.size());
		}

		void addState(StateController* stateController, Behavior* behavior) {
			stateControllers.emplace_back(stateController);
			behaviors.emplace_back(behavior);
		}

		~Sequence() {}
	};
} // namespace Pronounce
