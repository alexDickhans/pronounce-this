#pragma once

#ifndef DEBUG
#include "behavior.hpp"
#include "stateController.hpp"
#endif // !DEBUG

// TODO: add docstrings

#include <utility>
#include <vector>
#include <iostream>
namespace Pronounce {
	class Sequence : public Behavior {
	private:
		std::vector<std::shared_ptr<StateController>> stateControllers;
		std::vector<std::shared_ptr<Behavior>> behaviors;

		int currentIndex = 0;
	public:
		explicit Sequence(std::string name) : Behavior(std::move(name)) {}

		void initialize() override {
			currentIndex = 0;
			if (!behaviors.empty()) {
				stateControllers.at(currentIndex)->setCurrentBehavior(behaviors.at(currentIndex));
			}
		}

		void update() override {
			
			// If the current behavior is done then we will move onto the next behavior
			if (behaviors.at(currentIndex)->isDone()) {
				if (currentIndex < stateControllers.size() - 1) {
					currentIndex++;
					stateControllers.at(currentIndex)->setCurrentBehavior(behaviors.at(currentIndex));
				}
				else {
					// std::cout << this->getName() << "Ending" << std::endl;
					stateControllers.at(currentIndex)->useDefaultBehavior();
					currentIndex++;
					// Done
				}
			}
		}

		void exit() override {
			if (!this->isDone()) {
				stateControllers.at(currentIndex)->useDefaultBehavior();
				currentIndex = stateControllers.size() - 1;
			}
		}

		bool isDone() override {
			return currentIndex >= stateControllers.size();
		}

		void addState(std::shared_ptr<StateController> stateController, std::shared_ptr<Behavior> behavior) {
			stateControllers.emplace_back(stateController);
			behaviors.emplace_back(behavior);
		}

		~Sequence() = default;
	};
} // namespace Pronounce
