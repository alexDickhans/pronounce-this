#pragma once
//
// Created by alex on 9/7/23.
//

#include <utility>

#include "stateMachine/behavior.hpp"

namespace Pronounce {
	class Wings : public Behavior {
	private:
		pros::ADIDigitalOut& wingsSolenoid;
		bool state{false};
	public:
		Wings(std::string name, pros::ADIDigitalOut& wingsSolenoid, bool state) : Behavior(std::move(name)), wingsSolenoid(wingsSolenoid), state(state) {

		}

		void initialize() override {
			wingsSolenoid.set_value(state);
		}

		void update() override {
			// none
		}

		void exit() override {

		}

		bool isDone() override {
			return false;
		}
	};
}
