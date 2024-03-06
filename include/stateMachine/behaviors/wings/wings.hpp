#pragma once
//
// Created by alex on 9/7/23.
//

#include <utility>

#include "stateMachine/behavior.hpp"

namespace Pronounce {
	class Wing : public Behavior {
	private:
		pros::adi::DigitalOut& solenoid;
		bool state{false};
	public:
		Wing(std::string name, pros::adi::DigitalOut& solenoid, bool state) : Behavior(std::move(name)), solenoid(solenoid), state(state) {

		}

		void initialize() override {
			solenoid.set_value(state);
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
